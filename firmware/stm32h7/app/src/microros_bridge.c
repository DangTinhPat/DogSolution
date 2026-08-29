#include "microros_bridge.h"

#include "tick.h"
#include "actuator_if.h"
#include "motor_topology.h"   /* JOINT_COUNT */
#include "microros_transport.h"
#include "mpu6050.h"

#include <limits.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <main_bot_hardware_msgs/msg/imu_raw.h>
#include <main_bot_hardware_msgs/msg/joint_cmd.h>
#include <main_bot_hardware_msgs/msg/joint_diag.h>
#include <main_bot_hardware_msgs/msg/joint_fb.h>

/* JointCmd.msg dung mang co dinh [12]. Neu topology firmware doi kich thuoc
 * ma message chua regenerate, dung build ngay thay vi doc/ghi lech mang. */
_Static_assert(JOINT_COUNT == 12, "JointCmd/JointFb contract requires exactly 12 joints");

/* Gioi han spin o 1ms: benchmark voi timeout 5ms tung lam /joint_fb chi dat
 * khoang 143Hz thay vi 200Hz. RX da duoc ISR nap vao ring rieng, nen executor
 * khong can block lau de giu kip du lieu serial. */
#define EXECUTOR_SPIN_TIMEOUT_NS RCL_MS_TO_NS(1)
#define PING_INTERVAL_MS 2000U   /* health check khi da connected - ping moi lan spin se
                                   * chiem het bang thong UART */
#define PING_TIMEOUT_MS 50
#define PING_ATTEMPTS 2

typedef enum
{
    STATE_WAITING_AGENT,
    STATE_CONNECTED,
} BridgeState;

static BridgeState state = STATE_WAITING_AGENT;
static uint32_t last_ping_ms = 0U;
static uint32_t last_joint_cmd_ms = 0U;
static bool has_received_joint_cmd;
static uint32_t command_rx_count;
static uint32_t command_seq_gap_count;
static uint32_t command_seq_duplicate_count;
static uint32_t command_seq_out_of_order_count;
static uint32_t joint_fb_publish_fail_count;
static uint32_t imu_publish_fail_count;
static uint32_t joint_diag_publish_fail_count;
static uint8_t last_command_seq;
static bool has_last_command_seq;

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_publisher_t joint_fb_pub;
static rcl_publisher_t imu_raw_pub;
static rcl_publisher_t joint_diag_pub;
static rcl_subscription_t joint_cmd_sub;
static main_bot_hardware_msgs__msg__ImuRaw imu_raw_msg;
static main_bot_hardware_msgs__msg__JointDiag joint_diag_msg;
static main_bot_hardware_msgs__msg__JointFb joint_fb_msg;
static main_bot_hardware_msgs__msg__JointCmd joint_cmd_msg;

/* Entity creation may fail at any step while the serial agent is absent.
 * Track exactly what became live so cleanup never passes a zero/partial entity
 * into an rcl fini function, then reset every handle before the next retry. */
typedef struct
{
    bool support;
    bool node;
    bool joint_fb_publisher;
    bool imu_raw_publisher;
    bool joint_diag_publisher;
    bool joint_cmd_subscription;
    bool executor;
} EntityLifecycle;

static EntityLifecycle entity_lifecycle;

static void ResetEntityHandles(void)
{
    support = (rclc_support_t){0};
    node = rcl_get_zero_initialized_node();
    executor = rclc_executor_get_zero_initialized_executor();
    joint_fb_pub = rcl_get_zero_initialized_publisher();
    imu_raw_pub = rcl_get_zero_initialized_publisher();
    joint_diag_pub = rcl_get_zero_initialized_publisher();
    joint_cmd_sub = rcl_get_zero_initialized_subscription();
    entity_lifecycle = (EntityLifecycle){0};
}

static void JointCmdCallback(const void *msgin)
{
    const main_bot_hardware_msgs__msg__JointCmd *cmd =
        (const main_bot_hardware_msgs__msg__JointCmd *)msgin;

    command_rx_count++;
    if (has_last_command_seq)
    {
        const uint8_t delta = (uint8_t)(cmd->seq - last_command_seq);
        if (delta == 0U)
        {
            if (command_seq_duplicate_count != UINT32_MAX) { command_seq_duplicate_count++; }
        }
        else if (delta <= 128U)
        {
            const uint32_t missing = (uint32_t)delta - 1U;
            if (UINT32_MAX - command_seq_gap_count < missing)
            {
                command_seq_gap_count = UINT32_MAX;
            }
            else
            {
                command_seq_gap_count += missing;
            }
            last_command_seq = cmd->seq;
        }
        else if (command_seq_out_of_order_count != UINT32_MAX)
        {
            command_seq_out_of_order_count++;
        }
    }
    else
    {
        has_last_command_seq = true;
        last_command_seq = cmd->seq;
    }

    float angles_rad[JOINT_COUNT];
    float velocities_rad_s[JOINT_COUNT];
    float kp[JOINT_COUNT];
    float kd[JOINT_COUNT];
    float tau_ff_nm[JOINT_COUNT];
    for (uint32_t i = 0; i < JOINT_COUNT; i++)
    {
        angles_rad[i] = (float)cmd->target_angle_mrad[i] / 1000.0f;
        velocities_rad_s[i] = (float)cmd->target_velocity_mrad_s[i] / 1000.0f;
        kp[i] = (float)cmd->kp_x100[i] / 100.0f;
        kd[i] = (float)cmd->kd_x100[i] / 100.0f;
        tau_ff_nm[i] = (float)cmd->tau_ff_mnm[i] / 1000.0f;
    }

    Actuator_SetTarget(angles_rad, velocities_rad_s, kp, kd, tau_ff_nm);
    last_joint_cmd_ms = Tick_GetMs();
    has_received_joint_cmd = true;
}

/* rclc_executor capacity=1 because publishers are not executor handles; the sole
 * handle is /joint_cmd. colcon.meta reserves 1 subscription + 3 publishers. */
static bool CreateEntities(void)
{
    ResetEntityHandles();
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.support = true;
    if (rclc_node_init_default(&node, "stm32_joint_node", "", &support) != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.node = true;
    /* best_effort ca 2 chieu: mat 1 frame khong sao (frame sau bu, gui o nhip cao hon
     * timeout /joint_cmd nhieu lan) - tranh vong khu hoi ACK cua reliable lam giam
     * tan so thuc te, dung ly do da xac nhan qua IMU (xem OUT_SAVE/testSTM/README.md). */
    if (rclc_publisher_init_best_effort(
            &joint_fb_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(main_bot_hardware_msgs, msg, JointFb),
            "/joint_fb") != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.joint_fb_publisher = true;
    if (rclc_publisher_init_best_effort(
            &imu_raw_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(main_bot_hardware_msgs, msg, ImuRaw),
            "/imu/raw") != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.imu_raw_publisher = true;
    if (rclc_publisher_init_best_effort(
            &joint_diag_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(main_bot_hardware_msgs, msg, JointDiag),
            "/joint_diag") != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.joint_diag_publisher = true;
    if (rclc_subscription_init_best_effort(
            &joint_cmd_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(main_bot_hardware_msgs, msg, JointCmd),
            "/joint_cmd") != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.joint_cmd_subscription = true;
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
    {
        return false;
    }
    entity_lifecycle.executor = true;
    if (rclc_executor_add_subscription(
            &executor, &joint_cmd_sub, &joint_cmd_msg, &JointCmdCallback, ON_NEW_DATA) != RCL_RET_OK)
    {
        return false;
    }
    return true;
}

static void DestroyEntities(void)
{
    rcl_ret_t cleanup_result = RCL_RET_OK;

    if (entity_lifecycle.support)
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        if (rmw_context != NULL)
        {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    if (entity_lifecycle.executor)
    {
        (void)rclc_executor_fini(&executor);
    }
    if (entity_lifecycle.joint_cmd_subscription)
    {
        cleanup_result = rcl_subscription_fini(&joint_cmd_sub, &node);
    }
    if (entity_lifecycle.joint_diag_publisher)
    {
        cleanup_result = rcl_publisher_fini(&joint_diag_pub, &node);
    }
    if (entity_lifecycle.imu_raw_publisher)
    {
        cleanup_result = rcl_publisher_fini(&imu_raw_pub, &node);
    }
    if (entity_lifecycle.joint_fb_publisher)
    {
        cleanup_result = rcl_publisher_fini(&joint_fb_pub, &node);
    }
    if (entity_lifecycle.node)
    {
        cleanup_result = rcl_node_fini(&node);
    }
    if (entity_lifecycle.support)
    {
        (void)rclc_support_fini(&support);
    }
    (void)cleanup_result;
    ResetEntityHandles();
}

void MicroRosBridge_Begin(void)
{
    main_bot_hardware_msgs__msg__ImuRaw__init(&imu_raw_msg);
    main_bot_hardware_msgs__msg__JointDiag__init(&joint_diag_msg);
    main_bot_hardware_msgs__msg__JointFb__init(&joint_fb_msg);
    main_bot_hardware_msgs__msg__JointCmd__init(&joint_cmd_msg);

    last_joint_cmd_ms = 0U;
    has_received_joint_cmd = false;
    command_rx_count = 0U;
    command_seq_gap_count = 0U;
    command_seq_duplicate_count = 0U;
    command_seq_out_of_order_count = 0U;
    joint_fb_publish_fail_count = 0U;
    imu_publish_fail_count = 0U;
    joint_diag_publish_fail_count = 0U;
    last_command_seq = 0U;
    has_last_command_seq = false;

    /* Khong goi allocator rieng nhu ban OUT_SAVE/testSTM (MicroRos_InstallFreeRTOSAllocator) -
     * khong FreeRTOS o day, rcutils's default allocator da bọc san malloc/free/realloc/
     * calloc cua newlib (hoat dong qua nosys.specs + symbol _end trong linker script,
     * xem README.md), dung thang duoc. */
    MicroRos_RegisterTransport();
    ResetEntityHandles();
    state = STATE_WAITING_AGENT;
}

void MicroRosBridge_SpinSome(void)
{
    switch (state)
    {
        case STATE_WAITING_AGENT:
            if (CreateEntities())
            {
                state = STATE_CONNECTED;
                last_ping_ms = Tick_GetMs();
            }
            else
            {
                DestroyEntities();
                /* rclc_support_init() da tu cho/retry noi bo (UCLIENT_MIN_SESSION_
                 * CONNECTION_INTERVAL, xem toolchain.cmake) - vong lap superloop ben
                 * ngoai (main.c) la du, khong can them delay/ping o day. */
            }
            break;

        case STATE_CONNECTED:
        {
            const uint32_t now = Tick_GetMs();
            if ((now - last_ping_ms) >= PING_INTERVAL_MS)
            {
                last_ping_ms = now;
                if (rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS) != RMW_RET_OK)
                {
                    DestroyEntities();
                    state = STATE_WAITING_AGENT;
                    break;
                }
            }
            (void)rclc_executor_spin_some(&executor, EXECUTOR_SPIN_TIMEOUT_NS);
            break;
        }
    }
}

bool MicroRosBridge_IsConnected(void)
{
    return state == STATE_CONNECTED;
}

uint32_t MicroRosBridge_LastJointCmdMs(void)
{
    return last_joint_cmd_ms;
}

bool MicroRosBridge_HasReceivedJointCmd(void)
{
    return has_received_joint_cmd;
}

uint32_t MicroRosBridge_JointCmdCount(void)
{
    return command_rx_count;
}

void MicroRosBridge_ResetCommandSequence(void)
{
    has_last_command_seq = false;
}

static int16_t scale_to_i16(float value, float scale);

void MicroRosBridge_PublishJointFb(void)
{
    if (state != STATE_CONNECTED)
    {
        return;
    }

    float angles_rad[JOINT_COUNT];
    float velocities_rad_s[JOINT_COUNT];
    float efforts_nm[JOINT_COUNT];
    joint_fb_msg.valid_mask = Actuator_GetMeasured(
        angles_rad, velocities_rad_s, efforts_nm);
    for (uint32_t i = 0; i < JOINT_COUNT; i++)
    {
        joint_fb_msg.measured_angle_mrad[i] = scale_to_i16(angles_rad[i], 1000.0f);
        joint_fb_msg.measured_velocity_mrad_s[i] = scale_to_i16(velocities_rad_s[i], 1000.0f);
        joint_fb_msg.measured_effort_mnm[i] = scale_to_i16(efforts_nm[i], 1000.0f);
    }

    if (rcl_publish(&joint_fb_pub, &joint_fb_msg, NULL) != RCL_RET_OK &&
        joint_fb_publish_fail_count != UINT32_MAX)
    {
        joint_fb_publish_fail_count++;
    }
}

static int16_t scale_to_i16(float value, float scale)
{
    if (!isfinite(value))
    {
        return 0;
    }
    float scaled = value * scale;
    if (scaled > (float)INT16_MAX)
    {
        scaled = (float)INT16_MAX;
    }
    else if (scaled < (float)INT16_MIN)
    {
        scaled = (float)INT16_MIN;
    }
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

void MicroRosBridge_PublishJointDiag(void)
{
    if (state != STATE_CONNECTED) { return; }

    ActuatorDiagnostics diagnostics;
    Actuator_GetDiagnostics(&diagnostics);
    joint_diag_msg.ready_mask = diagnostics.ready_mask;
    joint_diag_msg.fresh_mask = diagnostics.fresh_mask;
    joint_diag_msg.runtime_fault_mask = diagnostics.runtime_fault_mask;
    joint_diag_msg.can_bus_off_mask = diagnostics.can_bus_off_mask;
    for (uint32_t i = 0U; i < JOINT_COUNT; i++)
    {
        joint_diag_msg.status_raw_le[i] = diagnostics.status_raw_le[i];
        joint_diag_msg.telemetry_age_ms[i] = diagnostics.telemetry_age_ms[i];
    }
    joint_diag_msg.can_tx_fail_count[0] = diagnostics.can_tx_fail_count[0];
    joint_diag_msg.can_tx_fail_count[1] = diagnostics.can_tx_fail_count[1];
    joint_diag_msg.command_rx_count = command_rx_count;
    joint_diag_msg.command_seq_gap_count = command_seq_gap_count;
    joint_diag_msg.command_seq_duplicate_count = command_seq_duplicate_count;
    joint_diag_msg.command_seq_out_of_order_count = command_seq_out_of_order_count;
    joint_diag_msg.joint_fb_publish_fail_count = joint_fb_publish_fail_count;
    joint_diag_msg.imu_publish_fail_count = imu_publish_fail_count;
    joint_diag_msg.joint_diag_publish_fail_count = joint_diag_publish_fail_count;

    if (rcl_publish(&joint_diag_pub, &joint_diag_msg, NULL) != RCL_RET_OK &&
        joint_diag_publish_fail_count != UINT32_MAX)
    {
        joint_diag_publish_fail_count++;
    }
}

void MicroRosBridge_PublishImuRaw(const MPU6050_Reading *reading,
                                  MicroRosImuStatus status,
                                  uint32_t stamp_ms)
{
    if (state != STATE_CONNECTED)
    {
        return;
    }

    for (uint32_t i = 0U; i < 3U; ++i)
    {
        const float acceleration = (reading != NULL) ? reading->linear_acceleration[i] : 0.0f;
        const float angular_velocity = (reading != NULL) ? reading->angular_velocity[i] : 0.0f;
        imu_raw_msg.linear_acceleration_milli_ms2[i] = scale_to_i16(acceleration, 1000.0f);
        imu_raw_msg.angular_velocity_mrad_s[i] = scale_to_i16(angular_velocity, 1000.0f);
    }
    imu_raw_msg.stamp_ms = stamp_ms;
    imu_raw_msg.status = (uint8_t)status;
    if (rcl_publish(&imu_raw_pub, &imu_raw_msg, NULL) != RCL_RET_OK &&
        imu_publish_fail_count != UINT32_MAX)
    {
        imu_publish_fail_count++;
    }
}
