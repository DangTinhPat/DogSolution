#ifndef MICROROS_BRIDGE_H
#define MICROROS_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

/* micro-ROS node "stm32_joint_node" voi subscriber "/joint_cmd" + publishers
 * "/joint_fb", "/joint_diag" and compact "/imu/raw" (main_bot_hardware_msgs), qua UART1/
 * micro_ros_agent thay vi CAN. Cung reconnect
 * state machine (ping dinh ky khi da connected) voi OUT_SAVE/testSTM/app/src/microros_bridge.c,
 * nhung KHONG FreeRTOS (dung Tick_GetMs() thay xTaskGetTickCount(), khong goi
 * MicroRos_InstallFreeRTOSAllocator() - xem microros_time.c va ghi chu trong .c).
 *
 * Callback subscription goi thang Actuator_SetTarget() (actuator_if.h) va tu cap nhat
 * timestamp lan cuoi nhan duoc /joint_cmd. main.c dung timestamp nay cho watchdog:
 * mat lenh thi ngat luc, khong chay FSM hay fallback ve goc co dinh. */
void MicroRosBridge_Begin(void);

/* Goi moi vong superloop cua main(). Khi connected, spin cho toi da 1ms va ping
 * health toi da 50ms x 2; khi waiting, rcl entity/session creation co the block
 * theo retry policy cua micro-ROS (motor da duoc watchdog disable truoc khi vao lai). */
void MicroRosBridge_SpinSome(void);

bool MicroRosBridge_IsConnected(void);

uint32_t MicroRosBridge_LastJointCmdMs(void);
bool MicroRosBridge_HasReceivedJointCmd(void);
uint32_t MicroRosBridge_JointCmdCount(void);

/* Drop only the previous publisher's sequence baseline after the command
 * watchdog expires. Diagnostic counters remain cumulative; the first command
 * from a restarted host establishes a new baseline instead of reporting a
 * false packet gap. */
void MicroRosBridge_ResetCommandSequence(void);

/* No-op (bo qua) neu chua connected - an toan goi vo dieu kien moi chu ky
 * JOINT_FB_SEND_PERIOD_MS trong main.c. Tu doc Actuator_GetMeasured() ben trong. */
void MicroRosBridge_PublishJointFb(void);

/* Low-rate actuator/CAN/transport counters; main.c calls at 10 Hz. */
void MicroRosBridge_PublishJointDiag(void);

typedef enum
{
    MICROROS_IMU_STATUS_OK = 1U,
    MICROROS_IMU_STATUS_INIT_FAILED = 2U,
    MICROROS_IMU_STATUS_READ_FAILED = 4U,
} MicroRosImuStatus;

struct MPU6050_Reading;

/* Publish one compact fixed-size IMU sample. reading may be NULL for a status-only
 * error frame; the EC Kalman node rejects any frame without STATUS_OK. */
void MicroRosBridge_PublishImuRaw(const struct MPU6050_Reading *reading,
                                  MicroRosImuStatus status,
                                  uint32_t stamp_ms);

#endif /* MICROROS_BRIDGE_H */
