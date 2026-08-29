#include "actuator_if.h"
#include "motor_topology.h"
#include "motor_calib.h"
#include "baby_alpha2_protocol.h"
#include "can.h"
#include "tick.h"
#include <string.h>
#include <stdbool.h>
#include <limits.h>

#define BA2_PI_F        3.14159265358979323846f
#define BA2_DEG2RAD(d)  ((d) * (BA2_PI_F / 180.0f))
#define BA2_RAD2DEG(r)  ((r) * (180.0f / BA2_PI_F))

/* Vị trí ngồi/crouch làm giá trị mặc định TRƯỚC KHI Actuator_Init() hiệu
 * chuẩn xong (hoặc cho khớp hiệu chuẩn thất bại) - khớp với sit_pos trong
 * main_bot/config/controllers.yaml (phía ROS2). KHÔNG GIAN LOGIC (xem
 * motor_calib.h header). */
static float g_last_target[JOINT_COUNT] = {
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f,
    0.0f, -1.231f, 2.462f
};
static float g_measured_angle[JOINT_COUNT];      /* khong gian LOGIC */
static float g_measured_velocity[JOINT_COUNT];   /* khong gian LOGIC */
static float g_measured_effort[JOINT_COUNT];     /* khong gian LOGIC */
static uint32_t g_last_telemetry_ms[JOINT_COUNT];
static uint16_t g_status_raw_le[JOINT_COUNT];
static int g_has_feedback[JOINT_COUNT];
static int g_feedback_required[JOINT_COUNT];
static int g_active_required[JOINT_COUNT];
static uint32_t g_last_set_target_ms;
static bool g_has_set_target;
static uint16_t g_runtime_fault_mask;
static uint16_t g_disable_pending_mask;
static uint32_t g_can_tx_fail_count[2];

/* true = khớp này hiệu chuẩn thành công lúc Actuator_Init() (PING/HANDSHAKE,
 * ENABLE/telemetry va HOME deu OK; SETUP ACK la best-effort vi khong phai
 * moi driver revision deu tra ve) - Actuator_SetTarget() BỎ QUA hoàn toàn khớp hiệu chuẩn thất bại,
 * không gửi lệnh gì (an toàn hơn là gửi lệnh với giới hạn không đáng tin). */
static int g_joint_ok[JOINT_COUNT];

/* Giới hạn hành trình TUYỆT ĐỐI (rad, KHÔNG GIAN LOGIC), tính từ HOME đo
 * được lúc boot + độ lệch tương đối cố định (motor_calib.h) - dùng làm lớp
 * kẹp AN TOÀN THỨ HAI trong Actuator_SetTarget(), độc lập với SETUP_LIMITS
 * đã gửi cho driver (oneLeg/HW.md mục 8 "lớp an toàn": 2 lớp bảo vệ nhau). */
static float g_limit_max_rad[JOINT_COUNT];
static float g_limit_min_rad[JOINT_COUNT];

/* Offset hieu chuan ADAPTIVE (rad, khong gian LOGIC) - bu vao moi lan chuyen
 * doi RAW<->LOGIC de "diem 0 tuyet doi" cua encoder driver (TROI moi lan mat
 * dien - da xac nhan qua oneLeg/main_adaptive_confirmed_v2.c.bak) khop voi
 * gia dinh ASSUMED_REST_LOGICAL_RAD duoi day. Tinh lai MOI LAN boot trong
 * Actuator_Init() tu chinh HOME do duoc phien nay - KHONG dung bang hang so
 * co dinh (offset thuc te doi tung phien cap dien). Mac dinh 0 (memset) cho
 * khop chua/khong hieu chuan duoc. */
static float g_home_offset_rad[JOINT_COUNT];

/* Gia tri LOGIC gia dinh dung cho tu the HOME (nam xap luc cap nguon, xem
 * motor_calib.h) - chi so theo JointType_t (1=Hang,2=Dui,3=Goi), [0] khong
 * dung. Moc 0.0 cho ca 3 loai da duoc dong bo voi origin/limit hieu chinh
 * trong babydog.xacro; firmware va URDF phai tiep tuc thay doi cung nhau. */
static const float ASSUMED_REST_LOGICAL_RAD[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* ===== Chuyen doi khong gian LOGIC <-> RAW (xem motor_calib.h header) =====
 * Diem DUY NHAT trong toan bo firmware biet ve MOTOR_JOINT_SIGN - moi noi
 * khac (gioi han, HOME, g_last_target/g_measured_*) deu lam viec thuan tuy
 * trong khong gian LOGIC (DA bao gom offset hieu chuan, xem g_home_offset_rad
 * o tren). */
static float LogicalToRaw(JointIndex_t joint, float logical_rad)
{
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * (logical_rad - g_home_offset_rad[joint]);
}

static float RawToLogical(JointIndex_t joint, float raw_rad)
{
    /* sign = +-1 nen dao nguoc dung la nhan lai chinh no (sign*sign=1). */
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * raw_rad + g_home_offset_rad[joint];
}

/* Torque khong co offset: chi doi dau theo cung mapping lap guong cua goc.
 * Neu lenh vi tri logic duong can torque logic duong de ho tro no, ca hai
 * phai toi motor voi cung phep doi dau. */
static float LogicalTorqueToRaw(JointIndex_t joint, float logical_nm)
{
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * logical_nm;
}

/* Velocity la dao ham cua position: cung doi dau lap guong, khong bao gio
 * cong home offset. Tach ham de khong vo tinh dung LogicalToRaw() (ham do co
 * offset vi tri) cho v_des. */
static float LogicalVelocityToRaw(JointIndex_t joint, float logical_rad_s)
{
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * logical_rad_s;
}

static float RawVelocityToLogical(JointIndex_t joint, float raw_rad_s)
{
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * raw_rad_s;
}

static float RawTorqueToLogical(JointIndex_t joint, float raw_nm)
{
    const int sign = MOTOR_JOINT_SIGN[Motor_LegGroupForJoint(joint)][Motor_JointTypeForJoint(joint)];
    return (float)sign * raw_nm;
}

static uint32_t bus_index(uint32_t instance)
{
    return (instance == CAN_INSTANCE_2) ? 1U : 0U;
}

static void record_tx_failure(uint32_t instance)
{
    const uint32_t index = bus_index(instance);
    if (g_can_tx_fail_count[index] != UINT32_MAX)
    {
        g_can_tx_fail_count[index]++;
    }
}

static bool can_tx(uint32_t instance, const CAN_Frame *f)
{
    for (uint32_t i = 0U; i < 200U; i++)
    {
        if (CAN_Transmit(instance, f)) { return true; }
    }
    record_tx_failure(instance);
    return false;
}

static void disable_all_drivers_best_effort(void)
{
    /* The STM32 can reboot while joint drivers remain powered and retain the
     * last active PD target. Overwrite that state before the relatively long
     * per-joint initialization sequence starts. Failed sends are counted and
     * the affected joint will still fail the later handshake/telemetry gates. */
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        const JointIndex_t joint = (JointIndex_t)j;
        const CAN_Frame disable = BA2_BuildSystemFrame(
            Motor_IdForJoint(joint), BA2_OPCODE_MOTOR_DISABLE);
        (void)can_tx(Motor_BusForJoint(joint), &disable);
    }
}

typedef bool (*MatchFn)(const CAN_Frame *rx, uint32_t id, const void *context);
static bool match_ping(const CAN_Frame *rx, uint32_t id, const void *context)
{
    (void)context;
    return rx->data_len >= 12U && rx->data[0] == BA2_REPLY_PING(id);
}
static bool match_hs(const CAN_Frame *rx, uint32_t id, const void *context)
{
    (void)context;
    return rx->data_len >= 12U && rx->data[0] == BA2_REPLY_HS(id);
}
static bool match_setup(const CAN_Frame *rx, uint32_t id, const void *context)
{
    (void)context;
    /* Driver revisions do not expose a verified, stable full-payload echo.
     * When an ACK is present, data[0] is the only validated identity field. */
    return rx->data_len >= 1U && rx->data[0] == BA2_REPLY_SETUP(id);
}
static bool match_page0(const CAN_Frame *rx, uint32_t id)
{
    return rx->data_len >= 12U && rx->data[0] == BA2_REPLY_PAGE0(id);
}

static bool status_is_explicitly_disabled(uint16_t status_raw_le);

/* Gửi rồi chờ đúng phản hồi mong đợi (retry nếu timeout) - MỌI phản hồi
 * BabyAlpha2 về CAN ID=0 (xem motor_topology.h header), phân biệt bằng
 * data[0] qua tham số m/id. CHỈ dùng trong Actuator_Init() (trước khi
 * main.c vào vòng lặp chính) - đây là nơi DUY NHẤT khác main.c tự gọi
 * CAN_Receive(), an toàn vì không có ai khác đọc FIFO cùng lúc lúc boot. */
static bool send_wait(uint32_t instance, const CAN_Frame *f, MatchFn m, uint32_t id,
                       const void *context, uint32_t timeout_ms, uint32_t retries)
{
    for (uint32_t a = 0U; a <= retries; a++)
    {
        if (!can_tx(instance, f)) { continue; }
        const uint32_t t0 = Tick_GetMs();
        while ((Tick_GetMs() - t0) < timeout_ms)
        {
            CAN_Frame rx;
            while (CAN_IsRxPending(instance) && CAN_Receive(instance, &rx))
            {
                if (rx.id == 0U && m(&rx, id, context)) { return true; }
            }
        }
    }
    return false;
}

/* Nhắc MOTOR_ENABLE cho MỌI khớp đã PING thành công TRỪ khớp đang xử lý -
 * dùng giữa các bước hiệu chuẩn TUẦN TỰ để không khớp nào bị "bỏ đói" quá
 * lâu trong lúc khớp khác đang hiệu chuẩn (driver tự rơi Standby nếu thiếu
 * nhắc định kỳ) - 12 khớp nên chuỗi hiệu chuẩn dài hơn nhiều so với 1 chân
 * (oneLeg/HW.md mục 8). */
static void keepalive_others(JointIndex_t skip_joint)
{
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        if ((JointIndex_t)j == skip_joint || !g_joint_ok[j]) { continue; }
        const JointIndex_t joint = (JointIndex_t)j;
        const CAN_Frame en = BA2_BuildSystemFrame(Motor_IdForJoint(joint), BA2_OPCODE_MOTOR_ENABLE);
        (void)can_tx(Motor_BusForJoint(joint), &en);
    }
}

/* Nhac lai MOTOR_ENABLE cho MOI khop da PING OK - PHAI goi dinh ky TRONG SUOT
 * luc chay (khong chi luc boot). Truoc day chi goi keepalive_others() ben
 * trong Actuator_Init() (chi luc hieu chuan), main.c's vong lap chinh KHONG
 * bao gio goi lai - driver BabyAlpha2 tu roi Standby (mat luc, bo qua lenh
 * PD) sau 1 khoang khong duoc nhac, dung sau khi Init() xong la het nhac -
 * gay hien tuong dong co "keu lach cach" (driver co gang xu ly frame nhung
 * khong o trang thai Enable de sinh momen) du PD van gui deu qua /joint_cmd.
 * Xac nhan qua oneLeg/main_12joint_hold_proven.c.bak (da chay that): ho nhac
 * lai moi RE_ENABLE_PERIOD_MS=500ms trong main loop, khong chi luc Init().
 * main.c goi ham nay voi cung chu ky 500ms. */
void Actuator_ReenableAll(void)
{
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        if (!g_joint_ok[j]) { continue; }
        const JointIndex_t joint = (JointIndex_t)j;
        const CAN_Frame en = BA2_BuildSystemFrame(Motor_IdForJoint(joint), BA2_OPCODE_MOTOR_ENABLE);
        (void)can_tx(Motor_BusForJoint(joint), &en);
    }
}

/* PING+HANDSHAKE+MOTOR_ENABLE - CHƯA gửi SETUP_LIMITS (chưa biết HOME) -
 * an toàn vì chưa gửi Kp/Kd khác 0 (oneLeg/HW.md mục 5). KHÔNG gửi
 * CALIB_ENCODER (0xF4, xem baby_alpha2_protocol.h) - tài liệu chính thức
 * liệt kê bước này trong chuỗi khởi tạo 5 bước, nhưng công thức tính offset
 * (Byte 1-2) chưa được xác nhận rõ ràng trên phần cứng của dự án này; thay
 * vào đó dùng cách hiệu chuẩn thích ứng (đọc HOME + giới hạn tương đối) đã
 * kiểm chứng qua oneLeg, không cần hiểu offset đó là gì. */
static bool ping_handshake_enable(JointIndex_t joint)
{
    const uint32_t instance = Motor_BusForJoint(joint);
    const uint32_t id = Motor_IdForJoint(joint);

    CAN_Frame f = BA2_BuildSystemFrame(id, BA2_OPCODE_PING);
    if (!send_wait(instance, &f, match_ping, id, NULL, 300U, 4U))
    {
        return false;
    }

    f = BA2_BuildHandshakeFrame(id);
    if (!send_wait(instance, &f, match_hs, id, NULL, 300U, 4U))
    {
        return false;
    }

    f = BA2_BuildSystemFrame(id, BA2_OPCODE_MOTOR_ENABLE);
    return can_tx(instance, &f);
}

/* Đọc vị trí thực bằng khung PD Kp=Kd=tau=0 (động cơ KHÔNG sinh lực trong
 * lúc đọc) - dùng làm HOME của phiên khởi động này (khong gian RAW, chuyen
 * sang LOGIC ngay truoc khi tra ve). */
static bool read_home_logical(JointIndex_t joint, float *logical_rad,
                              CAN_Frame *telemetry_out)
{
    const uint32_t instance = Motor_BusForJoint(joint);
    const uint32_t id = Motor_IdForJoint(joint);
    for (uint32_t a = 0U; a < 20U; a++)
    {
        const CAN_Frame probe = BA2_BuildPdFrame(
            id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, MOTOR_TAU_ABS_LIMIT_NM);
        if (!can_tx(instance, &probe)) { continue; }
        const uint32_t t0 = Tick_GetMs();
        while ((Tick_GetMs() - t0) < 20U)
        {
            CAN_Frame rx;
            while (CAN_IsRxPending(instance) && CAN_Receive(instance, &rx))
            {
                if (rx.id == 0U && match_page0(&rx, id))
                {
                    const uint16_t status_raw_le = (uint16_t)rx.data[10] |
                                                   ((uint16_t)rx.data[11] << 8);
                    if (status_is_explicitly_disabled(status_raw_le)) { continue; }
                    const float raw_rad = BA2_DecodePosAct(((uint16_t)rx.data[1] << 8) | rx.data[2]);
                    *logical_rad = RawToLogical(joint, raw_rad);
                    if (telemetry_out != NULL) { *telemetry_out = rx; }
                    return true;
                }
            }
        }
    }
    return false;
}

/* limit_max/min_logical: khong gian LOGIC (g_limit_max_rad/min_rad). Chuyen
 * sang RAW qua LogicalToRaw() truoc khi ma hoa - neu dau la -1, phep nhan
 * dao ca thu tu max/min nen phai tu sap xep lai (khong duoc gia dinh
 * raw_max > raw_min). */
static bool send_setup_limits(JointIndex_t joint, float limit_max_logical, float limit_min_logical)
{
    const uint32_t instance = Motor_BusForJoint(joint);
    const uint32_t id = Motor_IdForJoint(joint);

    const float raw_a = LogicalToRaw(joint, limit_max_logical);
    const float raw_b = LogicalToRaw(joint, limit_min_logical);
    const float raw_max = (raw_a > raw_b) ? raw_a : raw_b;
    const float raw_min = (raw_a > raw_b) ? raw_b : raw_a;

    const uint16_t max_raw = BA2_EncodePosAct(raw_max);
    const uint16_t min_raw = BA2_EncodePosAct(raw_min);
    const CAN_Frame f = BA2_BuildSetupLimitsFrame(id, max_raw, min_raw, MOTOR_VMAX_RAW);
    return send_wait(instance, &f, match_setup, id, &f, 300U, 4U);
}

void Actuator_Init(void)
{
    memset(g_measured_angle, 0, sizeof(g_measured_angle));
    memset(g_measured_velocity, 0, sizeof(g_measured_velocity));
    memset(g_measured_effort, 0, sizeof(g_measured_effort));
    memset(g_last_telemetry_ms, 0, sizeof(g_last_telemetry_ms));
    memset(g_status_raw_le, 0, sizeof(g_status_raw_le));
    memset(g_has_feedback, 0, sizeof(g_has_feedback));
    memset(g_feedback_required, 0, sizeof(g_feedback_required));
    memset(g_active_required, 0, sizeof(g_active_required));
    memset(g_joint_ok, 0, sizeof(g_joint_ok));
    memset(g_limit_max_rad, 0, sizeof(g_limit_max_rad));
    memset(g_limit_min_rad, 0, sizeof(g_limit_min_rad));
    memset(g_home_offset_rad, 0, sizeof(g_home_offset_rad));
    memset(g_can_tx_fail_count, 0, sizeof(g_can_tx_fail_count));
    g_last_set_target_ms = 0U;
    g_has_set_target = false;
    g_runtime_fault_mask = 0U;
    g_disable_pending_mask = 0U;

    disable_all_drivers_best_effort();

    /* Buoc 1: PING+HANDSHAKE+ENABLE tuan tu 12 khop (keepalive giua cac
     * buoc de khong khop nao bi bo doi qua lau). */
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        g_joint_ok[j] = ping_handshake_enable((JointIndex_t)j) ? 1 : 0;
        keepalive_others((JointIndex_t)j);
    }

    /* Buoc 2: doc HOME (Kp=Kd=tau=0, khong gian LOGIC) cho tung khop da PING
     * OK - GIA DINH robot dang nam xap (tu the nghi tu nhien) luc cap nguon,
     * nguoi van hanh PHAI dam bao dieu nay truoc khi bat nguon (xem
     * motor_calib.h). Tinh gioi han LOGIC tuyet doi = HOME + do lech tuong
     * doi (motor_calib.h, KHONG can mirror rieng - da chuyen het sang
     * LogicalToRaw() luc gui SETUP_LIMITS), roi moi gui SETUP_LIMITS. Khop
     * nao khong doc duoc HOME -> danh dau that bai, Actuator_SetTarget() se
     * bo qua hoan toan khop do. */
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        if (!g_joint_ok[j]) { continue; }
        const JointIndex_t joint = (JointIndex_t)j;

        float home_logical_rad = 0.0f;
        CAN_Frame home_telemetry = {0};
        const bool got_home = read_home_logical(
            joint, &home_logical_rad, &home_telemetry);
        keepalive_others(joint);
        if (!got_home)
        {
            g_joint_ok[j] = 0;
            continue;
        }

        const JointType_t jt = Motor_JointTypeForJoint(joint);

        /* Hieu chuan ADAPTIVE: home_logical_rad o day la gia tri CHUA co
         * offset (g_home_offset_rad[j] van = 0 tu memset, xem RawToLogical())
         * - tinh offset SAO CHO sau khi cong vao, HOME phien nay tro thanh
         * dung ASSUMED_REST_LOGICAL_RAD[jt]. Moi lan doc/gui RAW<->LOGIC SAU
         * DIEM NAY (limit, target, telemetry) deu tu dong di qua offset nay
         * qua LogicalToRaw()/RawToLogical(). */
        const float assumed_rest = ASSUMED_REST_LOGICAL_RAD[jt];
        g_home_offset_rad[j] = assumed_rest - home_logical_rad;

        const float home_deg = BA2_RAD2DEG(assumed_rest);   /* = HOME DA hieu chuan, hang so */
        g_limit_max_rad[j] = BA2_DEG2RAD(home_deg + MOTOR_LIMIT_MAX_REL_DEG[jt]);
        g_limit_min_rad[j] = BA2_DEG2RAD(home_deg + MOTOR_LIMIT_MIN_REL_DEG[jt]);

        /* Diem bat dau noi suy = chinh HOME vua do (DA hieu chuan = assumed_rest)
         * - lenh giu dau tien khong giat (sai so ban dau = 0, oneLeg/HW.md muc 8). */
        g_last_target[j] = assumed_rest;
        g_measured_angle[j] = assumed_rest;
        g_measured_velocity[j] = RawVelocityToLogical(
            joint, BA2_DecodeVelAct(
                ((uint16_t)home_telemetry.data[3] << 8) | home_telemetry.data[4]));
        g_measured_effort[j] = RawTorqueToLogical(
            joint, BA2_DecodeTauAct(
                ((uint16_t)home_telemetry.data[5] << 8) | home_telemetry.data[6]));
        g_status_raw_le[j] = (uint16_t)home_telemetry.data[10] |
                             ((uint16_t)home_telemetry.data[11] << 8);
        g_has_feedback[j] = 1;
        g_last_telemetry_ms[j] = Tick_GetMs();

        /* Some proven driver revisions apply SETUP_LIMITS without returning
         * the documented ACK. Do not discard valid HOME telemetry or disable
         * such a joint: Actuator_SetTarget() still enforces the same limits
         * independently on the MCU before every CAN command. */
        (void)send_setup_limits(joint, g_limit_max_rad[j], g_limit_min_rad[j]);
        keepalive_others(joint);
    }
}

static void latch_runtime_fault(JointIndex_t joint)
{
    const uint16_t bit = (uint16_t)(1U << (uint32_t)joint);
    if (!g_joint_ok[joint]) { return; }

    g_joint_ok[joint] = 0;
    g_feedback_required[joint] = 0;
    g_active_required[joint] = 0;
    g_runtime_fault_mask |= bit;
    g_disable_pending_mask |= bit;
}

static bool status_is_explicitly_disabled(uint16_t status_raw_le)
{
    /* 0x0001 is NOT a reliable Standby indication: all 12 physical drivers in
     * this robot returned it immediately after accepting an active PD command.
     * Keep revision-specific nonzero values observable through /joint_diag,
     * but only use the unambiguous all-zero Disabled value as a safety gate. */
    return status_raw_le == 0x0000U;
}

static bool joint_from_feedback_source(uint32_t instance, uint8_t motor_id, JointIndex_t *joint_out)
{
    if (joint_out == NULL || motor_id < 1U || motor_id > 6U) { return false; }

    if (instance == CAN_INSTANCE_1)
    {
        /* Front bus: physical id1-3 = RF, id4-6 = LF. */
        static const JointIndex_t kCan1IdToJoint[6] = {
            JOINT_RF_ABAD, JOINT_RF_HIP, JOINT_RF_KNEE,
            JOINT_LF_ABAD, JOINT_LF_HIP, JOINT_LF_KNEE,
        };
        *joint_out = kCan1IdToJoint[(uint32_t)motor_id - 1U];
        return true;
    }
    if (instance == CAN_INSTANCE_2)
    {
        /* Hind bus: physical id1-3 = RH, id4-6 = LH. */
        static const JointIndex_t kCan2IdToJoint[6] = {
            JOINT_RH_ABAD, JOINT_RH_HIP, JOINT_RH_KNEE,
            JOINT_LH_ABAD, JOINT_LH_HIP, JOINT_LH_KNEE,
        };
        *joint_out = kCan2IdToJoint[(uint32_t)motor_id - 1U];
        return true;
    }
    return false;
}

void Actuator_OnBabyAlpha2Frame(uint32_t instance, const CAN_Frame *frame)
{
    /* MOI phan hoi BabyAlpha2 (PING/HANDSHAKE/SETUP/telemetry Page0) ve
     * chung CAN ID=0 - o day chi quan tam telemetry Page0 (data[0]=1..6,
     * du dai >=12 byte); cac reply he thong khac da duoc send_wait() trong
     * Actuator_Init() xu ly rieng (khong can main.c dispatch toi day). */
    if (frame->id != 0U || frame->data_len < 12U) { return; }
    const uint8_t b0 = frame->data[0];
    if (b0 < 1U || b0 > 6U) { return; }

    JointIndex_t joint = JOINT_COUNT;
    if (!joint_from_feedback_source(instance, b0, &joint)) { return; }
    const uint32_t slot = (uint32_t)joint;

    const uint16_t position_raw = ((uint16_t)frame->data[1] << 8) | frame->data[2];
    const uint16_t velocity_raw = ((uint16_t)frame->data[3] << 8) | frame->data[4];
    const uint16_t effort_raw = ((uint16_t)frame->data[5] << 8) | frame->data[6];
    const uint16_t status_raw_le = (uint16_t)frame->data[10] |
                                   ((uint16_t)frame->data[11] << 8);
    const uint32_t now_ms = Tick_GetMs();

    g_measured_angle[slot] = RawToLogical(joint, BA2_DecodePosAct(position_raw));
    g_measured_velocity[slot] = RawVelocityToLogical(joint, BA2_DecodeVelAct(velocity_raw));
    g_measured_effort[slot] = RawTorqueToLogical(joint, BA2_DecodeTauAct(effort_raw));
    g_status_raw_le[slot] = status_raw_le;
    g_last_telemetry_ms[slot] = now_ms;
    g_has_feedback[slot] = 1;

    /* Driver was active and receiving a continuous command stream, but now
     * explicitly reports Disabled. This is the observable signature of
     * reset/hot-plug or a lost enable state; its RAM limits may be gone, so
     * never resume force without a full reboot/recalibration. */
    if (g_active_required[slot] && status_is_explicitly_disabled(status_raw_le))
    {
        latch_runtime_fault(joint);
    }
}

void Actuator_SetTarget(const float angles_rad[12], const float velocities_rad_s[12],
                        const float kp[12],
                        const float kd[12], const float tau_ff_nm[12])
{
    const uint32_t now_ms = Tick_GetMs();
    if (!g_has_set_target ||
        (now_ms - g_last_set_target_ms) >= ACTUATOR_TELEMETRY_TIMEOUT_MS)
    {
        /* No commands means no telemetry replies. On stream restart, require
         * one zero-gain probe before treating old feedback as a runtime loss. */
        memset(g_feedback_required, 0, sizeof(g_feedback_required));
        memset(g_active_required, 0, sizeof(g_active_required));
    }
    g_last_set_target_ms = now_ms;
    g_has_set_target = true;

    /* Kep velocity/KP/KD/Tff LOP 2 - doc lap voi phia ROS2/EC (xem
     * StandSitController.h). BA2_EncodeGainX100() chi kep san = 0, chua co
     * tran - kep o day truoc, phong gia tri loi (YAML sai don vi, hoac
     * kp_x100/kd_x100 tu /joint_cmd bi loi) truyen thang xuong dong co that. */
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        /* Khop hieu chuan that bai luc Actuator_Init() -> KHONG gui lenh gi
         * ca (an toan hon la gui voi gioi han khong dang tin). */
        if (!g_joint_ok[j]) { continue; }
        const JointIndex_t joint = (JointIndex_t)j;
        const bool feedback_fresh = g_has_feedback[j] &&
            ((now_ms - g_last_telemetry_ms[j]) < ACTUATOR_TELEMETRY_TIMEOUT_MS);
        if (!feedback_fresh && g_feedback_required[j])
        {
            latch_runtime_fault(joint);
            continue;
        }
        float joint_kp = kp[j];
        float joint_kd = kd[j];
        float joint_velocity = velocities_rad_s[j];
        float joint_tau_ff = tau_ff_nm[j];
        if (joint_velocity > MOTOR_VELOCITY_ABS_LIMIT_RAD_S[j])
        {
            joint_velocity = MOTOR_VELOCITY_ABS_LIMIT_RAD_S[j];
        }
        if (joint_velocity < -MOTOR_VELOCITY_ABS_LIMIT_RAD_S[j])
        {
            joint_velocity = -MOTOR_VELOCITY_ABS_LIMIT_RAD_S[j];
        }
        if (joint_kp < 0.0f) { joint_kp = 0.0f; }
        if (joint_kp > MOTOR_KP_ABS_LIMIT) { joint_kp = MOTOR_KP_ABS_LIMIT; }
        if (joint_kd < 0.0f) { joint_kd = 0.0f; }
        if (joint_kd > MOTOR_KD_ABS_LIMIT) { joint_kd = MOTOR_KD_ABS_LIMIT; }
        if (joint_tau_ff > MOTOR_TAU_ABS_LIMIT_NM) { joint_tau_ff = MOTOR_TAU_ABS_LIMIT_NM; }
        if (joint_tau_ff < -MOTOR_TAU_ABS_LIMIT_NM) { joint_tau_ff = -MOTOR_TAU_ABS_LIMIT_NM; }
        const bool active_requested =
            joint_kp > 0.0f || joint_kd > 0.0f || joint_tau_ff != 0.0f;
        if (feedback_fresh && active_requested && g_active_required[j] &&
            status_is_explicitly_disabled(g_status_raw_le[j]))
        {
            latch_runtime_fault(joint);
            continue;
        }

        /* Kep gioi han hanh trinh LOP 2 TRONG KHONG GIAN LOGIC - doc lap voi
         * SETUP_LIMITS da gui driver (oneLeg/HW.md muc 8, 2 lop bao ve
         * nhau). */
        float target_logical = angles_rad[j];
        if (target_logical > g_limit_max_rad[j]) { target_logical = g_limit_max_rad[j]; }
        if (target_logical < g_limit_min_rad[j]) { target_logical = g_limit_min_rad[j]; }
        /* Neu target dang o bien, khong cho D-term tiep tuc yeu cau van toc
         * huong RA NGOAI hanh trinh. Van toc huong vao trong van duoc giu de
         * khop co the roi khoi bien mot cach binh thuong. */
        if ((target_logical >= g_limit_max_rad[j] && joint_velocity > 0.0f) ||
            (target_logical <= g_limit_min_rad[j] && joint_velocity < 0.0f))
        {
            joint_velocity = 0.0f;
        }
        g_last_target[j] = target_logical;

        /* Chuyen position/velocity/torque sang khong gian RAW NGAY TRUOC KHI
         * ma hoa. Position co home offset; velocity va torque chi doi dau,
         * tuyet doi khong duoc cong offset goc vao hai dai luong dao ham. */
        const bool safe_probe = !feedback_fresh;
        const float transmitted_target = safe_probe
            ? (g_has_feedback[j] ? g_measured_angle[j] : g_last_target[j])
            : target_logical;
        const float target_raw = LogicalToRaw(joint, transmitted_target);
        const float velocity_raw = LogicalVelocityToRaw(
            joint, safe_probe ? 0.0f : joint_velocity);
        const float tau_ff_raw = LogicalTorqueToRaw(
            joint, safe_probe ? 0.0f : joint_tau_ff);
        const CAN_Frame frame = BA2_BuildPdFrame(
            Motor_IdForJoint(joint), target_raw, velocity_raw,
            safe_probe ? 0.0f : joint_kp, safe_probe ? 0.0f : joint_kd,
            tau_ff_raw, MOTOR_TAU_ABS_LIMIT_NM);
        const uint32_t instance = Motor_BusForJoint(joint);
        if (!CAN_Transmit(instance, &frame))
        {
            record_tx_failure(instance);
        }
        else if (!safe_probe)
        {
            g_feedback_required[j] = 1;
            g_active_required[j] = active_requested ? 1 : 0;
        }
    }
}

void Actuator_ServiceSafety(void)
{
    for (uint32_t instance = CAN_INSTANCE_1; instance <= CAN_INSTANCE_2; instance++)
    {
        if (CAN_IsBusOff(instance))
        {
            const int first = (instance == CAN_INSTANCE_2) ? 6 : 0;
            const int end = first + 6;
            for (int j = first; j < end; j++)
            {
                if (g_joint_ok[j]) { latch_runtime_fault((JointIndex_t)j); }
            }
            continue;
        }

        const int first = (instance == CAN_INSTANCE_2) ? 6 : 0;
        const int end = first + 6;
        for (int j = first; j < end; j++)
        {
            const uint16_t bit = (uint16_t)(1U << (uint32_t)j);
            if ((g_disable_pending_mask & bit) == 0U) { continue; }

            const JointIndex_t joint = (JointIndex_t)j;
            const CAN_Frame disable = BA2_BuildSystemFrame(
                Motor_IdForJoint(joint), BA2_OPCODE_MOTOR_DISABLE);
            if (CAN_Transmit(instance, &disable))
            {
                g_disable_pending_mask &= (uint16_t)~bit;
            }
            else
            {
                record_tx_failure(instance);
                break;
            }
        }
    }
}

void Actuator_Disable(void)
{
    /* Watchdog fail-soft frame: zero every torque-producing term. This is
     * intentionally softer than MOTOR_DISABLE so a brief EC reconnect can
     * recover without re-running physical home calibration. */
    float target[JOINT_COUNT];
    float velocity[JOINT_COUNT];
    float kp[JOINT_COUNT];
    float kd[JOINT_COUNT];
    float tau_ff[JOINT_COUNT];
    Actuator_GetLastTarget(target);
    for (int j = 0; j < (int)JOINT_COUNT; ++j)
    {
        velocity[j] = 0.0f;
        kp[j] = 0.0f;
        kd[j] = 0.0f;
        tau_ff[j] = 0.0f;
    }
    memset(g_feedback_required, 0, sizeof(g_feedback_required));
    memset(g_active_required, 0, sizeof(g_active_required));
    Actuator_SetTarget(target, velocity, kp, kd, tau_ff);
    memset(g_feedback_required, 0, sizeof(g_feedback_required));
    memset(g_active_required, 0, sizeof(g_active_required));
    g_has_set_target = false;
}

void Actuator_GetLastTarget(float angles_rad[12])
{
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        angles_rad[j] = g_has_feedback[j] ? g_measured_angle[j] : g_last_target[j];
    }
}

uint16_t Actuator_GetMeasured(float angles_rad[12], float velocities_rad_s[12],
                              float efforts_nm[12])
{
    const uint32_t now_ms = Tick_GetMs();
    uint16_t valid_mask = 0U;
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        angles_rad[j] = g_has_feedback[j] ? g_measured_angle[j] : 0.0f;
        velocities_rad_s[j] = g_has_feedback[j] ? g_measured_velocity[j] : 0.0f;
        efforts_nm[j] = g_has_feedback[j] ? g_measured_effort[j] : 0.0f;
        if (g_joint_ok[j] && g_has_feedback[j] &&
            ((now_ms - g_last_telemetry_ms[j]) < ACTUATOR_TELEMETRY_TIMEOUT_MS))
        {
            valid_mask |= (uint16_t)(1U << (uint32_t)j);
        }
    }
    return valid_mask;
}

void Actuator_GetDiagnostics(ActuatorDiagnostics *diagnostics)
{
    if (diagnostics == NULL) { return; }

    const uint32_t now_ms = Tick_GetMs();
    memset(diagnostics, 0, sizeof(*diagnostics));
    diagnostics->runtime_fault_mask = g_runtime_fault_mask;
    diagnostics->can_bus_off_mask =
        (CAN_IsBusOff(CAN_INSTANCE_1) ? 0x01U : 0U) |
        (CAN_IsBusOff(CAN_INSTANCE_2) ? 0x02U : 0U);
    diagnostics->can_tx_fail_count[0] = g_can_tx_fail_count[0];
    diagnostics->can_tx_fail_count[1] = g_can_tx_fail_count[1];

    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        const uint16_t bit = (uint16_t)(1U << (uint32_t)j);
        if (g_joint_ok[j]) { diagnostics->ready_mask |= bit; }
        diagnostics->status_raw_le[j] = g_status_raw_le[j];

        uint32_t age_ms = UINT16_MAX;
        if (g_has_feedback[j])
        {
            age_ms = now_ms - g_last_telemetry_ms[j];
            if (age_ms < ACTUATOR_TELEMETRY_TIMEOUT_MS)
            {
                diagnostics->fresh_mask |= bit;
            }
            if (age_ms > UINT16_MAX) { age_ms = UINT16_MAX; }
        }
        diagnostics->telemetry_age_ms[j] = (uint16_t)age_ms;
    }
}

int Actuator_IsJointOk(int joint)
{
    if (joint < 0 || joint >= (int)JOINT_COUNT) { return 0; }
    return g_joint_ok[joint];
}
