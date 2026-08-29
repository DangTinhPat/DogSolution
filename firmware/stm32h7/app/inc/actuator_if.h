/**
 ******************************************************************************
 * @file    actuator_if.h
 * @brief   Lớp trừu tượng cho 12 khớp chân - mỗi khớp có 1 board driver
 *          BabyAlpha2 riêng (tự làm PD cục bộ, đọc encoder tại chỗ), nói
 *          chuyện qua CAN-FD 12 byte (baby_alpha2_protocol.h, motor_topology.h)
 *          - giao thức THẬT, kế thừa từ /home/dvt/OUT_SAVE/babyDog_test/oneLeg (đã
 *          chạy trên phần cứng thật), thay cho giao thức tự bịa trước đây.
 *
 *          Actuator_Init() giờ chạy CẢ chuỗi hiệu chuẩn thích ứng 12 khớp
 *          (PING/HANDSHAKE/SETUP_LIMITS/ENABLE + đọc HOME) - BLOCKING, tốn
 *          vài giây ở boot - vì driver BabyAlpha2 mất điểm 0 mỗi lần mất
 *          điện (đặc tả driver mục 7.3), không thể hardcode giới hạn 1 lần
 *          dùng mãi. Xem motor_calib.h cho bảng giới hạn/độ cứng PD.
 ******************************************************************************
 */
#ifndef ACTUATOR_IF_H
#define ACTUATOR_IF_H

#include <stdint.h>
#include "can.h"

#define ACTUATOR_TELEMETRY_TIMEOUT_MS 100U

typedef struct
{
    uint16_t ready_mask;
    uint16_t fresh_mask;
    uint16_t runtime_fault_mask;
    uint16_t status_raw_le[12];
    uint16_t telemetry_age_ms[12];
    uint8_t can_bus_off_mask;
    uint32_t can_tx_fail_count[2];
} ActuatorDiagnostics;

void Actuator_Init(void);

/* Nhac lai MOTOR_ENABLE cho moi khop da hieu chuan OK - PHAI goi dinh ky (vd
 * moi 500ms, xem oneLeg/main_12joint_hold_proven.c.bak's RE_ENABLE_PERIOD_MS)
 * TRONG SUOT luc chay tu main.c's vong lap chinh, KHONG chi luc boot - driver
 * BabyAlpha2 tu roi Standby (mat luc) neu thieu nhac dinh ky. Thieu buoc nay
 * gay dong co "keu lach cach, khong sinh momen" du PD van gui deu. */
void Actuator_ReenableAll(void);

/* Kiem tra bus-off va gui MOTOR_DISABLE dang cho cho cac khop da bi latch
 * runtime fault. Goi moi vong superloop; ham khong block va khong tu dong
 * re-enable/recalibrate khop sau hot-plug. */
void Actuator_ServiceSafety(void);

/* Gọi bởi main.c cho MỖI frame nhận được từ CAN_Receive() có id==0 trên bất
 * kỳ instance nào (MỌI phản hồi BabyAlpha2 - PING/HANDSHAKE/SETUP/telemetry
 * Page0 - đều về chung ID này, phân biệt bằng data[0], xem motor_topology.h
 * header) - main.c là nơi duy nhất gọi CAN_Receive() cho mỗi instance
 * (tránh 2 nơi cùng rút frame khỏi 1 FIFO phần cứng). */
void Actuator_OnBabyAlpha2Frame(uint32_t instance, const CAN_Frame *frame);

/* Moi mang co 12 phan tu theo JointIndex_t, khop dung thu tu joints phia
 * ROS2. angles_rad/velocities_rad_s/tau_ff_nm o khong gian LOGIC;
 * actuator_if.c la noi duy nhat doi dau sang RAW. Velocity/Kp/Kd/Tff duoc
 * clamp doc lap cho tung khop. */
void Actuator_SetTarget(const float angles_rad[12], const float velocities_rad_s[12],
                        const float kp[12],
                        const float kd[12], const float tau_ff_nm[12]);

/* Watchdog fail-soft: gui position hien tai voi velocity/Kp/Kd/Tff deu bang
 * 0 de xoa lenh PD cu ma van cho phep khoi phuc sau mot lan mat EC ngan. */
void Actuator_Disable(void);

/* Vị trí khớp dùng làm điểm bắt đầu nội suy lần tiếp theo: vị trí ĐO ĐƯỢC
 * thật (từ feedback frame) nếu đã nhận được ít nhất 1 lần cho khớp đó, nếu
 * chưa thì tạm dùng mục tiêu đã yêu cầu gần nhất (trước khi có feedback đầu
 * tiên, ví dụ ngay lúc mới cấp nguồn). */
void Actuator_GetLastTarget(float angles_rad[12]);

/* Tra position/velocity/torque moi nhat va bit mask freshness. Gia tri trong
 * mang chi hop le khi bit tuong ung trong return mask = 1. */
uint16_t Actuator_GetMeasured(float angles_rad[12], float velocities_rad_s[12],
                              float efforts_nm[12]);

void Actuator_GetDiagnostics(ActuatorDiagnostics *diagnostics);

/* true neu khop nay HIEU CHUAN THANH CONG luc Actuator_Init() (PING+HANDSHAKE+
 * MOTOR_ENABLE/telemetry va HOME deu dung; SETUP_LIMITS ACK la best-effort
 * vi khong phai moi driver revision deu tra ve) - tuc la CAN-FD 2 chieu
 * voi driver vat ly gan dung bus/id cua khop nay dang hoat dong that. Dung de
 * bao cao/debug (vd main.c in trang thai bring-up qua UART roi, xem
 * usart2_debug_init()) - KHONG dung de dieu khien logic, Actuator_SetTarget()
 * da tu bo qua khop khong ok. joint ngoai [0,11] tra ve 0. */
int Actuator_IsJointOk(int joint);

#endif /* ACTUATOR_IF_H */
