/**
 ******************************************************************************
 * @file    main.c
 * @brief   Firmware STM32H7 (board FK743M5-XIH6) - giai đoạn đứng lên/ngồi
 *          xuống, dùng thư viện lib/ (rcc/gpio/can/usart/tick).
 *
 *          - RDK X5/laptop <-> STM32 (đường ros2_control, main_bot_hardware):
 *            micro-ROS qua UART1 (PA9/PA10) - subscriber "/joint_cmd" +
 *            publisher "/joint_fb" (xem microros_bridge.c) - THAY CAN, không
 *            đụng chân với FDCAN1/2 (PA11/12, PB5/6) hay board driver khớp.
 *          - STM32 <-> 12 board driver khớp (driver "BabyAlpha2"): CAN-FD 12
 *            byte (baby_alpha2_protocol.h/motor_topology.h - giao thức THẬT,
 *            kế thừa từ /home/dvt/OUT_SAVE/babyDog_test/oneLeg đã chạy trên phần cứng
 *            thật), 6 khớp trên CAN_INSTANCE_1 (cùng bus RDK-link cũ), 6 khớp
 *            trên CAN_INSTANCE_2 (PB5/PB6). Mỗi board driver tự chạy PD cục
 *            bộ (kp/kd do STM32 gửi xuống) bằng encoder tại chỗ - MỌI phản
 *            hồi (PING/HANDSHAKE/SETUP/telemetry) từ driver trên 1 bus đều
 *            về CHUNG CAN ID=0, phân biệt bằng byte data[0] (KHÔNG phải theo
 *            CAN ID riêng từng khớp như thiết kế tự bịa trước đây).
 *
 *          CHƯA TEST TRÊN PHẦN CỨNG THẬT Ở MỨC TOÀN HỆ THỐNG - lib/can.c đã
 *          test loopback nội bộ thật (xem can.h), giao thức BabyAlpha2 đã
 *          test thật trên board khác (oneLeg) nhưng CHƯA test trên chính
 *          board/động cơ của dự án này - xem Actuator_Init()/motor_calib.h.
 ******************************************************************************
 */

#include <stdint.h>
#include "rcc.h"
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "tick.h"
#include "app_i2c.h"
#include "mpu6050.h"
#include "motor_topology.h"
#include "baby_alpha2_protocol.h"
#include "actuator_if.h"
#include "microros_bridge.h"
#include "microros_transport.h"

/* Baudrate cho USART2 debug console (PA2=TX/PA3=RX, qua adapter CH340 rời) -
 * KHAC USART1 (micro-ROS/UART1, PA9/PA10) - dung TAM THOI de bring-up doc lap
 * (khong can laptop/ROS2) xac nhan tung khop co PING/HANDSHAKE/ENABLE/doc HOME
 * thanh cong voi driver BabyAlpha2 that hay khong, truoc khi tin dung buoc di
 * chuyen. Dung chung USART_Init() (usart.c) - hop le vi board nay D2PPRE1
 * (APB1, cap cho USART2) va D2PPRE2 (APB2, cap cho USART1) LUON dat cung gia
 * tri /2 (xem rcc.c) nen SystemPCLK2Clock dung so cho ca 2 - KHONG phai dung
 * chung cho moi board/cau hinh clock khac. */
#define DEBUG_UART_BAUDRATE 115200U

/* Che do bring-up TOI GIAN: firmware CHI gui PING (0xF0) lap lai va bao PASS/
 * FAIL qua USART2 - KHONG bao gio goi HANDSHAKE/MOTOR_ENABLE/SETUP_LIMITS/PD,
 * KHONG chay FSM/Actuator_Init() day du. Dung khi CHI can xac nhan tin hieu
 * CAN-FD 2 chieu voi 1 driver BabyAlpha2 co thong hay khong (vd MCU moi noi
 * day, chua muon bat bat ky kha nang sinh luc nao). Dat = 0 de quay lai
 * firmware van hanh binh thuong (FSM day du + micro-ROS + ros2_control). */
#define BRINGUP_PING_ONLY 0

/* Che do bring-up TOI GIAN THU 2: CHI test lop vat ly UART1 (micro-ROS/PA9-
 * PA10) - gui lap lai 1 chuoi co dinh qua USART_SendString() (polling, KHONG
 * dung transport XRCE cua microros_transport.c), nhay LED PC13 moi lan
 * gui - hoan toan KHONG dung toi CAN/Actuator_Init()/micro-ROS. Dung de tach
 * bach: neu chuoi nay KHONG bao gio toi duoc may tinh (vd qua `cat -v
 * /dev/ttyUSB0` hoac python pyserial raw), loi chac chan o lop vat ly (day/
 * adapter/GPIO/clock) - khong lien quan gi toi CAN hay giao thuc micro-ROS.
 * Dat = 0 de quay lai firmware van hanh binh thuong. */
#define BRINGUP_UART_ECHO_ONLY 0

/* Che do bring-up TOI GIAN THU 3: test chieu NGUOC LAI cua UART1 (PC -> MCU,
 * PA10/RX) - BRINGUP_UART_ECHO_ONLY o tren CHI test MCU->PC (TX), chua tung
 * test rieng PC->MCU. Vong lap nay POLLING THUAN TUY (USART_IsDataAvailable/
 * USART_ReceiveByte/USART_SendByte cua usart.c) - KHONG dung ngat RXNE, KHONG
 * dung rx_ring cua microros_transport.c - de tach bach hoan toan lop
 * thanh ghi USART1 + GPIO PA9/PA10 that (co dung AF7/pull-up hay khong) ra
 * khoi logic ISR/ring buffer. Echo lai NGUYEN VEN tung byte nhan duoc + nhay
 * LED PC13 moi byte. Neu gui 1 chuoi da biet tu PC (vd `cat` mot file lon qua
 * `/dev/ttyUSB0` roi doc lai) va nhan ve KHONG loi kieu nay, nghia la thanh
 * ghi/GPIO RX hoan toan dung - loi (neu con) chac chan nam o tang ISR/ring
 * buffer hoac tang giao thuc XRCE, khong phai o day. Dat = 0 de quay lai
 * firmware van hanh binh thuong. */
#define BRINGUP_UART_RX_ECHO_ONLY 0

/* Che do bring-up TOI GIAN THU 4: giong BRINGUP_UART_RX_ECHO_ONLY o tren nhung
 * dung DUNG duong ISR RXNE + rx_ring[] cua microros_transport.c (qua
 * MicroRosTransport_DebugRxIsrOpen()/DebugRxIsrRead()) - cung 1 ISR/ring buffer
 * ma tang XRCE that su dung khi chay san xuat, nhung KHONG dung toi session/
 * entity XRCE nao ca. Neu test nay cung sach (nhu BRINGUP_UART_RX_ECHO_ONLY da
 * xac nhan sach o lop thanh ghi/GPIO thuan polling), nghia la ISR+ring buffer
 * CUNG dung - loi con lai chi co the o tang giao thuc XRCE/agent. Dat = 0 de
 * quay lai firmware van hanh binh thuong. */
#define BRINGUP_UART_RX_ISR_ECHO_ONLY 0

/* LED PC13 nhay lien tuc neu con nhan "/joint_cmd" qua micro-ROS trong
 * LED_ACTIVE_WINDOW_MS gan day - ngung gui la LED tat lai (xem led_init()/
 * vong lap chinh trong main()). */
#define LED_BLINK_PERIOD_MS  150U
#define LED_ACTIVE_WINDOW_MS 1000U

#define JOINT_FB_SEND_PERIOD_MS 5U /* ~200Hz, khớp nhịp update_rate cua controllers_real.yaml */
#define JOINT_DIAG_SEND_PERIOD_MS 100U
#define JOINT_CMD_TIMEOUT_MS 200U /* mat /joint_cmd -> ngat luc, khong fallback goc */
#define IMU_SAMPLE_PERIOD_MS 10U  /* MPU6050 configured at 100Hz */
#define IMU_RETRY_PERIOD_MS 1000U
#define IMU_MAX_CONSECUTIVE_READ_ERRORS 3U

/* Cung gia tri toi uu da xac nhan qua ~/OUT_SAVE/testSTM (18.4Hz -> 41.6Hz o baudrate
 * 115200 cu, xem ~/OUT_SAVE/testSTM/README.md) - agent phia laptop (micro_ros_agent
 * serial -b ...) PHAI khop dung gia tri nay. */
#define UART_BAUDRATE 921600U

/* Nominal 1Mbit/s tai kernel clock 25MHz (HSE): 1 + TSEG1(19) + TSEG2(5) = 25
 * Tq, BRP=1 -> 25MHz/25 = 1Mbit/s, sample point (1+19)/25 = 80%.
 *
 * Data phase: KHOP CHINH XAC voi OUT_SAVE/babyDog_test/oneLeg/snapshots/
 * leg1_proven_2026-08-08 (da xac nhan chay that tren phan cung, cung dung
 * motor-link CAN-FD 12-byte) thay vi tu suy "khong dung toi vi
 * bit_rate_switch=false". Truoc day dat data phase = nominal (19/5, 1Mbit/s)
 * voi ly do "BRS=false nen DBTP khong anh huong" - GIA DINH NAY CO THE SAI:
 * theo Bosch M_CAN, truong CRC cua frame FD dung co che "fixed stuff bit"
 * rieng, vi tri cac bit nhoi co the phu thuoc cau hinh data-phase ke ca khi
 * BRS=0 tren tung frame - chua xac nhan chac chan day co phai nguyen nhan
 * PING khong co phan hoi hay khong, nhung day la diem KHAC BIET CU THE duy
 * nhat con lai so voi ban da chay duoc that, nen doi khop truoc. 1 + TSEG1(3)
 * + TSEG2(1) = 5 Tq, BRP=1 -> 25MHz/5 = 5Mbit/s. */
static const CAN_InitConfig CAN_BUS_CONFIG = {
    .nominal_prescaler = 1U,
    .nominal_tseg1 = 19U,
    .nominal_tseg2 = 5U,
    .nominal_sjw = 5U,
    .data_prescaler = 1U,
    .data_tseg1 = 3U,
    .data_tseg2 = 1U,
    .data_sjw = 1U,
    .mode = CAN_MODE_NORMAL,
};

static void configure_can_gpio(void)
{
    RCC_GPIO_ClockEnable(GPIOA);
    RCC_GPIO_ClockEnable(GPIOB);

    /* CAN_INSTANCE_1 = FDCAN1 = PA11 (RX) / PA12 (TX), AF9 */
    const GPIO_InitConfig pa11 = {
        .pin = 11U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_VERYHIGH, .pull = GPIO_PULL_NONE, .af = 9U,
    };
    const GPIO_InitConfig pa12 = {
        .pin = 12U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_VERYHIGH, .pull = GPIO_PULL_NONE, .af = 9U,
    };
    GPIO_Init(GPIOA, &pa11);
    GPIO_Init(GPIOA, &pa12);

    /* CAN_INSTANCE_2 = FDCAN2 = PB5 (RX) / PB6 (TX), AF9 */
    const GPIO_InitConfig pb5 = {
        .pin = 5U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_VERYHIGH, .pull = GPIO_PULL_NONE, .af = 9U,
    };
    const GPIO_InitConfig pb6 = {
        .pin = 6U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_VERYHIGH, .pull = GPIO_PULL_NONE, .af = 9U,
    };
    GPIO_Init(GPIOB, &pb5);
    GPIO_Init(GPIOB, &pb6);
}

/* UART1 cho micro-ROS (microros_bridge.c) - PA9=TX/PA10=RX, AF7, khong dung chan voi
 * CAN o tren (PA11/12, PB5/6). Port tu OUT_SAVE/testSTM/main.c's usart1_init(), cung board. */
static void usart1_init(void)
{
    RCC_GPIO_ClockEnable(GPIOA);

    const GPIO_InitConfig pa9_tx = {
        .pin = 9U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_VERYHIGH, .pull = GPIO_PULL_NONE, .af = 7U,
    };
    const GPIO_InitConfig pa10_rx = {
        .pin = 10U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_VERYHIGH, .pull = GPIO_PULL_UP, .af = 7U,
    };
    GPIO_Init(GPIOA, &pa9_tx);
    GPIO_Init(GPIOA, &pa10_rx);

    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC_APB2ENR;

    USART_Init(USART1, UART_BAUDRATE);
}

/* USART2 rieng cho debug console TAM THOI (xem DEBUG_UART_BAUDRATE) - PA2=TX/
 * PA3=RX, AF7, khong dung chan voi USART1 (PA9/10) hay CAN (PA11/12, PB5/6). */
static void usart2_debug_init(void)
{
    RCC_GPIO_ClockEnable(GPIOA);

    const GPIO_InitConfig pa2_tx = {
        .pin = 2U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_VERYHIGH, .pull = GPIO_PULL_NONE, .af = 7U,
    };
    const GPIO_InitConfig pa3_rx = {
        .pin = 3U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_VERYHIGH, .pull = GPIO_PULL_UP, .af = 7U,
    };
    GPIO_Init(GPIOA, &pa2_tx);
    GPIO_Init(GPIOA, &pa3_rx);

    RCC_APB1LENR |= RCC_APB1LENR_USART2EN;
    (void)RCC_APB1LENR;

    USART_Init(USART2, DEBUG_UART_BAUDRATE);
}

/* LED nguoi dung tren board FK743M5-XIH6 - PC13, active LOW (xem
 * BOARD_FK743M5-XIH6.md) - dung de bao hieu bang mat "/joint_cmd" qua micro-
 * ROS/UART1 co toi MCU hay khong, khong can terminal/SWD (xem vong lap chinh:
 * nhay lien tuc khi con nhan lenh gan day, tat khi ngung nhan). */
static void led_init(void)
{
    RCC_GPIO_ClockEnable(GPIOC);
    const GPIO_InitConfig pc13 = {
        .pin = 13U, .mode = GPIO_MODE_OUTPUT, .otype = GPIO_OTYPE_PUSHPULL,
        .speed = GPIO_SPEED_LOW, .pull = GPIO_PULL_NONE, .af = 0U,
    };
    GPIO_Init(GPIOC, &pc13);
    GPIO_WritePin(GPIOC, 13U, 1U); /* active LOW -> 1 = tat luc khoi dong */
}

/* In so nguyen khong dau (0..999) dang thap phan qua USART - khong dung
 * sprintf/stdio (codebase nay khong lien ket libc "day du" cho cac ham nay,
 * xem nhan xet .specs=nano/nosys trong Makefile) - chi can du cho joint index
 * (0-11)/id (1-6)/bus (1-2), khong can tong quat hoa qua muc can thiet. */
static void send_udec(USART_TypeDef *u, uint32_t v)
{
    char digits[4];
    int n = 0;
    if (v == 0U) { USART_SendByte(u, '0'); return; }
    while (v > 0U && n < 4)
    {
        digits[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n > 0) { USART_SendByte(u, (uint8_t)digits[--n]); }
}

/* Bao cao trang thai bring-up cua 12 khop qua USART2 (xem usart2_debug_init())
 * - goi 1 lan ngay sau Actuator_Init(),
 * blocking). Muc dich DUY NHAT: xac nhan bang mat tung khop co that su nhan
 * duoc phan hoi CAN-FD tu driver BabyAlpha2 vat ly hay khong (PING+HANDSHAKE+
 * MOTOR_ENABLE+doc HOME deu OK -> "READY"), truoc khi tin dung buoc di chuyen
 * nao - dung cho bring-up doc lap (chi MCU+1 driver, khong laptop, xem
 * Actuator_IsJointOk() header). Khop khong noi day day du du kien se bao
 * "TIMEOUT" (binh thuong, khong phai loi). */
static void print_joint_bringup_report(bool imu_ready)
{
    USART_SendString(USART2, "\r\n=== BabyAlpha2 bring-up ===\r\n");
    for (int j = 0; j < (int)JOINT_COUNT; j++)
    {
        const JointIndex_t joint = (JointIndex_t)j;
        USART_SendString(USART2, "joint ");
        send_udec(USART2, (uint32_t)j);
        USART_SendString(USART2, " (CAN");
        send_udec(USART2, (Motor_BusForJoint(joint) == CAN_INSTANCE_2) ? 2U : 1U);
        USART_SendString(USART2, " id=");
        send_udec(USART2, Motor_IdForJoint(joint));
        USART_SendString(USART2, "): ");
        USART_SendString(USART2, Actuator_IsJointOk(j) ? "READY\r\n" : "TIMEOUT (khong noi day/driver chua bat)\r\n");
    }
    USART_SendString(USART2, "imu (MPU6050 I2C1 PB7/PB8): ");
    USART_SendString(USART2, imu_ready ? "READY\r\n" : "INIT FAILED (WHO_AM_I sai hoac khong phan hoi I2C)\r\n");
    USART_SendString(USART2, "=== het bao cao ===\r\n");
}

#if BRINGUP_PING_ONLY
/* Che do bring-up TOI GIAN (xem BRINGUP_PING_ONLY o tren) - CO Y giu TACH
 * BIET hoan toan khoi actuator_if.c (khong goi ham nao cua file do), tu
 * gui/nhan CAN truc tiep bang can.h + baby_alpha2_protocol.h -
 * de nguoi doc/soat code chi can doc DUY NHAT ham nay la du xac nhan: KHONG
 * co opcode nao khac 0xF0 (PING) tung duoc gui di trong toan bo vong lap nay,
 * tuc la KHONG co kha nang dong co sinh luc trong che do nay du bat ky loi
 * logic nao khac. Lap vo han, khong tra ve - main() sau ham nay se khong
 * chay toi (co y, xem loi goi trong main()). CHI BIEN DICH khi
 * BRINGUP_PING_ONLY=1 (dat lai 0 o dinh file de dung firmware van hanh binh
 * thuong + test led/micro-ROS ben duoi).
 *
 * In qua USART1 (PA9/PA10) CHU KHONG PHAI USART2 nhu print_joint_bringup_report()
 * - vi che do nay khong bao gio goi toi MicroRosBridge_Begin() (xem main(),
 * ping_only_bringup_loop() khong tra ve), nen USART1 hoan toan ranh, khop dung
 * day CH340 da noi san (PA9/PA10) khong can dau lai. Baudrate = UART_BAUDRATE
 * (921600, da usart1_init() cau hinh san) - chinh terminal dung toc do nay. */
static void ping_only_bringup_loop(void)
{
    USART_SendString(USART1, "\r\n=== CHE DO BRING-UP: CHI PING, KHONG ENABLE/HANDSHAKE ===\r\n");
    while (1)
    {
        for (int j = 0; j < (int)JOINT_COUNT; j++)
        {
            const JointIndex_t joint = (JointIndex_t)j;
            const uint32_t instance = Motor_BusForJoint(joint);
            const uint32_t id = Motor_IdForJoint(joint);
            const CAN_Frame ping = BA2_BuildSystemFrame(id, BA2_OPCODE_PING);

            int got_reply = 0;
            for (uint32_t retry = 0U; retry < 3U && !got_reply; retry++)
            {
                (void)CAN_Transmit(instance, &ping);
                const uint32_t t0 = Tick_GetMs();
                while (!got_reply && (Tick_GetMs() - t0) < 100U)
                {
                    CAN_Frame rx;
                    while (CAN_IsRxPending(instance) && CAN_Receive(instance, &rx))
                    {
                        if (rx.id == 0U && rx.data_len >= 1U && rx.data[0] == BA2_REPLY_PING(id))
                        {
                            got_reply = 1;
                        }
                    }
                }
            }

            USART_SendString(USART1, "joint ");
            send_udec(USART1, (uint32_t)j);
            USART_SendString(USART1, " (CAN");
            send_udec(USART1, (instance == CAN_INSTANCE_2) ? 2U : 1U);
            USART_SendString(USART1, " id=");
            send_udec(USART1, id);
            USART_SendString(USART1, "): ");
            USART_SendString(USART1, got_reply ? "PING OK\r\n" : "khong phan hoi\r\n");
        }
        USART_SendString(USART1, "--- lap lai sau ~1s ---\r\n");
        const uint32_t loop_t0 = Tick_GetMs();
        while ((Tick_GetMs() - loop_t0) < 1000U) { /* cho */ }
    }
}
#endif /* BRINGUP_PING_ONLY */

#if BRINGUP_UART_ECHO_ONLY
/* Test toi gian lop vat ly UART1 - xem giai thich day du o BRINGUP_UART_ECHO_ONLY
 * o tren. Lap vo han, khong tra ve (main() sau ham nay khong chay toi, co y). */
static void uart_echo_only_bringup_loop(void)
{
    uint32_t counter = 0U;
    while (1)
    {
        USART_SendString(USART1, "BABYDOG_UART_TEST seq=");
        send_udec(USART1, counter);
        USART_SendString(USART1, "\r\n");
        GPIO_TogglePin(GPIOC, 13U);
        counter++;

        const uint32_t t0 = Tick_GetMs();
        while ((Tick_GetMs() - t0) < 500U) { /* cho 500ms, khong lam gi khac */ }
    }
}
#endif /* BRINGUP_UART_ECHO_ONLY */

#if BRINGUP_UART_RX_ECHO_ONLY
/* Test toi gian chieu RX cua UART1 (PC->MCU) - xem giai thich day du o
 * BRINGUP_UART_RX_ECHO_ONLY o tren. Lap vo han, khong tra ve. */
static void uart_rx_echo_only_bringup_loop(void)
{
    while (1)
    {
        if (USART_IsDataAvailable(USART1))
        {
            const uint8_t b = USART_ReceiveByte(USART1);
            USART_SendByte(USART1, b);   /* echo nguyen ven, khong doi */
            GPIO_TogglePin(GPIOC, 13U);
        }
    }
}
#endif /* BRINGUP_UART_RX_ECHO_ONLY */

#if BRINGUP_UART_RX_ISR_ECHO_ONLY
/* Test RX qua DUNG ISR/ring buffer production - xem giai thich day du o
 * BRINGUP_UART_RX_ISR_ECHO_ONLY o tren. Lap vo han, khong tra ve. */
static void uart_rx_isr_echo_only_bringup_loop(void)
{
    MicroRosTransport_DebugRxIsrOpen();
    uint8_t buf[64];
    while (1)
    {
        const size_t n = MicroRosTransport_DebugRxIsrRead(buf, sizeof(buf));
        for (size_t i = 0U; i < n; i++)
        {
            USART_SendByte(USART1, buf[i]);   /* echo nguyen ven, khong doi */
            GPIO_TogglePin(GPIOC, 13U);
        }
    }
}
#endif /* BRINGUP_UART_RX_ISR_ECHO_ONLY */

int main(void)
{
    /* Cau hinh PLL 480MHz - da xac nhan chay dung tren board that (xem rcc.h) -
     * best-effort neu that bai (timeout HSE/VOS/PLL/switch), SystemCoreClock roi ve
     * HSI 64MHz mac dinh, he thong van chay dung (chi cham hon) - khong co gi trong
     * firmware nay phu thuoc viec PLL co len duoc hay khong (CAN dung HSE truc tiep,
     * Tick dung SystemCoreClock de tu tinh prescaler). */
    (void)RCC_SystemClock_Config_HSE_480MHz();

    Tick_Init();
    configure_can_gpio();
    usart1_init();
    usart2_debug_init();
    led_init();
    AppI2c1_Init();

    /* IMU is observational only: a missing sensor never blocks CAN, motor
     * watchdog, or Stand/Sit. The superloop retries it periodically below. */
    bool imu_ready = MPU6050_Init();

#if BRINGUP_UART_ECHO_ONLY
    uart_echo_only_bringup_loop(); /* khong tra ve - dat TRUOC CAN_Init, xem chi tiet o duoi */
#endif

#if BRINGUP_UART_RX_ECHO_ONLY
    uart_rx_echo_only_bringup_loop(); /* khong tra ve - dat TRUOC CAN_Init, xem chi tiet o duoi */
#endif

#if BRINGUP_UART_RX_ISR_ECHO_ONLY
    uart_rx_isr_echo_only_bringup_loop(); /* khong tra ve - dat TRUOC CAN_Init, xem chi tiet o duoi */
#endif

    if (!CAN_Init(CAN_INSTANCE_1, &CAN_BUS_CONFIG))
    {
        while (1) { /* HSE khong len duoc - treo co chu dich, xem README.md */ }
    }
    if (!CAN_Init(CAN_INSTANCE_2, &CAN_BUS_CONFIG))
    {
        while (1) { /* HSE khong len duoc - treo co chu dich, xem README.md */ }
    }
    /* KHONG treo (1) nhu CAN_Init o tren neu that bai - CAN hong van AN TOAN (khong
     * khop nao nhan lenh, Actuator_Init()'s PING se tu that bai het, g_joint_ok=0 het,
     * Actuator_SetTarget bo qua hoan toan) nhung treo cung o day se chan LUON ca
     * micro-ROS/UART khoi chay - mat kha nang debug tu xa qua /joint_fb (khong co
     * USART2 debug console vat ly de xem ly do). Bo qua loi, van tiep tuc boot -
     * Actuator_Init() se tu bao khop TIMEOUT binh thuong qua duong ROS2. */
    (void)CAN_Start(CAN_INSTANCE_1);
    (void)CAN_Start(CAN_INSTANCE_2);

#if BRINGUP_PING_ONLY
    ping_only_bringup_loop(); /* khong tra ve - xem BRINGUP_PING_ONLY */
#endif

    Actuator_Init();
    print_joint_bringup_report(imu_ready);

    MicroRosBridge_Begin();

    uint32_t last_joint_fb_ms = 0U;
    uint32_t last_joint_diag_ms = 0U;
    uint32_t last_imu_sample_ms = 0U;
    uint32_t last_imu_retry_ms = Tick_GetMs();
    uint32_t imu_consecutive_read_errors = 0U;
    uint32_t last_led_toggle_ms = 0U;
    uint32_t last_reenable_ms = 0U;
    /* Command counter lam epoch cho watchdog; khong dung timestamp=0 lam
     * sentinel vi TIM2 co the wrap dung ve 0 sau khi chay dai ngay. */
    uint32_t joint_cmd_disable_latch_count = 0U;

    while (1)
    {
        MicroRosBridge_SpinSome(); /* xu ly callback subscription /joint_cmd neu co du
                                     * lieu moi (goi Actuator_SetTarget() ben trong) */
        /* Lay timestamp SAU spin: callback co the cap nhat last_joint_cmd_ms
         * va di qua ranh gioi 1ms. Dung timestamp truoc spin de tru timestamp
         * moi se underflow uint32_t va kich watchdog gia ngay lap tuc. */
        const uint32_t now_ms = Tick_GetMs();

        /* main() la noi DUY NHAT goi CAN_Receive() cho moi instance, roi tu
         * dinh tuyen theo id - tranh 2 ham khac nhau cung rut frame khoi 1
         * FIFO phan cung (rut xong la mat, khong "xem truoc roi tra lai"
         * duoc). */
        CAN_Frame rx;

        /* Instance 1: phan hoi BabyAlpha2 (id=0) cua 6 dong co chan truoc. */
        while (CAN_IsRxPending(CAN_INSTANCE_1) && CAN_Receive(CAN_INSTANCE_1, &rx))
        {
            if (rx.id == 0U)
            {
                Actuator_OnBabyAlpha2Frame(CAN_INSTANCE_1, &rx);
            }
        }

        /* Instance 2: chi co phan hoi BabyAlpha2 (id=0) cua 6 dong co chan sau. */
        while (CAN_IsRxPending(CAN_INSTANCE_2) && CAN_Receive(CAN_INSTANCE_2, &rx))
        {
            if (rx.id == 0U)
            {
                Actuator_OnBabyAlpha2Frame(CAN_INSTANCE_2, &rx);
            }
        }

        /* Watchdog cho /joint_cmd (micro-ROS, duong robot THAT). Neu laptop
         * crash/rut UART giua luc robot dang dung, khong co watchdog nay thi
         * moi driver board cu giu nguyen lenh PD cuoi cung MAI MAI
         * (khong ai chu dong tat luc). Da xac nhan qua security review truoc
         * khi ket noi phan cung that - fix nay chu dong Actuator_Disable()
         * MOT LAN moi lan phat hien mat lien ket (dung last_joint_cmd_ms lam
         * "epoch" de tu reset khi co lenh moi toi, tranh spam CAN moi vong lap
         * sau khi da that luc). Bo qua neu chua tung nhan lenh nao ca; trang
         * thai nay duoc theo doi bang co rieng, khong suy ra tu timestamp=0. */
        const uint32_t last_joint_cmd_ms = MicroRosBridge_LastJointCmdMs();
        const uint32_t joint_cmd_count = MicroRosBridge_JointCmdCount();
        if (MicroRosBridge_HasReceivedJointCmd() &&
            (!MicroRosBridge_IsConnected() ||
             (now_ms - last_joint_cmd_ms) >= JOINT_CMD_TIMEOUT_MS) &&
            joint_cmd_disable_latch_count != joint_cmd_count)
        {
            Actuator_Disable();
            /* A host/controller restart starts JointCmd.seq again at zero. The
             * watchdog boundary separates publisher epochs, so the first new
             * command must establish a baseline instead of being counted as a
             * large packet gap against the old process's final sequence. */
            MicroRosBridge_ResetCommandSequence();
            joint_cmd_disable_latch_count = joint_cmd_count;
        }

        Actuator_ServiceSafety();

        if ((now_ms - last_joint_fb_ms) >= JOINT_FB_SEND_PERIOD_MS)
        {
            last_joint_fb_ms = now_ms;
            MicroRosBridge_PublishJointFb();
        }

        if ((now_ms - last_joint_diag_ms) >= JOINT_DIAG_SEND_PERIOD_MS)
        {
            last_joint_diag_ms = now_ms;
            MicroRosBridge_PublishJointDiag();
        }

        if (imu_ready && ((now_ms - last_imu_sample_ms) >= IMU_SAMPLE_PERIOD_MS))
        {
            last_imu_sample_ms = now_ms;
            MPU6050_Reading reading;
            if (MPU6050_Read(&reading))
            {
                imu_consecutive_read_errors = 0U;
                MicroRosBridge_PublishImuRaw(
                    &reading, MICROROS_IMU_STATUS_OK, now_ms);
            }
            else
            {
                MicroRosBridge_PublishImuRaw(
                    NULL, MICROROS_IMU_STATUS_READ_FAILED, now_ms);
                imu_consecutive_read_errors++;
                if (imu_consecutive_read_errors >= IMU_MAX_CONSECUTIVE_READ_ERRORS)
                {
                    imu_ready = false;
                    last_imu_retry_ms = now_ms;
                }
            }
        }
        else if (!imu_ready && ((now_ms - last_imu_retry_ms) >= IMU_RETRY_PERIOD_MS))
        {
            last_imu_retry_ms = now_ms;
            imu_ready = MPU6050_Init();
            imu_consecutive_read_errors = 0U;
            if (!imu_ready)
            {
                MicroRosBridge_PublishImuRaw(
                    NULL, MICROROS_IMU_STATUS_INIT_FAILED, now_ms);
            }
        }

        /* Nhac lai MOTOR_ENABLE dinh ky - xem Actuator_ReenableAll() header,
         * thieu buoc nay khien driver tu roi Standby sau 1 khoang khong duoc
         * nhac (chi nhac 1 lan luc Actuator_Init() la khong du). 500ms khop
         * dung chu ky da proven tren oneLeg (RE_ENABLE_PERIOD_MS). */
        if ((now_ms - last_reenable_ms) >= 500U)
        {
            last_reenable_ms = now_ms;
            Actuator_ReenableAll();
        }

        /* LED PC13 = bang chung bang mat "/joint_cmd" toi duoc MCU qua micro-
         * ROS/UART1 hay khong - nhay lien tuc trong luc con nhan lenh gan day,
         * tat han khi ngung gui (khong lien quan CAN/dong co, xem
         * LED_ACTIVE_WINDOW_MS o dau file). */
        const uint32_t led_last_joint_cmd_ms = MicroRosBridge_LastJointCmdMs();
        if (MicroRosBridge_HasReceivedJointCmd() &&
            (now_ms - led_last_joint_cmd_ms) < LED_ACTIVE_WINDOW_MS)
        {
            if ((now_ms - last_led_toggle_ms) >= LED_BLINK_PERIOD_MS)
            {
                last_led_toggle_ms = now_ms;
                GPIO_TogglePin(GPIOC, 13U);
            }
        }
        else
        {
            GPIO_WritePin(GPIOC, 13U, 1U); /* active LOW -> 1 = tat */
        }

    }
}
