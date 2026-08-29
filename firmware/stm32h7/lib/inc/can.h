/**
 ******************************************************************************
 * @file    can.h
 * @brief   Driver FDCAN (lõi Bosch M_CAN) cho STM32H743 - hỗ trợ CẢ
 *          Classic CAN (2.0A/2.0B, tối đa 8 byte data) LẪN CAN FD (tối đa
 *          64 byte data, tùy chọn tăng tốc pha data - BRS) TRÊN CÙNG 1
 *          ngoại vi FDCAN1 (STM32H743 không có bxCAN cổ điển riêng - mọi
 *          "Classic CAN" đều chạy qua lõi FDCAN ở chế độ tương thích).
 *
 * ✅ ĐÃ KIỂM THỬ ỔN ĐỊNH TRÊN BOARD THẬT (FK743M5-XIH6, qua Loopback nội bộ):
 *   7 test case đa dạng (Classic/FD, ID chuẩn/mở rộng, 0-64 byte, có/không
 *   BRS) đều gửi+nhận khớp tuyệt đối 100%, lặp lại ổn định qua nhiều lần
 *   reset. Quá trình kiểm thử phát hiện và sửa được 4 lỗi thật:
 *       1. GFC.ANFS/ANFE phải = 00 (không phải 01 như đoán ban đầu) -
 *          giá trị 01 định tuyến nhầm frame sang Rx FIFO1 chưa cấp phát,
 *          làm mất frame hoàn toàn dù TX vẫn báo thành công.
 *       2. DLC nằm ở bit[19:16] của T1/R1 (không phải bit[3:0] như suy
 *          đoán ban đầu từ trí nhớ) - tìm ra bằng cách ghi trực tiếp qua
 *          GDB/ST-Link (bỏ qua firmware) rồi đối chiếu giá trị đọc lại.
 *       3. Message RAM của FDCAN CHỈ chấp nhận truy cập 32-bit (word) -
 *          copy dữ liệu byte-by-byte bị phần cứng âm thầm bỏ qua gần hết.
 *       4. Rx FIFO0 cấu hình 8 phần tử nhưng get_index chỉ mask 2 bit
 *          (đúng cho 0-3) thay vì 3 bit (0-7) - đã mở rộng mask.
 *   Vị trí bit DLC[19:16]/BRS[20]/FDF[21] đã đối chiếu khớp chính xác với
 *   header chính thức của ST (stm32h7xx_hal_fdcan.h: FDCAN_ELEMENT_MASK_DLC
 *   =0x000F0000, _BRS=0x00100000, _FDF=0x00200000) - không còn nghi vấn.
 *
 *   Giới hạn còn lại (không phải lỗi code, chỉ là chưa có điều kiện kiểm
 *   tra): bit-timing (NBTP/DBTP - tốc độ baud thực trên bus) chưa xác minh
 *   bằng bus CAN thật + transceiver + máy phân tích logic, vì Loopback nội
 *   bộ không đi qua đường truyền vật lý nên không phản ánh đúng tốc độ.
 ******************************************************************************
 */

#ifndef CAN_H
#define CAN_H

#include <stdint.h>
#include <stdbool.h>

/* FDCAN1 = 0x4000A000, FDCAN2 = 0x4000A400 (chỉ mới test FDCAN1) */
#define CAN_INSTANCE_1   0U
#define CAN_INSTANCE_2   1U

typedef enum
{
    CAN_MODE_NORMAL             = 0,  /* hoat dong binh thuong tren bus that       */
    CAN_MODE_LOOPBACK_INTERNAL  = 1,  /* TX noi bo vong lai RX, KHONG dung transceiver, dung de tu-test */
    CAN_MODE_LOOPBACK_EXTERNAL  = 2,  /* TX ra chan that + tu nhan lai, van can transceiver/bus */
    CAN_MODE_BUS_MONITORING     = 3,  /* chi nghe, khong ACK/khong TX (silent) */
} CAN_Mode;

typedef struct
{
    /* Bit timing pha "Nominal" (dung cho ca Classic CAN va phan arbitration
     * cua FD frame). Cong thuc: 1 bit = (1 + TSEG1 + TSEG2) time-quantum,
     * time-quantum = (BRP) / f_kernel. Vi du 25MHz kernel, BRP=1,
     * TSEG1=39,TSEG2=10 -> 50 tq/bit -> 25MHz/50 = 500 kbit/s, sample 80%. */
    uint32_t nominal_prescaler;   /* BRP, 1..512   */
    uint32_t nominal_tseg1;       /* 1..256        */
    uint32_t nominal_tseg2;       /* 1..128        */
    uint32_t nominal_sjw;         /* 1..128        */

    /* Bit timing pha "Data" - CHI dung khi truyen FD frame co BRS=1 (tang
     * toc doan du lieu). Neu khong dung FD/BRS, dat = nominal_* la du. */
    uint32_t data_prescaler;      /* BRP, 1..32    */
    uint32_t data_tseg1;          /* 1..32         */
    uint32_t data_tseg2;          /* 1..16         */
    uint32_t data_sjw;            /* 1..16         */

    CAN_Mode mode;
} CAN_InitConfig;

typedef struct
{
    uint32_t id;             /* Standard ID (0..0x7FF) hoac Extended ID (0..0x1FFFFFFF) */
    bool     extended_id;    /* true = 29-bit extended, false = 11-bit standard         */
    bool     fd_format;      /* true = CAN FD frame (toi da 64 byte), false = Classic (toi da 8 byte) */
    bool     bit_rate_switch;/* true = dung data_* bit timing cho doan data (chi khi fd_format=true) */
    uint8_t  data[64];
    uint8_t  data_len;       /* so byte thuc (0..8 cho Classic, 0..64 cho FD)           */
} CAN_Frame;

/* Khoi tao FDCAN instance (goi 1 lan, khi CCCR.INIT dang =1 mac dinh sau
 * reset). Ham tu bat clock HSE (nguon kernel clock mac dinh cua FDCAN)
 * va clock APB cho FDCAN - khong phu thuoc RCC_SystemClock_Config da
 * chay hay chua. */
bool CAN_Init(uint32_t instance, const CAN_InitConfig *cfg);

/* Roi khoi che do Init (CCCR.INIT=0), FDCAN bat dau hoat dong. */
bool CAN_Start(uint32_t instance);
void CAN_Stop(uint32_t instance);

/* Doc thanh ghi trang thai loi (Bosch M_CAN) - dung khi debug bus im lang
 * hoan toan tren 1 instance cu the (xem oneLeg's can.c, cung ham nay). */
uint32_t CAN_DebugReadPSR(uint32_t instance);   /* Protocol Status Register - LEC/ACT/EP/EW/BO */
uint32_t CAN_DebugReadECR(uint32_t instance);   /* Error Counter Register */
uint32_t CAN_DebugReadCCCR(uint32_t instance);
uint32_t CAN_DebugReadTXFQS(uint32_t instance);
bool CAN_IsBusOff(uint32_t instance);

/* Gui 1 frame (Classic hoac FD tuy frame->fd_format) qua Tx FIFO/Queue.
 * Tra ve false neu Tx FIFO day (khong con cho trong). */
bool CAN_Transmit(uint32_t instance, const CAN_Frame *frame);

/* true neu Rx FIFO0 co it nhat 1 frame dang cho */
bool CAN_IsRxPending(uint32_t instance);

/* Lay 1 frame tu Rx FIFO0 (khong block - phai CAN_IsRxPending() truoc).
 * Tra ve false neu FIFO rong luc goi. */
bool CAN_Receive(uint32_t instance, CAN_Frame *frame);

/* Debug: gia tri R0/R1 tho (raw) cua lan CAN_Receive() gan nhat - dung de
 * chan doan bit-field khi phat trien/kiem thu driver, doc qua ST-Link. */
extern volatile uint32_t CAN_DebugLastR0;
extern volatile uint32_t CAN_DebugLastR1;
extern volatile uint32_t CAN_DebugLastGetIndex;
extern volatile uint32_t CAN_DebugLastElemAddr;

#endif /* CAN_H */
