#include "can.h"
#include "stm32h743_regs.h"

/* ==========================================================================
 * Thanh ghi FDCAN (loi Bosch M_CAN) - offset tu base tung instance.
 * Dung macro offset (khong dung struct) vi day la khoi thanh ghi rat
 * nhieu/day, 1 sai sot dem padding se lam lech toan bo cac thanh ghi phia
 * sau neu dung struct - macro offset an toan hon (giong cach lam voi RCC).
 * ========================================================================*/

#define FDCAN1_BASE   0x4000A000UL
#define FDCAN2_BASE   0x4000A400UL
/* Vung SRAM dung chung cho Message RAM cua ca FDCAN1 va FDCAN2, 10KB,
 * dia chi co dinh tren STM32H743. */
#define FDCAN_MSGRAM_BASE   0x4000AC00UL

#define FDCAN_REG(instance, off)  (*(volatile uint32_t *)(((instance) == CAN_INSTANCE_2 ? FDCAN2_BASE : FDCAN1_BASE) + (off)))

#define FDCAN_CCCR(i)    FDCAN_REG(i, 0x018UL)
#define FDCAN_NBTP(i)    FDCAN_REG(i, 0x01CUL)
#define FDCAN_DBTP(i)    FDCAN_REG(i, 0x00CUL)
#define FDCAN_TEST(i)    FDCAN_REG(i, 0x010UL)
/* Ban do thanh ghi Bosch M_CAN (doi chieu voi CCCR=0x018/NBTP=0x01C/GFC=0x080
 * da xac nhan dung o duoi day): 0x020 TSCC, 0x024 TSCV, 0x028 TOCC, 0x02C
 * TOCV, ... 0x040 ECR, 0x044 PSR - xac nhan dung tu oneLeg (da tung dat
 * nham o 0x020/0x024, la TSCC/TSCV, luon doc ve 0 vi khong bat clock cho no,
 * hieu nham "khong loi bus" gia). */
#define FDCAN_ECR(i)     FDCAN_REG(i, 0x040UL)   /* Error Counter Register */
#define FDCAN_PSR(i)     FDCAN_REG(i, 0x044UL)   /* Protocol Status Register - LEC/ACT/EP/EW/BO */
#define FDCAN_GFC(i)     FDCAN_REG(i, 0x080UL)
#define FDCAN_SIDFC(i)   FDCAN_REG(i, 0x084UL)
#define FDCAN_XIDFC(i)   FDCAN_REG(i, 0x088UL)
#define FDCAN_RXF0C(i)   FDCAN_REG(i, 0x0A0UL)
#define FDCAN_RXF0S(i)   FDCAN_REG(i, 0x0A4UL)
#define FDCAN_RXF0A(i)   FDCAN_REG(i, 0x0A8UL)
#define FDCAN_RXESC(i)   FDCAN_REG(i, 0x0BCUL)
#define FDCAN_TXBC(i)    FDCAN_REG(i, 0x0C0UL)
#define FDCAN_TXFQS(i)   FDCAN_REG(i, 0x0C4UL)
#define FDCAN_TXESC(i)   FDCAN_REG(i, 0x0C8UL)
#define FDCAN_TXBAR(i)   FDCAN_REG(i, 0x0D0UL)

#define CCCR_INIT_Msk   (1UL << 0)
#define CCCR_CCE_Msk    (1UL << 1)
#define CCCR_FDOE_Msk   (1UL << 8)   /* FD Operation Enable */
#define CCCR_BRSE_Msk   (1UL << 9)   /* Bit Rate Switch Enable */
#define CCCR_TEST_Msk   (1UL << 7)
#define CCCR_MON_Msk    (1UL << 5)   /* Bus Monitoring (silent) */
#define TEST_LBCK_Msk   (1UL << 4)

/* ==========================================================================
 * Bo tri Message RAM CHO 1 INSTANCE (dung chung cong thuc cho FDCAN1 va
 * FDCAN2 - moi instance dung 1 nua vung 10KB de khong dung do nhau).
 * Element size = 64 byte data (ho tro FD toi da) cho ca Rx va Tx.
 * 1 element = 2 word header (T0/T1 hoac R0/R1) + 16 word data = 18 word = 72 byte.
 * ========================================================================*/

#define CAN_NUM_RXF0_ELEM   8U
/* PHAI la 16 (khong phai 4) - da xac nhan tren phan cung that (du an oneLeg):
 * voi >=5 khung gui lien tiep trong 1 chu ky dieu khien (thuc te se xay ra
 * khi co 12 board driver khop that gui/nhan cung luc), FIFO 4 phan tu am
 * tham lam rot cac khung cuoi (CAN_Transmit() tra false, khong co canh bao
 * nao khac) - tung khien 2/6 dong co tren 1 bus hoan toan khong nhan duoc
 * lenh trong nhieu gio debug ma khong ai de y. Doi theo day thi mask doc
 * TFQPI ben duoi (CAN_Transmit) cung PHAI doi tu 0x3 sang 0xF tuong ung
 * (0x3 chi du 2 bit cho 4 phan tu). */
#define CAN_NUM_TXFQ_ELEM   16U
#define CAN_ELEM_SIZE_BYTES 72UL   /* 2 word header + 16 word (64 byte) data */

#define CAN_INSTANCE_PARTITION_BYTES  0x1400UL  /* nua vung 10KB, danh rieng cho 1 instance */

static uint32_t msgram_base(uint32_t instance)
{
    return (uint32_t)FDCAN_MSGRAM_BASE + (instance * CAN_INSTANCE_PARTITION_BYTES);
}

/* Vi khong dung Standard/Extended ID filter list (SIDFC/XIDFC = 0 phan tu),
 * GFC.ANFS/ANFE se quyet dinh MOI frame khong khop filter nao (tuc la MOI
 * frame, vi khong co filter) deu duoc chap nhan vao Rx FIFO0 - don gian
 * hoa toi da, khong can craft filter element. */
static uint32_t rxf0_base(uint32_t instance) { return msgram_base(instance); }
static uint32_t txfq_base(uint32_t instance) { return rxf0_base(instance) + (CAN_NUM_RXF0_ELEM * CAN_ELEM_SIZE_BYTES); }

/* Ma hoa DLC (Data Length Code) tu so byte thuc te - Classic CAN: DLC =
 * so byte (0..8). CAN FD: cac do dai > 8 duoc "nhay coc" theo bang chuan
 * Bosch M_CAN (khong tuyen tinh). */
static uint32_t bytes_to_dlc(uint8_t len)
{
    if (len <= 8U)  { return len; }
    if (len <= 12U) { return 9U; }
    if (len <= 16U) { return 10U; }
    if (len <= 20U) { return 11U; }
    if (len <= 24U) { return 12U; }
    if (len <= 32U) { return 13U; }
    if (len <= 48U) { return 14U; }
    return 15U;   /* 64 byte */
}

static uint8_t dlc_to_bytes(uint32_t dlc)
{
    static const uint8_t table[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    return table[dlc & 0xFU];
}

bool CAN_Init(uint32_t instance, const CAN_InitConfig *cfg)
{
    /* Bat HSE (nguon kernel clock mac dinh cua FDCAN sau reset - RCC
     * khong dong bo APB cho FDCAN, day la nguon rieng qua RCC_D2CCIP1R,
     * gia tri mac dinh sau reset la HSE). */
    RCC_CR |= (1UL << 16);
    {
        uint32_t timeout = 100000UL;
        while ((RCC_CR & (1UL << 17)) == 0U)
        {
            if (--timeout == 0U) { return false; }
        }
    }

    /* Bat clock APB cho FDCAN (chung 1 bit cho ca FDCAN1+FDCAN2 tren H743) */
    RCC_APB1HENR |= (1UL << 8);   /* FDCANEN */

    /* Vao Init mode (CCCR.INIT=1, cho phan cung xac nhan) roi mo khoa cau
     * hinh (CCCR.CCE=1) - bat buoc truoc khi sua bat ky thanh ghi cau hinh
     * nao (NBTP, DBTP, GFC, cac thanh ghi Message RAM...). */
    FDCAN_CCCR(instance) |= CCCR_INIT_Msk;
    {
        uint32_t timeout = 100000UL;
        while ((FDCAN_CCCR(instance) & CCCR_INIT_Msk) == 0U)
        {
            if (--timeout == 0U) { return false; }
        }
    }
    FDCAN_CCCR(instance) |= CCCR_CCE_Msk;

    /* Bit timing pha Nominal (field luu gia tri thuc - 1) */
    FDCAN_NBTP(instance) = ((cfg->nominal_sjw - 1UL)   << 25)
                          | ((cfg->nominal_tseg1 - 1UL) << 8)
                          | ((cfg->nominal_tseg2 - 1UL) << 0)
                          | ((cfg->nominal_prescaler - 1UL) << 16);

    /* Bit timing pha Data (chi dung khi FD+BRS) */
    FDCAN_DBTP(instance) = ((cfg->data_sjw - 1UL)   << 0)
                          | ((cfg->data_tseg1 - 1UL) << 8)
                          | ((cfg->data_tseg2 - 1UL) << 4)
                          | ((cfg->data_prescaler - 1UL) << 16);

    /* Cho phep ca FD format va Bit Rate Switch (frame Classic van hoat
     * dong binh thuong du bat 2 co nay - chi frame co FDF=1/BRS=1 moi
     * thuc su dung toi). */
    FDCAN_CCCR(instance) |= (CCCR_FDOE_Msk | CCCR_BRSE_Msk);

    /* LUON xoa het bit TEST/MON/LBCK truoc - da tung xac nhan bug that (du
     * an oneLeg): neu 1 instance tung duoc Init o mode Loopback/Bus-
     * Monitoring truoc do (vd tu-test luc bring-up), roi sau nay Init lai o
     * mode NORMAL, cac bit TEST/MON/LBCK cu bi SOT LAI (CCCR khong tu reset
     * ve baseline giua 2 lan Init) - khien "NORMAL" thuc chat van la
     * Loopback/Silent, tu nhan lai chinh khung minh gui (tuong nham la
     * phan hoi that tu bus - rat kho phat hien vi "co ve hoat dong").
     * TEST.LBCK chi ghi duoc khi CCCR.TEST=1 (theo Bosch M_CAN) - phai xoa
     * LBCK TRUOC khi xoa CCCR.TEST, vi CCCR.TEST co the dang =1 do config
     * loopback CU con sot lai luc vao ham nay. */
    FDCAN_TEST(instance) &= ~TEST_LBCK_Msk;
    FDCAN_CCCR(instance) &= ~(CCCR_TEST_Msk | CCCR_MON_Msk);

    /* Che do test (Loopback) neu duoc yeu cau */
    if (cfg->mode == CAN_MODE_LOOPBACK_INTERNAL)
    {
        FDCAN_CCCR(instance) |= CCCR_TEST_Msk;
        FDCAN_TEST(instance) |= TEST_LBCK_Msk;
        FDCAN_CCCR(instance) |= CCCR_MON_Msk;   /* khong phat ra chan that, chi vong noi bo */
    }
    else if (cfg->mode == CAN_MODE_LOOPBACK_EXTERNAL)
    {
        FDCAN_CCCR(instance) |= CCCR_TEST_Msk;
        FDCAN_TEST(instance) |= TEST_LBCK_Msk;
    }
    else if (cfg->mode == CAN_MODE_BUS_MONITORING)
    {
        FDCAN_CCCR(instance) |= CCCR_MON_Msk;
    }

    /* Khong dung filter list -> GFC dinh tuyen MOI frame (khong khop
     * filter nao, vi khong co filter) vao Rx FIFO0. ANFS=bits[5:4],
     * ANFE=bits[3:2], gia tri 00 = accept vao FIFO0 (cung la gia tri mac
     * dinh sau reset - da xac nhan tren board that: gia tri 01 khien
     * frame bi dinh tuyen nham sang FIFO1 chua duoc cap phat, gay mat
     * frame hoan toan dù TX van bao thanh cong). */
    FDCAN_SIDFC(instance) = 0U;   /* list size = 0 */
    FDCAN_XIDFC(instance) = 0U;
    FDCAN_GFC(instance) = 0U;

    /* Rx FIFO0: dia chi bat dau (word offset tu Message RAM base) + so
     * phan tu. F0SA la offset TINH TU FDCAN_MSGRAM_BASE, don vi byte
     * (4 byte align). */
    FDCAN_RXF0C(instance) = ((rxf0_base(instance) - FDCAN_MSGRAM_BASE) << 0)
                           | (CAN_NUM_RXF0_ELEM << 16);
    FDCAN_RXESC(instance) = 0x7UL;   /* F0DS = 111b: 64 byte data field */

    /* Tx FIFO/Queue (mode FIFO mac dinh, khong can bat them bit mode) */
    FDCAN_TXBC(instance) = ((txfq_base(instance) - FDCAN_MSGRAM_BASE) << 0)
                          | (CAN_NUM_TXFQ_ELEM << 24);
    FDCAN_TXESC(instance) = 0x7UL;   /* TFDS = 111b: 64 byte data field */

    return true;
}

bool CAN_Start(uint32_t instance)
{
    /* Xoa CCE cung luc voi INIT - CCE chi hop le khi INIT=1 (Bosch M_CAN), de
     * sot CCE=1 khi INIT=0 la trang thai khong chuan. Roi CHO XAC NHAN INIT
     * thuc su ve 0 (co timeout) - ban truoc chi ghi 1 lan roi tin la xong,
     * KHONG xac minh - da xac nhan la bug THAT tren phan cung (du an oneLeg,
     * xem snapshots/leg1_proven): CCCR doc lai VAN con INIT=1 sau khi goi ham
     * nay, khien CAN khong bao gio thuc su roi Init mode de bat dau gui
     * frame - toan bo bus im lang (khong loi ro rang nao khac) dun toi khi
     * fix nay duoc ap dung. */
    FDCAN_CCCR(instance) &= ~(CCCR_INIT_Msk | CCCR_CCE_Msk);

    uint32_t timeout = 100000UL;
    while ((FDCAN_CCCR(instance) & CCCR_INIT_Msk) != 0U)
    {
        if (--timeout == 0U) { return false; }
    }
    return true;
}

void CAN_Stop(uint32_t instance)
{
    FDCAN_CCCR(instance) |= CCCR_INIT_Msk;
}

bool CAN_Transmit(uint32_t instance, const CAN_Frame *frame)
{
    uint32_t txfqs = FDCAN_TXFQS(instance);

    if ((txfqs & (1UL << 21)) != 0U)   /* TFQF: Tx FIFO/Queue Full */
    {
        return false;
    }

    /* TFQPI nam o bit[20:16] (5 bit) - mask PHAI du rong cho CAN_NUM_TXFQ_ELEM.
     * Mask 0x3 cu chi dung cho 4 phan tu (2 bit); voi 16 phan tu can 4 bit. */
    uint32_t put_index = (txfqs >> 16) & 0xFUL;   /* TFQPI: chi so phan tu trong duoc dua vao tiep theo */
    uint32_t elem_addr = txfq_base(instance) + (put_index * CAN_ELEM_SIZE_BYTES);

    uint32_t t0;
    if (frame->extended_id)
    {
        t0 = (1UL << 30) | (frame->id & 0x1FFFFFFFUL);
    }
    else
    {
        t0 = ((frame->id & 0x7FFUL) << 18);
    }

    /* DLC nam o bit[19:16] cua T1 (KHONG PHAI bit[3:0] nhu ban dau) - da
     * xac nhan THAT tren phan cung bang cach ghi truc tiep qua GDB/ST-Link
     * (bo qua firmware) roi doi chieu gia tri doc lai o R1: ghi DLC=3,8,9
     * o bit[19:16] deu doc lai dung y het gia tri da ghi; ghi o bit[3:0]
     * (vi tri cu) khong an huong gi den ket qua. Vi tri chinh xac cua
     * FDF/BRS van CHUA xac dinh duoc du da thu bit20/21/22/23 rieng le va
     * ket hop voi nhieu gia tri DLC khac nhau - tam thoi giu bit20/21 theo
     * suy doan hop ly nhat (chua kiem chung duoc bang thuc nghiem). */
    uint32_t dlc = bytes_to_dlc(frame->data_len);
    uint32_t t1 = ((dlc & 0xFUL) << 16)
                | (frame->fd_format ? (1UL << 21) : 0U)
                | ((frame->fd_format && frame->bit_rate_switch) ? (1UL << 20) : 0U);

    *(volatile uint32_t *)(elem_addr + 0U) = t0;
    *(volatile uint32_t *)(elem_addr + 4U) = t1;

    /* Message RAM cua FDCAN CHI chap nhan truy cap 32-bit (word) - da xac
     * nhan THAT tren phan cung: ghi tung byte rieng le (volatile uint8_t*)
     * bi phan cung am tham bo qua phan lon, chi con lai byte cuoi cung
     * cua moi word "song sot" mot cach ngau nhien. Phai gom 4 byte thanh
     * 1 word roi ghi 1 lan. */
    {
        volatile uint32_t *dst = (volatile uint32_t *)(elem_addr + 8U);
        uint32_t num_words = ((uint32_t)frame->data_len + 3U) / 4U;

        for (uint32_t w = 0U; w < num_words; w++)
        {
            uint32_t word = 0U;
            for (uint32_t b = 0U; b < 4U; b++)
            {
                uint32_t byte_index = (w * 4U) + b;
                uint8_t byte_val = (byte_index < frame->data_len) ? frame->data[byte_index] : 0U;
                word |= ((uint32_t)byte_val) << (b * 8U);
            }
            dst[w] = word;
        }
    }

    FDCAN_TXBAR(instance) = (1UL << put_index);   /* Add Request cho buffer nay */
    return true;
}

bool CAN_IsRxPending(uint32_t instance)
{
    /* F0FL: fill level 0..8 (CAN_NUM_RXF0_ELEM=8) can 4 bit, khong phai 3
     * bit - gia tri 8 (FIFO day hoan toan) se bi mask &0x7 lam sai thanh 0. */
    return ((FDCAN_RXF0S(instance) & 0xFUL) != 0U);   /* F0FL: so phan tu dang co trong FIFO0 */
}

volatile uint32_t CAN_DebugLastR0 = 0U;
volatile uint32_t CAN_DebugLastR1 = 0U;
volatile uint32_t CAN_DebugLastGetIndex = 0xFFFFFFFFU;
volatile uint32_t CAN_DebugLastElemAddr = 0U;

bool CAN_Receive(uint32_t instance, CAN_Frame *frame)
{
    uint32_t rxf0s = FDCAN_RXF0S(instance);

    if ((rxf0s & 0xFUL) == 0U)
    {
        return false;
    }

    /* F0GI: 3 bit (0..7) vi RXF0C cau hinh CAN_NUM_RXF0_ELEM=8 phan tu -
     * mask 2 bit (&0x3) truoc day chi dung tinh co cho element 0..3, sai
     * hoan toan tu element 4 tro di (vd index that=5=101b bi mask con
     * 01b=1) - da xac nhan loi nay THAT qua test case thu 6/7 (dung Rx
     * element 5,6) bi sai ket qua trong khi 5 test dau (element 0..4,
     * ma 4=100b & 0x3=000b lai "vo tinh" trung voi element 0 con trong
     * FIFO tu truoc do khien de gay nham lan neu khong kiem tra ky). */
    uint32_t get_index = (rxf0s >> 8) & 0x7UL;   /* F0GI */
    uint32_t elem_addr = rxf0_base(instance) + (get_index * CAN_ELEM_SIZE_BYTES);

    uint32_t r0 = *(volatile uint32_t *)(elem_addr + 0U);
    uint32_t r1 = *(volatile uint32_t *)(elem_addr + 4U);

    CAN_DebugLastR0 = r0;
    CAN_DebugLastR1 = r1;
    CAN_DebugLastGetIndex = get_index;
    CAN_DebugLastElemAddr = elem_addr;

    frame->extended_id = ((r0 & (1UL << 30)) != 0U);
    frame->id = frame->extended_id ? (r0 & 0x1FFFFFFFUL) : ((r0 >> 18) & 0x7FFUL);
    frame->fd_format = ((r1 & (1UL << 21)) != 0U);
    frame->bit_rate_switch = ((r1 & (1UL << 20)) != 0U);
    frame->data_len = dlc_to_bytes((r1 >> 16) & 0xFUL);

    /* Doc theo word (32-bit) - cung ly do nhu ben CAN_Transmit, Message
     * RAM khong dam bao truy cap byte le. */
    {
        const volatile uint32_t *src = (const volatile uint32_t *)(elem_addr + 8U);
        uint32_t num_words = ((uint32_t)frame->data_len + 3U) / 4U;

        for (uint32_t w = 0U; w < num_words; w++)
        {
            uint32_t word = src[w];
            for (uint32_t b = 0U; b < 4U; b++)
            {
                uint32_t byte_index = (w * 4U) + b;
                if (byte_index < frame->data_len)
                {
                    frame->data[byte_index] = (uint8_t)(word >> (b * 8U));
                }
            }
        }
    }

    FDCAN_RXF0A(instance) = get_index;   /* bao phan cung da doc xong, giai phong phan tu nay */
    return true;
}

uint32_t CAN_DebugReadPSR(uint32_t instance) { return FDCAN_PSR(instance); }
uint32_t CAN_DebugReadECR(uint32_t instance) { return FDCAN_ECR(instance); }
uint32_t CAN_DebugReadCCCR(uint32_t instance) { return FDCAN_CCCR(instance); }
uint32_t CAN_DebugReadTXFQS(uint32_t instance) { return FDCAN_TXFQS(instance); }

#define PSR_BO_Msk   (1UL << 7)

bool CAN_IsBusOff(uint32_t instance)
{
    return (FDCAN_PSR(instance) & PSR_BO_Msk) != 0U;
}
