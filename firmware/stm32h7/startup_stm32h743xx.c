/**
 ******************************************************************************
 * @file    startup_stm32h743xx.c
 * @brief   Startup file cho STM32H743XIH6 (Arm Cortex-M7), toolchain GCC
 *          (arm-none-eabi-gcc). Không phụ thuộc CMSIS/HAL - toàn bộ định
 *          nghĩa thanh ghi lõi (SCB, MPU) được khai báo cục bộ trong file
 *          này để có thể build độc lập ngay lập tức.
 *
 * Nhiệm vụ của file:
 *   1. Định nghĩa Vector Table đặt tại 0x08000000 (đầu Flash).
 *   2. Reset_Handler - điểm vào đầu tiên của CPU sau khi thoát reset:
 *        - Bật FPU (CPACR - CP10/CP11 Full Access)
 *        - Bật I-Cache và D-Cache của Cortex-M7
 *        - Cấu hình 1 vùng MPU "Non-cacheable, Shareable" dành cho buffer
 *          DMA (giải quyết vấn đề Cache Coherency giữa CPU và DMA)
 *        - Copy .data từ Flash -> RAM, xoá .bss, xoá .noncacheable
 *        - Set lại VTOR trỏ đúng vào Flash (0x08000000)
 *        - Gọi __libc_init_array(), SystemInit(), main()
 *
 * Ghi chú kiến trúc quan trọng:
 *   - Với GCC, Reset_Handler có thể viết bằng C bình thường (không cần
 *     "naked"/asm) vì phần cứng Cortex-M7 đã tự động nạp MSP từ
 *     vector[0] và PC từ vector[1] (Reset_Handler) TRƯỚC khi hàm này
 *     chạy dòng lệnh đầu tiên -> stack đã sẵn sàng, hàm C hoạt động bình
 *     thường như 1 lời gọi hàm thông thường.
 *   - Việc bật D-Cache đòi hỏi vòng lặp invalidate theo set/way (đọc
 *     CCSIDR để biết số set/way) - viết tay bằng Assembly rất dễ sai,
 *     nên toàn bộ phần cache/MPU được viết bằng C cho an toàn & dễ đọc.
 ******************************************************************************
 */

#include <stdint.h>

/* ==========================================================================
 * 1. ĐỊNH NGHĨA THANH GHI LÕI CORTEX-M7 CẦN DÙNG (SCB, MPU)
 *    (Tự khai báo, không include core_cm7.h để file build độc lập)
 * ========================================================================*/

#define SCB_VTOR    (*(volatile uint32_t *)0xE000ED08U)  /* Vector Table Offset Register        */
#define SCB_CPACR   (*(volatile uint32_t *)0xE000ED88U)  /* Coprocessor Access Control Register */
#define SCB_CCR     (*(volatile uint32_t *)0xE000ED14U)  /* Configuration Control Register      */
#define SCB_ICIALLU (*(volatile uint32_t *)0xE000EF50U)  /* I-Cache Invalidate All to PoU       */
#define SCB_CSSELR  (*(volatile uint32_t *)0xE000ED84U)  /* Cache Size Selection Register       */
#define SCB_CCSIDR  (*(volatile uint32_t *)0xE000ED80U)  /* Cache Size ID Register              */
#define SCB_DCISW   (*(volatile uint32_t *)0xE000EF60U)  /* D-Cache Invalidate by Set/Way       */

#define SCB_CCR_IC_Msk  (1UL << 16)  /* Instruction cache enable bit trong CCR */
#define SCB_CCR_DC_Msk  (1UL << 17)  /* Data cache enable bit trong CCR        */

#define MPU_CTRL    (*(volatile uint32_t *)0xE000ED94U)
#define MPU_RNR     (*(volatile uint32_t *)0xE000ED98U)
#define MPU_RBAR    (*(volatile uint32_t *)0xE000ED9CU)
#define MPU_RASR    (*(volatile uint32_t *)0xE000EDA0U)

#define MPU_CTRL_ENABLE_Msk     (1UL << 0)
#define MPU_CTRL_PRIVDEFENA_Msk (1UL << 2)
#define MPU_RASR_ENABLE_Msk     (1UL << 0)

/* ==========================================================================
 * 2. SYMBOL ĐƯỢC LINKER SCRIPT (.ld) CUNG CẤP
 *    Đây là các ĐỊA CHỈ (không phải biến), phải lấy bằng toán tử &.
 * ========================================================================*/

extern uint32_t _estack;         /* Đỉnh stack = đỉnh vùng DTCM RAM (0x2002_0000)   */
extern uint32_t _sidata;         /* Địa chỉ .data trong FLASH (nguồn copy - LMA)    */
extern uint32_t _sdata;          /* Địa chỉ đầu .data trong RAM (đích copy - VMA)   */
extern uint32_t _edata;          /* Địa chỉ cuối .data trong RAM                    */
extern uint32_t _sbss;           /* Địa chỉ đầu .bss                                */
extern uint32_t _ebss;           /* Địa chỉ cuối .bss                               */
extern uint32_t _snoncacheable;  /* Đầu vùng .noncacheable (buffer DMA, SRAM1)      */
extern uint32_t _enoncacheable;  /* Cuối vùng .noncacheable                         */
extern uint32_t _siitcm;         /* Địa chỉ .itcm_text trong FLASH (nguồn copy)     */
extern uint32_t _sitcm;          /* Địa chỉ đầu .itcm_text trong ITCM (đích copy)   */
extern uint32_t _eitcm;          /* Địa chỉ cuối .itcm_text trong ITCM              */

extern void __libc_init_array(void);   /* newlib: gọi static constructors (C/C++) */
extern int  main(void);

/* ==========================================================================
 * 3. PROTOTYPE
 * ========================================================================*/

void Reset_Handler(void);
void Default_Handler(void);
void SystemInit(void);              /* Có thể override lại (không weak) nếu dùng CubeMX system file */
static void CPU_CACHE_Enable(void);
static void MPU_Config(void);

/* Toàn bộ IRQ Handler mặc định đều "weak" và alias sang Default_Handler.
 * Ứng dụng chỉ cần định nghĩa lại đúng tên hàm (không có weak) ở file khác
 * để override, ví dụ: void USART1_IRQHandler(void) { ... } */
#define WEAK_HANDLER __attribute__((weak, alias("Default_Handler")))

/* ---- Core exception handlers (Cortex-M7, không tính SP/Reset) ---- */
void NMI_Handler(void)             WEAK_HANDLER;
void HardFault_Handler(void)       WEAK_HANDLER;
void MemManage_Handler(void)       WEAK_HANDLER;
void BusFault_Handler(void)        WEAK_HANDLER;
void UsageFault_Handler(void)      WEAK_HANDLER;
void SVC_Handler(void)             WEAK_HANDLER;
void DebugMon_Handler(void)        WEAK_HANDLER;
void PendSV_Handler(void)          WEAK_HANDLER;
void SysTick_Handler(void)         WEAK_HANDLER;

/* ---- STM32H743xx peripheral IRQ handlers (IRQ0..IRQ149) ---- */
void WWDG_IRQHandler(void)                 WEAK_HANDLER;
void PVD_AVD_IRQHandler(void)              WEAK_HANDLER;
void TAMP_STAMP_IRQHandler(void)           WEAK_HANDLER;
void RTC_WKUP_IRQHandler(void)             WEAK_HANDLER;
void FLASH_IRQHandler(void)                WEAK_HANDLER;
void RCC_IRQHandler(void)                  WEAK_HANDLER;
void EXTI0_IRQHandler(void)                WEAK_HANDLER;
void EXTI1_IRQHandler(void)                WEAK_HANDLER;
void EXTI2_IRQHandler(void)                WEAK_HANDLER;
void EXTI3_IRQHandler(void)                WEAK_HANDLER;
void EXTI4_IRQHandler(void)                WEAK_HANDLER;
void DMA1_Stream0_IRQHandler(void)         WEAK_HANDLER;
void DMA1_Stream1_IRQHandler(void)         WEAK_HANDLER;
void DMA1_Stream2_IRQHandler(void)         WEAK_HANDLER;
void DMA1_Stream3_IRQHandler(void)         WEAK_HANDLER;
void DMA1_Stream4_IRQHandler(void)         WEAK_HANDLER;
void DMA1_Stream5_IRQHandler(void)         WEAK_HANDLER;
void DMA1_Stream6_IRQHandler(void)         WEAK_HANDLER;
void ADC_IRQHandler(void)                  WEAK_HANDLER;
void FDCAN1_IT0_IRQHandler(void)           WEAK_HANDLER;
void FDCAN2_IT0_IRQHandler(void)           WEAK_HANDLER;
void FDCAN1_IT1_IRQHandler(void)           WEAK_HANDLER;
void FDCAN2_IT1_IRQHandler(void)           WEAK_HANDLER;
void EXTI9_5_IRQHandler(void)              WEAK_HANDLER;
void TIM1_BRK_IRQHandler(void)             WEAK_HANDLER;
void TIM1_UP_IRQHandler(void)              WEAK_HANDLER;
void TIM1_TRG_COM_IRQHandler(void)         WEAK_HANDLER;
void TIM1_CC_IRQHandler(void)              WEAK_HANDLER;
void TIM2_IRQHandler(void)                 WEAK_HANDLER;
void TIM3_IRQHandler(void)                 WEAK_HANDLER;
void TIM4_IRQHandler(void)                 WEAK_HANDLER;
void I2C1_EV_IRQHandler(void)              WEAK_HANDLER;
void I2C1_ER_IRQHandler(void)              WEAK_HANDLER;
void I2C2_EV_IRQHandler(void)              WEAK_HANDLER;
void I2C2_ER_IRQHandler(void)              WEAK_HANDLER;
void SPI1_IRQHandler(void)                 WEAK_HANDLER;
void SPI2_IRQHandler(void)                 WEAK_HANDLER;
void USART1_IRQHandler(void)               WEAK_HANDLER;
void USART2_IRQHandler(void)               WEAK_HANDLER;
void USART3_IRQHandler(void)               WEAK_HANDLER;
void EXTI15_10_IRQHandler(void)            WEAK_HANDLER;
void RTC_Alarm_IRQHandler(void)            WEAK_HANDLER;
void TIM8_BRK_TIM12_IRQHandler(void)       WEAK_HANDLER;
void TIM8_UP_TIM13_IRQHandler(void)        WEAK_HANDLER;
void TIM8_TRG_COM_TIM14_IRQHandler(void)   WEAK_HANDLER;
void TIM8_CC_IRQHandler(void)              WEAK_HANDLER;
void DMA1_Stream7_IRQHandler(void)         WEAK_HANDLER;
void FMC_IRQHandler(void)                  WEAK_HANDLER;
void SDMMC1_IRQHandler(void)               WEAK_HANDLER;
void TIM5_IRQHandler(void)                 WEAK_HANDLER;
void SPI3_IRQHandler(void)                 WEAK_HANDLER;
void UART4_IRQHandler(void)                WEAK_HANDLER;
void UART5_IRQHandler(void)                WEAK_HANDLER;
void TIM6_DAC_IRQHandler(void)             WEAK_HANDLER;
void TIM7_IRQHandler(void)                 WEAK_HANDLER;
void DMA2_Stream0_IRQHandler(void)         WEAK_HANDLER;
void DMA2_Stream1_IRQHandler(void)         WEAK_HANDLER;
void DMA2_Stream2_IRQHandler(void)         WEAK_HANDLER;
void DMA2_Stream3_IRQHandler(void)         WEAK_HANDLER;
void DMA2_Stream4_IRQHandler(void)         WEAK_HANDLER;
void ETH_IRQHandler(void)                  WEAK_HANDLER;
void ETH_WKUP_IRQHandler(void)             WEAK_HANDLER;
void FDCAN_CAL_IRQHandler(void)            WEAK_HANDLER;
void DMA2_Stream5_IRQHandler(void)         WEAK_HANDLER;
void DMA2_Stream6_IRQHandler(void)         WEAK_HANDLER;
void DMA2_Stream7_IRQHandler(void)         WEAK_HANDLER;
void USART6_IRQHandler(void)               WEAK_HANDLER;
void I2C3_EV_IRQHandler(void)              WEAK_HANDLER;
void I2C3_ER_IRQHandler(void)              WEAK_HANDLER;
void OTG_HS_EP1_OUT_IRQHandler(void)       WEAK_HANDLER;
void OTG_HS_EP1_IN_IRQHandler(void)        WEAK_HANDLER;
void OTG_HS_WKUP_IRQHandler(void)          WEAK_HANDLER;
void OTG_HS_IRQHandler(void)               WEAK_HANDLER;
void DCMI_IRQHandler(void)                 WEAK_HANDLER;
void RNG_IRQHandler(void)                  WEAK_HANDLER;
void FPU_IRQHandler(void)                  WEAK_HANDLER;
void UART7_IRQHandler(void)                WEAK_HANDLER;
void UART8_IRQHandler(void)                WEAK_HANDLER;
void SPI4_IRQHandler(void)                 WEAK_HANDLER;
void SPI5_IRQHandler(void)                 WEAK_HANDLER;
void SPI6_IRQHandler(void)                 WEAK_HANDLER;
void SAI1_IRQHandler(void)                 WEAK_HANDLER;
void LTDC_IRQHandler(void)                 WEAK_HANDLER;
void LTDC_ER_IRQHandler(void)              WEAK_HANDLER;
void DMA2D_IRQHandler(void)                WEAK_HANDLER;
void SAI2_IRQHandler(void)                 WEAK_HANDLER;
void QUADSPI_IRQHandler(void)              WEAK_HANDLER;
void LPTIM1_IRQHandler(void)               WEAK_HANDLER;
void CEC_IRQHandler(void)                  WEAK_HANDLER;
void I2C4_EV_IRQHandler(void)              WEAK_HANDLER;
void I2C4_ER_IRQHandler(void)              WEAK_HANDLER;
void SPDIF_RX_IRQHandler(void)             WEAK_HANDLER;
void OTG_FS_EP1_OUT_IRQHandler(void)       WEAK_HANDLER;
void OTG_FS_EP1_IN_IRQHandler(void)        WEAK_HANDLER;
void OTG_FS_WKUP_IRQHandler(void)          WEAK_HANDLER;
void OTG_FS_IRQHandler(void)               WEAK_HANDLER;
void DMAMUX1_OVR_IRQHandler(void)          WEAK_HANDLER;
void HRTIM1_Master_IRQHandler(void)        WEAK_HANDLER;
void HRTIM1_TIMA_IRQHandler(void)          WEAK_HANDLER;
void HRTIM1_TIMB_IRQHandler(void)          WEAK_HANDLER;
void HRTIM1_TIMC_IRQHandler(void)          WEAK_HANDLER;
void HRTIM1_TIMD_IRQHandler(void)          WEAK_HANDLER;
void HRTIM1_TIME_IRQHandler(void)          WEAK_HANDLER;
void HRTIM1_FLT_IRQHandler(void)           WEAK_HANDLER;
void DFSDM1_FLT0_IRQHandler(void)          WEAK_HANDLER;
void DFSDM1_FLT1_IRQHandler(void)          WEAK_HANDLER;
void DFSDM1_FLT2_IRQHandler(void)          WEAK_HANDLER;
void DFSDM1_FLT3_IRQHandler(void)          WEAK_HANDLER;
void SAI3_IRQHandler(void)                 WEAK_HANDLER;
void SWPMI1_IRQHandler(void)               WEAK_HANDLER;
void TIM15_IRQHandler(void)                WEAK_HANDLER;
void TIM16_IRQHandler(void)                WEAK_HANDLER;
void TIM17_IRQHandler(void)                WEAK_HANDLER;
void MDIOS_WKUP_IRQHandler(void)           WEAK_HANDLER;
void MDIOS_IRQHandler(void)                WEAK_HANDLER;
void JPEG_IRQHandler(void)                 WEAK_HANDLER;
void MDMA_IRQHandler(void)                 WEAK_HANDLER;
void SDMMC2_IRQHandler(void)               WEAK_HANDLER;
void HSEM1_IRQHandler(void)                WEAK_HANDLER;
void ADC3_IRQHandler(void)                 WEAK_HANDLER;
void DMAMUX2_OVR_IRQHandler(void)          WEAK_HANDLER;
void BDMA_Channel0_IRQHandler(void)        WEAK_HANDLER;
void BDMA_Channel1_IRQHandler(void)        WEAK_HANDLER;
void BDMA_Channel2_IRQHandler(void)        WEAK_HANDLER;
void BDMA_Channel3_IRQHandler(void)        WEAK_HANDLER;
void BDMA_Channel4_IRQHandler(void)        WEAK_HANDLER;
void BDMA_Channel5_IRQHandler(void)        WEAK_HANDLER;
void BDMA_Channel6_IRQHandler(void)        WEAK_HANDLER;
void BDMA_Channel7_IRQHandler(void)        WEAK_HANDLER;
void COMP1_IRQHandler(void)                WEAK_HANDLER;
void LPTIM2_IRQHandler(void)               WEAK_HANDLER;
void LPTIM3_IRQHandler(void)               WEAK_HANDLER;
void LPTIM4_IRQHandler(void)               WEAK_HANDLER;
void LPTIM5_IRQHandler(void)               WEAK_HANDLER;
void LPUART1_IRQHandler(void)              WEAK_HANDLER;
void CRS_IRQHandler(void)                  WEAK_HANDLER;
void SAI4_IRQHandler(void)                 WEAK_HANDLER;
void WAKEUP_PIN_IRQHandler(void)           WEAK_HANDLER;

/* ==========================================================================
 * 4. VECTOR TABLE
 *    Đặt vào section riêng ".isr_vector" - linker script sẽ ép section này
 *    nằm ở địa chỉ 0x08000000 (đầu Flash), đúng vị trí BOOT của STM32H7 khi
 *    chân BOOT0 chọn boot từ Flash chính.
 *
 *    Phần tử [0]  = giá trị nạp cho MSP (không phải con trỏ hàm!) -> ép kiểu
 *                    con trỏ hàm để tránh warning nhưng bản chất là địa chỉ.
 *    Phần tử [1]  = Reset_Handler
 *    Phần tử [2..14] = core exception handlers
 *    Phần tử [16..165] = IRQ0..IRQ149 ngoại vi STM32H743 (vị trí "0" là các
 *                    IRQ dự trữ/Reserved - PHẢI giữ nguyên vị trí, không được
 *                    bỏ, vì thứ tự trong bảng = số IRQ tra trong NVIC).
 * ========================================================================*/

typedef void (*pFunc)(void);

__attribute__((section(".isr_vector"), used))
const pFunc g_pfnVectors[] =
{
    (pFunc)&_estack,                     /* Initial Stack Pointer */
    Reset_Handler,                       /* Reset Handler          */
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    /* ---------------- External Interrupts (IRQ0..IRQ149) ---------------- */
    WWDG_IRQHandler,                     /* 0   Window WatchDog                      */
    PVD_AVD_IRQHandler,                  /* 1   PVD/AVD qua EXTI                     */
    TAMP_STAMP_IRQHandler,               /* 2   Tamper & TimeStamp qua EXTI          */
    RTC_WKUP_IRQHandler,                 /* 3   RTC Wakeup qua EXTI                  */
    FLASH_IRQHandler,                    /* 4   Flash memory                         */
    RCC_IRQHandler,                      /* 5   RCC                                  */
    EXTI0_IRQHandler,                    /* 6   EXTI Line0                           */
    EXTI1_IRQHandler,                    /* 7   EXTI Line1                           */
    EXTI2_IRQHandler,                    /* 8   EXTI Line2                           */
    EXTI3_IRQHandler,                    /* 9   EXTI Line3                           */
    EXTI4_IRQHandler,                    /* 10  EXTI Line4                           */
    DMA1_Stream0_IRQHandler,             /* 11  DMA1 Stream0                         */
    DMA1_Stream1_IRQHandler,             /* 12  DMA1 Stream1                         */
    DMA1_Stream2_IRQHandler,             /* 13  DMA1 Stream2                         */
    DMA1_Stream3_IRQHandler,             /* 14  DMA1 Stream3                         */
    DMA1_Stream4_IRQHandler,             /* 15  DMA1 Stream4                         */
    DMA1_Stream5_IRQHandler,             /* 16  DMA1 Stream5                         */
    DMA1_Stream6_IRQHandler,             /* 17  DMA1 Stream6                         */
    ADC_IRQHandler,                      /* 18  ADC1, ADC2                           */
    FDCAN1_IT0_IRQHandler,               /* 19  FDCAN1 interrupt line 0              */
    FDCAN2_IT0_IRQHandler,               /* 20  FDCAN2 interrupt line 0              */
    FDCAN1_IT1_IRQHandler,               /* 21  FDCAN1 interrupt line 1              */
    FDCAN2_IT1_IRQHandler,               /* 22  FDCAN2 interrupt line 1              */
    EXTI9_5_IRQHandler,                  /* 23  EXTI Line[9:5]                       */
    TIM1_BRK_IRQHandler,                 /* 24  TIM1 Break                           */
    TIM1_UP_IRQHandler,                  /* 25  TIM1 Update                          */
    TIM1_TRG_COM_IRQHandler,             /* 26  TIM1 Trigger & Commutation           */
    TIM1_CC_IRQHandler,                  /* 27  TIM1 Capture Compare                 */
    TIM2_IRQHandler,                     /* 28  TIM2                                 */
    TIM3_IRQHandler,                     /* 29  TIM3                                 */
    TIM4_IRQHandler,                     /* 30  TIM4                                 */
    I2C1_EV_IRQHandler,                  /* 31  I2C1 Event                           */
    I2C1_ER_IRQHandler,                  /* 32  I2C1 Error                           */
    I2C2_EV_IRQHandler,                  /* 33  I2C2 Event                           */
    I2C2_ER_IRQHandler,                  /* 34  I2C2 Error                           */
    SPI1_IRQHandler,                     /* 35  SPI1                                 */
    SPI2_IRQHandler,                     /* 36  SPI2                                 */
    USART1_IRQHandler,                   /* 37  USART1                               */
    USART2_IRQHandler,                   /* 38  USART2                               */
    USART3_IRQHandler,                   /* 39  USART3                               */
    EXTI15_10_IRQHandler,                /* 40  EXTI Line[15:10]                     */
    RTC_Alarm_IRQHandler,                /* 41  RTC Alarm (A/B) qua EXTI             */
    0,                                    /* 42  Reserved                             */
    TIM8_BRK_TIM12_IRQHandler,           /* 43  TIM8 Break & TIM12                   */
    TIM8_UP_TIM13_IRQHandler,            /* 44  TIM8 Update & TIM13                  */
    TIM8_TRG_COM_TIM14_IRQHandler,       /* 45  TIM8 Trigger/Commutation & TIM14     */
    TIM8_CC_IRQHandler,                  /* 46  TIM8 Capture Compare                 */
    DMA1_Stream7_IRQHandler,             /* 47  DMA1 Stream7                         */
    FMC_IRQHandler,                      /* 48  FMC                                  */
    SDMMC1_IRQHandler,                   /* 49  SDMMC1                               */
    TIM5_IRQHandler,                     /* 50  TIM5                                 */
    SPI3_IRQHandler,                     /* 51  SPI3                                 */
    UART4_IRQHandler,                    /* 52  UART4                                */
    UART5_IRQHandler,                    /* 53  UART5                                */
    TIM6_DAC_IRQHandler,                 /* 54  TIM6 & DAC1/2 underrun               */
    TIM7_IRQHandler,                     /* 55  TIM7                                 */
    DMA2_Stream0_IRQHandler,             /* 56  DMA2 Stream0                         */
    DMA2_Stream1_IRQHandler,             /* 57  DMA2 Stream1                         */
    DMA2_Stream2_IRQHandler,             /* 58  DMA2 Stream2                         */
    DMA2_Stream3_IRQHandler,             /* 59  DMA2 Stream3                         */
    DMA2_Stream4_IRQHandler,             /* 60  DMA2 Stream4                         */
    ETH_IRQHandler,                      /* 61  Ethernet                             */
    ETH_WKUP_IRQHandler,                 /* 62  Ethernet Wakeup qua EXTI             */
    FDCAN_CAL_IRQHandler,                /* 63  FDCAN calibration unit               */
    0,                                    /* 64  Reserved                             */
    0,                                    /* 65  Reserved                             */
    0,                                    /* 66  Reserved                             */
    0,                                    /* 67  Reserved                             */
    DMA2_Stream5_IRQHandler,             /* 68  DMA2 Stream5                         */
    DMA2_Stream6_IRQHandler,             /* 69  DMA2 Stream6                         */
    DMA2_Stream7_IRQHandler,             /* 70  DMA2 Stream7                         */
    USART6_IRQHandler,                   /* 71  USART6                               */
    I2C3_EV_IRQHandler,                  /* 72  I2C3 Event                           */
    I2C3_ER_IRQHandler,                  /* 73  I2C3 Error                           */
    OTG_HS_EP1_OUT_IRQHandler,           /* 74  USB OTG HS End Point 1 Out           */
    OTG_HS_EP1_IN_IRQHandler,            /* 75  USB OTG HS End Point 1 In            */
    OTG_HS_WKUP_IRQHandler,              /* 76  USB OTG HS Wakeup qua EXTI           */
    OTG_HS_IRQHandler,                   /* 77  USB OTG HS                           */
    DCMI_IRQHandler,                     /* 78  DCMI                                 */
    0,                                    /* 79  Reserved                             */
    RNG_IRQHandler,                      /* 80  RNG                                  */
    FPU_IRQHandler,                      /* 81  FPU                                  */
    UART7_IRQHandler,                    /* 82  UART7                                */
    UART8_IRQHandler,                    /* 83  UART8                                */
    SPI4_IRQHandler,                     /* 84  SPI4                                 */
    SPI5_IRQHandler,                     /* 85  SPI5                                 */
    SPI6_IRQHandler,                     /* 86  SPI6                                 */
    SAI1_IRQHandler,                     /* 87  SAI1                                 */
    LTDC_IRQHandler,                     /* 88  LTDC                                 */
    LTDC_ER_IRQHandler,                  /* 89  LTDC Error                           */
    DMA2D_IRQHandler,                    /* 90  DMA2D                                */
    SAI2_IRQHandler,                     /* 91  SAI2                                 */
    QUADSPI_IRQHandler,                  /* 92  QUADSPI                              */
    LPTIM1_IRQHandler,                   /* 93  LPTIM1                               */
    CEC_IRQHandler,                      /* 94  HDMI-CEC                             */
    I2C4_EV_IRQHandler,                  /* 95  I2C4 Event                           */
    I2C4_ER_IRQHandler,                  /* 96  I2C4 Error                           */
    SPDIF_RX_IRQHandler,                 /* 97  SPDIF-RX                             */
    OTG_FS_EP1_OUT_IRQHandler,           /* 98  USB OTG FS End Point 1 Out           */
    OTG_FS_EP1_IN_IRQHandler,            /* 99  USB OTG FS End Point 1 In            */
    OTG_FS_WKUP_IRQHandler,              /* 100 USB OTG FS Wakeup qua EXTI           */
    OTG_FS_IRQHandler,                   /* 101 USB OTG FS                           */
    DMAMUX1_OVR_IRQHandler,              /* 102 DMAMUX1 Overrun                      */
    HRTIM1_Master_IRQHandler,            /* 103 HRTIM Master Timer                   */
    HRTIM1_TIMA_IRQHandler,              /* 104 HRTIM Timer A                        */
    HRTIM1_TIMB_IRQHandler,              /* 105 HRTIM Timer B                        */
    HRTIM1_TIMC_IRQHandler,              /* 106 HRTIM Timer C                        */
    HRTIM1_TIMD_IRQHandler,              /* 107 HRTIM Timer D                        */
    HRTIM1_TIME_IRQHandler,              /* 108 HRTIM Timer E                        */
    HRTIM1_FLT_IRQHandler,               /* 109 HRTIM Fault                          */
    DFSDM1_FLT0_IRQHandler,              /* 110 DFSDM Filter0                        */
    DFSDM1_FLT1_IRQHandler,              /* 111 DFSDM Filter1                        */
    DFSDM1_FLT2_IRQHandler,              /* 112 DFSDM Filter2                        */
    DFSDM1_FLT3_IRQHandler,              /* 113 DFSDM Filter3                        */
    SAI3_IRQHandler,                     /* 114 SAI3                                 */
    SWPMI1_IRQHandler,                   /* 115 Serial Wire Interface 1              */
    TIM15_IRQHandler,                    /* 116 TIM15                                */
    TIM16_IRQHandler,                    /* 117 TIM16                                */
    TIM17_IRQHandler,                    /* 118 TIM17                                */
    MDIOS_WKUP_IRQHandler,               /* 119 MDIOS Wakeup                         */
    MDIOS_IRQHandler,                    /* 120 MDIOS                                */
    JPEG_IRQHandler,                     /* 121 JPEG                                 */
    MDMA_IRQHandler,                     /* 122 MDMA                                 */
    0,                                    /* 123 Reserved                             */
    SDMMC2_IRQHandler,                   /* 124 SDMMC2                               */
    HSEM1_IRQHandler,                    /* 125 HSEM1                                */
    0,                                    /* 126 Reserved                             */
    ADC3_IRQHandler,                     /* 127 ADC3                                 */
    DMAMUX2_OVR_IRQHandler,              /* 128 DMAMUX2 Overrun                      */
    BDMA_Channel0_IRQHandler,            /* 129 BDMA Channel0                        */
    BDMA_Channel1_IRQHandler,            /* 130 BDMA Channel1                        */
    BDMA_Channel2_IRQHandler,            /* 131 BDMA Channel2                        */
    BDMA_Channel3_IRQHandler,            /* 132 BDMA Channel3                        */
    BDMA_Channel4_IRQHandler,            /* 133 BDMA Channel4                        */
    BDMA_Channel5_IRQHandler,            /* 134 BDMA Channel5                        */
    BDMA_Channel6_IRQHandler,            /* 135 BDMA Channel6                        */
    BDMA_Channel7_IRQHandler,            /* 136 BDMA Channel7                        */
    COMP1_IRQHandler,                    /* 137 COMP1                                */
    LPTIM2_IRQHandler,                   /* 138 LPTIM2                               */
    LPTIM3_IRQHandler,                   /* 139 LPTIM3                               */
    LPTIM4_IRQHandler,                   /* 140 LPTIM4                               */
    LPTIM5_IRQHandler,                   /* 141 LPTIM5                               */
    LPUART1_IRQHandler,                  /* 142 LPUART1                              */
    0,                                    /* 143 Reserved                             */
    CRS_IRQHandler,                      /* 144 Clock Recovery System                */
    0,                                    /* 145 Reserved                             */
    SAI4_IRQHandler,                     /* 146 SAI4                                 */
    0,                                    /* 147 Reserved                             */
    0,                                    /* 148 Reserved                             */
    WAKEUP_PIN_IRQHandler,               /* 149 Wakeup pin (WKUP1..6)                */
};

/* ==========================================================================
 * 5. RESET_HANDLER
 * ========================================================================*/

void Reset_Handler(void)
{
    /* ---- Bước 1: Bật FPU --------------------------------------------- */
    /* CPACR (0xE000ED88) bit[23:20] = CP11,CP10 -> ghi 11 vào cả 2 field
     * (Full Access) để cho phép dùng lệnh FPU (VFP) ngay từ đầu, tránh
     * UsageFault khi trình biên dịch chèn lệnh FPU trước khi ta kịp bật. */
    SCB_CPACR |= (0xFUL << 20);
    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");

    /* ---- Bước 2: Bật I-Cache và D-Cache -------------------------------- */
    CPU_CACHE_Enable();

    /* ---- Bước 3: Cấu hình MPU cơ bản cho vùng DMA non-cacheable -------- */
    /* Phải làm TRƯỚC khi bất kỳ code nào ghi dữ liệu vào buffer DMA, để
     * đảm bảo vùng đó luôn "Non-cacheable/Shareable" -> CPU và DMA luôn
     * nhìn thấy cùng 1 dữ liệu mà không cần clean/invalidate cache thủ công. */
    MPU_Config();

    /* ---- Bước 4: Copy .data từ Flash (LMA) sang RAM (VMA) -------------- */
    {
        uint32_t *src = &_sidata;
        uint32_t *dst = &_sdata;
        while (dst < &_edata)
        {
            *dst++ = *src++;
        }
    }

    /* ---- Bước 5: Xoá vùng .bss (zero-init) ------------------------------ */
    {
        uint32_t *dst = &_sbss;
        while (dst < &_ebss)
        {
            *dst++ = 0U;
        }
    }

    /* ---- Bước 6: Xoá vùng .noncacheable (buffer DMA) -------------------- */
    /* Không bắt buộc theo chuẩn C (vùng NOLOAD không có "giá trị khởi tạo"),
     * nhưng zero-init giúp tránh dùng phải rác cũ trong SRAM khi debug. */
    {
        uint32_t *dst = &_snoncacheable;
        while (dst < &_enoncacheable)
        {
            *dst++ = 0U;
        }
    }

    /* ---- Bước 6b: Copy .itcm_text từ Flash sang ITCM --------------------
     * Tương tự .data - code dùng macro ITCM_FUNC (mem_attr.h) được LƯU
     * trong Flash, phải copy sang ITCM TRƯỚC khi có bất kỳ lệnh CALL/BL
     * nào nhảy vào đó, nếu không sẽ fetch nhầm vùng ITCM trống/rác. */
    {
        uint32_t *src = &_siitcm;
        uint32_t *dst = &_sitcm;
        while (dst < &_eitcm)
        {
            *dst++ = *src++;
        }
    }

    /* ---- Bước 7: Set VTOR trỏ đúng vào bảng vector trong Flash ---------- */
    /* Mặc định sau reset VTOR = 0x00000000, nhưng do STM32H7 alias vùng
     * boot Flash vào địa chỉ 0x00000000 nên vector fetch đầu tiên vẫn đúng.
     * Ở đây ta set tường minh VTOR = 0x08000000 để chương trình không phụ
     * thuộc vào cấu hình BOOT pin/alias, đảm bảo đúng chuẩn. */
    SCB_VTOR = 0x08000000U;
    __asm volatile ("dsb 0xF" ::: "memory");

    /* ---- Bước 8: Gọi __libc_init_array() (static constructors newlib) -- */
    __libc_init_array();

    /* ---- Bước 9: Gọi SystemInit() (cấu hình clock/PLL, ...) ------------ */
    SystemInit();

    /* ---- Bước 10: Gọi main() -------------------------------------------- */
    main();

    /* main() không được phép return trong hệ thống nhúng - nếu lỡ return,
     * treo CPU tại đây thay vì "rơi" vào vùng nhớ không xác định. */
    while (1)
    {
    }
}

/* ==========================================================================
 * 6. CPU_CACHE_Enable() - Bật I-Cache / D-Cache của Cortex-M7
 * ========================================================================*/

static void CPU_CACHE_Enable(void)
{
    /* ---- I-Cache -------------------------------------------------------- */
    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");
    SCB_ICIALLU = 0UL;                       /* Invalidate toàn bộ I-Cache      */
    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");
    SCB_CCR |= SCB_CCR_IC_Msk;               /* Enable I-Cache                  */
    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");

    /* ---- D-Cache ---------------------------------------------------------
     * Trước khi enable, phải invalidate toàn bộ D-Cache theo set/way vì
     * cache có thể chứa dữ liệu "rác" còn sót từ debugger/bootloader.
     * CCSIDR cho biết số lượng set (dòng) và way (associativity) của
     * D-Cache Cortex-M7 (thường 16KB/32KB tuỳ dòng H7: 16 way x N set). */
    SCB_CSSELR = 0U;                         /* Chọn Level 1 Data cache          */
    __asm volatile ("dsb 0xF" ::: "memory");

    {
        uint32_t ccsidr = SCB_CCSIDR;
        /* CCSIDR: bits[27:13] = NumWays-1 (associativity), bits[12:3] = NumSets-1 */
        uint32_t sets = (ccsidr >> 13) & 0x7FFFU;
        uint32_t ways;

        do
        {
            ways = (ccsidr >> 3) & 0x3FFU;
            do
            {
                /* DCISW encoding (Cortex-M7, line size cố định 32 byte):
                 *   Way -> bits[31:32-Log2(Assoc)], Set -> bits[SetShift+ :5]
                 * Công thức này khớp với CMSIS SCB_EnableDCache(). */
                uint32_t dcisw = ((sets << 5)  & 0x00007FE0U) |
                                  ((ways << 30) & 0xC0000000U);
                SCB_DCISW = dcisw;
            } while (ways-- != 0U);
        } while (sets-- != 0U);
    }

    __asm volatile ("dsb 0xF" ::: "memory");

    SCB_CCR |= SCB_CCR_DC_Msk;               /* Enable D-Cache                  */

    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");
}

/* ==========================================================================
 * 7. MPU_Config() - Cấu hình 1 region MPU "Normal, Non-cacheable, Shareable"
 *    bao phủ đúng vùng .noncacheable (32KB đầu SRAM1 @ 0x30000000) để dùng
 *    làm buffer DMA an toàn (DMA-safe buffer), không cần clean/invalidate
 *    cache thủ công quanh mỗi lần DMA transfer.
 *
 *    Region size PHẢI khớp với vùng RAM_NOCACHE khai báo trong linker
 *    script (32KB, base 0x30000000, aligned tự nhiên theo size - yêu cầu
 *    bắt buộc của MPU ARMv7-M: base address phải chia hết cho size).
 * ========================================================================*/

#define MPU_REGION_NOCACHE_BASE   0x30000000U
#define MPU_REGION_NOCACHE_SIZE   14U   /* SIZE field: vùng = 2^(SIZE+1) = 2^15 = 32KB */
#define MPU_REGION_NUMBER_0       0U

/* Encode TEX/S/C/B = Normal memory, Non-cacheable, Shareable (TEX=001,C=0,B=0,S=1) */
#define MPU_TEX_NORMAL_NONCACHEABLE (0x1UL << 19)
#define MPU_S_SHAREABLE              (0x1UL << 18)
#define MPU_AP_FULL_ACCESS           (0x3UL << 24)  /* Privileged/Unprivileged R/W */
#define MPU_XN_EXECUTE_NEVER         (0x1UL << 28)  /* Cam CPU thuc thi code tu vung nay */

static void MPU_Config(void)
{
    /* Disable MPU trước khi cấu hình */
    MPU_CTRL = 0U;

    MPU_RNR  = MPU_REGION_NUMBER_0;
    MPU_RBAR = MPU_REGION_NOCACHE_BASE;
    MPU_RASR = MPU_AP_FULL_ACCESS            |
               MPU_XN_EXECUTE_NEVER          |
               MPU_TEX_NORMAL_NONCACHEABLE   |
               MPU_S_SHAREABLE               |
               (MPU_REGION_NOCACHE_SIZE << 1) |
               MPU_RASR_ENABLE_Msk;

    /* Bật MPU:
     *  - ENABLE       : bật MPU
     *  - PRIVDEFENA   : với các vùng địa chỉ KHÔNG được map bởi region nào,
     *                    dùng lại "default memory map" nền tảng của
     *                    Cortex-M7 (giữ nguyên hành vi cacheable mặc định
     *                    cho toàn bộ SRAM/Flash còn lại - chỉ vùng buffer
     *                    DMA 32KB ở trên là bị override thành non-cacheable). */
    MPU_CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;

    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");
}

/* ==========================================================================
 * 8. SystemInit() - weak stub
 *    CubeMX/CubeIDE thường sinh ra file system_stm32h7xx.c với hàm này để
 *    cấu hình PLL/clock tree, VOS (voltage scaling), v.v. Ở đây khai báo
 *    "weak" để project build/link được ngay cả khi CHƯA có file đó; khi có
 *    file system_stm32h7xx.c thật, hàm "strong" trong file đó sẽ override
 *    hàm rỗng này.
 * ========================================================================*/

__attribute__((weak)) void SystemInit(void)
{
    /* Placeholder: cấu hình clock tree (HSE/PLL), Power Supply (VOS), v.v.
     * nên được thực hiện ở đây hoặc trong system_stm32h7xx.c của CubeMX. */
}

/* ==========================================================================
 * 9. Default_Handler - bẫy toàn bộ ngắt/exception chưa được xử lý
 * ========================================================================*/

void Default_Handler(void)
{
    /* Treo tại đây để debug (xem lại thanh ghi LR/IPSR để biết ngắt nào
     * đang gây lỗi). Trong sản phẩm thực tế có thể thay bằng reset hệ thống
     * hoặc ghi log lỗi trước khi reset. */
    while (1)
    {
    }
}
