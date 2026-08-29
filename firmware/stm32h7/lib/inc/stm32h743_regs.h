/**
 ******************************************************************************
 * @file    stm32h743_regs.h
 * @brief   Register map dùng chung cho toàn bộ thư viện lib/ - KHÔNG phụ
 *          thuộc CMSIS/HAL, tự định nghĩa struct/địa chỉ để build độc lập
 *          (giống tinh thần của startup_stm32h743xx.c).
 *
 * Quy ước:
 *   - Peripheral có layout thanh ghi liên tục, đơn giản (GPIO, TIM, USART,
 *     DMA stream) -> dùng struct để code gọn, an toàn kiểu.
 *   - Peripheral có layout thưa/nhiều "reserved" xen kẽ dễ đếm nhầm padding
 *     (RCC, PWR, SYSCFG) -> dùng macro offset trực tiếp, tránh 1 lỗi đếm
 *     padding làm lệch TOÀN BỘ các thanh ghi phía sau trong struct.
 ******************************************************************************
 */

#ifndef STM32H743_REGS_H
#define STM32H743_REGS_H

#include <stdint.h>

/* ==========================================================================
 * RCC (Reset and Clock Control) - base 0x58024400
 * ========================================================================*/
#define RCC_BASE            0x58024400UL

#define RCC_CR               (*(volatile uint32_t *)(RCC_BASE + 0x00UL))
#define RCC_CFGR             (*(volatile uint32_t *)(RCC_BASE + 0x10UL))
#define RCC_D1CFGR            (*(volatile uint32_t *)(RCC_BASE + 0x18UL))
#define RCC_D2CFGR            (*(volatile uint32_t *)(RCC_BASE + 0x1CUL))
#define RCC_D3CFGR            (*(volatile uint32_t *)(RCC_BASE + 0x20UL))
#define RCC_PLLCKSELR         (*(volatile uint32_t *)(RCC_BASE + 0x28UL))
#define RCC_PLLCFGR           (*(volatile uint32_t *)(RCC_BASE + 0x2CUL))
#define RCC_PLL1DIVR          (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_PLL1FRACR         (*(volatile uint32_t *)(RCC_BASE + 0x34UL))
#define RCC_AHB3ENR           (*(volatile uint32_t *)(RCC_BASE + 0xD4UL))
#define RCC_AHB1ENR           (*(volatile uint32_t *)(RCC_BASE + 0xD8UL))
#define RCC_AHB2ENR           (*(volatile uint32_t *)(RCC_BASE + 0xDCUL))
#define RCC_AHB4ENR           (*(volatile uint32_t *)(RCC_BASE + 0xE0UL))
#define RCC_APB3ENR           (*(volatile uint32_t *)(RCC_BASE + 0xE4UL))
#define RCC_APB1LENR          (*(volatile uint32_t *)(RCC_BASE + 0xE8UL))
#define RCC_APB1HENR          (*(volatile uint32_t *)(RCC_BASE + 0xECUL))
#define RCC_APB2ENR           (*(volatile uint32_t *)(RCC_BASE + 0xF0UL))
#define RCC_APB4ENR           (*(volatile uint32_t *)(RCC_BASE + 0xF4UL))

/* Bit clock-enable cho GPIOA..GPIOK trong AHB4ENR (bit0..bit10) */
#define RCC_AHB4ENR_GPIOAEN   (1UL << 0)
#define RCC_AHB4ENR_GPIOBEN   (1UL << 1)
#define RCC_AHB4ENR_GPIOCEN   (1UL << 2)
#define RCC_AHB4ENR_GPIODEN   (1UL << 3)
#define RCC_AHB4ENR_GPIOEEN   (1UL << 4)
#define RCC_AHB4ENR_GPIOFEN   (1UL << 5)
#define RCC_AHB4ENR_GPIOGEN   (1UL << 6)
#define RCC_AHB4ENR_GPIOHEN   (1UL << 7)
#define RCC_AHB4ENR_GPIOIEN   (1UL << 8)
#define RCC_AHB4ENR_GPIOJEN   (1UL << 9)
#define RCC_AHB4ENR_GPIOKEN   (1UL << 10)
#define RCC_APB4ENR_SYSCFGEN  (1UL << 1)

#define RCC_AHB1ENR_DMA1EN    (1UL << 0)
#define RCC_AHB1ENR_DMA2EN    (1UL << 1)

#define RCC_APB2ENR_TIM1EN    (1UL << 0)
#define RCC_APB2ENR_USART1EN  (1UL << 4)

#define RCC_APB1LENR_TIM2EN   (1UL << 0)
#define RCC_APB1LENR_TIM3EN   (1UL << 1)
#define RCC_APB1LENR_TIM4EN   (1UL << 2)
#define RCC_APB1LENR_TIM5EN   (1UL << 3)
#define RCC_APB1LENR_USART2EN (1UL << 17)
#define RCC_APB1LENR_USART3EN (1UL << 18)

/* ==========================================================================
 * PWR (Power Control) - base 0x58024800
 * ========================================================================*/
#define PWR_BASE             0x58024800UL
#define PWR_CSR1               (*(volatile uint32_t *)(PWR_BASE + 0x04UL))
#define PWR_CR3                (*(volatile uint32_t *)(PWR_BASE + 0x0CUL))
#define PWR_D3CR                (*(volatile uint32_t *)(PWR_BASE + 0x18UL))
#define PWR_D3CR_VOS_Pos       14U
#define PWR_D3CR_VOS_Msk       (0x3UL << PWR_D3CR_VOS_Pos)
#define PWR_D3CR_VOSRDY_Msk    (1UL << 13)
#define PWR_CSR1_ACTVOSRDY_Msk (1UL << 13)

/* SYSCFG - base 0x58000400 */
#define SYSCFG_BASE          0x58000400UL
#define SYSCFG_PWRCR          (*(volatile uint32_t *)(SYSCFG_BASE + 0x04UL))
#define SYSCFG_PWRCR_ODEN_Msk (1UL << 0)

/* FLASH interface (Flash latency) - base 0x52002000 */
#define FLASH_IF_BASE        0x52002000UL
#define FLASH_ACR              (*(volatile uint32_t *)(FLASH_IF_BASE + 0x00UL))

/* ==========================================================================
 * GPIO - 11 cổng GPIOA..GPIOK, mỗi cổng cách nhau 0x400, base 0x58020000
 * ========================================================================*/
typedef struct
{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];   /* AFR[0]=AFRL (chan 0-7), AFR[1]=AFRH (chan 8-15) */
} GPIO_TypeDef;

#define GPIO_PORT_BASE(port_index)  (0x58020000UL + ((uint32_t)(port_index) * 0x400UL))
#define GPIOA   ((GPIO_TypeDef *)GPIO_PORT_BASE(0))
#define GPIOB   ((GPIO_TypeDef *)GPIO_PORT_BASE(1))
#define GPIOC   ((GPIO_TypeDef *)GPIO_PORT_BASE(2))
#define GPIOD   ((GPIO_TypeDef *)GPIO_PORT_BASE(3))
#define GPIOE   ((GPIO_TypeDef *)GPIO_PORT_BASE(4))
#define GPIOF   ((GPIO_TypeDef *)GPIO_PORT_BASE(5))
#define GPIOG   ((GPIO_TypeDef *)GPIO_PORT_BASE(6))
#define GPIOH   ((GPIO_TypeDef *)GPIO_PORT_BASE(7))
#define GPIOI   ((GPIO_TypeDef *)GPIO_PORT_BASE(8))
#define GPIOJ   ((GPIO_TypeDef *)GPIO_PORT_BASE(9))
#define GPIOK   ((GPIO_TypeDef *)GPIO_PORT_BASE(10))

/* ==========================================================================
 * TIM - timer đa năng (general purpose: TIM2/3/4/5), layout giống nhau
 * ========================================================================*/
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RESERVED1;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t RESERVED2;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef;

#define TIM2   ((TIM_TypeDef *)0x40000000UL)
#define TIM3   ((TIM_TypeDef *)0x40000400UL)
#define TIM4   ((TIM_TypeDef *)0x40000800UL)
#define TIM5   ((TIM_TypeDef *)0x40000C00UL)

/* ==========================================================================
 * USART - layout giống nhau cho USART1/2/3/6 (khác offset base)
 * ========================================================================*/
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t BRR;
    volatile uint32_t GTPR;
    volatile uint32_t RTOR;
    volatile uint32_t RQR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t RDR;
    volatile uint32_t TDR;
    volatile uint32_t PRESC;
} USART_TypeDef;

#define USART1   ((USART_TypeDef *)0x40011000UL)
#define USART2   ((USART_TypeDef *)0x40004400UL)
#define USART3   ((USART_TypeDef *)0x40004800UL)

#define USART_CR1_UE_Msk      (1UL << 0)   /* USART enable          */
#define USART_CR1_RE_Msk      (1UL << 2)   /* Receiver enable       */
#define USART_CR1_TE_Msk      (1UL << 3)   /* Transmitter enable    */
#define USART_CR1_RXNEIE_Msk  (1UL << 5)   /* RXNE ngat khi co du lieu den */
#define USART_CR1_TXEIE_Msk   (1UL << 7)   /* TXE ngat khi thanh ghi truyen rong */
#define USART_ISR_TXE_Msk     (1UL << 7)   /* Transmit data reg empty */
#define USART_ISR_RXNE_Msk    (1UL << 5)   /* Read data reg not empty */
#define USART_ISR_TC_Msk      (1UL << 6)   /* Transmission complete */

/* ==========================================================================
 * NVIC (loi Cortex-M, khong doi giua cac dong STM32) - base 0xE000E100.
 * ISER0 = IRQ 0..31, ISER1 = IRQ 32..63. IPR: 1 byte/IRQ, 4 bit cao dung
 * (khong co configPRIO_BITS vi khong FreeRTOS o day - vd USART1_IRQn=37 ->
 * IPR[37], dung chung group priority voi microros_transport.c cua OUT_SAVE/testSTM).
 * ========================================================================*/
#define NVIC_ISER0            (*(volatile uint32_t *)0xE000E100UL)
#define NVIC_ISER1            (*(volatile uint32_t *)0xE000E104UL)
#define NVIC_IPR              ((volatile uint8_t *)0xE000E400UL)
#define USART1_IRQn           37

/* DWT (Data Watchpoint and Trace) - bo dem cycle CPU co san tren moi loi Cortex-M co
 * debug unit (kem ca M7) - dung do do chinh xac thoi gian thuc thi 1 doan code (vd
 * benchmark truoc/sau khi doi USART TX tu polling sang ngat) ma khong can bat ky ngoai
 * vi/timer nao khac, do phan giai bang dung 1 chu ky CPU (~2.08ns o 480MHz). */
#define CoreDebug_DEMCR       (*(volatile uint32_t *)0xE000EDFCUL)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL              (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)
#define DWT_CYCCNT            (*(volatile uint32_t *)0xE0001004UL)

/* ==========================================================================
 * DMA - DMA1/DMA2, mỗi controller có 8 stream (0..7), layout giống nhau
 * ========================================================================*/
typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t NDTR;
    volatile uint32_t PAR;
    volatile uint32_t M0AR;
    volatile uint32_t M1AR;
    volatile uint32_t FCR;
} DMA_Stream_TypeDef;

#define DMA1_BASE            0x40020000UL
#define DMA2_BASE            0x40020400UL
/* Thanh ghi LISR/HISR/LIFCR/HIFCR nam o offset 0x00-0x0C tinh tu base
 * controller (KHONG phai base stream) - stream 0 bat dau tu offset 0x10 */
#define DMA1_LISR             (*(volatile uint32_t *)(DMA1_BASE + 0x00UL))
#define DMA1_HISR             (*(volatile uint32_t *)(DMA1_BASE + 0x04UL))
#define DMA1_LIFCR            (*(volatile uint32_t *)(DMA1_BASE + 0x08UL))
#define DMA1_HIFCR            (*(volatile uint32_t *)(DMA1_BASE + 0x0CUL))
#define DMA2_LISR             (*(volatile uint32_t *)(DMA2_BASE + 0x00UL))
#define DMA2_HISR             (*(volatile uint32_t *)(DMA2_BASE + 0x04UL))
#define DMA2_LIFCR             (*(volatile uint32_t *)(DMA2_BASE + 0x08UL))
#define DMA2_HIFCR             (*(volatile uint32_t *)(DMA2_BASE + 0x0CUL))

#define DMA1_STREAM(n)   ((DMA_Stream_TypeDef *)(DMA1_BASE + 0x10UL + ((uint32_t)(n) * 0x18UL)))
#define DMA2_STREAM(n)   ((DMA_Stream_TypeDef *)(DMA2_BASE + 0x10UL + ((uint32_t)(n) * 0x18UL)))

/* DMAMUX1 - anh xa request ngoai vi vao stream, base 0x40020800 */
#define DMAMUX1_BASE          0x40020800UL
#define DMAMUX1_CxCR(ch)      (*(volatile uint32_t *)(DMAMUX1_BASE + ((uint32_t)(ch) * 0x04UL)))

#define DMA_CR_EN_Msk         (1UL << 0)
#define DMA_CR_DIR_Pos        6U
#define DMA_CR_MINC_Msk       (1UL << 10)
#define DMA_CR_PINC_Msk       (1UL << 9)
#define DMA_CR_TCIE_Msk       (1UL << 4)

/* ==========================================================================
 * SysTick (core Cortex-M7, khong doi giua cac dong ARM) - base 0xE000E010
 * ========================================================================*/
#define SYSTICK_BASE         0xE000E010UL
#define SYSTICK_CTRL           (*(volatile uint32_t *)(SYSTICK_BASE + 0x00UL))
#define SYSTICK_LOAD            (*(volatile uint32_t *)(SYSTICK_BASE + 0x04UL))
#define SYSTICK_VAL              (*(volatile uint32_t *)(SYSTICK_BASE + 0x08UL))
#define SYSTICK_CTRL_ENABLE_Msk   (1UL << 0)
#define SYSTICK_CTRL_TICKINT_Msk  (1UL << 1)
#define SYSTICK_CTRL_CLKSOURCE_Msk (1UL << 2)
#define SYSTICK_CTRL_COUNTFLAG_Msk (1UL << 16)

/* ==========================================================================
 * SCB cache maintenance-by-address (core Cortex-M7) - dung cho lib/cache.c
 * ========================================================================*/
#define SCB_DCIMVAC   (*(volatile uint32_t *)0xE000EF5CU)  /* Invalidate by addr       */
#define SCB_DCCMVAC   (*(volatile uint32_t *)0xE000EF68U)  /* Clean by addr            */
#define SCB_DCCIMVAC  (*(volatile uint32_t *)0xE000EF70U)  /* Clean+Invalidate by addr */
#define CACHE_LINE_SIZE  32U   /* Cortex-M7: kich thuoc 1 dong cache co dinh, 32 byte */

#endif /* STM32H743_REGS_H */
