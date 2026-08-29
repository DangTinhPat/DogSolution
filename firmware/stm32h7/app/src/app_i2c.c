#include "app_i2c.h"

#include "gpio.h"
#include "rcc.h"

#include <stddef.h>

/* Minimal STM32H743 I2C-v2 register view kept in app/ so the shared lib/
 * register layer remains untouched. */
typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t TIMINGR;
    volatile uint32_t TIMEOUTR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t PECR;
    volatile uint32_t RXDR;
    volatile uint32_t TXDR;
} AppI2cRegisters;

#define APP_I2C1 ((AppI2cRegisters *)0x40005400UL)

#define APP_RCC_BASE                  0x58024400UL
#define APP_RCC_D2CCIP2R              (*(volatile uint32_t *)(APP_RCC_BASE + 0x54UL))
#define APP_RCC_APB1LRSTR             (*(volatile uint32_t *)(APP_RCC_BASE + 0x90UL))
#define APP_RCC_APB1LENR_I2C1EN       (1UL << 21)
#define APP_RCC_APB1LRSTR_I2C1RST     (1UL << 21)
#define APP_RCC_D2CCIP2R_I2C123SEL_Pos 12U
#define APP_RCC_D2CCIP2R_I2C123SEL_Msk (3UL << APP_RCC_D2CCIP2R_I2C123SEL_Pos)
#define APP_RCC_D2CCIP2R_I2C123SEL_HSI (2UL << APP_RCC_D2CCIP2R_I2C123SEL_Pos)

#define APP_I2C_CR1_PE                (1UL << 0)
#define APP_I2C_CR2_RD_WRN            (1UL << 10)
#define APP_I2C_CR2_START             (1UL << 13)
#define APP_I2C_CR2_STOP              (1UL << 14)
#define APP_I2C_CR2_AUTOEND           (1UL << 25)
#define APP_I2C_CR2_SADD_Pos          1U
#define APP_I2C_CR2_NBYTES_Pos        16U
#define APP_I2C_ISR_TXIS              (1UL << 1)
#define APP_I2C_ISR_RXNE              (1UL << 2)
#define APP_I2C_ISR_NACKF             (1UL << 4)
#define APP_I2C_ISR_STOPF             (1UL << 5)
#define APP_I2C_ISR_TC                (1UL << 6)
#define APP_I2C_ISR_BERR              (1UL << 8)
#define APP_I2C_ISR_ARLO              (1UL << 9)
#define APP_I2C_ISR_OVR               (1UL << 10)
#define APP_I2C_ISR_TIMEOUT           (1UL << 12)
#define APP_I2C_ISR_BUSY              (1UL << 15)
#define APP_I2C_ICR_NACKCF            (1UL << 4)
#define APP_I2C_ICR_STOPCF            (1UL << 5)
#define APP_I2C_ICR_BERRCF            (1UL << 8)
#define APP_I2C_ICR_ARLOCF            (1UL << 9)
#define APP_I2C_ICR_OVRCF             (1UL << 10)
#define APP_I2C_ICR_TIMOUTCF          (1UL << 12)
#define APP_I2C_ISR_ERROR_MASK        (APP_I2C_ISR_NACKF | APP_I2C_ISR_BERR | \
                                      APP_I2C_ISR_ARLO | APP_I2C_ISR_OVR | \
                                      APP_I2C_ISR_TIMEOUT)
#define APP_I2C_ICR_ERROR_MASK        (APP_I2C_ICR_NACKCF | APP_I2C_ICR_BERRCF | \
                                      APP_I2C_ICR_ARLOCF | APP_I2C_ICR_OVRCF | \
                                      APP_I2C_ICR_TIMOUTCF)

/* Explicitly select the always-on 64 MHz HSI kernel for I2C123, independent
 * of whether the CPU successfully switches to the 480 MHz PLL. This proven
 * 100 kHz timing value therefore stays correct in both clock modes. */
#define APP_I2C1_TIMINGR_100KHZ_HSI64 0x3040404BUL

/* Bounded polling prevents an absent/stuck IMU from starving CAN and the
 * motor watchdog indefinitely. A NACK exits immediately. */
#define APP_I2C_TIMEOUT_LOOPS 100000UL

static void abort_transfer(void)
{
    if ((APP_I2C1->ISR & APP_I2C_ISR_BUSY) != 0U)
    {
        APP_I2C1->CR2 |= APP_I2C_CR2_STOP;
    }
    APP_I2C1->ICR = APP_I2C_ICR_ERROR_MASK | APP_I2C_ICR_STOPCF;

    /* Clear a potentially half-finished CR2 transaction before retrying. */
    APP_I2C1->CR1 &= ~APP_I2C_CR1_PE;
    APP_I2C1->CR2 = 0U;
    APP_I2C1->CR1 |= APP_I2C_CR1_PE;
}

static bool wait_flag(uint32_t mask)
{
    uint32_t timeout = APP_I2C_TIMEOUT_LOOPS;
    while (timeout-- > 0U)
    {
        const uint32_t status = APP_I2C1->ISR;
        /* Check errors first: NACKF and STOPF may become set together after
         * the target rejects the final byte. Treating STOPF first would
         * incorrectly report a successful register write. */
        if ((status & APP_I2C_ISR_ERROR_MASK) != 0U)
        {
            abort_transfer();
            return false;
        }
        if ((status & mask) != 0U)
        {
            return true;
        }
    }
    abort_transfer();
    return false;
}

static bool start_transfer(uint8_t addr7, uint32_t nbytes, bool read,
                           bool autoend, bool wait_not_busy)
{
    if (wait_not_busy)
    {
        uint32_t timeout = APP_I2C_TIMEOUT_LOOPS;
        while (((APP_I2C1->ISR & APP_I2C_ISR_BUSY) != 0U) && (timeout-- > 0U))
        {
        }
        if ((APP_I2C1->ISR & APP_I2C_ISR_BUSY) != 0U)
        {
            abort_transfer();
            return false;
        }
    }

    uint32_t cr2 = ((uint32_t)addr7 << APP_I2C_CR2_SADD_Pos) |
                   (nbytes << APP_I2C_CR2_NBYTES_Pos) |
                   APP_I2C_CR2_START;
    if (read)
    {
        cr2 |= APP_I2C_CR2_RD_WRN;
    }
    if (autoend)
    {
        cr2 |= APP_I2C_CR2_AUTOEND;
    }
    APP_I2C1->CR2 = cr2;
    return true;
}

void AppI2c1_Init(void)
{
    RCC_GPIO_ClockEnable(GPIOB);

    const GPIO_InitConfig pb8_scl = {
        .pin = 8U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_OPENDRAIN,
        .speed = GPIO_SPEED_HIGH, .pull = GPIO_PULL_UP, .af = 4U,
    };
    const GPIO_InitConfig pb7_sda = {
        .pin = 7U, .mode = GPIO_MODE_AF, .otype = GPIO_OTYPE_OPENDRAIN,
        .speed = GPIO_SPEED_HIGH, .pull = GPIO_PULL_UP, .af = 4U,
    };
    GPIO_Init(GPIOB, &pb8_scl);
    GPIO_Init(GPIOB, &pb7_sda);

    APP_RCC_D2CCIP2R =
        (APP_RCC_D2CCIP2R & ~APP_RCC_D2CCIP2R_I2C123SEL_Msk) |
        APP_RCC_D2CCIP2R_I2C123SEL_HSI;
    RCC_APB1LENR |= APP_RCC_APB1LENR_I2C1EN;
    (void)RCC_APB1LENR;

    APP_RCC_APB1LRSTR |= APP_RCC_APB1LRSTR_I2C1RST;
    APP_RCC_APB1LRSTR &= ~APP_RCC_APB1LRSTR_I2C1RST;

    APP_I2C1->CR1 = 0U;
    APP_I2C1->TIMINGR = APP_I2C1_TIMINGR_100KHZ_HSI64;
    APP_I2C1->CR1 = APP_I2C_CR1_PE;
}

bool AppI2c1_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    if (!start_transfer(addr7, 2U, false, true, true) ||
        !wait_flag(APP_I2C_ISR_TXIS))
    {
        return false;
    }
    APP_I2C1->TXDR = reg;
    if (!wait_flag(APP_I2C_ISR_TXIS))
    {
        return false;
    }
    APP_I2C1->TXDR = value;
    if (!wait_flag(APP_I2C_ISR_STOPF))
    {
        return false;
    }
    APP_I2C1->ICR = APP_I2C_ICR_STOPCF;
    return true;
}

bool AppI2c1_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if ((buffer == NULL) || (length == 0U))
    {
        return false;
    }

    /* Register pointer write followed by a repeated START. BUSY must remain
     * set between the two phases; waiting for it to clear here is a known
     * failure mode caught in the earlier board bring-up. */
    if (!start_transfer(addr7, 1U, false, false, true) ||
        !wait_flag(APP_I2C_ISR_TXIS))
    {
        return false;
    }
    APP_I2C1->TXDR = reg;
    if (!wait_flag(APP_I2C_ISR_TC) ||
        !start_transfer(addr7, length, true, true, false))
    {
        return false;
    }

    for (uint8_t i = 0U; i < length; ++i)
    {
        if (!wait_flag(APP_I2C_ISR_RXNE))
        {
            return false;
        }
        buffer[i] = (uint8_t)APP_I2C1->RXDR;
    }
    if (!wait_flag(APP_I2C_ISR_STOPF))
    {
        return false;
    }
    APP_I2C1->ICR = APP_I2C_ICR_STOPCF;
    return true;
}
