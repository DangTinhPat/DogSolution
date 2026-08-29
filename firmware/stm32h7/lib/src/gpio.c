#include "gpio.h"

void GPIO_Init(GPIO_TypeDef *port, const GPIO_InitConfig *cfg)
{
    uint32_t pin = cfg->pin;

    /* MODER: 2 bit/chan */
    port->MODER &= ~(0x3UL << (pin * 2U));
    port->MODER |=  ((uint32_t)cfg->mode << (pin * 2U));

    /* OTYPER: 1 bit/chan - chi co y nghia voi OUTPUT/AF, ghi luon cho don gian */
    port->OTYPER &= ~(0x1UL << pin);
    port->OTYPER |=  ((uint32_t)cfg->otype << pin);

    /* OSPEEDR: 2 bit/chan */
    port->OSPEEDR &= ~(0x3UL << (pin * 2U));
    port->OSPEEDR |=  ((uint32_t)cfg->speed << (pin * 2U));

    /* PUPDR: 2 bit/chan */
    port->PUPDR &= ~(0x3UL << (pin * 2U));
    port->PUPDR |=  ((uint32_t)cfg->pull << (pin * 2U));

    /* AFR[0]=chan 0-7 (AFRL), AFR[1]=chan 8-15 (AFRH), 4 bit/chan */
    if (cfg->mode == GPIO_MODE_AF)
    {
        uint32_t afr_index = pin / 8U;
        uint32_t afr_shift = (pin % 8U) * 4U;
        port->AFR[afr_index] &= ~(0xFUL << afr_shift);
        port->AFR[afr_index] |=  (cfg->af << afr_shift);
    }
}

void GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, uint32_t state)
{
    /* BSRR: bit [pin] = Set (keo HIGH), bit [pin+16] = Reset (keo LOW).
     * Dung BSRR thay vi doc-sua-ghi ODR de thao tac atomic (an toan khi
     * co ISR/DMA khac cung truy cap chan khac tren cung port). */
    if (state != 0U)
    {
        port->BSRR = (1UL << pin);
    }
    else
    {
        port->BSRR = (1UL << (pin + 16U));
    }
}

void GPIO_TogglePin(GPIO_TypeDef *port, uint32_t pin)
{
    /* Doc ODR (khong atomic nhu BSRR, nhung toggle von khong the atomic
     * hoa toan bang BSRR don thuan - chap nhan duoc cho GPIO thong thuong). */
    if ((port->ODR & (1UL << pin)) != 0U)
    {
        port->BSRR = (1UL << (pin + 16U));
    }
    else
    {
        port->BSRR = (1UL << pin);
    }
}

uint32_t GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR >> pin) & 0x1UL;
}
