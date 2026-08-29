/**
 ******************************************************************************
 * @file    gpio.h
 * @brief   API cấu hình/đọc/ghi GPIO cho STM32H743, dùng chung cho mọi
 *          cổng (GPIOA..GPIOK) - truyền con trỏ port (vd GPIOC) + số chân.
 ******************************************************************************
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "stm32h743_regs.h"

typedef enum
{
    GPIO_MODE_INPUT     = 0x0U,
    GPIO_MODE_OUTPUT    = 0x1U,
    GPIO_MODE_AF        = 0x2U,
    GPIO_MODE_ANALOG    = 0x3U,
} GPIO_Mode;

typedef enum
{
    GPIO_OTYPE_PUSHPULL  = 0x0U,
    GPIO_OTYPE_OPENDRAIN = 0x1U,
} GPIO_OType;

typedef enum
{
    GPIO_SPEED_LOW       = 0x0U,
    GPIO_SPEED_MEDIUM    = 0x1U,
    GPIO_SPEED_HIGH      = 0x2U,
    GPIO_SPEED_VERYHIGH  = 0x3U,
} GPIO_Speed;

typedef enum
{
    GPIO_PULL_NONE = 0x0U,
    GPIO_PULL_UP   = 0x1U,
    GPIO_PULL_DOWN = 0x2U,
} GPIO_Pull;

typedef struct
{
    uint32_t     pin;      /* 0..15 */
    GPIO_Mode    mode;
    GPIO_OType   otype;    /* chi co y nghia khi mode = OUTPUT hoac AF     */
    GPIO_Speed   speed;
    GPIO_Pull    pull;
    uint32_t     af;       /* 0..15, chi dung khi mode = GPIO_MODE_AF     */
} GPIO_InitConfig;

/* Lưu ý: gọi RCC_GPIO_ClockEnable(port) (rcc.h) trước khi cấu hình. */
void GPIO_Init(GPIO_TypeDef *port, const GPIO_InitConfig *cfg);

void    GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, uint32_t state);
void    GPIO_TogglePin(GPIO_TypeDef *port, uint32_t pin);
uint32_t GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin);

#endif /* GPIO_H */
