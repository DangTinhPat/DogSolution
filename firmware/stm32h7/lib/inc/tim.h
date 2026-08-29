/**
 ******************************************************************************
 * @file    tim.h
 * @brief   Driver timer đa năng (TIM2/3/4/5) - timebase cơ bản + PWM 1 kênh.
 *          Nhớ bật clock timer qua RCC_APB1LENR_TIMxEN (rcc.h/regs.h)
 *          trước khi gọi các hàm dưới đây.
 ******************************************************************************
 */

#ifndef TIM_H
#define TIM_H

#include <stdint.h>
#include "stm32h743_regs.h"

typedef struct
{
    uint32_t prescaler;   /* PSC: chia tan so dau vao timer, 0..65535 (chia cho psc+1) */
    uint32_t period;      /* ARR: gia tri dem toi da (auto-reload), 0..65535         */
} TIM_TimeBaseConfig;

typedef enum
{
    TIM_CHANNEL_1 = 0,
    TIM_CHANNEL_2 = 1,
    TIM_CHANNEL_3 = 2,
    TIM_CHANNEL_4 = 3,
} TIM_Channel;

void TIM_TimeBaseInit(TIM_TypeDef *tim, const TIM_TimeBaseConfig *cfg);
void TIM_Start(TIM_TypeDef *tim);
void TIM_Stop(TIM_TypeDef *tim);
uint32_t TIM_GetCounter(TIM_TypeDef *tim);

/* PWM mode 1, output active khi CNT < CCR. Goi TIM_TimeBaseInit() truoc
 * de dat tan so PWM (= f_timer / (period+1)), sau do goi ham nay 1 lan
 * cho moi kenh muon dung, roi dieu chinh duty bang TIM_PWM_SetDuty(). */
void TIM_PWM_ConfigChannel(TIM_TypeDef *tim, TIM_Channel ch, uint32_t pulse);
void TIM_PWM_SetDuty(TIM_TypeDef *tim, TIM_Channel ch, uint32_t pulse);

#endif /* TIM_H */
