#include "tim.h"

#define TIM_CR1_CEN_Msk    (1UL << 0)
#define TIM_CR1_ARPE_Msk   (1UL << 7)
#define TIM_EGR_UG_Msk     (1UL << 0)

void TIM_TimeBaseInit(TIM_TypeDef *tim, const TIM_TimeBaseConfig *cfg)
{
    tim->PSC = cfg->prescaler;
    tim->ARR = cfg->period;
    tim->CR1 |= TIM_CR1_ARPE_Msk;   /* nap ARR qua shadow reg, tranh glitch giua chu ky */
    tim->EGR |= TIM_EGR_UG_Msk;    /* force update: nap PSC/ARR vao ngay, khong cho den lan tran dau */
}

void TIM_Start(TIM_TypeDef *tim)
{
    tim->CR1 |= TIM_CR1_CEN_Msk;
}

void TIM_Stop(TIM_TypeDef *tim)
{
    tim->CR1 &= ~TIM_CR1_CEN_Msk;
}

uint32_t TIM_GetCounter(TIM_TypeDef *tim)
{
    return tim->CNT;
}

void TIM_PWM_ConfigChannel(TIM_TypeDef *tim, TIM_Channel ch, uint32_t pulse)
{
    uint32_t ocm_pwm1 = 0x6UL;   /* OCxM = 110b: PWM mode 1 */

    switch (ch)
    {
        case TIM_CHANNEL_1:
            tim->CCMR1 = (tim->CCMR1 & ~(0x7UL << 4)) | (ocm_pwm1 << 4) | (1UL << 3); /* OC1M + OC1PE */
            tim->CCR1  = pulse;
            tim->CCER |= (1UL << 0);   /* CC1E */
            break;
        case TIM_CHANNEL_2:
            tim->CCMR1 = (tim->CCMR1 & ~(0x7UL << 12)) | (ocm_pwm1 << 12) | (1UL << 11);
            tim->CCR2  = pulse;
            tim->CCER |= (1UL << 4);   /* CC2E */
            break;
        case TIM_CHANNEL_3:
            tim->CCMR2 = (tim->CCMR2 & ~(0x7UL << 4)) | (ocm_pwm1 << 4) | (1UL << 3);
            tim->CCR3  = pulse;
            tim->CCER |= (1UL << 8);   /* CC3E */
            break;
        case TIM_CHANNEL_4:
        default:
            tim->CCMR2 = (tim->CCMR2 & ~(0x7UL << 12)) | (ocm_pwm1 << 12) | (1UL << 11);
            tim->CCR4  = pulse;
            tim->CCER |= (1UL << 12);  /* CC4E */
            break;
    }
}

void TIM_PWM_SetDuty(TIM_TypeDef *tim, TIM_Channel ch, uint32_t pulse)
{
    switch (ch)
    {
        case TIM_CHANNEL_1: tim->CCR1 = pulse; break;
        case TIM_CHANNEL_2: tim->CCR2 = pulse; break;
        case TIM_CHANNEL_3: tim->CCR3 = pulse; break;
        case TIM_CHANNEL_4:
        default:            tim->CCR4 = pulse; break;
    }
}
