/**
 ******************************************************************************
 * @file    systick.h
 * @brief   Delay chính xác dựa trên SysTick (thay cho vòng lặp busy-wait
 *          đếm số vô căn cứ) - tự động bù đúng theo SystemCoreClock hiện
 *          tại (rcc.h), nên delay_ms()/delay_us() LUÔN đúng thời gian dù
 *          bạn đổi cấu hình clock (HSI 64MHz mặc định hay PLL 480MHz).
 *
 * Cách dùng: gọi SYSTICK_Init() 1 lần sau khi đã chốt xong SystemCoreClock
 * (tức là gọi SAU RCC_SystemClock_Config_HSE_480MHz() nếu có dùng).
 ******************************************************************************
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

void SYSTICK_Init(void);
void SYSTICK_DelayMs(uint32_t ms);
void SYSTICK_DelayUs(uint32_t us);

#endif /* SYSTICK_H */
