/**
 ******************************************************************************
 * @file    rcc.h
 * @brief   Điều khiển clock: bật clock ngoại vi + cấu hình System Clock
 *          (PLL1: HSE 25MHz -> 480MHz) cho board FK743M5-XIH6.
 ******************************************************************************
 */

#ifndef RCC_H
#define RCC_H

#include <stdint.h>
#include "stm32h743_regs.h"

/* SystemCoreClock: tần số CPU hiện tại (Hz) - các driver khác (SysTick,
 * USART baudrate, TIM prescaler...) đọc biến này để tự tính toán, KHÔNG
 * hard-code tần số. Mặc định = 64 MHz (HSI, giá trị sau reset) cho tới khi
 * RCC_SystemClock_Config_HSE_480MHz() chạy thành công. */
extern volatile uint32_t SystemCoreClock;

/* Tan so APB2 (Hz) - USART1/TIM1/TIM8 dung lam clock nguon. Mac dinh =
 * SystemCoreClock (HSI, khong chia domain nao) cho toi khi
 * RCC_SystemClock_Config_HSE_480MHz() chay xong (luc do = 120 MHz). */
extern volatile uint32_t SystemPCLK2Clock;

/* Bật clock cho 1 cổng GPIO bất kỳ (GPIOA..GPIOK) - tự suy ra đúng bit
 * trong RCC_AHB4ENR từ địa chỉ base của port, không cần nhớ bit thủ công. */
void RCC_GPIO_ClockEnable(GPIO_TypeDef *port);

/* Cấu hình System Clock: HSE (25MHz) -> PLL1 -> 480MHz (CPU), AHB=240MHz,
 * APB1/2/3/4=120MHz - đúng cấu hình tối đa theo datasheet STM32H743
 * (VOS0 - hiệu năng cao nhất).
 *
 * Có timeout cho MỌI bước chờ cờ trạng thái phần cứng (HSE ready, VOS
 * ready, PLL lock, clock switch) - nếu 1 bước timeout, hàm dừng lại NGAY,
 * KHÔNG chuyển SYSCLK sang PLL -> hệ thống vẫn chạy an toàn trên HSI mặc
 * định thay vì treo cứng vô thời hạn chờ 1 cờ không bao giờ set.
 *
 * ⚠ ĐÃ KIỂM THỬ TRÊN BOARD THẬT (FK743M5-XIH6) - KẾT QUẢ (cập nhật):
 *   - HSE 25MHz lên đúng (HSERDY=1) - OK.
 *   - Bước nâng VOS lên Scale 0 + bật Overdrive TỪNG timeout thật
 *     (RCC_CLOCK_ERR_VOS_TIMEOUT) do thiếu bước cấu hình PWR_CR3
 *     (LDO/SMPS) trước khi đổi VOS, VÀ do chỉ chờ mỗi PWR_D3CR.VOSRDY mà
 *     thiếu PWR_CSR1.ACTVOSRDY - đã fix cả 2 (xem PWR_CR3 SCUEN/LDOEN +
 *     chờ ACTVOSRDY rồi mới chờ VOSRDY trong hàm dưới).
 *   - Sau fix: đã xác nhận qua debug UART trên board thật - PLL lên đúng
 *     480MHz (RCC_CLOCK_OK, SystemCoreClock=480000000) ổn định, không còn
 *     rơi về HSI nữa.
 *
 * @return 0 nếu thành công (SystemCoreClock đã cập nhật = 480000000),
 *         khác 0 nếu thất bại ở bước tương ứng (xem enum RCC_ClockStatus) -
 *         hàm có timeout an toàn ở MỌI bước nên dù thất bại cũng không
 *         treo máy, chỉ rơi về HSI 64MHz mặc định.
 */
typedef enum
{
    RCC_CLOCK_OK = 0,
    RCC_CLOCK_ERR_HSE_TIMEOUT,
    RCC_CLOCK_ERR_VOS_TIMEOUT,
    RCC_CLOCK_ERR_PLL_TIMEOUT,
    RCC_CLOCK_ERR_SWITCH_TIMEOUT,
} RCC_ClockStatus;

RCC_ClockStatus RCC_SystemClock_Config_HSE_480MHz(void);

#endif /* RCC_H */
