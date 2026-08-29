/**
 ******************************************************************************
 * @file    tick.h
 * @brief   Bộ đếm mili-giây chạy tự do (free-running), dùng TIM2 (32-bit,
 *          đếm lên, không ngắt) thay vì lib/systick.h - systick.h của thư
 *          viện dùng cho delay CHẶN (blocking, tự bật/tắt SysTick mỗi lần
 *          gọi), không phù hợp để vừa đo thời gian trôi qua vừa chạy vòng
 *          lặp chính không-chặn (non-blocking) và watchdog /joint_cmd cần.
 *
 * ===== LỊCH SỬ LỖI (đã đo thật trên phần cứng, xác nhận qua dự án oneLeg) =====
 *
 * Bản cũ đặt prescaler = SystemCoreClock/1000 - 1, mắc HAI lỗi chồng nhau
 * (chính comment cũ của file này đã CẢNH BÁO TRƯỚC điều này sẽ xảy ra khi
 * PLL 480MHz lên được, nhưng chưa từng sửa - giờ đã lên được thật, xác nhận
 * trên board khác cùng dòng):
 *
 *   1. Lấy SystemCoreClock (xung CPU, 480MHz) làm xung của timer. Sai: TIM2
 *      nằm trên APB1 (domain D2). rcc.c đặt HPRE=/2 (AHB=240MHz) và
 *      D2PPRE1=/2 (APB1=120MHz). Theo quy tắc STM32H7, khi APB prescaler
 *      khác /1 thì xung kernel của timer = 2 x APB1 = 240MHz, KHÔNG phải
 *      480MHz.
 *   2. Nghiêm trọng hơn: PSC là thanh ghi 16-BIT. Giá trị 479999 (0x752FF)
 *      bị CẮT ÂM THẦM còn 21247 (0x52FF), trình biên dịch không báo gì.
 *
 * Hậu quả đo được (dự án oneLeg): 240.2MHz / 21248 ≈ 11306 tick/giây thay vì
 * 1000 -> MỌI mốc thời gian trong firmware chạy NHANH GẤP ~11.3 LẦN.
 *
 * ===== CÁCH SỬA =====
 *
 * Một PSC 16-bit KHÔNG THỂ chia 240MHz xuống thẳng 1kHz (cần chia 240000 >
 * 65535). Chia xuống TICK_HZ=4kHz (PSC+1=60000, vừa khít 16-bit) rồi chia 4
 * trong phần mềm ở Tick_GetMs(). Độ phân giải 0.25ms - thừa cho vòng điều
 * khiển 200Hz (chu kỳ 5ms).
 *
 * Tick_GetTimerClockHz() tự suy xung kernel timer thật từ SystemCoreClock
 * (240MHz nếu PLL 480MHz thành công, bằng thẳng SystemCoreClock nếu đang ở
 * fallback HSI - lúc đó rcc.c thoát sớm trước khi đặt prescaler domain nào,
 * APB1 = SystemCoreClock không chia) - KHÔNG hardcode 1 giá trị, tự đúng cho
 * cả 2 trường hợp clock hiện có của board này.
 ******************************************************************************
 */
#ifndef TICK_H
#define TICK_H

#include <stdint.h>

/* Tần số đếm của TIM2 sau prescaler. Tick_GetMs() = CNT / (TICK_HZ/1000). */
#define TICK_HZ   4000UL

void Tick_Init(void);

/** @brief Số mili-giây đã trôi qua kể từ Tick_Init(). Độ phân giải 0.25ms. */
uint32_t Tick_GetMs(void);

/** @brief Số đếm thô (0.25ms mỗi đơn vị) - dùng khi cần đo mịn hơn 1ms. */
uint32_t Tick_GetRaw(void);

/** @brief Xung kernel thực tế của TIM2 (Hz), suy từ cấu hình clock đang chạy.
 *         Công khai để test/chẩn đoán kiểm chứng được. */
uint32_t Tick_GetTimerClockHz(void);

#endif /* TICK_H */
