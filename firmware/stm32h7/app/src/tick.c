#include "tick.h"
#include "stm32h743_regs.h"
#include "tim.h"
#include "rcc.h"

/* PSC la thanh ghi 16-bit - gia tri lon hon 65535 bi CAT AM THAM. */
#define TICK_PSC_MAX   65535UL

uint32_t Tick_GetTimerClockHz(void)
{
    /* TIM2 nam tren APB1 (domain D2). Quy tac STM32H7: neu APB prescaler
     * khac /1 thi xung kernel timer = 2 x APB1, con neu = /1 thi = APB1.
     *
     *   - Cau hinh PLL 480MHz (rcc.c) thanh cong: HPRE=/2 -> AHB=240MHz,
     *     D2PPRE1=/2 -> APB1=120MHz  =>  TIM2 = 2 x 120 = 240MHz.
     *   - Neu cau hinh PLL that bai (VOS0/Overdrive timeout, xem rcc.c), he
     *     van chay HSI 64MHz va rcc.c thoat truoc khi dat cac bo chia domain,
     *     luc do APB1 = 64MHz voi prescaler /1  =>  TIM2 = 64MHz.
     *
     * Suy tu SystemCoreClock de tu dong dung ca hai truong hop. */
    return (SystemCoreClock == 480000000UL) ? 240000000UL : SystemCoreClock;
}

void Tick_Init(void)
{
    RCC_APB1LENR |= RCC_APB1LENR_TIM2EN;

    /* TIM2 la 32-bit -> ARR = 0xFFFFFFFF de dem tu do lien tuc, khong can ngat.
     *
     * KHONG chia thang xuong 1kHz: 240MHz/1kHz = 240000 > 65535 nen se bi cat
     * am tham (chinh la loi cu). Chia xuong TICK_HZ = 4kHz -> PSC+1 = 60000,
     * vua khit thanh ghi 16-bit. Tick_GetMs() chia them 4 trong phan mem. */
    uint32_t div = Tick_GetTimerClockHz() / TICK_HZ;
    if (div == 0UL) { div = 1UL; }

    /* Chot chan: neu mot cau hinh clock tuong lai lam div vuot 16-bit thi kep
     * lai thay vi de no bi cat am tham. Thoi gian se sai, nhung sai theo huong
     * CHAM hon (an toan hon) va lo ra ngay khi do, chu khong am tham nhanh len. */
    if (div > (TICK_PSC_MAX + 1UL)) { div = TICK_PSC_MAX + 1UL; }

    const TIM_TimeBaseConfig cfg = {
        .prescaler = div - 1UL,
        .period = 0xFFFFFFFFUL,
    };
    TIM_TimeBaseInit(TIM2, &cfg);
    TIM_Start(TIM2);
}

uint32_t Tick_GetRaw(void)
{
    return TIM_GetCounter(TIM2);
}

uint32_t Tick_GetMs(void)
{
    return TIM_GetCounter(TIM2) / (TICK_HZ / 1000UL);
}
