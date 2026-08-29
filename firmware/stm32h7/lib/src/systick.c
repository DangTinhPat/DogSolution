#include "systick.h"
#include "stm32h743_regs.h"
#include "rcc.h"

/* Dung 1 lan RELOAD = SystemCoreClock/1000 -1  de counter tran dung 1ms,
 * roi poll co COUNTFLAG (khong dung ngat) - don gian, du dung cho delay
 * blocking thong thuong (khong can Reset_Handler dang ky SysTick_Handler). */

void SYSTICK_Init(void)
{
    SYSTICK_CTRL = 0U;   /* tat truoc khi cau hinh lai */
}

static void systick_wait_one_period(uint32_t reload)
{
    SYSTICK_LOAD = reload;
    SYSTICK_VAL  = 0U;                                  /* clear counter hien tai */
    SYSTICK_CTRL = SYSTICK_CTRL_CLKSOURCE_Msk            /* dung xung CPU (processor clock) */
                  | SYSTICK_CTRL_ENABLE_Msk;

    while ((SYSTICK_CTRL & SYSTICK_CTRL_COUNTFLAG_Msk) == 0U)
    {
        /* doi den khi counter tran ve 0 1 lan (COUNTFLAG tu dong clear khi doc CTRL) */
    }

    SYSTICK_CTRL = 0U;   /* tat lai, tranh chiem dung SysTick ngoai y muon */
}

void SYSTICK_DelayMs(uint32_t ms)
{
    uint32_t reload_1ms = (SystemCoreClock / 1000UL) - 1UL;

    for (uint32_t i = 0; i < ms; i++)
    {
        systick_wait_one_period(reload_1ms);
    }
}

void SYSTICK_DelayUs(uint32_t us)
{
    /* SysTick RELOAD toi da 24-bit (16,777,216) - voi SystemCoreClock=480MHz,
     * 1 lan nap chi cover toi da ~34ms tinh theo us, thua du cho ham nay
     * (thuong goi voi us nho). Voi delay us lon, chia nho tung 1000us. */
    uint32_t reload_1us = (SystemCoreClock / 1000000UL) - 1UL;

    while (us >= 1000U)
    {
        SYSTICK_DelayMs(1);
        us -= 1000U;
    }

    for (uint32_t i = 0; i < us; i++)
    {
        systick_wait_one_period(reload_1us);
    }
}
