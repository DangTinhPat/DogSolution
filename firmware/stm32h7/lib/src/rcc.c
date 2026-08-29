#include "rcc.h"

volatile uint32_t SystemCoreClock  = 64000000UL;  /* HSI mac dinh sau reset */
volatile uint32_t SystemPCLK2Clock = 64000000UL;  /* HSI, khong chia domain nao luc mac dinh */

#define RCC_TIMEOUT_LOOPS   1000000UL  /* vong lap busy-wait toi da cho moi buoc cho co */

void RCC_GPIO_ClockEnable(GPIO_TypeDef *port)
{
    /* Cac GPIOx cach nhau deu 0x400 byte, bit enable trong AHB4ENR cung
     * tang deu theo dung thu tu A,B,C... -> tinh bit tu khoang cach dia
     * chi thay vi phai viet 11 ham/case rieng cho tung cong. */
    uint32_t port_index = ((uint32_t)port - (uint32_t)GPIOA) / 0x400UL;
    RCC_AHB4ENR |= (1UL << port_index);
    /* Doc lai de dam bao clock da thuc su on dinh truoc khi ham goi tiep
     * theo dung vao thanh ghi GPIO (khuyen nghi cua RM0433). */
    (void)RCC_AHB4ENR;
}

/* Cho co bit "mask" trong "reg" duoc set, toi da RCC_TIMEOUT_LOOPS vong.
 * Tra ve 1 neu thanh cong, 0 neu timeout. */
static int wait_flag_set(volatile uint32_t *reg, uint32_t mask)
{
    uint32_t timeout = RCC_TIMEOUT_LOOPS;
    while (((*reg) & mask) == 0U)
    {
        if (--timeout == 0U)
        {
            return 0;
        }
    }
    return 1;
}

RCC_ClockStatus RCC_SystemClock_Config_HSE_480MHz(void)
{
    /* ---- Buoc 1: Bat HSE (thach anh ngoai 25MHz) va cho on dinh -------- */
    RCC_CR |= (1UL << 16);   /* HSEON */
    if (!wait_flag_set((volatile uint32_t *)&RCC_CR, (1UL << 17)))  /* HSERDY */
    {
        return RCC_CLOCK_ERR_HSE_TIMEOUT;
    }

    /* ---- Buoc 2: Nang Voltage Scaling len VOS0 (hieu nang cao nhat) ----
     * VOS0 la dieu kien bat buoc de CPU chay duoc 480MHz.
     *
     * LUU Y QUAN TRONG (loi thuc te da gap tren board that): thieu buoc
     * cau hinh PWR_CR3 (kich hoat LDO) TRUOC khi doi VOS se khien VOSRDY
     * KHONG BAO GIO len, du doi bao lau. PWR_CR3 phai duoc ghi qua 2 buoc
     * rieng: ghi bit "SCUEN" (single-write-enable, tu xoa sau khi dung)
     * roi MOI ghi bit LDOEN thuc su - ghi gop 1 lan khong co tac dung. */
    RCC_APB4ENR |= RCC_APB4ENR_SYSCFGEN;

    PWR_CR3 = 0x00000004U;   /* SCUEN: cho phep ghi 1 lan cau hinh cap nguon */
    PWR_CR3 = 0x00000002U;   /* LDOEN: bat LDO che do hoat dong binh thuong  */

    /* Yeu cau VOS Scale 0 VA bat Overdrive (ODEN) CUNG LUC truoc khi cho
     * VOSRDY. */
    PWR_D3CR = (PWR_D3CR & ~PWR_D3CR_VOS_Msk) | PWR_D3CR_VOS_Msk; /* VOS = 11b (Scale 0) */
    SYSCFG_PWRCR |= SYSCFG_PWRCR_ODEN_Msk;                         /* Bat overdrive       */

    /* Phai cho CA 2 thanh ghi: PWR_CSR1.ACTVOSRDY (trang thai THUC te dang
     * ap dung) roi moi den PWR_D3CR.VOSRDY (xac nhan yeu cau da duoc dap
     * ung) - thieu buoc cho CSR1 truoc la nguyen nhan khien D3CR.VOSRDY
     * khong bao gio len o lan thu truoc do. */
    if (!wait_flag_set((volatile uint32_t *)&PWR_CSR1, PWR_CSR1_ACTVOSRDY_Msk))
    {
        return RCC_CLOCK_ERR_VOS_TIMEOUT;
    }
    if (!wait_flag_set((volatile uint32_t *)&PWR_D3CR, PWR_D3CR_VOSRDY_Msk))
    {
        return RCC_CLOCK_ERR_VOS_TIMEOUT;
    }

    /* ---- Buoc 3: Tang Flash wait-state TRUOC khi tang xung nhip --------
     * AXI bus (noi chua Flash) chay o 240MHz (480MHz CPU / HPRE=2) o VOS0
     * -> can 4 wait-state + WRHIGHFREQ=2 (theo bang AN xung nhip STM32H7). */
    FLASH_ACR = 4UL | (2UL << 4);

    /* ---- Buoc 4: Cau hinh PLL1 : ref = HSE/DIVM1 = 25/5 = 5MHz
     *              VCO = ref * DIVN1 = 5 * 192 = 960MHz (wide range)
     *              PLL1P = VCO / DIVP1 = 960 / 2 = 480MHz = SYSCLK ------- */
    RCC_PLLCKSELR = (5UL << 4)     /* DIVM1 = 5   */
                   | (2UL << 0);   /* PLLSRC = HSE (10b) */

    RCC_PLLCFGR = (2UL << 2)       /* PLL1RGE = 4-8MHz input range (10b) */
                | (0UL << 1)       /* PLL1VCOSEL = 0 -> wide VCO 192-960MHz */
                | (1UL << 16);     /* DIVP1EN = 1 (bat ngo ra P dung cho SYSCLK) */

    RCC_PLL1DIVR = (191UL << 0)    /* DIVN1 = 192 (field = N-1) */
                  | (1UL << 9);    /* DIVP1 = 2   (field = P-1) */

    RCC_CR |= (1UL << 24);   /* PLL1ON */
    if (!wait_flag_set((volatile uint32_t *)&RCC_CR, (1UL << 25)))  /* PLL1RDY */
    {
        return RCC_CLOCK_ERR_PLL_TIMEOUT;
    }

    /* ---- Buoc 5: Chia domain TRUOC khi chuyen SYSCLK sang PLL, tranh cac
     * domain AHB/APB bi vuot toc do toi da trong khoanh khac chuyen doi:
     *   D1CPRE = /1  -> CPU  = 480MHz
     *   HPRE   = /2  -> AHB  = 240MHz (AXI, D1/D2/D3 domain)
     *   D1PPRE = /2  -> APB3 = 120MHz
     *   D2PPRE1= /2  -> APB1 = 120MHz
     *   D2PPRE2= /2  -> APB2 = 120MHz
     *   D3PPRE = /2  -> APB4 = 120MHz -------------------------------- */
    RCC_D1CFGR = (0UL << 8)    /* D1CPRE = /1  (0xxx) */
               | (4UL << 4)    /* D1PPRE = /2  (100b) */
               | (8UL << 0);   /* HPRE   = /2  (1000b) */

    RCC_D2CFGR = (4UL << 8)    /* D2PPRE2 = /2 */
               | (4UL << 4);   /* D2PPRE1 = /2 */

    RCC_D3CFGR = (4UL << 4);   /* D3PPRE = /2 */

    /* ---- Buoc 6: Chuyen SYSCLK sang PLL1 va cho xac nhan ---------------- */
    RCC_CFGR = (RCC_CFGR & ~0x7UL) | 0x3UL;   /* SW = 011b (PLL1) */

    {
        uint32_t timeout = RCC_TIMEOUT_LOOPS;
        while (((RCC_CFGR >> 3) & 0x7UL) != 0x3UL)   /* cho SWS = 011b */
        {
            if (--timeout == 0U)
            {
                return RCC_CLOCK_ERR_SWITCH_TIMEOUT;
            }
        }
    }

    SystemCoreClock  = 480000000UL;
    SystemPCLK2Clock = 120000000UL;   /* APB2 = AHB(240) / D2PPRE2(2) = 120 MHz */
    return RCC_CLOCK_OK;
}
