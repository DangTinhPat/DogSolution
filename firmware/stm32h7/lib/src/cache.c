#include "cache.h"
#include "stm32h743_regs.h"

/* Căn địa chỉ bắt đầu XUỐNG mốc 32 byte gần nhất (đầu dòng cache chứa nó),
 * để đảm bảo thao tác cache bao trùm đủ toàn bộ vùng nhớ được yêu cầu -
 * nếu không căn, byte đầu buffer có thể nằm giữa 1 dòng cache và bị bỏ sót. */
static uint32_t align_down(uint32_t addr)
{
    return addr & ~((uint32_t)CACHE_LINE_SIZE - 1U);
}

void CACHE_CleanDCache_by_Addr(void *addr, int32_t size_bytes)
{
    if (size_bytes <= 0)
    {
        return;
    }

    uint32_t start = align_down((uint32_t)addr);
    uint32_t end   = (uint32_t)addr + (uint32_t)size_bytes;

    __asm volatile ("dsb 0xF" ::: "memory");
    for (; start < end; start += CACHE_LINE_SIZE)
    {
        SCB_DCCMVAC = start;
    }
    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");
}

void CACHE_InvalidateDCache_by_Addr(void *addr, int32_t size_bytes)
{
    if (size_bytes <= 0)
    {
        return;
    }

    uint32_t start = align_down((uint32_t)addr);
    uint32_t end   = (uint32_t)addr + (uint32_t)size_bytes;

    __asm volatile ("dsb 0xF" ::: "memory");
    for (; start < end; start += CACHE_LINE_SIZE)
    {
        SCB_DCIMVAC = start;
    }
    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");
}

void CACHE_CleanInvalidateDCache_by_Addr(void *addr, int32_t size_bytes)
{
    if (size_bytes <= 0)
    {
        return;
    }

    uint32_t start = align_down((uint32_t)addr);
    uint32_t end   = (uint32_t)addr + (uint32_t)size_bytes;

    __asm volatile ("dsb 0xF" ::: "memory");
    for (; start < end; start += CACHE_LINE_SIZE)
    {
        SCB_DCCIMVAC = start;
    }
    __asm volatile ("dsb 0xF" ::: "memory");
    __asm volatile ("isb 0xF" ::: "memory");
}
