/**
 ******************************************************************************
 * @file    cache.h
 * @brief   Clean/Invalidate D-Cache theo địa chỉ (dùng cho buffer DMA nằm
 *          trong vùng RAM cacheable bình thường - ví dụ AXI SRAM - khi
 *          buffer LỚN HƠN 32KB nên không đặt vừa trong vùng .noncacheable
 *          cố định của MPU, xem mem_attr.h / MPU_Config()).
 *
 * Quy tắc dùng khi giao tiếp DMA với buffer cacheable:
 *   - CPU ghi dữ liệu vào buffer -> gọi CACHE_CleanDCache_by_Addr() TRƯỚC
 *     khi khởi động DMA đọc (đẩy dữ liệu từ cache xuống RAM thật, DMA mới
 *     thấy đúng dữ liệu).
 *   - DMA ghi dữ liệu vào buffer -> gọi CACHE_InvalidateDCache_by_Addr()
 *     SAU khi DMA báo hoàn tất, TRƯỚC khi CPU đọc (buộc CPU đọc lại từ RAM
 *     thật thay vì dùng dữ liệu cache cũ).
 *
 * Nếu buffer <= 32KB, ưu tiên dùng NONCACHEABLE_DATA (mem_attr.h) thay vì
 * cache.h - đơn giản hơn, không cần nhớ gọi clean/invalidate đúng chỗ.
 ******************************************************************************
 */

#ifndef CACHE_H
#define CACHE_H

#include <stdint.h>

void CACHE_CleanDCache_by_Addr(void *addr, int32_t size_bytes);
void CACHE_InvalidateDCache_by_Addr(void *addr, int32_t size_bytes);
void CACHE_CleanInvalidateDCache_by_Addr(void *addr, int32_t size_bytes);

#endif /* CACHE_H */
