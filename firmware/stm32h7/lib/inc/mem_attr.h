/**
 ******************************************************************************
 * @file    mem_attr.h
 * @brief   Macro tiện dụng để đặt biến/hàm vào đúng vùng RAM mong muốn,
 *          tận dụng đặc tính từng loại bộ nhớ trên STM32H743 thay vì luôn
 *          để trình biên dịch tự chọn vùng .data/.bss mặc định (AXI SRAM).
 *
 * Bảng lựa chọn nhanh:
 *   - DTCM_DATA        : biến truy cập RẤT thường xuyên (biến điều khiển
 *                        vòng lặp, state máy trạng thái tốc độ cao) -> DTCM
 *                        có độ trễ 0 wait-state, không qua bus matrix AXI.
 *   - ITCM_FUNC         : hàm cực nhạy timing (ISR tốc độ cao, vòng lặp
 *                        điều khiển động cơ...) -> ITCM cũng 0 wait-state
 *                        cho fetch lệnh, không tranh chấp bus với DMA/D-Cache.
 *   - AXI_SRAM_DATA     : mảng/bộ đệm lớn (ảnh camera, frame buffer LCD) ->
 *                        AXI SRAM 512KB, có D-Cache tăng tốc, băng thông cao
 *                        khi giao tiếp DMA2D/LTDC/DCMI.
 *   - NONCACHEABLE_DATA : buffer dùng chung với DMA (SPI/UART/SDMMC/QSPI DMA)
 *                        -> đặt trong 32KB vùng .noncacheable (SRAM1) đã được
 *                        MPU đánh dấu Non-cacheable/Shareable trong
 *                        startup_stm32h743xx.c - KHÔNG cần tự gọi
 *                        clean/invalidate D-Cache quanh transfer.
 *                        Ngân sách CHỈ 32KB - buffer lớn hơn hãy dùng
 *                        AXI_SRAM_DATA + cache.h (clean/invalidate thủ công).
 *
 * Lưu ý: DTCM_DATA và ITCM_FUNC hiện dùng chung section .data/.text mặc
 * định của DTCM/ITCM (linker script copy nguyên khối .data vào AXI SRAM,
 * không phải DTCM) - do đó 2 macro này THỰC SỰ hoạt động chỉ khi bạn đã bật
 * phần mở rộng ITCM copy-on-boot (xem ghi chú trong stm32h743xihx_flash.ld,
 * section .itcm_text). Với DTCM, cách đơn giản và luôn đúng nhất là khai
 * báo biến local trong hàm chạy trực tiếp trên stack (stack vốn đã nằm
 * trong DTCM) thay vì biến global.
 ******************************************************************************
 */

#ifndef MEM_ATTR_H
#define MEM_ATTR_H

/* Hàm đặt vào ITCM (nạp sẵn từ Flash lúc boot - xem Reset_Handler) */
#define ITCM_FUNC       __attribute__((section(".itcm_text")))

/* Biến đặt vào AXI SRAM tường minh (thực ra là mặc định của .data/.bss,
 * macro này chỉ để code tự mô tả rõ ý định, không đổi hành vi). */
#define AXI_SRAM_DATA   __attribute__((section(".axi_ram_data")))

/* Buffer DMA-safe: không qua D-Cache, MPU đã cấu hình sẵn - xem
 * MPU_Config() trong startup_stm32h743xx.c. Ngân sách 32KB. */
#define NONCACHEABLE_DATA(align_bytes) \
    __attribute__((section(".noncacheable"), aligned(align_bytes)))

#endif /* MEM_ATTR_H */
