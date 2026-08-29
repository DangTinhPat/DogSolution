/**
 ******************************************************************************
 * @file    dma.h
 * @brief   Cấu hình 1 DMA stream (DMA1/DMA2, stream 0..7) - chế độ direct
 *          (không FIFO/burst) cho use-case phổ biến: UART/SPI/ADC <-> RAM.
 *
 * LƯU Ý CACHE - đọc trước khi dùng với buffer bộ nhớ:
 *   - Nếu mem_addr nằm trong vùng .noncacheable (macro NONCACHEABLE_DATA,
 *     mem_attr.h, giới hạn 32KB @ SRAM1) -> KHÔNG cần làm gì thêm, CPU và
 *     DMA luôn thấy cùng dữ liệu.
 *   - Nếu mem_addr nằm ở vùng RAM cacheable bình thường (vd AXI SRAM,
 *     .data/.bss mặc định) -> PHẢI tự gọi cache.h quanh transfer:
 *       + Trước khi DMA ĐỌC từ RAM (hướng MEM_TO_PERIPH): gọi
 *         CACHE_CleanDCache_by_Addr() sau khi CPU ghi xong dữ liệu.
 *       + Sau khi DMA GHI vào RAM (hướng PERIPH_TO_MEM) hoàn tất: gọi
 *         CACHE_InvalidateDCache_by_Addr() trước khi CPU đọc kết quả.
 *   Quên bước này là nguyên nhân phổ biến nhất của lỗi "DMA chạy nhưng dữ
 *   liệu CPU đọc được vẫn cũ" trên các dòng STM32 có D-Cache (F7/H7).
 ******************************************************************************
 */

#ifndef DMA_H
#define DMA_H

#include <stdint.h>
#include "stm32h743_regs.h"

typedef enum
{
    DMA_DIR_PERIPH_TO_MEM = 0x0U,
    DMA_DIR_MEM_TO_PERIPH = 0x1U,
    DMA_DIR_MEM_TO_MEM    = 0x2U,
} DMA_Direction;

typedef enum
{
    DMA_DATASIZE_BYTE     = 0x0U,
    DMA_DATASIZE_HALFWORD = 0x1U,
    DMA_DATASIZE_WORD     = 0x2U,
} DMA_DataSize;

typedef struct
{
    uint32_t      dmamux_request;  /* so hieu DMA request cua ngoai vi, xem RM0433 bang DMAMUX */
    DMA_Direction direction;
    DMA_DataSize  data_size;
    uint32_t      periph_addr;
    uint32_t      mem_addr;
    uint32_t      data_count;      /* so phan tu (khong phai byte) can truyen */
    int           mem_inc;         /* 1 = tang dia chi memory sau moi phan tu */
    int           periph_inc;      /* 1 = tang dia chi peripheral (hiem dung) */
    int           circular;        /* 1 = tu dong nap lai NDTR/dia chi khi xong (circular mode) */
} DMA_StreamConfig;

/* dmamux_channel = so kenh DMAMUX (0..15 cho DMA1, 16..31 cho DMA2 - da
 * cong don voi offset DMA2 ben trong ham, chi truyen 0..15 cho ca 2). */
void DMA_StreamInit(DMA_Stream_TypeDef *stream, uint32_t dmamux_channel,
                     const DMA_StreamConfig *cfg);

void DMA_StreamStart(DMA_Stream_TypeDef *stream);
void DMA_StreamStop(DMA_Stream_TypeDef *stream);

/* global_index: 0..7 cho DMA1 stream0..7, hoac dung tham so rieng neu DMA2
 * (xem cai dat trong dma.c - can biet stream nay thuoc DMA1 hay DMA2 de
 * doc dung LISR/HISR). */
int  DMA_StreamIsComplete(DMA_Stream_TypeDef *stream, uint32_t controller_is_dma2, uint32_t global_index);
void DMA_StreamClearFlags(DMA_Stream_TypeDef *stream, uint32_t controller_is_dma2, uint32_t global_index);

#endif /* DMA_H */
