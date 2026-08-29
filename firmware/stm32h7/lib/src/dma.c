#include "dma.h"

#define DMA_CR_PSIZE_Pos   11U
#define DMA_CR_MSIZE_Pos   13U

void DMA_StreamInit(DMA_Stream_TypeDef *stream, uint32_t dmamux_channel,
                     const DMA_StreamConfig *cfg)
{
    /* Tat stream truoc khi cau hinh (bat buoc - EN phai = 0 moi duoc sua CR) */
    stream->CR &= ~DMA_CR_EN_Msk;
    while ((stream->CR & DMA_CR_EN_Msk) != 0U)
    {
        /* doi phan cung xac nhan da tat */
    }

    stream->PAR  = cfg->periph_addr;
    stream->M0AR = cfg->mem_addr;
    stream->NDTR = cfg->data_count;

    stream->CR = ((uint32_t)cfg->direction << DMA_CR_DIR_Pos)
               | ((uint32_t)cfg->data_size << DMA_CR_PSIZE_Pos)
               | ((uint32_t)cfg->data_size << DMA_CR_MSIZE_Pos)
               | (cfg->mem_inc    ? DMA_CR_MINC_Msk : 0U)
               | (cfg->periph_inc ? DMA_CR_PINC_Msk : 0U)
               | (cfg->circular   ? (1UL << 8)       : 0U);   /* CIRC */

    /* DMAMUX: gan request cua ngoai vi vao dung kenh DMAMUX tuong ung voi
     * stream nay. Kenh DMAMUX 0..7 <-> DMA1 stream0..7, kenh 8..15 <->
     * DMA2 stream0..7 (theo dung thu tu vat ly cua STM32H7). */
    DMAMUX1_CxCR(dmamux_channel) = cfg->dmamux_request;
}

void DMA_StreamStart(DMA_Stream_TypeDef *stream)
{
    stream->CR |= DMA_CR_EN_Msk;
}

void DMA_StreamStop(DMA_Stream_TypeDef *stream)
{
    stream->CR &= ~DMA_CR_EN_Msk;
}

/* Vi tri bit TCIF (Transfer Complete) trong LISR/HISR ung voi stream cuc
 * bo 0..3 trong tung thanh ghi - pattern chuan cua ho STM32 DMA (giong
 * nhau tren F4/F7/H7): shift = 6*local + (local>=2 ? 4 : 0), TCIF = +5. */
static uint32_t tcif_bit_pos(uint32_t local_index)
{
    uint32_t shift = (6UL * local_index) + ((local_index >= 2U) ? 4UL : 0UL);
    return shift + 5UL;
}

int DMA_StreamIsComplete(DMA_Stream_TypeDef *stream, uint32_t controller_is_dma2, uint32_t global_index)
{
    (void)stream;
    uint32_t local_index = global_index % 4U;
    uint32_t bit = tcif_bit_pos(local_index);
    uint32_t isr_val;

    if (global_index < 4U)
    {
        isr_val = controller_is_dma2 ? DMA2_LISR : DMA1_LISR;
    }
    else
    {
        isr_val = controller_is_dma2 ? DMA2_HISR : DMA1_HISR;
    }

    return ((isr_val & (1UL << bit)) != 0U) ? 1 : 0;
}

void DMA_StreamClearFlags(DMA_Stream_TypeDef *stream, uint32_t controller_is_dma2, uint32_t global_index)
{
    (void)stream;
    uint32_t local_index = global_index % 4U;
    /* Xoa ca 5 co (FEIF,DMEIF,TEIF,HTIF,TCIF) cua stream nay bang cach ghi
     * 1 vao toan bo cum bit tuong ung - don gian, khong can xoa tung co. */
    uint32_t shift = (6UL * local_index) + ((local_index >= 2U) ? 4UL : 0UL);
    uint32_t mask = (0x3FUL) << shift;   /* 6 bit lien tiep bao trum ca cum flag cua 1 stream */

    if (global_index < 4U)
    {
        if (controller_is_dma2) { DMA2_LIFCR = mask; } else { DMA1_LIFCR = mask; }
    }
    else
    {
        if (controller_is_dma2) { DMA2_HIFCR = mask; } else { DMA1_HIFCR = mask; }
    }
}
