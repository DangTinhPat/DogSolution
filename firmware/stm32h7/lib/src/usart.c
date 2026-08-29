#include "usart.h"
#include "rcc.h"

void USART_Init(USART_TypeDef *usart, uint32_t baudrate)
{
    usart->CR1 = 0U;   /* tat truoc khi cau hinh (UE=0) */

    /* Oversampling mac dinh = 16 (CR1.OVER8 = 0) -> BRR = fclk / baud */
    usart->BRR = SystemPCLK2Clock / baudrate;

    usart->CR1 = USART_CR1_TE_Msk | USART_CR1_RE_Msk | USART_CR1_UE_Msk;
}

void USART_SendByte(USART_TypeDef *usart, uint8_t byte)
{
    while ((usart->ISR & USART_ISR_TXE_Msk) == 0U)
    {
        /* doi thanh ghi TX rong */
    }
    usart->TDR = byte;
}

void USART_SendString(USART_TypeDef *usart, const char *str)
{
    while (*str != '\0')
    {
        USART_SendByte(usart, (uint8_t)*str);
        str++;
    }
}

uint8_t USART_ReceiveByte(USART_TypeDef *usart)
{
    while ((usart->ISR & USART_ISR_RXNE_Msk) == 0U)
    {
        /* doi co du lieu den */
    }
    return (uint8_t)usart->RDR;
}

int USART_IsDataAvailable(USART_TypeDef *usart)
{
    return ((usart->ISR & USART_ISR_RXNE_Msk) != 0U) ? 1 : 0;
}
