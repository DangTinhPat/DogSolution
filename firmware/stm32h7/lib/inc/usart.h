/**
 ******************************************************************************
 * @file    usart.h
 * @brief   Driver USART kiểu polling (không ngắt/DMA) - đủ dùng cho debug
 *          console qua UART1 (mặc định PA9=TX, PA10=RX trên board
 *          FK743M5-XIH6, xem BOARD_FK743M5-XIH6.md).
 *
 * Trước khi gọi USART_Init():
 *   1. Bật clock GPIO + USART tương ứng (rcc.h).
 *   2. Cấu hình chân TX/RX sang chế độ Alternate Function đúng số AF
 *      (USART1 trên PA9/PA10 dùng AF7) bằng gpio.h.
 ******************************************************************************
 */

#ifndef USART_H
#define USART_H

#include <stdint.h>
#include "stm32h743_regs.h"

/* baudrate tinh theo SystemPCLK2Clock (USART1) - xem rcc.h.
 * Neu dung USART2/3 (APB1) can peripheral clock APB1 rieng, hien tai
 * driver nay chi ho tro USART1. */
void USART_Init(USART_TypeDef *usart, uint32_t baudrate);

void    USART_SendByte(USART_TypeDef *usart, uint8_t byte);
void    USART_SendString(USART_TypeDef *usart, const char *str);
uint8_t USART_ReceiveByte(USART_TypeDef *usart);   /* blocking - cho den khi co du lieu */
int     USART_IsDataAvailable(USART_TypeDef *usart);

#endif /* USART_H */
