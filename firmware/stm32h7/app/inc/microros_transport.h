#ifndef MICROROS_TRANSPORT_H
#define MICROROS_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Custom micro-ROS transport over UART1 (PA9=TX/PA10=RX) - port nguyen ven tu
 * OUT_SAVE/testSTM/app/src/microros_transport.c (cung board FK743M5-XIH6, cung lib/usart.c),
 * KHONG doi RTOS-independent logic gi (file goc khong thuc su goi ham FreeRTOS nao,
 * chi include du thua). open/close/write/read la 4 callback rmw_uros_set_custom_
 * transport() (rmw_microros/custom_transport.h) can.
 *
 * USART1 phai da duoc khoi tao (usart1_init() trong main.c) truoc khi goi ham nay -
 * transport khong dung clock/GPIO/USART_Init, chi TX/RX byte. */
void MicroRos_RegisterTransport(void);

/* TAM THOI, chi cho BRINGUP_UART_RX_ISR_ECHO_ONLY (main.c): mo cung 1 duong RXNEIE+
 * NVIC va rut cung 1 rx_ring[] ma tang XRCE that su dung (Transport_Open()/
 * Transport_Read() noi bo, khong doi gi), nhung KHONG dung toi rmw_uros_set_custom_
 * transport()/session XRCE nao ca - de co lap dung lop ISR+ring buffer, tach khoi
 * giao thuc XRCE ben tren. */
void MicroRosTransport_DebugRxIsrOpen(void);
size_t MicroRosTransport_DebugRxIsrRead(uint8_t *buffer, size_t maxlen);

#endif /* MICROROS_TRANSPORT_H */
