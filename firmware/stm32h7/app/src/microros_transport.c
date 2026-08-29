#include "microros_transport.h"

#include "stm32h743_regs.h"
#include "usart.h"
#include "rmw_microros/rmw_microros.h"

#define MICROROS_UART USART1

/* Ring buffer nap boi USART1_IRQHandler() (ngat RXNE, 1 byte/lan) - Transport_Read()
 * chi rut ra, KHONG cho. Port nguyen ven tu OUT_SAVE/testSTM/app/src/microros_transport.c (da
 * xac nhan on dinh tren board that) theo dung sample chinh thuc cua micro-ROS cho STM32
 * (micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c) - ban
 * blocking-cho-timeout_ms ban dau la nguyen nhan gay ket noi khong on dinh (xem
 * OUT_SAVE/testSTM/README.md "Trang thai da kiem thu"): read() blocking het nguyen timeout_ms
 * trong 1 lan goi co the chan mat co hoi cho tang XRCE ben tren xen ke doc/ghi kip luc.
 *
 * head chi do task (Transport_Read) ghi, tail chi do ISR ghi -> khong can khoa/mutex,
 * an toan kieu single-producer-single-consumer lock-free (khong FreeRTOS o day, khac
 * ban goc OUT_SAVE/testSTM - nhung ISR + ring buffer khong phu thuoc RTOS ngay tu dau). */
#define RX_RING_SIZE 2048U
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile size_t rx_head = 0U;
static volatile size_t rx_tail = 0U;

void USART1_IRQHandler(void)
{
    if ((MICROROS_UART->ISR & USART_ISR_RXNE_Msk) != 0U)
    {
        const uint8_t byte = (uint8_t)MICROROS_UART->RDR;   /* doc RDR tu xoa co RXNE */
        const size_t next_tail = (rx_tail + 1U) % RX_RING_SIZE;
        if (next_tail != rx_head)   /* con cho - bo byte neu day thay vi ghi de */
        {
            rx_ring[rx_tail] = byte;
            rx_tail = next_tail;
        }
    }
}

static bool Transport_Open(struct uxrCustomTransport *transport)
{
    (void)transport;
    NVIC_IPR[USART1_IRQn] = (uint8_t)(5U << 4);
    NVIC_ISER1 = (1UL << (USART1_IRQn - 32));   /* IRQ 37 nam trong ISER1 (32..63) */
    MICROROS_UART->CR1 |= USART_CR1_RXNEIE_Msk;
    return true;
}

static bool Transport_Close(struct uxrCustomTransport *transport)
{
    (void)transport;
    MICROROS_UART->CR1 &= ~USART_CR1_RXNEIE_Msk;
    return true;
}

static size_t Transport_Write(struct uxrCustomTransport *transport, const uint8_t *buffer,
                               size_t length, uint8_t *error_code)
{
    (void)transport;
    (void)error_code;
    /* Keep TX polling/blocking: the previous asynchronous TX ring returned
     * before bytes reached the wire and repeatedly destabilized XRCE session
     * establishment. RX remains interrupt-driven and non-blocking. */
    for (size_t i = 0U; i < length; i++)
    {
        USART_SendByte(MICROROS_UART, buffer[i]);
    }
    return length;
}

static size_t Transport_Read(struct uxrCustomTransport *transport, uint8_t *buffer,
                              size_t length, int timeout_ms, uint8_t *error_code)
{
    (void)transport;
    (void)error_code;
    (void)timeout_ms;   /* khong doi - tra ngay nhung gi dang co trong ring buffer, dung
                          * y het contract cua it_transport.c's cubemx_transport_read():
                          * tang goi (uxr_run_session_until_...) tu quan ly vong lap cho/
                          * het gio bang dong ho rieng, khong dua vao read() blocking ho. */
    size_t received = 0U;
    while (received < length && rx_head != rx_tail)
    {
        buffer[received] = rx_ring[rx_head];
        rx_head = (rx_head + 1U) % RX_RING_SIZE;
        received++;
    }
    return received;
}

void MicroRosTransport_DebugRxIsrOpen(void)
{
    (void)Transport_Open(NULL);
}

size_t MicroRosTransport_DebugRxIsrRead(uint8_t *buffer, size_t maxlen)
{
    return Transport_Read(NULL, buffer, maxlen, 0, NULL);
}

void MicroRos_RegisterTransport(void)
{
    (void)rmw_uros_set_custom_transport(
        MICROROS_TRANSPORTS_FRAMING_MODE,
        NULL,
        Transport_Open,
        Transport_Close,
        Transport_Write,
        Transport_Read);
}
