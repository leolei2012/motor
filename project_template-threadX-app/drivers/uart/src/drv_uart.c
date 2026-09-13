#include "drv_uart.h"

#include <string.h>

#include "hal_usart2.h"
#include "hal_tim7.h"

/** uart_control RX 完成回调（ISR 上下文，只置标志） */
static void uart_on_rx_done(struct uart_control *uart, uint16_t len)
{
    struct drv_uart *self = (struct drv_uart *)uart;

    (void)len;
    self->rx_ready = true;
}

static void uart_on_tx_done(struct uart_control *uart)
{
    (void)uart;
}

void drv_uart_init(struct drv_uart *self, uint8_t *rx_buf, uint16_t rx_size, uint16_t timeout_tick)
{
    if ((self == NULL) || (rx_buf == NULL))
    {
        return;
    }

    memset(self, 0, sizeof(*self));

    hal_usart2_set_instance(&self->uart);

    (void)uart_control_init(&self->uart, hal_usart2_get_ops(), rx_buf, rx_size, timeout_tick,
                            uart_on_rx_done, uart_on_tx_done);
    (void)uart_control_enable_rx(&self->uart);

    /* 启动 TIM7 空闲超时定时器（1ms 节拍，驱动 uart_control_timer_isr） */
    hal_tim7_start();
}

uint8_t drv_uart_send(struct drv_uart *self, const uint8_t *buf, uint8_t len)
{
    if ((self == NULL) || (buf == NULL))
    {
        return 0u;
    }

    if (uart_control_send(&self->uart, buf, len) == UART_CONTROL_OK)
    {
        return len;
    }

    return 0u;
}

uint8_t drv_uart_tx_done(const struct drv_uart *self)
{
    if (self == NULL)
    {
        return 0u;
    }

    return (uint8_t)(self->uart.tx_busy ? 0u : 1u);
}

void drv_uart_bind_rx(struct drv_uart *self,
                      void (*cb)(void *arg, const uint8_t *data, uint16_t len),
                      void *arg)
{
    if (self == NULL)
    {
        return;
    }

    self->on_rx = cb;
    self->on_rx_arg = arg;
}

void drv_uart_dispatch_rx(struct drv_uart *self)
{
    uint16_t len;

    if (self == NULL)
    {
        return;
    }

    if (!self->rx_ready)
    {
        return;
    }

    len = uart_control_rx_len(&self->uart);
    if ((len > 0u) && (self->on_rx != NULL))
    {
        self->on_rx(self->on_rx_arg, self->uart.rx_buf, len);
    }

    self->rx_ready = false;
    (void)uart_control_enable_rx(&self->uart);
}
