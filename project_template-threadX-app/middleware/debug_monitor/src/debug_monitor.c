#include "debug_monitor.h"

#include <string.h>

#include "dm_adapter.h"

static dm_adapter s_adapter;

/** 发送回调：ctx 为 debug_monitor 实例 */
static uint8_t dm_uart_send(const uint8_t *buf, uint8_t len, void *ctx)
{
    struct debug_monitor *self = (struct debug_monitor *)ctx;

    return drv_uart_send(self->uart, buf, len);
}

/** RX 帧回调（drv_uart_dispatch_rx 派发） */
static void dm_on_rx_frame(void *arg, const uint8_t *data, uint16_t len)
{
    struct debug_monitor *self = (struct debug_monitor *)arg;

    if ((data == NULL) || (len == 0u) || (self == NULL))
    {
        return;
    }

    dm_adapter_refresh();
    debug_monitor_handle_frame(self, data, (uint8_t)len);
}

void debug_monitor_init(struct debug_monitor *self, uint8_t slave_addr, struct drv_uart *uart)
{
    const dm_adapter *adapter;

    if ((self == NULL) || (uart == NULL))
    {
        return;
    }

    memset(self, 0, sizeof(*self));
    self->uart = uart;

    adapter = dm_adapter_get();
    if (adapter != NULL)
    {
        s_adapter = *adapter;
    }
    else
    {
        memset(&s_adapter, 0, sizeof(s_adapter));
    }

    mb_slave_init(&self->slave, slave_addr, dm_adapter_reg_map());
    self->slave.base.send = dm_uart_send;
    self->slave.base.ctx  = self;

    if (s_adapter.init != NULL)
    {
        s_adapter.init(&self->slave);
    }

    drv_uart_bind_rx(uart, dm_on_rx_frame, self);
}

void debug_monitor_handle_frame(struct debug_monitor *self, const uint8_t *frame, uint8_t len)
{
    if ((self == NULL) || (frame == NULL) || (len == 0u))
    {
        return;
    }

    mb_slave_rx_frame(&self->slave, frame, len);
}

void debug_monitor_poll(struct debug_monitor *self)
{
    (void)self;

    if (s_adapter.poll != NULL)
    {
        s_adapter.poll();
    }
}

uint8_t debug_monitor_tx_done(struct debug_monitor *self)
{
    if (self == NULL)
    {
        return 0u;
    }

    return drv_uart_tx_done(self->uart);
}
