#ifndef DEBUG_MONITOR_H
#define DEBUG_MONITOR_H

#include <stdint.h>

#include "modbus/modbus_slave.h"
#include "drv_uart.h"

/**
 * @file    debug_monitor.h
 * @brief   Modbus RTU 调试从站（传输 + 从站 + adapter 钩子）。
 */

/** 适配器钩子：init / poll 扩展调试能力 */
typedef struct
{
    void (*init)(struct mb_slave_handle *slave); /**< mb_slave_init 之后调用（注册 IAP 回调等） */
    void (*poll)(void);                          /**< 周期调用 */
} dm_adapter;

/** debug_monitor 调试从站对象（单实例） */
struct debug_monitor
{
    struct mb_slave_handle slave; /**< 内嵌 Modbus 从站 */
    struct drv_uart *uart;        /**< 串口设备对象（注入） */
};

void debug_monitor_init(struct debug_monitor *self, uint8_t slave_addr, struct drv_uart *uart);
void debug_monitor_handle_frame(struct debug_monitor *self, const uint8_t *frame, uint8_t len);
void debug_monitor_poll(struct debug_monitor *self);

/** @brief 返回 1 表示串口 TX 管线完全空闲 */
uint8_t debug_monitor_tx_done(struct debug_monitor *self);

#endif /* DEBUG_MONITOR_H */
