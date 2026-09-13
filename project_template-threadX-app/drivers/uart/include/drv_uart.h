#ifndef DRV_UART_H
#define DRV_UART_H

#include "uart_control.h"

/**
 * @file drv_uart.h
 * @brief 串口设备驱动：每个物理串口一个 struct drv_uart 对象。
 *
 * 包装 uart_control（组帧）+ 消费者 RX 回调 + 帧完成标志。
 * ISR 置 rx_ready，任务侧调用 drv_uart_dispatch_rx 派发帧。
 */

/** 串口设备对象（一个物理串口一个实例） */
struct drv_uart
{
    struct uart_control uart;                                     /**< uart_control 实例（首成员） */
    void (*on_rx)(void *arg, const uint8_t *data, uint16_t len);  /**< 消费者帧回调 */
    void *on_rx_arg;                                              /**< 消费者帧回调参数 */
    volatile bool rx_ready;                                       /**< 帧完成标志：ISR 置位、任务清 */
};

/**
 * @brief 初始化串口设备（USART2）
 * @param timeout_tick 空闲帧判结束的 tick 数（TIM7 1ms 节拍，典型 3~10）
 */
void drv_uart_init(struct drv_uart *self, uint8_t *rx_buf, uint16_t rx_size, uint16_t timeout_tick);

/** 发送一帧（DMA），成功返回实际发送字节数，失败返回 0
 *  @note buf 必须位于 SRAM（DMA 可访问）；不能指向 const/FLASH 数据 */
uint8_t drv_uart_send(struct drv_uart *self, const uint8_t *buf, uint8_t len);

/** @brief 返回 1 表示 TX 管线完全空闲 */
uint8_t drv_uart_tx_done(const struct drv_uart *self);

/** 绑定消费者 RX 帧回调 */
void drv_uart_bind_rx(struct drv_uart *self,
                      void (*cb)(void *arg, const uint8_t *data, uint16_t len), void *arg);

/** 任务侧调用：有完整帧时派发给消费者回调，并重启接收 */
void drv_uart_dispatch_rx(struct drv_uart *self);

#endif /* DRV_UART_H */
