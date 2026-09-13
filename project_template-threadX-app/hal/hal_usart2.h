#ifndef HAL_USART2_H
#define HAL_USART2_H

#include "platform.h"

#include "uart_control.h"

/**
 * @file hal_usart2.h
 * @brief USART2 外设薄封装：提供 uart_control 的寄存器级 ops + ISR。
 *
 * 发送走 DMA（DMA1 CH2）；接收走逐字节 RXNE 中断；
 * 空闲帧结束由 TIM7 中断驱动 uart_control_timer_isr（见 bsp_isr.c）。
 * 引脚见 bsp_config.h（PD5 = TX，PD6 = RX）。
 */

void hal_usart2_init(void);

/** ——— DMA 发送 ——— */
void hal_usart2_dma_tx_start(const uint8_t *buf, uint16_t len);
void hal_usart2_dma_tx_isr(void);  /**< DMA1_CH2 中断入口 */

/** ——— 中断接收（逐字节） ——— */
void    hal_usart2_enable_it_rxne(void);
void    hal_usart2_disable_it_rxne(void);
uint8_t hal_usart2_read_byte(void);

/** ——— uart_control 硬件 ops ——— */

/** 返回 USART2 的 uart_control 硬件 ops */
const struct uart_control_hal_ops *hal_usart2_get_ops(void);

/** 注册使用 USART2 的 uart_control 实例（ISR 分发用） */
void hal_usart2_set_instance(struct uart_control *uart);

/** USART2 中断入口（bsp_isr.c 的 USART2_IRQHandler 调用） */
void hal_usart2_irq_handler(void);

#endif /* HAL_USART2_H */
