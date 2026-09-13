#ifndef HAL_TIM7_H
#define HAL_TIM7_H

#include "platform.h"

/**
 * @brief  TIM7 基本定时器 (APB1, 16-bit)
 *
 * 用于 UART RX 空闲检测，固定 1ms 节拍。
 * PSC=169 → 1MHz (1µs/tick)，ARR=999 → 1ms 周期。
 * 自由运行，UPDATE 中断中由 uart_control 累加软件计数器，
 * 达到阈值后判定帧接收完成。
 *
 * f_update = TIM7_CLK / ((PSC + 1) * (ARR + 1))
 */

#define HAL_TIM7_PRESCALER  169u  /**< 170MHz / 170 = 1MHz */

void hal_tim7_init(void);
void hal_tim7_start(void);
void hal_tim7_stop(void);

#endif /* HAL_TIM7_H */