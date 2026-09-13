#ifndef HAL_TIM6_H
#define HAL_TIM6_H

/*
 *  TIM6 基本定时器 (APB1, 16-bit)
 *
 *  用于传感器 ADC 定时触发，生成固定频率的更新中断。
 *  默认 1 kHz (PSC=169, ARR=999, TIM6_CLK=170MHz → 1MHz/1000)
 *
 *  频率公式: f_update = TIM6_CLK / ((PSC + 1) * (ARR + 1))
 */

#define HAL_TIM6_PRESCALER  169u  /**< 170MHz / 170 = 1MHz */
#define HAL_TIM6_PERIOD     999u  /**< 1MHz / 1000 = 1kHz */

void hal_tim6_init(void);
void hal_tim6_start(void);
void hal_tim6_stop(void);

#endif /* HAL_TIM6_H */