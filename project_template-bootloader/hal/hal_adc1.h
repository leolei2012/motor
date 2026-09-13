#ifndef HAL_ADC1_H
#define HAL_ADC1_H

#include "platform.h"

void hal_adc1_init(void);

/** 设置通道采样时间 (init 阶段调用) */
void hal_adc1_set_channel_sample_time(uint8_t channel, uint32_t sample_time);

/*
 * 非阻塞两拍采样:
 *   1. hal_adc1_reg_trigger_channel()  切换 sequencer + 触发一次规则组转换
 *   2. hal_adc1_reg_read_result()      读取上一次触发的转换结果 (不等待)
 * 转换在两次调用之间完成, 完全不阻塞。
 */
void     hal_adc1_reg_trigger_channel(uint8_t channel);
uint16_t hal_adc1_reg_read_result(void);

#endif /* HAL_ADC1_H */
