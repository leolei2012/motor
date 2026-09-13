#ifndef HAL_ADC1_H
#define HAL_ADC1_H

#include "platform.h"

void hal_adc1_init(void);

/** ——— 规则组 (regular group) ——— */

/** 设置通道采样时间 (init 阶段调用) */
void hal_adc1_set_channel_sample_time(uint8_t channel, uint32_t sample_time);

/*
 * 非阻塞两拍采样 (避免与 16kHz 注入组抢 ADC 时阻塞等待 EOC 饿死调度器):
 *   1. hal_adc1_reg_trigger_channel()  切换 sequencer + 触发一次规则组转换
 *   2. hal_adc1_reg_read_result()      读取上一次触发的转换结果 (不等待)
 * 转换在两次调用之间 (1ms) 完成, 完全无阻塞。
 */
void     hal_adc1_reg_trigger_channel(uint8_t channel);
uint16_t hal_adc1_reg_read_result(void);

/** ——— 注入组 (injected group) ——— */

/** 直接写 JSQR (用于 TIM1_UP 中的动态扇区切换) */
void hal_adc1_inj_set_jsqr(uint32_t jsqr);

/** 读 JDR1 (注入结果寄存器 rank1) */
uint16_t hal_adc1_inj_read_jdr1(void);

/** 读 JDR2 (注入结果寄存器 rank2) */
uint16_t hal_adc1_inj_read_jdr2(void);

/** JSQR 是否还在忙 (等待上一次注入完成) */
bool hal_adc1_inj_is_jsqr_busy(void);

#endif /* HAL_ADC1_H */