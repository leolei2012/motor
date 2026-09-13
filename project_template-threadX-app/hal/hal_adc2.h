#ifndef HAL_ADC2_H
#define HAL_ADC2_H

#include "platform.h"

void hal_adc2_init(void);

/** ——— 规则组 ——— */
uint16_t hal_adc2_reg_read_channel(uint8_t channel);

/** ——— 注入组 ——— */
void     hal_adc2_inj_set_jsqr(uint32_t jsqr);
uint16_t hal_adc2_inj_read_jdr1(void);
uint16_t hal_adc2_inj_read_jdr2(void);
bool     hal_adc2_inj_is_jsqr_busy(void);

#endif /* HAL_ADC2_H */