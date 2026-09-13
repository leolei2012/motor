#ifndef HAL_TIM1_H
#define HAL_TIM1_H

#include "platform.h"

/** STM32 MC Workbench generated TIM1 init constants, kept independent from MCSDK. */
#define TIM_CLOCK_DIVIDER           1U
#define PWM_FREQUENCY               16000U
#define REGULATION_EXECUTION_RATE   1U
#define ADV_TIM_CLK_MHz             170U
#define SW_DEADTIME_NS              200U
#define REP_COUNTER                 ((uint16_t)((REGULATION_EXECUTION_RATE * 2U) - 1U))
#define PWM_PERIOD_CYCLES           ((uint16_t)(((uint32_t)ADV_TIM_CLK_MHz * 1000000UL / PWM_FREQUENCY) & 0xFFFEU))
#define DEAD_TIME_ADV_TIM_CLK_MHz   (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)

#define DEAD_TIME_COUNTS_1          (DEAD_TIME_ADV_TIM_CLK_MHz * SW_DEADTIME_NS / 1000UL)

#if (DEAD_TIME_COUNTS_1 <= 255U)
#define DEAD_TIME_COUNTS    ((uint16_t)DEAD_TIME_COUNTS_1)
#elif (DEAD_TIME_COUNTS_1 <= 508U)
#define DEAD_TIME_COUNTS    ((uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * SW_DEADTIME_NS / 2U) / 1000UL) + 128U))
#elif (DEAD_TIME_COUNTS_1 <= 1008U)
#define DEAD_TIME_COUNTS    ((uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * SW_DEADTIME_NS / 8U) / 1000UL) + 320U))
#elif (DEAD_TIME_COUNTS_1 <= 2015U)
#define DEAD_TIME_COUNTS    ((uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * SW_DEADTIME_NS / 16U) / 1000UL) + 384U))
#else
#define DEAD_TIME_COUNTS    510U
#endif

#define HTMIN               1U
#define PWM_HALF_PERIOD     ((PWM_PERIOD_CYCLES) / 2)  /**< 5312, 中心对齐半周期 */

void hal_tim1_init(void);

/*
 *  hal_tim1 API
 *  ——————————————————————————————————————————————
 *  hal_tim1_start()            启动计数器
 *  hal_tim1_stop()             停止计数器
 *  hal_tim1_pwm_enable()       使能 PWM 输出 (MOE)
 *  hal_tim1_pwm_disable()      紧急关断，全部高阻
 *  hal_tim1_set_duty_a(cmp)    A 相占空比 (0 ~ PWM_HALF_PERIOD)
 *  hal_tim1_set_duty_b(cmp)    B 相占空比
 *  hal_tim1_set_duty_c(cmp)    C 相占空比
 *  hal_tim1_set_duty_abc(...)  三相同时设占空比
 *  hal_tim1_enable_adc_trigger()   TRGO = OC4REF
 *  hal_tim1_disable_adc_trigger()  TRGO = RESET
 *
 *  启动顺序: start() → pwm_enable()
 *  关断顺序: pwm_disable() → stop()
 */
void hal_tim1_start(void);
void hal_tim1_stop(void);

void hal_tim1_pwm_enable(void);
void hal_tim1_pwm_disable(void);

void hal_tim1_set_duty_a(uint16_t cmp);
void hal_tim1_set_duty_b(uint16_t cmp);
void hal_tim1_set_duty_c(uint16_t cmp);
void hal_tim1_set_duty_abc(uint16_t cmp_a, uint16_t cmp_b, uint16_t cmp_c);

void hal_tim1_enable_adc_trigger(void);
void hal_tim1_disable_adc_trigger(void);

/*
 * 三相下管导通 (自举电容充电)
 * 绕过预装载, CCR 立即生效 — 低边长通, 高边短通
 */
void hal_tim1_turn_on_low_sides(uint32_t ccr);

#endif /* HAL_TIM1_H */