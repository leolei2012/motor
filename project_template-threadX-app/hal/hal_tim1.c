#include "hal_tim1.h"

#include "bsp_config.h"

void hal_tim1_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    TIM_InitStruct.Prescaler = ((TIM_CLOCK_DIVIDER)-1);
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP;
    TIM_InitStruct.Autoreload = ((PWM_PERIOD_CYCLES) / 2);
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2;
    TIM_InitStruct.RepetitionCounter = (REP_COUNTER);
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = ((PWM_PERIOD_CYCLES) / 4);
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
    TIM_OC_InitStruct.CompareValue = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_ENABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_ENABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = ((DEAD_TIME_COUNTS) / 2);
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
    TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
    TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
    TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
    TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1_N8;
    TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

    GPIO_InitStruct.Pin = M1_PWM_UL_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(M1_PWM_UL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_PWM_VL_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(M1_PWM_VL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_PWM_WL_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    LL_GPIO_Init(M1_PWM_WL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_PWM_UH_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(M1_PWM_UH_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_PWM_VH_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(M1_PWM_VH_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_PWM_WH_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(M1_PWM_WH_GPIO_PORT, &GPIO_InitStruct);

    LL_TIM_SetCounter(TIM1, (uint32_t)PWM_HALF_PERIOD - 1U);
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_ClearFlag_UPDATE(TIM1);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);

    LL_TIM_EnableIT_UPDATE(TIM1);
}

/** ============================================================
   计数器启停
   ============================================================ */
void hal_tim1_start(void)
{
    LL_TIM_EnableCounter(TIM1);
}

void hal_tim1_stop(void)
{
    LL_TIM_DisableCounter(TIM1);
}

/** ============================================================
   PWM 输出总开关 (MOE)
   ============================================================ */
void hal_tim1_pwm_enable(void)
{
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
    LL_TIM_EnableAllOutputs(TIM1);
}

void hal_tim1_pwm_disable(void)
{
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
}

/** ============================================================
   占空比设置 (0 ~ PWM_HALF_PERIOD)
   ============================================================ */
void hal_tim1_set_duty_a(uint16_t cmp)
{
    LL_TIM_OC_SetCompareCH1(TIM1, cmp);
}

void hal_tim1_set_duty_b(uint16_t cmp)
{
    LL_TIM_OC_SetCompareCH2(TIM1, cmp);
}

void hal_tim1_set_duty_c(uint16_t cmp)
{
    LL_TIM_OC_SetCompareCH3(TIM1, cmp);
}

void hal_tim1_set_duty_abc(uint16_t cmp_a, uint16_t cmp_b, uint16_t cmp_c)
{
    LL_TIM_OC_SetCompareCH1(TIM1, cmp_a);
    LL_TIM_OC_SetCompareCH2(TIM1, cmp_b);
    LL_TIM_OC_SetCompareCH3(TIM1, cmp_c);
}

/** ============================================================
   ADC 触发控制
   ============================================================ */
void hal_tim1_enable_adc_trigger(void)
{
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
}

void hal_tim1_disable_adc_trigger(void)
{
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
}

/** ============================================================
   三相下管导通 (自举电容充电, 参考 ST R3_2_TurnOnLowSides)
   ============================================================ */
void hal_tim1_turn_on_low_sides(uint32_t ccr)
{
    /** 绕过预装载 → CCR 立即生效 */
    LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    LL_TIM_OC_SetCompareCH1(TIM1, ccr);
    LL_TIM_OC_SetCompareCH2(TIM1, ccr);
    LL_TIM_OC_SetCompareCH3(TIM1, ccr);

    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);

    LL_TIM_EnableAllOutputs(TIM1);
}