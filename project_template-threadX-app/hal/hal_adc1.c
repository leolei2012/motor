#include "hal_adc1.h"

#include "bsp_config.h"

void hal_adc1_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
    LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};
    uint32_t wait_loop_index;

    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_PLL);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    GPIO_InitStruct.Pin = M1_CURR_AMPL_U_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(M1_CURR_AMPL_U_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_CURR_AMPL_V_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(M1_CURR_AMPL_V_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_BUS_VOLTAGE_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(M1_BUS_VOLTAGE_GPIO_PORT, &GPIO_InitStruct);

    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);

    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

    LL_ADC_SetGainCompensation(ADC1, 0);
    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);

    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

    ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
    ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
    ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
    ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
    LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);
    LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);

    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000U * 2U))) / 10U);
    while (wait_loop_index != 0U)
    {
        wait_loop_index--;
    }

    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0U) { }

    /** ADC Enable (must be done after calibration) */
    /** Errata ADC5-140924: ADEN may not take effect on first write after */
    /** calibration at certain AHB/ADC clock ratios. Retry until ADRDY. */
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0U)
    {
        LL_ADC_Enable(ADC1);
    }

    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_7);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);

    LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_8);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED);

    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_EnableIT_JEOS(ADC1);

    /** Start+Stop 清空 JSQR 队列上下文, 再武装外部触发 (参考 MCSDK R3_2_ADCxInit) */
    LL_ADC_INJ_StartConversion(ADC1);
    LL_ADC_INJ_StopConversion(ADC1);
    while (LL_ADC_INJ_IsStopConversionOngoing(ADC1) != 0U) { }

    /*
     * 开环 VF 调试：无队列模式（QUEUE_DISABLE），每次 TRGO 触发都用
     * 同一个固定 JSQR 序列（U=CH7, V=CH8）重新采样，不依赖 TIM1_UP ISR
     * 维护双上下文队列。三相三电阻的扇区 JSQR 切换留待后续 FOC 闭环移植。
     */
    LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);

    /** 设置 JADSTART, ADC 开始响应 TIM1_TRGO 外部触发 */
    LL_ADC_INJ_StartConversion(ADC1);
}

/** ============================================================
   规则组
   ============================================================ */
void hal_adc1_set_channel_sample_time(uint8_t channel, uint32_t sample_time)
{
    LL_ADC_SetChannelSamplingTime(ADC1,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel),
                                  sample_time);
}

void hal_adc1_reg_trigger_channel(uint8_t channel)
{
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1,
                                 __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel));
    LL_ADC_ClearFlag_EOC(ADC1);
    LL_ADC_ClearFlag_EOS(ADC1);
    LL_ADC_ClearFlag_OVR(ADC1);
    LL_ADC_REG_StartConversion(ADC1);
}

uint16_t hal_adc1_reg_read_result(void)
{
    uint16_t raw = LL_ADC_REG_ReadConversionData12(ADC1);
    LL_ADC_ClearFlag_EOC(ADC1);
    LL_ADC_ClearFlag_OVR(ADC1);
    return raw;
}

/** ============================================================
   注入组
   ============================================================ */
void hal_adc1_inj_set_jsqr(uint32_t jsqr)
{
    ADC1->JSQR = jsqr;
}

uint16_t hal_adc1_inj_read_jdr1(void)
{
    return ADC1->JDR1;
}

uint16_t hal_adc1_inj_read_jdr2(void)
{
    return ADC1->JDR2;
}

bool hal_adc1_inj_is_jsqr_busy(void)
{
    /** JSQR != 0 表示队列中还有未被消费的上下文 (参考 MCSDK R3_2_TIMx_UP_IRQHandler) */
    return (ADC1->JSQR != 0u);
}