#include "hal_adc2.h"

#include "bsp_config.h"

void hal_adc2_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};
    uint32_t wait_loop_index;

    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_PLL);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    GPIO_InitStruct.Pin = M1_CURR_AMPL_V_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(M1_CURR_AMPL_V_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = M1_CURR_AMPL_W_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(M1_CURR_AMPL_W_GPIO_PORT, &GPIO_InitStruct);

    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_LEFT;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
    LL_ADC_Init(ADC2, &ADC_InitStruct);

    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
    ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
    LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

    LL_ADC_SetGainCompensation(ADC2, 0);
    LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);

    ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
    ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
    ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
    ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
    LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);
    LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
    LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

    LL_ADC_DisableDeepPowerDown(ADC2);
    LL_ADC_EnableInternalRegulator(ADC2);

    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000U * 2U))) / 10U);
    while (wait_loop_index != 0U)
    {
        wait_loop_index--;
    }

    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0U) { }

    while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0U)
    {
        LL_ADC_Enable(ADC2);
    }

    /** ADC2 JEOS 不使能中断 — 只用 ADC1 JEOS 进 ISR (与 ST MCSDK 一致) */
    LL_ADC_ClearFlag_JEOS(ADC2);

    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_8);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

    LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_9);
    LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_6CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);

    /** Start+Stop 清空 JSQR 队列上下文, 再武装外部触发 (参考 MCSDK R3_2_ADCxInit) */
    LL_ADC_INJ_StartConversion(ADC2);
    LL_ADC_INJ_StopConversion(ADC2);
    while (LL_ADC_INJ_IsStopConversionOngoing(ADC2) != 0U) { }

    /*
     * 开环 VF 调试：无队列模式（QUEUE_DISABLE），每次 TRGO 触发都用
     * 同一个固定 JSQR 序列（V=CH8, W=CH9）重新采样，不依赖 TIM1_UP ISR。
     */
    LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);

    /** 设置 JADSTART, ADC 开始响应 TIM1_TRGO 外部触发 */
    LL_ADC_INJ_StartConversion(ADC2);
}

/** ============================================================
   规则组
   ============================================================ */
uint16_t hal_adc2_reg_read_channel(uint8_t channel)
{
    LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1,
                                 __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel));
    LL_ADC_REG_ReadConversionData12(ADC2);
    LL_ADC_ClearFlag_EOC(ADC2);
    LL_ADC_REG_StartConversion(ADC2);

    /* 防跑飞: 规则组和注入组抢 ADC 时 EOC 可能偶发不置位, 加超时 */
    uint32_t eoc_timeout = 100000u;
    while (LL_ADC_IsActiveFlag_EOC(ADC2) == 0U && eoc_timeout > 0u)
    {
        eoc_timeout--;
    }

    uint16_t raw = LL_ADC_REG_ReadConversionData12(ADC2);
    LL_ADC_ClearFlag_EOC(ADC2);
    return raw;
}

/** ============================================================
   注入组
   ============================================================ */
void hal_adc2_inj_set_jsqr(uint32_t jsqr)
{
    ADC2->JSQR = jsqr;
}

uint16_t hal_adc2_inj_read_jdr1(void)
{
    return ADC2->JDR1;
}

uint16_t hal_adc2_inj_read_jdr2(void)
{
    return ADC2->JDR2;
}

bool hal_adc2_inj_is_jsqr_busy(void)
{
    /** JSQR != 0 表示队列中还有未被消费的上下文 (参考 MCSDK R3_2_TIMx_UP_IRQHandler) */
    return (ADC2->JSQR != 0u);
}