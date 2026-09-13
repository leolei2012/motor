#include "hal_adc1.h"

#include "bsp_config.h"

void hal_adc1_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
    uint32_t wait_loop_index;

    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_PLL);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    /** 模拟输入引脚：母线电压（PA3, ADC1 CH4） */
    GPIO_InitStruct.Pin = AIN_BUS_VOLTAGE_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(AIN_BUS_VOLTAGE_GPIO_PORT, &GPIO_InitStruct);

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

    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);

    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000U * 2U))) / 10U);
    while (wait_loop_index != 0U)
    {
        wait_loop_index--;
    }

    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0U) { }

    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0U)
    {
        LL_ADC_Enable(ADC1);
    }

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED);
}

/** 设置通道采样时间 (init 阶段调用) */
void hal_adc1_set_channel_sample_time(uint8_t channel, uint32_t sample_time)
{
    LL_ADC_SetChannelSamplingTime(ADC1,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel),
                                  sample_time);
}

/*
 * 非阻塞两拍采样:
 *   1. hal_adc1_reg_trigger_channel()  切换 sequencer + 触发一次规则组转换
 *   2. hal_adc1_reg_read_result()      读取上一次触发的转换结果 (不等待)
 */
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
