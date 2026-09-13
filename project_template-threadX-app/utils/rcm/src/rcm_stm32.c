/*
 *  RCM 的 STM32G4 LL 库适配层
 *  ---------------------------
 *  移植参考：实现 rcm_adc_ops_t 的 5 个函数，对接 STM32 LL 库。
 *  换其他 MCU 时只需改写这个文件。
 */

#include "rcm.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"

/** 平台相关上下文：告诉 ops 用哪个 ADC */
typedef struct
{
    ADC_TypeDef *adc;
} rcm_stm32_ctx_t;

/** ---------- 5 个硬件操作 ---------- */

static void stm32_start_conversion(uint8_t channel, void *user_data)
{
    rcm_stm32_ctx_t *ctx = (rcm_stm32_ctx_t *)user_data;

    LL_ADC_REG_SetSequencerRanks(ctx->adc,
                                 LL_ADC_REG_RANK_1,
                                 __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel));
    LL_ADC_REG_ReadConversionData12L(ctx->adc);
    LL_ADC_ClearFlag_EOC(ctx->adc);
    LL_ADC_REG_StartConversion(ctx->adc);
}

static bool stm32_is_conversion_done(void *user_data)
{
    rcm_stm32_ctx_t *ctx = (rcm_stm32_ctx_t *)user_data;
    return (LL_ADC_IsActiveFlag_EOC(ctx->adc) != 0U);
}

static uint16_t stm32_read_result(void *user_data)
{
    rcm_stm32_ctx_t *ctx = (rcm_stm32_ctx_t *)user_data;
    return LL_ADC_REG_ReadConversionData12L(ctx->adc);
}

static void stm32_init(void *user_data)
{
    rcm_stm32_ctx_t *ctx = (rcm_stm32_ctx_t *)user_data;

    if (LL_ADC_IsEnabled(ctx->adc) == 0U)
    {
        LL_ADC_DisableIT_EOC(ctx->adc);
        LL_ADC_ClearFlag_EOC(ctx->adc);
        LL_ADC_DisableIT_JEOC(ctx->adc);
        LL_ADC_ClearFlag_JEOC(ctx->adc);

        LL_ADC_StartCalibration(ctx->adc, LL_ADC_SINGLE_ENDED);
        while (LL_ADC_IsCalibrationOnGoing(ctx->adc) != 0U) {}

        while (LL_ADC_IsActiveFlag_ADRDY(ctx->adc) == 0U)
        {
            LL_ADC_Enable(ctx->adc);
        }
    }

    LL_ADC_REG_SetSequencerLength(ctx->adc, LL_ADC_REG_SEQ_SCAN_DISABLE);
}

static void stm32_set_sample_time(uint8_t channel, uint32_t sample_time, void *user_data)
{
    rcm_stm32_ctx_t *ctx = (rcm_stm32_ctx_t *)user_data;
    LL_ADC_SetChannelSamplingTime(ctx->adc,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(channel),
                                  sample_time);
}

/** ---------- 获取 ops 实例 ---------- */

const rcm_adc_ops_t *rcm_stm32_get_ops(void)
{
    static const rcm_adc_ops_t ops =
    {
        .start_conversion   = stm32_start_conversion,
        .is_conversion_done = stm32_is_conversion_done,
        .read_result        = stm32_read_result,
        .init               = stm32_init,
        .set_sample_time    = stm32_set_sample_time,
    };
    return &ops;
}