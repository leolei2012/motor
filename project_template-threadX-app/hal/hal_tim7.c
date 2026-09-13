#include "hal_tim7.h"

#include "bsp_config.h"

void hal_tim7_init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);

    TIM_InitStruct.Prescaler         = HAL_TIM7_PRESCALER;
    TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload        = 999u;  /**< 1ms 节拍 */
    TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0u;
    LL_TIM_Init(TIM7, &TIM_InitStruct);

    LL_TIM_DisableARRPreload(TIM7);
    LL_TIM_SetTriggerOutput(TIM7, LL_TIM_TRGO_RESET);

    LL_TIM_EnableIT_UPDATE(TIM7);
}

void hal_tim7_start(void)
{
    LL_TIM_SetCounter(TIM7, 0u);
    LL_TIM_ClearFlag_UPDATE(TIM7);
    LL_TIM_EnableCounter(TIM7);
}

void hal_tim7_stop(void)
{
    LL_TIM_DisableCounter(TIM7);
    LL_TIM_ClearFlag_UPDATE(TIM7);
}