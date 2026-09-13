#include "hal_tim6.h"

#include "bsp_config.h"

void hal_tim6_init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    TIM_InitStruct.Prescaler         = HAL_TIM6_PRESCALER;
    TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload        = HAL_TIM6_PERIOD;
    TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0u;
    LL_TIM_Init(TIM6, &TIM_InitStruct);

    LL_TIM_DisableARRPreload(TIM6);
    LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);

    LL_TIM_EnableIT_UPDATE(TIM6);
}

void hal_tim6_start(void)
{
    LL_TIM_SetCounter(TIM6, 0u);
    LL_TIM_ClearFlag_UPDATE(TIM6);
    LL_TIM_EnableCounter(TIM6);
}

void hal_tim6_stop(void)
{
    LL_TIM_DisableCounter(TIM6);
    LL_TIM_ClearFlag_UPDATE(TIM6);
}
