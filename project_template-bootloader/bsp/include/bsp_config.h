#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"

/** Pin defines for this board */

/** 模拟量输入：母线电压（ADC1 CH4, PA3, 1/25 分压） */
#define AIN_BUS_VOLTAGE_PIN         LL_GPIO_PIN_3
#define AIN_BUS_VOLTAGE_GPIO_PORT   GPIOA

/** 用户 LED */
#define LED_RED_PIN                 LL_GPIO_PIN_0
#define LED_RED_GPIO_PORT           GPIOE

#endif /* BSP_CONFIG_H */
