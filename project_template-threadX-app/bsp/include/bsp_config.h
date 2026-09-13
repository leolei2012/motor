#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

/** Pin defines for this board */

#define M1_EN_DRIVER_PIN            LL_GPIO_PIN_13
#define M1_EN_DRIVER_GPIO_PORT      GPIOC
#define M1_CURR_AMPL_U_PIN          LL_GPIO_PIN_1
#define M1_CURR_AMPL_U_GPIO_PORT    GPIOC
#define M1_CURR_AMPL_V_PIN          LL_GPIO_PIN_2
#define M1_CURR_AMPL_V_GPIO_PORT    GPIOC
#define M1_CURR_AMPL_W_PIN          LL_GPIO_PIN_3
#define M1_CURR_AMPL_W_GPIO_PORT    GPIOC
#define M1_BUS_VOLTAGE_PIN          LL_GPIO_PIN_3
#define M1_BUS_VOLTAGE_GPIO_PORT    GPIOA

#define M1_PWM_UL_PIN               LL_GPIO_PIN_13
#define M1_PWM_UL_GPIO_PORT         GPIOB
#define M1_PWM_VL_PIN               LL_GPIO_PIN_14
#define M1_PWM_VL_GPIO_PORT         GPIOB
#define M1_PWM_WL_PIN               LL_GPIO_PIN_15
#define M1_PWM_WL_GPIO_PORT         GPIOB
#define M1_PWM_UH_PIN               LL_GPIO_PIN_8
#define M1_PWM_UH_GPIO_PORT         GPIOA
#define M1_PWM_VH_PIN               LL_GPIO_PIN_9
#define M1_PWM_VH_GPIO_PORT         GPIOA
#define M1_PWM_WH_PIN               LL_GPIO_PIN_10
#define M1_PWM_WH_GPIO_PORT         GPIOA

#define UART_TX_PIN                 LL_GPIO_PIN_5
#define UART_TX_GPIO_PORT           GPIOD
#define UART_RX_PIN                 LL_GPIO_PIN_6
#define UART_RX_GPIO_PORT           GPIOD

#define LED_RED_PIN                 LL_GPIO_PIN_0
#define LED_RED_GPIO_PORT           GPIOE

#endif /* BSP_CONFIG_H */
