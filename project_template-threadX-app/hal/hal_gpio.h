#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>

#include "stm32g4xx.h"

enum
{
    HAL_GPIO_OUT0 = 0,   /**< 通用数字量输出 0（板载 LED_RED, PE0） */
    HAL_GPIO_OUT1 = 1,   /**< 通用数字量输出 1（电机驱动器使能 M1_EN_DRIVER, PC13） */
    HAL_GPIO_COUNT
};

typedef enum
{
    HAL_GPIO_LEVEL_LOW = 0,
    HAL_GPIO_LEVEL_HIGH = 1
} hal_gpio_level_t;


void hal_gpio_init(void);
void hal_gpio_write(uint16_t id, hal_gpio_level_t level);
void hal_gpio_toggle(uint16_t id);
hal_gpio_level_t hal_gpio_read(uint16_t id);

#endif /* HAL_GPIO_H */
