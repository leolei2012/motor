#include "hal_gpio.h"

#include <stdbool.h>
#include <stddef.h>

#include "bsp_config.h"


void hal_gpio_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);

    /** M1_EN_DRIVER（电机驱动器使能）：默认拉低（禁用），由业务主动使能 */
    LL_GPIO_ResetOutputPin(M1_EN_DRIVER_GPIO_PORT, M1_EN_DRIVER_PIN);

    GPIO_InitStruct.Pin = M1_EN_DRIVER_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(M1_EN_DRIVER_GPIO_PORT, &GPIO_InitStruct);

    /** LED */
    LL_GPIO_ResetOutputPin(LED_RED_GPIO_PORT, LED_RED_PIN);

    GPIO_InitStruct.Pin = LED_RED_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED_RED_GPIO_PORT, &GPIO_InitStruct);
}



typedef struct
{
    GPIO_TypeDef *port;
    uint32_t pin;
} hal_gpio_desc_t;

static const hal_gpio_desc_t s_gpio_desc[HAL_GPIO_COUNT] =
{
    [HAL_GPIO_OUT0] =
    {
        .port = LED_RED_GPIO_PORT,
        .pin = LED_RED_PIN,
    },
    [HAL_GPIO_OUT1] =
    {
        .port = M1_EN_DRIVER_GPIO_PORT,
        .pin = M1_EN_DRIVER_PIN,
    },
};

static const hal_gpio_desc_t *hal_gpio_desc_get(uint16_t id)
{
    if (id >= HAL_GPIO_COUNT)
    {
        return NULL;
    }

    return &s_gpio_desc[id];
}

void hal_gpio_write(uint16_t id, hal_gpio_level_t level)
{
    const hal_gpio_desc_t *desc = hal_gpio_desc_get(id);

    if (desc == NULL)
    {
        return;
    }

    if (level == HAL_GPIO_LEVEL_HIGH)
    {
        LL_GPIO_SetOutputPin(desc->port, desc->pin);
    }
    else
    {
        LL_GPIO_ResetOutputPin(desc->port, desc->pin);
    }
}

void hal_gpio_toggle(uint16_t id)
{
    const hal_gpio_desc_t *desc = hal_gpio_desc_get(id);

    if (desc == NULL)
    {
        return;
    }

    LL_GPIO_TogglePin(desc->port, desc->pin);
}

hal_gpio_level_t hal_gpio_read(uint16_t id)
{
    const hal_gpio_desc_t *desc = hal_gpio_desc_get(id);

    if (desc == NULL)
    {
        return HAL_GPIO_LEVEL_LOW;
    }

    return (LL_GPIO_IsInputPinSet(desc->port, desc->pin) == 1U)
           ? HAL_GPIO_LEVEL_HIGH
           : HAL_GPIO_LEVEL_LOW;
}




