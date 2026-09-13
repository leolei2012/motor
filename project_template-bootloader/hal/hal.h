#ifndef HAL_H
#define HAL_H


#include <stdbool.h>
#include <stdint.h>

#include "hal_adc1.h"
#include "bsp_board.h"
#include "hal_gpio.h"
#include "hal_tim6.h"

struct hal
{
    uint8_t dummy;
};

extern struct hal g_hal;

int hal_init(void);

#endif /* HAL_H */
