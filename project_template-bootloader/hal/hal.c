#include "hal.h"

struct hal g_hal;

int hal_init(void)
{
    hal_gpio_init();
    hal_adc1_init();
    hal_tim6_init();
    bsp_board_nvic_init();

    return 0;
}
