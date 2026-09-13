#include "hal.h"
#include "hal_gpio.h"
#include "bsp_config.h"

#include "drv.h"
#include "app.h"

const uint32_t g_version = 0x010000u;  /**< 主版本号(高字节) + 次版本号(低字节) + 修订号(第三字节) */

#define LED_BLINK_PERIOD_MS 500u

int main(void)
{
    uint32_t led_last = 0u;
    uint32_t sensor_last = 0u;
    uint32_t alarm_last = 0u;

    /** 裸机启动：按依赖顺序手动初始化各层 */
    bsp_board_early_init();
    hal_init();
    drv_init();
    app_init();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /** LED 心跳 */
        if (now - led_last >= LED_BLINK_PERIOD_MS)
        {
            led_last = now;
            hal_gpio_toggle(HAL_GPIO_LED_RED);
        }

        /** 模拟量轮询（物理量转换） */
        if (now - sensor_last >= DRV_AIN_SENSOR_TASK_PERIOD)
        {
            sensor_last = now;
            drv_ain_sensor_poll(g_drv.ain_sensor);
        }

        /** 告警系统轮询 */
        if (now - alarm_last >= APP_ALARM_SYSTEM_TASK_PERIOD)
        {
            alarm_last = now;
            app_alarm_system_poll(g_app.alarm_system);
        }
    }
}
