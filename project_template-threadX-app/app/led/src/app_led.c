#include "app_led.h"

/*
 *  app_led：通过注入的 drv_output 控制板载 LED。
 *
 *  业务规则：每 APP_LED_BLINK_PERIOD_MS 翻转一次，作为系统心跳指示。
 *  不直接调用 hal_gpio，全部经由 drivers 层（构造注入，见 architecture.md §4）。
 */

/** ============================================================
   初始化
   ============================================================ */
void app_led_init(struct app_led *self, struct drv_output *output)
{
    if (self == NULL)
    {
        return;
    }

    self->output    = output;
    self->last_tick = 0u;
    self->state     = 0u;

    /* 初始熄灭 */
    drv_output_set(self->output, DRV_OUTPUT_OUT0, DRV_OUTPUT_OFF);
}

/** ============================================================
   周期翻转
   ============================================================ */
void app_led_poll(struct app_led *self, uint32_t now)
{
    if (self == NULL)
    {
        return;
    }

    if ((now - self->last_tick) >= APP_LED_BLINK_PERIOD_MS)
    {
        self->last_tick = now;
        self->state     = (uint8_t)(self->state ^ 1u);
        drv_output_toggle(self->output, DRV_OUTPUT_OUT0);
    }
}
