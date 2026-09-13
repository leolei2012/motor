#ifndef APP_LED_H
#define APP_LED_H

#include "platform.h"

#include "drv_output.h"

/*
 *  app_led：LED 心跳指示（应用层业务逻辑）
 *
 *  通过 drv_output 抽象控制板载 LED，不直接接触 GPIO。
 *  依赖经 init 构造注入，不内部 include 对方实现（architecture.md §4）。
 *  层次依赖：app_led → drv_output → hal_gpio → bsp_config
 */

/** LED 心跳周期（ms） */
#define APP_LED_BLINK_PERIOD_MS  500u

struct app_led
{
    struct drv_output *output;   /**< 依赖：输出驱动（注入） */
    uint32_t           last_tick; /**< 上次翻转的系统 tick (ms) */
    uint8_t            state;     /**< 当前 LED 状态 (0=灭, 1=亮) */
};

/** ============================================================
   API
   ============================================================ */

/**
 * @brief 初始化：注入输出驱动 + 熄灭 LED
 * @param self    LED 指示灯对象指针
 * @param output  依赖：输出驱动（由组合根注入）
 */
void app_led_init(struct app_led *self, struct drv_output *output);

/**
 * @brief 周期翻转（由主循环驱动）
 * @param self LED 指示灯对象指针
 * @param now  当前系统 tick（HAL_GetTick()）
 */
void app_led_poll(struct app_led *self, uint32_t now);

#endif /* APP_LED_H */
