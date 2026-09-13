#include "app_task.h"

#include "app.h"
#include "drv.h"

#include "stm32g4xx_hal.h"

/*
 *  app_task：应用层周期线程。
 *
 *  - LED 心跳（app_led_poll，500ms）
 *  - 传感器轮询（drv_ain_sensor_poll，100ms）
 *  - 告警轮询（app_alarm_system_poll，1000ms）
 *  - 电机开环 VF 测试（上电后启动一次，验证 PWM 输出 + 接线）
 *
 *  线程入口只做周期调度，业务逻辑在各自模块内。
 *  以 ThreadX 1 个 tick（1ms，TX_TIMER_TICKS_PER_SECOND=1000）为节拍，
 *  用绝对时间戳（HAL_GetTick，ms）判周期。
 */

/** 开环 VF 测试：小电压幅值 + 低转速（安全起步，逐步调大） */
#define VF_TEST_VOLTAGE     0.05f     /**< 电压幅值（标幺，0.05 = 5% 母线 24V ≈ 1.2V） */
#define VF_TEST_SPEED_RPM   20.0f     /**< 目标机械转速 rpm */

/** ============================================================
   线程主体
   ============================================================ */
void app_task_entry(ULONG thread_input)
{
    (void)thread_input;

    uint32_t sensor_last = 0u;
    uint32_t alarm_last  = 0u;
    uint8_t  vf_started  = 0u;

    while (1)
    {
        /* 让出 CPU 一个 tick（1ms），作为周期调度节拍 */
        tx_thread_sleep(1u);

        uint32_t now = HAL_GetTick();

        /* —— 电机开环 VF 测试：上电后启动一次（验证 PWM + 接线 + 电流环节拍）—— */
        if (!vf_started && g_drv.motor != NULL)
        {
            /* 先设 VF 开环指令，再启动（启动会开 PWM + 使能驱动器） */
            drv_motor_set_openloop_vf(g_drv.motor, VF_TEST_VOLTAGE, VF_TEST_SPEED_RPM);
            drv_motor_start(g_drv.motor);
            vf_started = 1u;
        }

        /* LED 心跳（app 层，500ms 翻转） */
        app_led_poll(g_app.led, now);

        /* 传感器轮询（drivers 层，100ms） */
        if ((now - sensor_last) >= DRV_AIN_SENSOR_TASK_PERIOD)
        {
            sensor_last = now;
            drv_ain_sensor_poll(g_drv.ain_sensor);
        }

        /* 告警轮询（app 层，1000ms） */
        if ((now - alarm_last) >= APP_ALARM_SYSTEM_TASK_PERIOD)
        {
            alarm_last = now;
            app_alarm_system_poll(g_app.alarm_system);
        }
    }
}

/** ============================================================
   创建线程
   ============================================================ */
void app_task_create(TX_THREAD *thread_ptr, VOID *stack_ptr)
{
    tx_thread_create(thread_ptr,
                     "app_task",
                     app_task_entry,
                     0u,
                     stack_ptr,
                     APP_TASK_STACK_SIZE,
                     APP_TASK_PRIORITY,
                     APP_TASK_PRIORITY,
                     TX_NO_TIME_SLICE,
                     TX_AUTO_START);
}
