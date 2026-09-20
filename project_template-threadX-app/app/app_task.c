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
 *  - 电机测试（上电启动一次，TEST_MODE_* 宏二选一）：
 *      IF 开环（电流闭环+相位开环） / 无感速度闭环（flux 观测器 + PLL）
 *
 *  线程入口只做周期调度，业务逻辑在各自模块内。
 *  以 ThreadX 1 个 tick（1ms，TX_TIMER_TICKS_PER_SECOND=1000）为节拍，
 *  用绝对时间戳（HAL_GetTick，ms）判周期。
 */

/**
 *  测试模式选择（二选一）：
 *   - TEST_MODE_IF_OPENLOOP：开环 IF（电流闭环 + 相位开环），稳定成果，已验证；
 *   - TEST_MODE_SENSORLESS：无感速度闭环（flux 观测器 + PLL，自动开环启动
 *     ：锁定 0.2s → 斜坡 → 50rpm 拖动 → seed → 切闭环 → 速度环升到目标转速）。
 */
#define TEST_MODE_SENSORLESS   1
#define TEST_MODE_IF_OPENLOOP  0

/** 开环 IF 测试：电流闭环 + 相位开环（稳定成果，电机平稳正转） */
#define IF_TEST_CURRENT    1.2f     /**< 电流幅值 A（q 轴电流参考） */
#define IF_TEST_SPEED_RPM  100.0f   /**< 目标机械转速 rpm */

/** 无感速度闭环测试：flux 观测器估相位（自动开环启动后切闭环，速度环控速） */
#define CL_TEST_SPEED_RPM  300.0f  /**< 闭环目标机械转速 rpm（=拖动转速 300rpm，反电动势充足，观测器信号强） */

/** ============================================================
   线程主体
   ============================================================ */
void app_task_entry(ULONG thread_input)
{
    (void)thread_input;

    uint32_t sensor_last = 0u;
    uint32_t alarm_last  = 0u;
    uint32_t if_ramp_last = 0u;
    uint8_t  if_started  = 0u;
    float    if_speed    = 0.0f;

    while (1)
    {
        /* 让出 CPU 一个 tick（1ms），作为周期调度节拍 */
        tx_thread_sleep(1u);

        uint32_t now = HAL_GetTick();

        /* —— 电机测试：IF 开环 / 无感速度闭环（二选一，见顶部宏）—— */
        if (!if_started && g_drv.motor != NULL)
        {
#if TEST_MODE_SENSORLESS
            drv_motor_set_speed(g_drv.motor, CL_TEST_SPEED_RPM);
#else
            /* IF 开环：低速 15rpm 起步（场频 1.57Hz，转子可靠牵入），
               每 150ms +5rpm 斜坡升到目标（直接 100rpm 起步场频 16.7Hz，
               转子从静止跟不上 → 只能原地抖动，实测 v∥i 无反电动势确认）。 */
            if_speed = 15.0f;
            drv_motor_set_openloop_if(g_drv.motor, IF_TEST_CURRENT, if_speed);
#endif
            drv_motor_start(g_drv.motor);
            if_started = 1u;
            if_ramp_last = now;
        }

        /* IF 开环速度斜坡（15 → IF_TEST_SPEED_RPM） */
#if !TEST_MODE_SENSORLESS
        if (if_started && if_speed < IF_TEST_SPEED_RPM &&
            (uint32_t)(now - if_ramp_last) >= 150u)
        {
            if_ramp_last = now;
            if_speed += 5.0f;
            if (if_speed > IF_TEST_SPEED_RPM)
            {
                if_speed = IF_TEST_SPEED_RPM;
            }
            drv_motor_set_openloop_if(g_drv.motor, IF_TEST_CURRENT, if_speed);
        }
#endif

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
