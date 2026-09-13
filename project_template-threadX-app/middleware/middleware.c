#include "middleware.h"

#include "drv.h"
#include "platform.h"
#include "tx_api.h"

#include "dm_motor.h"

struct middleware g_middleware;

static struct debug_monitor s_debug_monitor;

int middleware_init(void)
{
    /* 绑定电机数据源（drivers 层 mcl 遥测 → debug_monitor 观测段 0x2000） */
    dm_motor_bind(g_drv.motor);

    /* debug_monitor：绑定调试串口（g_drv.uart）+ 寄存器表 */
    debug_monitor_init(&s_debug_monitor, 0x01, g_drv.uart);

    g_middleware.debug_monitor = &s_debug_monitor;

    return 0;
}

/* —— middleware 通信线程：周期派发 RX 帧 + 驱动 debug_monitor —— */
#define MIDDLEWARE_TASK_STACK_SIZE 2048u
#define MIDDLEWARE_TASK_PRIORITY   15u

static TX_THREAD s_middleware_thread;
static uint64_t s_middleware_stack[MIDDLEWARE_TASK_STACK_SIZE / sizeof(uint64_t)];

static void middleware_task_entry(ULONG parameter)
{
    (void)parameter;

    while (1)
    {
        drv_uart_dispatch_rx(g_drv.uart);
        debug_monitor_poll(&s_debug_monitor);

        /* 1ms 节拍（ThreadX 1ms/tick） */
        tx_thread_sleep(1u);
    }
}

int middleware_task_init(void)
{
    return (int)tx_thread_create(&s_middleware_thread, "middleware",
                                 middleware_task_entry, 0u,
                                 s_middleware_stack, sizeof(s_middleware_stack),
                                 MIDDLEWARE_TASK_PRIORITY, 0u,
                                 TX_NO_TIME_SLICE, TX_AUTO_START);
}
