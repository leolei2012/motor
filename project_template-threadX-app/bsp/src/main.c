#include "hal.h"
#include "bsp_config.h"

#include "drv.h"
#include "app.h"
#include "app_task.h"
#include "middleware.h"

#include "tx_api.h"

const uint32_t g_version = 0x010000u;  /**< 主版本号(高字节) + 次版本号(低字节) + 修订号(第三字节) */

/** app_task 线程控制块与栈（由 main 持有，静态分配） */
static TX_THREAD s_app_task_thread;
static ULONG     s_app_task_stack[APP_TASK_STACK_SIZE / sizeof(ULONG)];

/** ThreadX 内核可用内存（byte pool，供内核/驱动分配） */
static ULONG s_byte_pool_memory[2048u];
static TX_BYTE_POOL s_byte_pool;

/**
 * @brief 应用定义：在内核调度前创建线程（ThreadX 约定入口）
 * @param first_unused_memory 内核未使用的内存起始地址
 */
void tx_application_define(void *first_unused_memory)
{
    (void)first_unused_memory;

    /* 创建 byte pool，供内核/应用按需分配 */
    tx_byte_pool_create(&s_byte_pool,
                        "app byte pool",
                        s_byte_pool_memory,
                        sizeof(s_byte_pool_memory));

    /* 创建 app_task 线程（承载 LED / 传感器 / 告警周期任务） */
    app_task_create(&s_app_task_thread, s_app_task_stack);

    /* 创建 middleware 通信线程（debug_monitor RX 派发） */
    middleware_task_init();
}

/**
 * @brief 程序入口：板级/外设初始化后进入 ThreadX 内核
 */
int main(void)
{
    /* 板级早期初始化：HAL + 时钟树（170MHz） */
    bsp_board_early_init();

    /* 外设初始化：GPIO / ADC / TIM / CORDIC / USART + NVIC */
    hal_init();

    /* 驱动与应用的实例化 + 依赖注入 */
    drv_init();
    app_init();
    middleware_init();

    /* 进入 ThreadX 内核（永不返回）；线程创建在 tx_application_define 中 */
    tx_kernel_enter();
}
