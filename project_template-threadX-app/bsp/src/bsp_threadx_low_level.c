/*
 *  bsp_threadx_low_level.c —— ThreadX Cortex-M4 低层初始化（AC6 / Keil）
 *
 *  适配本工程现有 CubeMX 启动文件（startup_stm32g474xx.s）：
 *  - 向量表 VTOR 已由 SystemInit 设置，这里不再重复。
 *  - SysTick 配置为 ThreadX 内核 tick（1ms，TX_TIMER_TICKS_PER_SECOND=1000）。
 *  - 报告未用内存起始地址（供 tx_application_define 使用）。
 *  - 配置 PendSV / SVCall / SysTick 异常优先级。
 *
 *  !!! 关键约束（参考 MC-COMPRESSOR bsp_threadx.c）：
 *  SysTick（内核节拍）优先级必须比 PendSV 高，否则空闲循环里 SysTick
 *  无法抢占正在执行的 PendSV，睡眠线程永远不会被唤醒，卡死在 __tx_ts_wait。
 */
#include "tx_api.h"

#include "stm32g4xx.h"

/* ThreadX 内核期望的首个未用内存指针（由低层初始化赋值） */
extern VOID *_tx_initialize_unused_memory;

/* ThreadX 内部符号：系统栈指针 */
extern VOID *_tx_thread_system_stack_ptr;

/* 占位的未用内存（模板使用静态线程栈，内核 byte pool 也用静态数组） */
static __attribute__((aligned(8))) UCHAR s_unused_memory[64];

/*
 * @brief ThreadX 低层初始化
 * 由 _tx_initialize_kernel_enter 在内核进入前、中断已屏蔽时调用。
 */
VOID _tx_initialize_low_level(VOID)
{
    /* 关闭中断，保护初始化过程 */
    __disable_irq();

    /*
     * 保存初始系统栈指针（用于中断嵌套时的系统栈）。
     * 此时内核尚未启动，MSP 即上电后的初始系统栈顶。
     */
    _tx_thread_system_stack_ptr = (VOID *)__get_MSP();

    /* 报告未用内存起始地址（ThreadX 会把它传给 tx_application_define） */
    _tx_initialize_unused_memory = (VOID *)s_unused_memory;

    /* 配置 SysTick 为 ThreadX 内核 tick（1ms @ 170MHz，TX_TIMER_TICKS_PER_SECOND=1000） */
    SysTick->LOAD  = (SystemCoreClock / TX_TIMER_TICKS_PER_SECOND) - 1u;
    SysTick->VAL   = 0u;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |   /* 内部处理器时钟 */
                     SysTick_CTRL_TICKINT_Msk   |   /* 使能中断 */
                     SysTick_CTRL_ENABLE_Msk;       /* 使能计数器 */

    /* NVIC 优先级分组：无分组（全抢占优先级） */
    NVIC_SetPriorityGrouping(4u);

    /*
     * 配置内核异常优先级（关键！）：
     *  - PendSV / SVCall：最低优先级（STM32G4 为 15，4 位优先级）
     *  - SysTick：必须比 PendSV 高（这里用 1），否则空闲循环里 SysTick
     *    无法抢占 PendSV，睡眠线程永远不被唤醒，卡死在 __tx_ts_wait。
     *
     * NVIC_SetPriority 内部会按优先级位数（4 位）自动左移，改用它会比
     * 直接写 SHP 字节更安全可靠。
     */
    NVIC_SetPriority(PendSV_IRQn, 15u);   /* PendSV = 最低 (15) */
    NVIC_SetPriority(SVCall_IRQn, 15u);   /* SVCall = 最低 (15) */
    NVIC_SetPriority(SysTick_IRQn, 1u);   /* SysTick = 比 PendSV 高 */

    /* 使能 FPU（Cortex-M4F） */
    SCB->CPACR |= ((3UL << (10u * 2u)) | (3UL << (11u * 2u)));
}
