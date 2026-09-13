/*
 *  tx_user.h —— ThreadX 用户配置
 *
 *  本工程的最小可行配置：保留默认 timer 线程（不启用 TX_TIMER_PROCESS_IN_ISR），
 *  关闭性能统计与 trace，减小体积、降低复杂度。
 */
#ifndef TX_USER_H
#define TX_USER_H

/* 优先级数：32（默认，与 tx_port.h 一致） */
#define TX_MAX_PRIORITIES                       32

/*
 * 内核 tick 频率：1000Hz（1ms/tick）。
 * 使 tx_thread_sleep(1) = 1ms，满足 app_task 1ms 节拍需求。
 * SysTick 重载值在 bsp_threadx_low_level.c 里据此自动计算。
 */
#define TX_TIMER_TICKS_PER_SECOND               (1000UL)

/* 最小化体积与开销（可选，按需放开） */
#define TX_DISABLE_PREEMPTION_THRESHOLD
#define TX_DISABLE_REDUNDANT_CLEARING
#define TX_DISABLE_NOTIFY_CALLBACKS
#define TX_NO_FILEX_POINTER

/* 不使用 event trace */
/* #define TX_ENABLE_EVENT_TRACE */

#endif /* TX_USER_H */
