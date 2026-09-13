#ifndef APP_TASK_H
#define APP_TASK_H

#include "platform.h"

#include "tx_api.h"

/*
 *  app_task：应用层线程（ThreadX）
 *
 *  承载所有 app 层周期任务：LED 心跳、传感器轮询（drivers）、告警轮询。
 *  由 tx_application_define 创建，通过 tx_thread_sleep 做周期调度。
 *
 *  层次依赖：app_task → app_led / app_alarm_system / drivers
 */

/** 线程栈大小（16 字节对齐） */
#define APP_TASK_STACK_SIZE   1024u

/** 线程优先级（数值越小优先级越高） */
#define APP_TASK_PRIORITY     16u

/** 线程主体入口，作为 tx_thread_create 的 entry 传入 */
void app_task_entry(ULONG thread_input);

/*
 * 在 tx_application_define 里创建 app_task 线程。
 * @param thread_ptr  由调用方持有的 TX_THREAD 控制块
 * @param stack_ptr   线程栈（调用方分配）
 */
void app_task_create(TX_THREAD *thread_ptr, VOID *stack_ptr);

#endif /* APP_TASK_H */
