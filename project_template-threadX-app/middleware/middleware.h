#ifndef MIDDLEWARE_H
#define MIDDLEWARE_H

#include "debug_monitor.h"

extern struct middleware g_middleware;

/**
 * @brief  中间件载体：集成 debug_monitor 调试接口等中间件模块
 */
struct middleware
{
    struct debug_monitor *debug_monitor;  /**< debug_monitor 调试接口（Modbus 从站 + 调试寄存器） */
};

int middleware_init(void);
int middleware_task_init(void);

#endif /* MIDDLEWARE_H */
