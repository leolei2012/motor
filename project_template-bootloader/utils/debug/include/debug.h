/**
 * @file  debug.h
 * @brief 轻量级调试日志宏
 *
 * 用法：
 *   LOG_INFO("voltage = %d mV", voltage);
 *   LOG_WARN("temp high: %d", temp);
 *   LOG_ERR("init failed: %d", err);
 *
 * 通过 project_config.h 中的 CONFIG_DEBUG_UART_ENABLE 开关控制。
 * 替换 debug_putchar 为实际的串口发送函数。
 */
#ifndef DEBUG_H
#define DEBUG_H

#include "project_config.h"
#include <stdio.h>

void debug_putchar(char c);  /**< 在 utils/src/debug.c 中由用户实现 */

#if CONFIG_DEBUG_UART_ENABLE
#  define LOG(level, fmt, ...) \
       do { \
           char _buf[128]; \
           int  _n = snprintf(_buf, sizeof(_buf), \
               "[" level "] %s:%d " fmt "\r\n", \
               __FILE__, __LINE__, ##__VA_ARGS__); \
           for (int _i = 0; _i < _n; _i++) debug_putchar(_buf[_i]); \
       } while (0)
#  define LOG_INFO(fmt, ...)  LOG("I", fmt, ##__VA_ARGS__)
#  define LOG_WARN(fmt, ...)  LOG("W", fmt, ##__VA_ARGS__)
#  define LOG_ERR(fmt, ...)   LOG("E", fmt, ##__VA_ARGS__)
#else
#  define LOG_INFO(...)  do {} while (0)
#  define LOG_WARN(...)  do {} while (0)
#  define LOG_ERR(...)   do {} while (0)
#endif

#endif /* DEBUG_H */