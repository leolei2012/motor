#include "debug.h"

/**
 * 将此函数替换为实际的串口发送实现。
 * 例如：hal_uart_transmit(&g_debug_uart, (uint8_t *)&c, 1, 10);
 */
__attribute__((weak))
void debug_putchar(char c)
{
    (void)c;
    /** 默认空实现，由用户在 bsp 层覆盖 */
}