#include "drv.h"

#include "hal.h"

struct drv g_drv;

static struct drv_ain_sensor s_ain_sensor;
static struct drv_output     s_output;
static struct drv_uart       s_uart;
static struct drv_motor      s_motor;

/* 调试串口接收缓冲区 */
#define DEBUG_UART_RX_BUF_SIZE 256u
static uint8_t s_debug_uart_rx_buf[DEBUG_UART_RX_BUF_SIZE];

int drv_init(void)
{
    drv_ain_sensor_init(&s_ain_sensor);
    g_drv.ain_sensor = &s_ain_sensor;

    drv_output_init(&s_output);
    g_drv.output = &s_output;

    /* 调试串口：USART2，帧结束由 TIM7 空闲超时判定（timeout_tick = 3ms） */
    drv_uart_init(&s_uart, s_debug_uart_rx_buf, DEBUG_UART_RX_BUF_SIZE, 3u);
    g_drv.uart = &s_uart;

    /* 电机驱动：实例化 mcl（FOC 无感算法） */
    drv_motor_init(&s_motor);
    g_drv.motor = &s_motor;

    return 0;
}
