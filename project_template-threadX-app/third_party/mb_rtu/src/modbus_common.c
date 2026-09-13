/** @file modbus_common.c
 *  @brief Modbus RTU 协议栈公共实现：CRC16 与基类初始化
 */

#include "modbus/modbus_common.h"

/**
 * @brief 计算 CRC16（多项式 0xA001，初值 0xFFFF）
 */
uint16_t mb_crc16(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i;
    uint8_t j;
    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * @brief 在帧尾追加 CRC16（低字节在前）
 */
void mb_append_crc(uint8_t *frame, uint8_t len)
{
    uint16_t crc = mb_crc16(frame, len);
    frame[len] = (uint8_t)(crc & 0xFF);
    frame[len + 1] = (uint8_t)(crc >> 8);
}

/**
 * @brief 校验帧 CRC16（帧尾 2 字节）
 */
uint8_t mb_check_crc(const uint8_t *frame, uint8_t len)
{
    uint16_t crc_calc = mb_crc16(frame, (uint8_t)(len - 2));
    uint16_t crc_recv = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    return (uint8_t)(crc_calc == crc_recv);
}

/**
 * @brief 初始化基类（memset 清零后设置发送回调与上下文）
 */
void mb_base_init(struct mb_base *base, mb_send_func_t send, void *ctx)
{
    memset(base, 0, sizeof(*base));
    base->send = send;
    base->ctx = ctx;
}
