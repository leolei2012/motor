/** @file modbus_common.h
 *  @brief Modbus RTU 协议栈公共定义
 *
 *  包含功能码、异常码、错误码、基类（mb_base）、发送回调与 CRC16 接口。
 *  Master 与 Slave 共享本头，不面向具体角色。
 */

#ifndef __MODBUS_COMMON_H__
#define __MODBUS_COMMON_H__

/**
 * @brief 用户适配层
 *
 * 提供 uint8_t / uint16_t 等基础类型与 memset，由用户工程按自身平台提供
 * （本仓库不含此文件，examples/platform.h 仅作示例）。
 */
#include "platform.h"
#include "modbus/modbus_cfg.h"

/** @name 版本号 */
/** @{ */
#define MB_RTU_VERSION_MAJOR   1
#define MB_RTU_VERSION_MINOR   0
#define MB_RTU_VERSION_PATCH   0

/** 版本号字符串（如 "1.0.0"） */
#define MB_RTU_VERSION_STRING  "1.0.0"

/** 版本号数值：major<<16 | minor<<8 | patch，用于编译期/运行期比较 */
#define MB_RTU_VERSION_NUM     ((MB_RTU_VERSION_MAJOR << 16) | (MB_RTU_VERSION_MINOR << 8) | MB_RTU_VERSION_PATCH)
/** @} */

/** @name Modbus 标准功能码 */
/** @{ */
#define MB_FC_READ_COILS          0x01  /**< 读线圈 */
#define MB_FC_READ_DISCRETE_INPUTS 0x02  /**< 读离散输入 */
#define MB_FC_READ_HOLDING_REGS   0x03  /**< 读保持寄存器 */
#define MB_FC_READ_INPUT_REGS     0x04  /**< 读输入寄存器 */
#define MB_FC_WRITE_SINGLE_COIL   0x05  /**< 写单线圈 */
#define MB_FC_WRITE_SINGLE_REG    0x06  /**< 写单寄存器 */
#define MB_FC_WRITE_MULTI_COILS   0x0F  /**< 写多线圈 */
#define MB_FC_WRITE_MULTI_REGS    0x10  /**< 写多寄存器 */
#define MB_FC_IAP                 0x41  /**< IAP 升级（自定义） */
/** @} */

/** @name Modbus 标准异常码 */
/** @{ */
#define MB_EX_ILLEGAL_FUNCTION      0x01  /**< 非法功能码 */
#define MB_EX_ILLEGAL_DATA_ADDRESS  0x02  /**< 非法数据地址 */
#define MB_EX_ILLEGAL_DATA_VALUE    0x03  /**< 非法数据值 */
#define MB_EX_SLAVE_DEVICE_FAILURE  0x04  /**< 从机设备故障 */
/** @} */

/** Modbus 错误码（协议栈内部返回） */
enum mb_err_t
{
    MB_OK = 0,           /**< 成功 */
    MB_ERR_CRC = -1,     /**< CRC 校验失败 */
    MB_ERR_ADDR = -2,    /**< 地址不匹配 / 无此地址 */
    MB_ERR_FC = -3,      /**< 功能码错误 / 从机返回异常 */
    MB_ERR_RANGE = -4,   /**< 寄存器范围越界 */
    MB_ERR_LEN = -5,     /**< 帧长度错误 */
    MB_ERR_BUSY = -6,    /**< 主机忙（上一请求未完成） */
    MB_ERR_TIMEOUT = -7, /**< 响应超时 */
};

/** 发送回调：把协议栈组好的帧发往物理链路（UART / RS-485），ctx 为注册时透传的用户上下文 */
typedef uint8_t (*mb_send_func_t)(const uint8_t *buf, uint8_t len, void *ctx);

/** Modbus 基类：Master / Slave 句柄内嵌，持有发送回调、用户上下文与发送帧缓冲 */
struct mb_base
{
    mb_send_func_t send;                 /**< 发送回调 */
    void          *ctx;                  /**< 用户上下文（回调时透传） */
    uint8_t        tx_frame[MB_TX_FRAME_SIZE];  /**< 发送帧缓冲（拼帧工作区） */
};

/**
 * @brief 初始化基类（memset 清零后设置发送回调与上下文）
 * @param base 基类指针
 * @param send 发送回调（可为 NULL）
 * @param ctx  用户上下文（回调时透传）
 */
void mb_base_init(struct mb_base *base, mb_send_func_t send, void *ctx);

/**
 * @brief 计算 CRC16（多项式 0xA001，初值 0xFFFF）
 * @param buf 数据指针
 * @param len 数据长度（字节）
 * @return CRC16 校验值
 */
uint16_t mb_crc16(const uint8_t *buf, uint8_t len);

/**
 * @brief 在帧尾追加 CRC16（低字节在前）
 * @param frame 帧缓冲（需预留 2 字节空间）
 * @param len 帧内已有数据长度（不含 CRC）
 */
void mb_append_crc(uint8_t *frame, uint8_t len);

/**
 * @brief 校验帧 CRC16（帧尾 2 字节）
 * @param frame 完整帧（含 CRC）
 * @param len 帧总长（含 CRC）
 * @return 1=校验通过，0=校验失败
 */
uint8_t mb_check_crc(const uint8_t *frame, uint8_t len);

#endif
