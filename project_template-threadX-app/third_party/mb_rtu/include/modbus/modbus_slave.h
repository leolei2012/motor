/** @file modbus_slave.h
 *  @brief Modbus RTU 从机（Slave）
 *
 *  分段寄存器映射（线圈 / 离散输入 / 保持寄存器 / 输入寄存器），
 *  支持「连续数组」与「回调（getter/setter）」两种绑定，回调优先。
 */

#ifndef __MODBUS_SLAVE_H__
#define __MODBUS_SLAVE_H__

#include "modbus/modbus_common.h"
#include "modbus/modbus_iap.h"

/** 位读回调（getter） */
typedef enum mb_err_t (*mb_bit_read_cb_t)(uint16_t addr, uint8_t *out_val);
/** 位写回调（setter） */
typedef enum mb_err_t (*mb_bit_write_cb_t)(uint16_t addr, uint8_t val);
/** 寄存器读回调（getter） */
typedef enum mb_err_t (*mb_reg_read_cb_t)(uint16_t addr, uint16_t *out_val);
/** 寄存器写回调（setter） */
typedef enum mb_err_t (*mb_reg_write_cb_t)(uint16_t addr, uint16_t val);

/** 位段：每字节存 1 个线圈 / 离散输入（0 或 1） */
struct mb_bit_seg
{
    uint16_t start_addr;         /**< 起始地址 */
    uint16_t num;                /**< 数量 */
    uint8_t *data;               /**< 连续数组（可选） */
    mb_bit_read_cb_t on_read;    /**< 回调读（可选） */
    mb_bit_write_cb_t on_write;  /**< 回调写（可选） */
};

/** 寄存器段 */
struct mb_reg_seg
{
    uint16_t start_addr;         /**< 起始地址 */
    uint16_t num;                /**< 数量 */
    uint16_t *data;              /**< 连续数组（可选） */
    mb_reg_read_cb_t on_read;    /**< 回调读（可选） */
    mb_reg_write_cb_t on_write;  /**< 回调写（可选） */
};

/** @name 各寄存器类型的段数量（可按需扩充） */
/** @{ */
#define MB_COIL_SEGS 1        /**< 线圈段数量 */
#define MB_DISCRETE_SEGS 1    /**< 离散输入段数量 */
#define MB_HOLDING_SEGS 4     /**< 保持寄存器段数量 */
#define MB_INPUT_SEGS 1       /**< 输入寄存器段数量 */
/** @} */

/** 寄存器映射表：从机数据空间 */
struct mb_reg_map
{
    struct mb_bit_seg coils[MB_COIL_SEGS];        /**< 线圈段数组 */
    uint8_t coils_num;                            /**< 线圈段数量 */
    struct mb_bit_seg discrete[MB_DISCRETE_SEGS]; /**< 离散输入段数组 */
    uint8_t discrete_num;                         /**< 离散输入段数量 */
    struct mb_reg_seg holding[MB_HOLDING_SEGS];   /**< 保持寄存器段数组 */
    uint8_t holding_num;                          /**< 保持寄存器段数量 */
    struct mb_reg_seg input[MB_INPUT_SEGS];       /**< 输入寄存器段数组 */
    uint8_t input_num;                            /**< 输入寄存器段数量 */
};

#if MB_SLAVE_EN

/** 从机句柄 */
struct mb_slave_handle
{
    struct mb_base base;       /**< 基类：发送回调 + 发送帧缓冲 */
    uint8_t slave_addr;        /**< 从站地址 */
    struct mb_reg_map reg_map; /**< 寄存器映射表 */
#if MB_FC41_EN
    struct mb_iap_slave iap;   /**< IAP 状态块 */
#endif
};

/**
 * @brief 处理一帧请求（验 CRC → 比对地址 → 分派处理；广播仅执行不应答）
 * @param self 从机句柄
 * @param p_data 完整请求帧（含 CRC）
 * @param len 帧长
 * @return 1=已处理（广播仅执行不应答），0=未处理（CRC 错 / 地址不匹配 / 帧过短）
 */
uint8_t mb_slave_rx_frame(struct mb_slave_handle *self, const uint8_t *p_data, uint8_t len);

/**
 * @brief 初始化从机（清零句柄后设置地址并拷贝寄存器映射表）
 * @param self 从机句柄
 * @param addr 从站地址
 * @param reg_map 寄存器映射表（拷贝进句柄，可为 NULL）
 */
void mb_slave_init(struct mb_slave_handle *self,
                   uint8_t addr,
                   const struct mb_reg_map *reg_map);

#endif /* MB_SLAVE_EN */

#endif
