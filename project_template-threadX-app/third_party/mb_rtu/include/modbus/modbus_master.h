/** @file modbus_master.h
 *  @brief Modbus RTU 主机（Master）
 *
 *  异步请求、忙检测、超时处理与响应回调。单请求模型：一次只挂一个未完成请求。
 */

#ifndef __MODBUS_MASTER_H__
#define __MODBUS_MASTER_H__

#include "modbus/modbus_common.h"

#if MB_MASTER_EN

/** 主机响应回调 */
typedef void (*mb_master_rsp_cb_t)(enum mb_err_t err, const uint8_t *raw, uint8_t len, void *arg);

/** 主机请求状态（单请求模型） */
struct mb_master_req
{
    uint8_t slave_addr;          /**< 目标从站地址 */
    uint8_t fc;                  /**< 功能码 */
    uint16_t reg_addr;           /**< 寄存器起始地址 */
    uint16_t reg_num;            /**< 寄存器数量 */
    uint16_t timeout_reload;     /**< 超时阈值（tick） */
    uint16_t timeout_cnt;        /**< 超时计数（tick 驱动） */

    mb_master_rsp_cb_t on_rsp;   /**< 响应回调 */
    void *rsp_arg;               /**< 回调透传参数 */
    unsigned waiting : 1;        /**< 是否有未完成请求 */
};

/** 主机句柄 */
struct mb_master_handle
{
    struct mb_base base;         /**< 基类：发送回调 + 发送帧缓冲 */
    struct mb_master_req req;    /**< 请求状态 */
};

/**
 * @brief 初始化主机（memset 清零后设置发送回调与上下文）
 * @param self 主机句柄
 * @param send 发送回调
 * @param ctx  用户上下文（回调时透传）
 */
void mb_master_init(struct mb_master_handle *self, mb_send_func_t send, void *ctx);

/**
 * @brief 处理收到的响应帧（验 CRC 后匹配等待中的请求）
 * @param self 主机句柄
 * @param p_data 完整响应帧（含 CRC）
 * @param len 帧长
 */
void mb_master_rx_frame(struct mb_master_handle *self, const uint8_t *p_data, uint8_t len);

/**
 * @brief 周期调用，驱动请求超时
 * @param self 主机句柄
 */
void mb_master_tick(struct mb_master_handle *self);

/**
 * @brief 发起读请求（FC01–FC04）
 * @param self 主机句柄
 * @param slave_addr 从站地址
 * @param fc 功能码
 * @param reg_addr 起始地址
 * @param reg_num 数量
 * @param timeout_tick 超时阈值（tick）
 * @param on_rsp 响应回调
 * @param rsp_arg 回调透传参数
 * @return MB_OK 成功，MB_ERR_BUSY 忙
 */
enum mb_err_t mb_master_read(struct mb_master_handle *self,
                             uint8_t slave_addr,
                             uint8_t fc,
                             uint16_t reg_addr,
                             uint16_t reg_num,
                             uint16_t timeout_tick,
                             mb_master_rsp_cb_t on_rsp,
                             void *rsp_arg);

/**
 * @brief 发起写单请求（FC05 / FC06）
 * @param self 主机句柄
 * @param slave_addr 从站地址
 * @param fc 功能码
 * @param addr 寄存器地址
 * @param value 写入值
 * @param timeout_tick 超时阈值（tick）
 * @param on_rsp 响应回调
 * @param rsp_arg 回调透传参数
 * @return MB_OK 成功，MB_ERR_BUSY 忙
 */
enum mb_err_t mb_master_write_single(struct mb_master_handle *self,
                                     uint8_t slave_addr,
                                     uint8_t fc,
                                     uint16_t addr,
                                     uint16_t value,
                                     uint16_t timeout_tick,
                                     mb_master_rsp_cb_t on_rsp,
                                     void *rsp_arg);

/**
 * @brief 发起写多请求（FC0F / FC10）
 * @param self 主机句柄
 * @param slave_addr 从站地址
 * @param fc 功能码
 * @param start_addr 起始地址
 * @param num 数量
 * @param p_data 数据（按功能码对应的位 / 字节格式）
 * @param data_len 数据长度（字节）
 * @param timeout_tick 超时阈值（tick）
 * @param on_rsp 响应回调
 * @param rsp_arg 回调透传参数
 * @return MB_OK 成功，MB_ERR_BUSY 忙
 */
enum mb_err_t mb_master_write_multi(struct mb_master_handle *self,
                                    uint8_t slave_addr,
                                    uint8_t fc,
                                    uint16_t start_addr,
                                    uint16_t num,
                                    const uint8_t *p_data,
                                    uint8_t data_len,
                                    uint16_t timeout_tick,
                                    mb_master_rsp_cb_t on_rsp,
                                    void *rsp_arg);

#if MB_FC41_EN
/**
 * @brief IAP：开始升级（携带固件总大小与镜像 CRC32）
 * @param self 主机句柄
 * @param slave_addr 从站地址
 * @param total_size 固件总大小（字节）
 * @param fw_crc32 固件镜像 CRC32（标准 CRC-32，0xEDB88320）
 * @param timeout_tick 超时阈值（tick）
 * @param on_rsp 响应回调
 * @param rsp_arg 回调透传参数
 * @return MB_OK 成功，MB_ERR_BUSY 忙
 */
enum mb_err_t mb_master_iap_start(struct mb_master_handle *self,
                                  uint8_t slave_addr,
                                  uint32_t total_size,
                                  uint32_t fw_crc32,
                                  uint16_t timeout_tick,
                                  mb_master_rsp_cb_t on_rsp,
                                  void *rsp_arg);

/**
 * @brief IAP：写数据块
 * @param self 主机句柄
 * @param slave_addr 从站地址
 * @param block_no 块号（从 0 起连续）
 * @param p_data 数据
 * @param data_len 数据长度（字节，≤ MB_IAP_MAX_DATA）
 * @param timeout_tick 超时阈值（tick）
 * @param on_rsp 响应回调
 * @param rsp_arg 回调透传参数
 * @return MB_OK 成功，MB_ERR_BUSY 忙
 */
enum mb_err_t mb_master_iap_write(struct mb_master_handle *self,
                                  uint8_t slave_addr,
                                  uint16_t block_no,
                                  const uint8_t *p_data,
                                  uint8_t data_len,
                                  uint16_t timeout_tick,
                                  mb_master_rsp_cb_t on_rsp,
                                  void *rsp_arg);

/**
 * @brief IAP：结束升级（携带总块数）
 * @param self 主机句柄
 * @param slave_addr 从站地址
 * @param total_blocks 总块数
 * @param timeout_tick 超时阈值（tick）
 * @param on_rsp 响应回调
 * @param rsp_arg 回调透传参数
 * @return MB_OK 成功，MB_ERR_BUSY 忙
 */
enum mb_err_t mb_master_iap_end(struct mb_master_handle *self,
                                uint8_t slave_addr,
                                uint16_t total_blocks,
                                uint16_t timeout_tick,
                                mb_master_rsp_cb_t on_rsp,
                                void *rsp_arg);

/**
 * @brief IAP：查询状态（响应携带期望块号）
 * @param self 主机句柄
 * @param slave_addr 从站地址
 * @param timeout_tick 超时阈值（tick）
 * @param on_rsp 响应回调
 * @param rsp_arg 回调透传参数
 * @return MB_OK 成功，MB_ERR_BUSY 忙
 */
enum mb_err_t mb_master_iap_status(struct mb_master_handle *self,
                                   uint8_t slave_addr,
                                   uint16_t timeout_tick,
                                   mb_master_rsp_cb_t on_rsp,
                                   void *rsp_arg);
#endif /* MB_FC41_EN */

#endif /* MB_MASTER_EN */

#endif
