/** @file modbus_iap.h
 *  @brief IAP 升级（自定义功能码 0x41）从机侧定义
 *
 *  只负责「接数据」：把主机发来的固件字节流按块、按序交给用户回调。
 *  不关心 flash 分区 / 跳转 / 激活回滚，这些由用户在回调里自行处理。
 */

#ifndef __MODBUS_IAP_H__
#define __MODBUS_IAP_H__

#include "modbus/modbus_common.h"

/** @name IAP 子命令 */
/** @{ */
#define MB_IAP_CMD_START  0x01  /**< 开始：携带 total_size(4B) + fw_crc32(4B) */
#define MB_IAP_CMD_DATA   0x02  /**< 数据块：block_no(2B) + data(N) */
#define MB_IAP_CMD_END    0x03  /**< 结束：携带 total_blocks(2B) */
#define MB_IAP_CMD_STATUS 0x04  /**< 查询状态：响应携带 next_block(2B) */
/** @} */

/** IAP 状态码（正常响应内携带，非标准 Modbus 异常） */
enum mb_iap_status_t
{
    MB_IAP_OK         = 0x00,  /**< 成功 */
    MB_IAP_BAD_BLOCK  = 0x01,  /**< 块号不连续 */
    MB_IAP_BAD_LEN    = 0x02,  /**< 数据长度错误 */
    MB_IAP_NOT_ACTIVE = 0x03,  /**< 未 START 就 DATA/END */
    MB_IAP_INTERNAL   = 0x04,  /**< 用户回调返回错误 / 缺少必须的 on_data 回调 */
};

/** 从机数据回调：用户实现「接数据」 */
typedef enum mb_err_t (*mb_iap_start_cb_t)(uint32_t total_size, uint32_t crc32);          /**< START 回调：携带固件总大小与镜像 CRC32 */
typedef enum mb_err_t (*mb_iap_data_cb_t)(uint16_t block_no, const uint8_t *data, uint8_t len); /**< DATA 回调：接收一块数据 */
typedef enum mb_err_t (*mb_iap_end_cb_t)(void);                                           /**< END 回调：升级结束 */

/** 从机 IAP 状态块（内嵌于 mb_slave_handle） */
struct mb_iap_slave
{
    mb_iap_start_cb_t on_start;  /**< START 回调（可选） */
    mb_iap_data_cb_t  on_data;   /**< DATA 回调（必须：接数据） */
    mb_iap_end_cb_t   on_end;    /**< END 回调（可选） */
    uint32_t total_size;         /**< 固件总大小（START 时记录） */
    uint32_t expected_crc32;     /**< 期望的固件镜像 CRC32（START 时记录，END 时由用户比对） */
    uint16_t next_block;         /**< 期望的下一个块号 */
    uint8_t  active;             /**< 是否处于升级中 */
};

#endif /* __MODBUS_IAP_H__ */
