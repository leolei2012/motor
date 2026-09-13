/** @file modbus_cfg.h
 *  @brief Modbus RTU 协议栈编译配置
 *
 *  集中控制角色开关、功能码开关、帧缓冲大小与寄存器数量上限。
 *  本文件已按宿主工程定制：IAP 开启（128 字节块攒页）、发送帧 256、读寄存器上限 63。
 */

#ifndef __MODBUS_CFG_H__
#define __MODBUS_CFG_H__

/** @name 角色选择（可同时开启） */
/** @{ */
#define MB_SLAVE_EN 1   /**< 启用 Slave（1=启用 0=禁用） */
#define MB_MASTER_EN 1  /**< 启用 Master（1=启用 0=禁用） */
/** @} */

/** @name 功能码选择 */
/** @{ */
#define MB_FC01_EN 0  /**< 读线圈 */
#define MB_FC02_EN 0  /**< 读离散输入 */
#define MB_FC03_EN 1  /**< 读保持寄存器 */
#define MB_FC04_EN 0  /**< 读输入寄存器 */
#define MB_FC05_EN 0  /**< 写单线圈 */
#define MB_FC06_EN 0  /**< 写单寄存器 */
#define MB_FC0F_EN 0  /**< 写多线圈 */
#define MB_FC10_EN 1  /**< 写多寄存器 */
#define MB_FC41_EN 1  /**< IAP 升级（自定义功能码 0x41）—— Modbus 在线升级，128 字节块攒页写 APP1 */
/** @} */

/** @name 帧缓冲大小 */
/** @{ */
#define MB_TX_FRAME_SIZE 256  /**< 发送帧缓冲（字节）—— 63 regs×2+5=131B，留余量 */
#define MB_IAP_MAX_DATA 128  /**< IAP 单块数据字节数（128×2=flash 页 256，攒页写） */
/** @} */

/** @name 寄存器数量上限（用于越界检查） */
/** @{ */
#define MB_MAX_READ_COILS  2000  /**< 单次读线圈 / 离散输入数量上限 */
#define MB_MAX_READ_REGS   63    /**< 单次读寄存器数量上限 */
#define MB_MAX_WRITE_COILS 1968  /**< 单次写线圈数量上限 */
#define MB_MAX_WRITE_REGS  50    /**< 单次写寄存器数量上限 */
/** @} */

#endif /* __MODBUS_CFG_H__ */
