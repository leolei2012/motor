#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <stdint.h>

#include "modbus/modbus_slave.h"

/** 前向声明：避免 middleware 层 include drivers 头，仅作不透明指针传递 */
struct drv_motor;

/**
 * @file    dm_motor.h
 * @brief   debug_monitor 的电机观测段：把 mcl 全量可观测变量映射到 Modbus 保持寄存器。
 *
 * 观测段从 0x2000 起，只读。变量类型三类：
 *   - uint16 枚举（state/fault/ctrl_mode/mode/ol_stage）占 1 寄存器
 *   - uint32（tick_count）占 2 寄存器（高字在前）
 *   - float32（其余浮点量）占 2 寄存器（IEEE754 大端，高字在前）
 *
 * 数据源由 dm_motor_bind() 注入（drivers 层 mcl 实例），
 * middleware 层通过 mcl 结构体公开字段直接读取（mcl.h 中字段为 public）。
 */

/** 观测段起始地址 */
#define DM_MOTOR_REG_BASE 0x2000u

/** 观测段寄存器总数 */
#define DM_MOTOR_REG_NUM  0x3Eu   /* 0x2000 ~ 0x203D，追加 4 个 SMO 内部状态 float32（0x2036 起） */

/**
 * @brief 绑定电机数据源（drivers 层在 init 时调用）
 * @param motor drv_motor 实例（内部持有 mcl；可为 NULL 以解绑）
 */
void dm_motor_bind(const struct drv_motor *motor);

/**
 * @brief 返回电机观测段的寄存器段（只读，on_read 回调动态取值）
 *        可直接并入 mb_reg_map 的 holding 段。
 */
void dm_motor_reg_seg(struct mb_reg_seg *out_seg);

#endif /* DM_MOTOR_H */
