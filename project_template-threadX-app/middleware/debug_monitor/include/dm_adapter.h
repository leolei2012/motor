#ifndef DM_ADAPTER_H
#define DM_ADAPTER_H

#include "debug_monitor.h"

/**
 * @file    dm_adapter.h
 * @brief   debug_monitor 库的测试适配器
 *
 * 暴露一组测试寄存器（只读：魔数 / 版本 / 自增计数；可写：测试字 / 命令），
 * 用于验证 Modbus 从站的读 / 写链路是否可用。
 */

/** 返回测试寄存器映射表（连续保持寄存器 0x0000 起） */
const struct mb_reg_map *dm_adapter_reg_map(void);

/** 返回适配器钩子（init 时做一次数据源绑定） */
const dm_adapter *dm_adapter_get(void);

/** 刷新测试寄存器（收到 Modbus 帧前调用，保证读到最新值） */
void dm_adapter_refresh(void);

#endif /* DM_ADAPTER_H */
