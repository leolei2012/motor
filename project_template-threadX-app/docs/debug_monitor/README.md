# debug_monitor 点位表（Modbus 寄存器点表）

> 对应固件：`project_template-threadX-app`（STM32G474VET6，ThreadX + mcl FOC）
> 调试接口：`middleware/debug_monitor`（Modbus RTU 从站，地址 0x01）
> 传输：USART2 9600 8N1，FC03 读保持寄存器

本目录存放 debug_monitor 的**点位表**，供 ProbeStation 调试平台按「分组」导入/导出寄存器点。

---

## 一、点位表文件

| 文件 | 说明 |
|------|------|
| `debug_monitor-pointsheet.xlsx` | 点位表模板（分组=sheet，可导回 ProbeStation） |
| `gen_pointsheet.py` | 生成上面 xlsx 的脚本（纯 Python 标准库，无第三方依赖） |

---

## 二、点位表格式（ProbeStation「分组 = sheet」规范）

一份点表 xlsx 中：

- **每个寄存器分组 = 一个 sheet**（sheet 名 = 分组名）；
- 另有一个「设备信息」sheet 记录连接参数（导入时被忽略，仅人读）。

当前点表只有一个分组 **`MCL_OBS`**（mcl 全量可观测变量）。

### 分组 sheet 的行布局

| 行号 | 内容 |
|------|------|
| 1 | `分组` \| `<分组名>` |
| 2 | `从站`=1 `功能码`=3 `起始地址`=0x2000 `数量`=54 |
| 3 | （空行） |
| 4 | 列头：`别名 \| 数据类型 \| 单位 \| 系数 \| 偏移 \| 枚举 \| 功能码 \| 起始地址 \| 数量` |
| 5+ | 每个寄存器点一行 |

### 列头含义

| 列 | 含义 | 本表取值 |
|----|------|---------|
| 别名 | 点位名 | 见下方寄存器表 |
| 数据类型 | 解码类型 | `uint16` / `uint32` / `float32` |
| 单位 | 工程单位 | rpm / rad / A / V / ℃ / rad/s / pu |
| 系数 | 物理值 = 裸值 × 系数 | 1 |
| 偏移 | 物理值 = 裸值 × 系数 + 偏移 | 0 |
| 枚举 | 离散标签（`值=标签;…`） | 状态/故障/模式字段有，连续量为空 |
| 功能码 | Modbus 功能码 | 3（读保持寄存器） |
| 起始地址 | 寄存器起始地址（十进制） | 0x2000 起 |
| 数量 | 该点占字宽（16 位 = 1；32 位 = 2） | 见下表 |

### 编码约定

- **uint16**：直接 1 寄存器，物理值 = 裸值。
- **uint32**：2 寄存器，**高 16 位在前**。`value = (reg[N] << 16) | reg[N+1]`。
- **float32**：2 寄存器，IEEE754 大端，**高 16 位在前**。`float = floatFromBits((reg[N] << 16) | reg[N+1])`。
  （与 `dm_motor.c`、ProbeStation `codec.ts` 的 `float32` 编解码一致。）

---

## 三、mcl 全量观测寄存器表（0x2000 起，只读）

数据来源：直接读 mcl 结构体公开字段（`mcl.h` 中 motor 对象字段为 public），
由 `dm_motor.c` 的 `on_read` 回调按地址动态取值。

### 3.1 状态与计数段（uint16 / uint32）

| 起始地址 | 别名 | 类型 | 枚举 / 说明 |
|---------|------|------|------|
| 0x2000 | 运行状态 | uint16 | 0=IDLE 1=ALIGN 2=RUN 3=FAULT |
| 0x2001 | 故障码 | uint16 | 0=无 1=过流 2=过压 3=欠压 4=过温 5=堵转 6=门驱 |
| 0x2002 | 控制模式 | uint16 | 0=电流 1=速度 2=位置 3=VF 4=IF 5=预定位 |
| 0x2003 | 运行模式 | uint16 | 0=FOC有感 1=FOC无感 2=BLDC霍尔 3=BLDC无感 |
| 0x2004 | 开环阶段 | uint16 | 0=未开环 1=锁定 2=拖动 |
| 0x2005 | 控制周期计数 | uint32 | 高字在前，`mcl.tick_count` |
| 0x2007 | （保留） | uint16 | 对齐填充，恒 0 |

### 3.2 实时量段（float32，每 2 寄存器）

| 起始地址 | 别名 | 单位 | 字段 |
|---------|------|------|------|
| 0x2008 | 转速 | rpm | `speed_rad_s ÷ pole_pairs × 60/(2π)` |
| 0x200A | 位置 | rad | `phase_rad` |
| 0x200C | Iq电流 | A | `iq_now` |
| 0x200E | Id电流 | A | `id_now` |
| 0x2010 | 母线电压 | V | `vbus` |
| 0x2012 | 母线电流 | A | 恒 0（无传感器） |
| 0x2014 | 占空比 | pu | `duty_now` [-1,1] |
| 0x2016 | 电机温度 | ℃ | 恒 0（未接） |
| 0x2018 | FET温度 | ℃ | 恒 0（未接） |
| 0x201A | 估计电角度 | rad | `phase_rad`（观测器） |
| 0x201C | 估计角速度 | rad/s | `speed_rad_s` |
| 0x201E | Iq目标 | A | `iq_ref` |
| 0x2020 | 速度目标 | rpm | `speed_ref_rpm` |
| 0x2022 | 开环幅值 | pu | `openloop_mag` |
| 0x2024 | 开环角速度 | rad/s | `openloop_speed` |
| 0x2026 | 开环相位 | rad | `openloop_angle` |
| 0x2028 | Vα上周期 | V | `v_alpha_prev` |
| 0x202A | Vβ上周期 | V | `v_beta_prev` |

### 3.3 故障快照段（float32 + uint32）

| 起始地址 | 别名 | 类型 | 说明 |
|---------|------|------|------|
| 0x202C | 故障电流 | float32 | `fault_info.current` A |
| 0x202E | 故障电压 | float32 | `fault_info.voltage` V |
| 0x2030 | 故障转速 | float32 | `fault_info.speed` rad/s |
| 0x2032 | 故障温度 | float32 | `fault_info.temp` ℃ |
| 0x2034 | 故障周期计数 | uint32 | `fault_info.tick` |

### 3.4 SMO 观测器内部状态段（float32，诊断观测器是否跟踪转子）

| 起始地址 | 别名 | 单位 | 说明 |
|---------|------|------|------|
| 0x2036 | SMO反电动势α | V | `observer.e_alpha_final`（二级滤波反电动势 α） |
| 0x2038 | SMO反电动势β | V | `observer.e_beta_final`（二级滤波反电动势 β） |
| 0x203A | SMO角增量w | — | `observer.w_est`（每采样角增量 ω·Ts，无量纲） |
| 0x203C | SMO估计电流α | A | `observer.i_alpha_hat`（滑模电流观测器估计值） |

> 诊断用法：开环 IF 拖动电机时 SMO 后台运行，观察 `e_alpha_final`/`e_beta_final`
> 是否形成稳定旋转矢量（两相正弦），`w_est` 是否稳定在 `ω·Ts` 附近，
> 即可判断 SMO 是否正常跟踪转子反电动势。

> 共 34 个字段，寄存器地址 0x2000 ~ 0x203D（62 个保持寄存器）。

---

## 四、与其它调试段的关系

debug_monitor 的完整 holding 寄存器布局（`dm_adapter.c`）：

| 地址段 | 用途 | 读/写 |
|--------|------|-------|
| 0x0000 ~ 0x0001 | 测试写段（`CTRL_REG_*`，写回显） | 写 |
| 0x2000 ~ 0x203D | **mcl 全量观测段 + SMO 内部状态**（本点位表） | 只读 |
| 0xE000 ~ 0xE002 | 测试读段（`TEST_REG_*`，魔数/版本/计数） | 只读 |

本点位表**只覆盖 0x2000 观测段**；如需纳入测试段/写段，向 `gen_pointsheet.py`
的 `FIELDS` 列表或新增 sheet 扩展即可。

---

## 五、重新生成 / 扩展

```bash
python docs/debug_monitor/gen_pointsheet.py
```

脚本内部维护 `FIELDS` 列表（`别名, 数据类型, 单位, 枚举, 寄存器数` 五元组），
字段顺序与 `middleware/debug_monitor/src/dm_motor.c` 的地址映射一一对应。
改观测字段时**两处必须同步**：

1. `dm_motor.c`（固件侧 `dm_motor_read_half` 的地址分派）
2. `gen_pointsheet.py` 的 `FIELDS`（点位表侧）
