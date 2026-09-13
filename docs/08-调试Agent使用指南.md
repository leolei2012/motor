# 08 · 调试 Agent 使用指南（实操手册）

> 状态：v1.0 · 日期：2026-08-14
> 读者：**调试 agent**（用 MCP 工具 + YAML 语义清单去调试嵌入式设备的 AI agent）。
> 前置：请先读 `03-语义清单契约-YAML规范.md`（寄存器说明书格式）、`05-Agent调试接口PRD.md`（接口设计）。
> 本文回答：**你拿到 MCP 工具后，怎么一步步把一台设备调起来、读出问题、写参数验证。**

---

## 0. 一句话原则

> **平台是"原始数据通道"：只帮你读/写 Modbus 寄存器的原始值，不做任何翻译。**
> 一个寄存器读回来是 `254`，平台不会告诉你它是 `25.4℃`——这个换算（scale/unit/enum）由你**拿 YAML 说明书自己算**。

所以你的每一步都是两段式：**用 MCP 拿原始值 → 查 YAML 把它翻译成物理含义**。

---

## 1. 前置条件

| 项 | 说明 |
|---|---|
| 连接方式 | MCP（streamable-http），端点 `http://<host>:8081/mcp`，工具名前缀 `mcp__probestation__` |
| 语义清单 | 固件方产出的 YAML 说明书（03 号规范），**随固件一起提交**，你调试前先拿到它 |
| 设备已配置 | 目标设备已在平台配好（有 `device_id` + 寄存器组 + 寄存器定义），用 `list_devices` 确认 |

**调试前必做**：先 `list_devices` 看设备在不在、`list_registers` 看寄存器在不在，再 `read`。不要盲读盲写。

---

## 2. 工具全景（按用途分组）

### 2.1 看设备/寄存器（先认识目标）

| 工具 | 参数 | 返回 |
|---|---|---|
| `list_devices` | — | 设备列表（`id`/`name`/`ip`/`port`/`mode`/`is_active`/`transport`/`slave_id`） |
| `list_registers` | `device_id` | 该设备寄存器定义（`id`/`alias`/`address`/`data_type`） |
| `list_alarm_rules` | `device_id?` | 已配告警规则 |

### 2.2 读数据（观测）

| 工具 | 参数 | 返回 |
|---|---|---|
| `read_register` | `device_id, register_id` | `{ register_id, address, value, timestamp, quality }` |
| `get_device_snapshot` | `device_id` | 整机所有寄存器快照，每个含 `alias`/`address`/`data_type`/`value`/`timestamp`/`quality` |
| `query_history` | `device_id, register_id, start, end` | 时间序列 `[{ ts, value, quality }]` |
| `get_device_health` | `device_id` | `{ connected, polling, last_sample_time, register_count, ... }` |

### 2.3 写数据（控制，危险）

| 工具 | 参数 | 返回 |
|---|---|---|
| `write_register` | `device_id, register_id, value` | 写原始值（FC16），**忠实执行，无拦截** |

### 2.4 配置（改采集拓扑，默认不用于调试流程）

`create_group` / `update_group` / `create_register` / `update_register` / `delete_register` / `delete_group` —— 改的是"平台要轮询哪些寄存器"，**不属于常规调试动作**，谨慎使用。

### 2.5 OTA（固件升级，仅在需要升级时）

`upload_firmware` / `list_firmwares` / `ota_upgrade` / `ota_status` / `ota_abort` —— 见 `07-OTA固件升级PRD.md`。

---

## 3. 标准调试工作流

```
① list_devices               → 拿到目标 device_id，确认 is_active、transport、slave_id
② list_registers(device_id)  → 拿到所有寄存器，记下「address ↔ register_id ↔ alias」对应
③ get_device_health          → 确认 connected=true、polling=true、last_sample_time 新鲜
④ get_device_snapshot        → 一次拿整机原始值（比逐个 read 快）
⑤ 对照 YAML 翻译             → 把原始值 ×scale、查 enum/bitfield，变成物理含义
⑥ 定位问题                   → 关联多个寄存器（温度+电流+状态字+故障码）做根因推断
⑦ write_register 干预        → 写设定值/命令，写前查 YAML 确认 access、scale 方向
⑧ read_register 验证         → 读回确认生效（volatile 命令位读回 0 属正常）
```

---

## 4. 关键工具详解（含示例）

### 4.1 `read_register` —— 读单个寄存器

```
read_register(device_id=1, register_id=7)
→ { register_id: 7, address: 100, value: 254, timestamp: "2026-08-14T10:00:00.000Z", quality: "good" }
```

**要点**：
- `value` 是**按 `data_type` 解码后的原始数值**（int16/float32 已拼好），但**没有乘 scale、没有 enum 翻译**。
- `address` 是十进制（100 = 0x0064），`timestamp`/`quality` 用于判断数据新旧（`quality != "good"` 或时间戳很旧 = 数据不可信）。

### 4.2 `get_device_snapshot` —— 一次读整机（调试首选）

```
get_device_snapshot(device_id=1)
→ [
    { register_id: 7, alias: "motor_temp", address: 100, data_type: "int16", value: 254, timestamp: "...", quality: "good" },
    { register_id: 8, alias: "status_word", address: 102, data_type: "uint16", value: 18, timestamp: "...", quality: "good" },
    ...
  ]
```

**要点**：一次拿到全设备所有寄存器 + alias + 类型，省去逐个 read 的往返。**调试时优先用这个**。

### 4.3 `write_register` —— 写原始值（危险）

```
write_register(device_id=1, register_id=9, value=700)   → 写原始值 700
```

**写前必查 YAML**：
1. `access`：确认该寄存器可写（`write` / `read_write`）。
2. `scale` 方向：YAML 写 `scale: 0.1` = **物理值 = 裸值 × 0.1**。你要设 70.0℃，裸值应传 `700`，不是 `70`。**别写反**。
3. `volatile: true` 的命令位：写完读回 0 是正常的，别误判写失败。
4. 64 位寄存器（int64/uint64）：`value` 传**字符串**（如 `"9007199254740993"`），避免大数丢精度。

### 4.4 `query_history` —— 查历史趋势

```
query_history(device_id=1, register_id=7, start="2026-08-14T09:00:00Z", end="2026-08-14T10:00:00Z")
→ [ { ts: "...", value: 253, quality: "good" }, { ts: "...", value: 254, quality: "good" }, ... ]
```

**用途**：看某个量随时间的变化（升温曲线、波动、是否卡死）。

### 4.5 `get_device_health` —— 先确认设备活着

```
get_device_health(device_id=1)
→ { connected: true, polling: true, last_sample_time: "2026-08-14T10:00:01.000Z", register_count: 12, ... }
```

**用途**：调试前先确认 `connected` + `polling` + `last_sample_time` 新鲜，否则读到的是陈旧缓存。

---

## 5. 与 YAML 语义清单的配合（核心）

平台给你的是 `value`（原始值），**YAML 告诉你它是什么意思**。翻译规则（见 03 号）：

| 场景 | YAML 字段 | 怎么翻译 |
|---|---|---|
| 有缩放 | `scale: 0.1` | 物理值 = `value × 0.1`（254 → 25.4） |
| 有单位 | `unit: "℃"` | 单位补上 |
| 枚举状态 | `enum: {0: 停机, 1: 运行}` | `value=1` → "运行" |
| 位域 | `bitfield: [...]` | 按 bit 拆开解读（18 = bit1 置位 + bit4 置位） |
| 命令位 | `volatile: true` | 写后读回 0 属正常 |

**寻址**：平台用 `register_id` 或 `address`，**不认 YAML 的 `name`**。`name → address` 的对应关系写在 YAML 里，你自己查表。

**新鲜度**：`firmware.version` 必填，调试前核对"YAML 版本 == 设备当前固件"，过期语义比没有语义更危险。

---

## 6. 端到端示例（调试"冰沙机"）

假设固件方给了 YAML：
```yaml
firmware: { name: 雪融机控制器, version: "1.2.3" }
registers:
  - { name: motor_temp,      address: 0x0064, data_type: int16,  scale: 0.1, unit: "℃", access: read }
  - { name: motor_state,     address: 0x0065, data_type: uint16, enum: {0: 停机, 1: 运行, 2: 故障}, access: read }
  - { name: status_word,     address: 0x0066, data_type: uint16, bitfield: [{bit:0, name: enable}, {bit:15, name: fault_flag}] }
  - { name: heater_setpoint, address: 0x0100, data_type: int16, scale: 0.1, unit: "℃", access: write }
```

调试对话：
```
1. list_devices → 找到 device_id=3（雪融机，transport=tcp）
2. list_registers(3) → address 100↔motor_temp、101↔motor_state、102↔status_word、256↔heater_setpoint
3. get_device_snapshot(3)
     → motor_temp value=254, motor_state value=1, status_word value=0x8002, heater_setpoint value=700
4. 查 YAML 翻译：
     motor_temp 254×0.1 = 25.4℃（正常）
     motor_state 1 = "运行"（正常）
     status_word 0x8002 = fault_flag=1 + enable=1 → 有故障标志，且已使能
5. 定位：有故障标志 fault_flag=1，但 state 还显示运行 → 疑似电机过温/堵转，需进一步查故障码寄存器
6. 干预（若需降负载）：write_register(3, heater_setpoint_id, 600)  → 设 60.0℃
7. 验证：read_register(3, heater_setpoint_id) → value=600，确认写入
```

---

## 7. 注意事项 / 陷阱清单

| # | 陷阱 | 正确做法 |
|---|---|---|
| 1 | **写是危险的**：写错地址/值 = 真机动作 | 写前查 YAML 的 `access`、`scale` 方向，写后读回验证 |
| 2 | `value` 是**原始值**，不是物理值 | 有 scale 的一定要自己乘（254 = 25.4℃），写时也要先反算（70℃ = 写 700） |
| 3 | `volatile` 命令位读回 0 | 正常，别当写失败 |
| 4 | 64 位寄存器写大数 | `value` 传字符串，避免 JS 丢精度 |
| 5 | 数据陈旧 | 读前先 `get_device_health` 看 `last_sample_time`，`quality != "good"` 不可信 |
| 6 | 多字寄存器（float32/int32/int64）跨地址 | 平台已按 `data_type` 自动拼好，你无需手动拼；但 `list_registers` 里可能显示多个地址被合并 |
| 7 | 从站 id（slave_id） | 多从站总线上，确认目标设备的 `slave_id`（在 `list_devices` 结果里），写操作平台按组 slaveId 路由 |
| 8 | 平台不拦截 | 写越界值会**静默截断/回绕**（int16 写 99999 会变成别的数），务必自己在 YAML 值域内写 |

---

## 8. 一句话总结

**先看（list + snapshot + health）→ 再译（查 YAML）→ 后动（write）→ 必验（read 回读）。** 平台给你原始值，YAML 给你含义，判断和动作的责任在你。
