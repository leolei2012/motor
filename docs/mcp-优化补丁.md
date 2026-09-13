# MCP 优化补丁（3 处真实 bug）

> 目标文件：`C:\Users\22671\Desktop\ProbeStation\packages\mcp\src\index.ts`
> 说明：以下是 3 处精确修改（old → new），可直接手动应用，或授权后在目标目录执行 `edit`。

---

## 改动 1：`write_register` 丢失 slaveId + 64 位精度（P0）

**问题**：`poller.write(...)` 没传 `slaveId`（默认 1），从站 id ≠ 1 时写错设备；`value: zz.number()` 对 int64/uint64 丢精度。对照 `packages/api/src/index.ts` 的写路径（`grp?.slaveId ?? 1` + `BigInt(String(value))`）是对齐依据。

### 1a. import 加 `baseType`

```ts
// old
import { decodeRawByAddr, encodeRegister, registerWidth } from '@probebench/core'

// new
import { baseType, decodeRawByAddr, encodeRegister, registerWidth } from '@probebench/core'
```

### 1b. `write_register` 工具体替换

```ts
// old
    server.registerTool('write_register', {
      title: 'Write register', description: '写某设备单个寄存器（FC16，控制真机，危险操作）',
      inputSchema: { device_id: zz.number(), register_id: zz.number(), value: zz.number() },
    }, async (args) => {
      const reg = cfg.getRegister(args.register_id)
      if (!reg || reg.objectId !== args.device_id) return { content: [{ type: 'text', text: 'register not found' }], isError: true }
      const words = encodeRegister(reg.dataType ?? 'int16', args.value)
      await poller.write(reg.objectId, reg.startAddress, words, 'multiple')
      cfg.log('INFO', 'mcp', 'write register ' + args.register_id + ' = ' + args.value)
      return { content: [{ type: 'text', text: JSON.stringify({ register_id: args.register_id, value: args.value }) }] }
    })
```

```ts
// new
    server.registerTool('write_register', {
      title: 'Write register', description: '写某设备单个寄存器（FC16，控制真机，危险操作）。64 位寄存器 value 传字符串避免丢精度',
      inputSchema: { device_id: zz.number(), register_id: zz.number(), value: zz.union([zz.number(), zz.string()]) },
    }, async (args) => {
      const reg = cfg.getRegister(args.register_id)
      if (!reg || reg.objectId !== args.device_id) return { content: [{ type: 'text', text: 'register not found' }], isError: true }
      const base = baseType(reg.dataType ?? 'int16')
      const is64 = base.endsWith('64') && base !== 'float64'
      const value = is64 ? BigInt(String(args.value)) : Number(args.value)
      const words = encodeRegister(reg.dataType ?? 'int16', value)
      const grp = cfg.getGroup(reg.groupId)
      await poller.write(reg.objectId, reg.startAddress, words, 'multiple', grp?.slaveId ?? 1)
      cfg.log('INFO', 'mcp', 'write register ' + args.register_id + ' = ' + String(value))
      return { content: [{ type: 'text', text: JSON.stringify({ register_id: args.register_id, value: String(value) }) }] }
    })
```

---

## 改动 2：`query_history` 缺设备归属校验（P1）

**问题**：只校验 `register_id` 存在，没校验它属于 `device_id`；跨设备传参会用错误 dataType 解码目标设备数据。

```ts
// old
    }, async (args) => {
      const reg = cfg.getRegister(args.register_id)
      if (!reg) return { content: [{ type: 'text', text: 'register not found' }], isError: true }
      const points = await store.queryObject(args.device_id, args.start, args.end)
```

```ts
// new
    }, async (args) => {
      const reg = cfg.getRegister(args.register_id)
      if (!reg || reg.objectId !== args.device_id) return { content: [{ type: 'text', text: 'register not found' }], isError: true }
      const points = await store.queryObject(args.device_id, args.start, args.end)
```

---

## 改动 3：更新 `packages/mcp/README.md` 工具清单（P2，文档过时）

当前 README 仍是旧版（列 `read_all`、缺 snapshot/health/alarm/OTA）。建议按 `mcp/src/index.ts` 实际工具补齐为：

| 工具 | 说明 |
|---|---|
| `list_devices` | 设备列表 |
| `list_registers` | 某设备寄存器定义 |
| `read_register` | 读单寄存器（含 address/value/timestamp/quality） |
| `get_device_snapshot` | 整机快照（alias/address/data_type/value/timestamp/quality） |
| `query_history` | 历史时序 |
| `write_register` | 写原始值（FC16） |
| `get_device_health` | 连接/轮询/最近采样 |
| `list_alarm_rules` | 告警规则 |
| `create_group`/`update_group`/`create_register`/`update_register`/`delete_register`/`delete_group` | 配置 CRUD |
| `upload_firmware`/`list_firmwares`/`ota_upgrade`/`ota_status`/`ota_abort` | OTA 固件升级 |

---

## 未做（有意跳过）

- **值域校验 / 越界报错**：产品决策已明确"平台忠实执行、不做拦截"，故不拦，仅在本指南 §7 提示 agent 自查。
- **按名寻址 / 语义翻译**：产品决策已砍掉，不恢复。
