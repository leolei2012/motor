# ProbeStation 反馈：RTU 设备轮询静默卡死

> 日期：2026-08-16
> 涉及组件：ProbeStation（砺台）packages/modbus（SerialDriver）、packages/poller
> 触发场景：用 ProbeStation 调试 bootloader（Modbus RTU 从站，USB 转串口）

## 1. 问题概述

ProbeStation 连接 **RTU 串口设备**时，轮询会**静默挂起**：无数据、无错误上报、串口通信灯不闪。
设备明明在跑、串口号对、线也交叉正确，但界面始终读不到寄存器，也看不到任何故障提示。

## 2. 测试环境

| 项 | 值 |
|---|---|
| 目标设备 | STM32G474 bootloader（Modbus RTU 从站，USART2，地址 0x01） |
| 串口参数 | 9600 8N1（无校验），从站地址 1 |
| 寄存器 | 5 个保持寄存器 0x0000–0x0004（FC03 读） |
| 串口 | COM3（USB 转串口，CH340/CP2102 类），TX/RX 已交叉 |
| ProbeStation | serialport v13 + jsmodbus |

## 3. 现象

- 设备（transport=rtu，serialPath=COM3）**读不到数据**，latest 为空。
- **没有** poller/result，**也没有** group-error / 超时上报。
- 串口工具通信灯**不闪**（无 TX）。
- 但用 .NET SerialPort 直连 COM3 时报 Access denied —— 说明 **ProbeStation 底层确实打开了并占用了 COM3**。

对比：同平台两台 TCP 演示设备轮询正常、每秒实时更新，说明轮询引擎本身没坏。

## 4. 已排除项（排查过程）

1. bootloader 在跑 —— 板载 LED 心跳正常闪烁（已进入 IAP 主循环）。
2. 串口号正确 —— 接的是 COM3。
3. 线序正确 —— TX/RX 交叉。
4. 轮询引擎正常 —— TCP 设备实时更新。
5. 串口被打开 —— COM3 被 ProbeStation 占用（.NET 直连被拒）。

排除以上后，问题收敛到 **ProbeStation 的 RTU 串口路径**。

## 5. 根因分析（源码定位）

packages/modbus/src/index.ts 的 SerialDriver.connect()：

```ts
async connect(opts: ConnectOptions): Promise<void> {
  this.disconnect()
  ...
  this.serialPort = await new Promise<any>((resolve, reject) => {
    const sp = new this.portCtor({ path, baudRate, ..., autoOpen: true },
      (err) => { if (err) reject(err); else resolve(sp) })
    sp.on('error', () => {})   // <- 吞掉了 error 事件
  })
  this.ready = true
}
```

serialport v13（@serialport/stream）的 open()：

```ts
open(openCallback) {
  ...
  this.settings.binding.open(openOptions).then(port => {
    ...
    if (openCallback) openCallback.call(this, null);   // 成功才回调
  }, err => {
    this._error(err, openCallback);                     // 失败走 _error
  });
}
```

**推断**：binding.open() 底层成功占用了 COM3（所以 .NET 直连报 Access denied），但
**open 回调没有触发**（binding.open 的 Promise 未 resolve / 或回调未到达），导致：

1. connect() 里的 await new Promise(...) **永久 pending**；
2. poller.getDriver() 里 await driver.connect() 卡死；
3. 轮询对该设备**既不读、也不报错**（getDriver 没抛错，读也没执行）。

这正好解释了「无数据 + 无错误 + 灯不闪」三合一，且 error 事件被 sp.on('error', () => {}) 吞掉，
连最后的兜底信号也丢掉了。

## 6. 建议修复

1. **connect 加超时**：Promise.race 包一层，超时（如 3s）即 reject，不让轮询永久卡死。
2. **不要吞 error**：把 sp.on('error', ...) 接到 reject（或至少记录日志），open 失败要能冒出来。
3. **确认 serialport v13 对该 USB 转串口驱动的 open 回调时序**：CH340/CP2102 类驱动的 open 是否可能
   出现「底层已开、回调不触发」的情况，必要时改用监听 sp.on('open') 事件而非构造回调。
4. 轮询侧加**兜底**：某设备连续 N 轮无进展时，主动标记故障（而不是静默挂起）。

## 7. 复现步骤

1. 启动 ProbeStation。
2. 新建 RTU 设备：transport=rtu、serialPath=COM3、9600 8N1、slaveId=1。
3. 新建 FC03 寄存器组（startAddress=0，quantity=5）。
4. 观察：无数据、无错误、通信灯不闪，latest 恒为空。

## 8. 待进一步验证

- 建议停掉 ProbeStation 后，用独立串口工具直连 COM3 读 bootloader（FC03 读 0x0000×5），
  确认目标设备与线缆 100% 正常，从而把问题彻底锁定在 ProbeStation 侧。

## 9. RTU 设备停用/断开后不释放串口（COM 一直被占用）

**现象**：设备（transport=rtu，serialPath=COM3）停用（toggle，isActive=0）后，COM3 仍被
ProbeStation 占用，外部工具（.NET SerialPort / 串口助手）直连报 `Access denied`。

**根因**：packages/poller/src/index.ts 的 `markDisconnected()` 对 RTU 设备**故意不 disconnect**：

```ts
/** 设备被断开/停用：... 并释放连接 */
private markDisconnected(objectId: number): void {
    ...
    // RTU：共享串口可能还有别的设备在用，不释放；TCP 释放该设备自己的连接
    if (!obj || obj.transport !== 'rtu') {
      const d = this.drivers.get('tcp:' + objectId)
      if (d) { d.disconnect(); this.drivers.delete(key) }
    }
}
```

因为一条 RTU 串口可能挂多台 slaveId 从站、共享一个 SerialDriver，停其中一台不应关整条串口。
但副作用是：**只要连过一次 RTU，停用/删除设备都不还串口**，只能重启进程释放。

**建议**：RTU 串口做引用计数，当「最后一台使用该串口的设备」被停用或删除时，也调用 disconnect 释放串口。

## 10. SerialDriver.disconnect() 未 await close()

**现象**：PUT 编辑设备触发 `poller.reconnectDevice(id)` → `driver.disconnect()`，但 COM3 仍被占用。

**根因**：packages/modbus/src/index.ts 的 `disconnect()`：

```ts
disconnect(): void {
    this.ready = false
    this.clients.clear()
    if (this.serialPort) {
      try { this.serialPort.close() } catch { /* ignore */ }  // <- close() 是异步的，未 await
      this.serialPort = null
    }
}
```

serialport v13 的 `close()` 返回 Promise，这里未 `await` 就置空引用，串口可能没真正关闭，
导致 reconnect 后旧句柄仍占用 COM3。

**建议**：`disconnect()` 改为 `async`，`await this.serialPort.close()` 后再置空。

## 11. 反复 open/close 导致 CH340 串口设备掉线（COM 号消失）

**现象**：调试中反复激活/停用 RTU 设备后，COM3 突然从系统串口列表消失——
`GetPortNames()` 只剩 J-Link 的 COM6，COM3 完全识别不到。

**排查**：CH340（`VID_1A86&PID_7523`，USB 转串口芯片）在注册表里仍有 3 条历史枚举记录
（曾映射 COM3/COM4/COM5），但当前无在线实例；`SERIALCOMM` 里 COM3 的映射已消失。

**推断**：SerialDriver 反复 open/close（叠加第 9 条「停用不释放」+ 第 10 条「close 未 await」），
累积的句柄/异常操作导致 CH340 驱动（`CH341SER_A64`）异常，Windows 将设备重新枚举/移除，串口号随之消失。

**建议**：
1. 与第 9/10 条一并修复：`disconnect()` 改为 `await close()` + RTU 串口引用计数释放，避免无效的重复 open/close。
2. 若复现，先物理拔插 USB 或重启恢复；若驱动反复掉线，需排查 serialport v13 对 CH340 驱动的兼容性。