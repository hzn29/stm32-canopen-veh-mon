# 节点 A 监控 B、C 的 Heartbeat 实验

## 固件配置

三块 NUCLEO-G474RE 使用同一份对象字典，但编译时通过 `canopen_node_config.h` 选择角色：

| 板卡 | CANopen Node-ID | `0x1017` Producer Heartbeat | `0x1016` Consumer Heartbeat |
|---|---:|---:|---|
| A | 1 | 1000 ms | 监控 B、C，各 1500 ms |
| B | 2 | 1000 ms | 未配置 |
| C | 3 | 1000 ms | 未配置 |

`0x1016` 的编码为 `(Node-ID << 16) | timeout_ms`：

- A 的第 1 项：`0x020005DC`，表示监控 Node-ID 2（B），超时 1500 ms。
- A 的第 2 项：`0x030005DC`，表示监控 Node-ID 3（C），超时 1500 ms。
- B、C 的所有监控项为 `0x00000000`，表示不监控远程节点。

`CanOpenPort_Process(1000U)` 每 1 ms 调用 `CO_process()`；CANopenNode 会自动处理 Heartbeat Consumer，不需要在应用层手写轮询。

## 烧录顺序

1. 编译并烧录 A：`CAN_NODE_ROLE` 保持 `CAN_NODE_ROLE_A`。
2. 将 `CAN_NODE_ROLE` 改为 `CAN_NODE_ROLE_B`，重新编译并烧录 B。
3. 将 `CAN_NODE_ROLE` 改为 `CAN_NODE_ROLE_C`，重新编译并烧录 C。
4. 恢复配置为 A，避免下次编译误烧录错误 Node-ID。

## 验证步骤

1. 三节点上电并接入同一条 CAN 总线。
2. 在 PCAN-View 中确认收到：B 的 Heartbeat `0x702`，C 的 Heartbeat `0x703`，周期约 1000 ms。
3. 只断开 B 的供电或 CAN 线，等待约 1500 ms；A 应产生 Heartbeat Consumer 超时错误。
4. 只断开 C 的供电或 CAN 线，等待约 1500 ms；A 应产生 Heartbeat Consumer 超时错误。
5. 恢复线路或重新上电后，A 再次收到对应 Heartbeat，监控状态恢复。
6. 当前 NMT 控制策略由健康状态明确控制；A 会在 `RECOVERING` 稳定确认期间保持 `0x7F`，确认 2000 ms 后，PCAN-View 中的 `0x701` 应恢复为 `0x05`。

## 观察要点

- B、C 不监控其他节点，因此它们不会因为对方停止发送而产生 Heartbeat Consumer 超时。
- A 的超时检测依赖 `CO_process()` 周期调用；默认任务每 1 ms 调用一次。
- 由于 Heartbeat 周期为 1000 ms、超时阈值为 1500 ms，实际超时点会受到任务调度和总线传输延迟影响，通常在 1500 ms 附近。
- 适配层诊断快照可通过 `CanOpenPort_GetDiagnostics()` 读取 B/C 的状态、超时次数、恢复次数和最近一次恢复耗时。
- Heartbeat 超时 EMCY 的标准错误码为 `0x8130`；信息索引 `0` 表示 B，`1` 表示 C。

## 健康状态策略

代码中的 `CanOpenPortHeartbeatStatus_t.healthState` 使用以下状态：

| 状态 | 含义 |
|---|---|
| `UNCONFIGURED` | 当前监控项未配置 |
| `UNKNOWN` | 已配置，但还没有收到有效 Heartbeat |
| `ACTIVE` | Heartbeat 正常 |
| `TIMEOUT` | 本次 Heartbeat 超时 |
| `RECOVERING` | 已重新收到 Heartbeat，等待 2000 ms 稳定确认 |
| `FAULT` | 累计 3 次独立超时，或单次故障超过 5000 ms 未恢复 |

这些阈值集中定义在 `canopen_node_config.h`：

```c
#define CANOPEN_HEALTH_TIMEOUT_EPISODE_LIMIT 3U
#define CANOPEN_HEALTH_RECOVERY_TIMEOUT_MS 5000U
#define CANOPEN_HEALTH_STABLE_TIME_MS 2000U
```

单次故障恢复流程为：

```text
ACTIVE -> TIMEOUT -> RECOVERING -> ACTIVE
```

若累计超时达到 3 次，或 5000 ms 内没有恢复，则进入 `FAULT`。PCAN-View 仍通过 `0x702`、`0x703`、`0x081` 和 `0x701` 验证实际总线现象。

## 在 Keil 中查看诊断变量

`main.c` 中的全局变量 `diagnostics` 每 1 ms 更新一次。进入 Debug 模式后，在 **View -> Watch Windows -> Watch 1** 中添加：

```text
diagnostics.heartbeatB.timeoutCount
diagnostics.heartbeatB.recoveryCount
diagnostics.heartbeatB.lastRecoveryTimeMs
diagnostics.heartbeatB.healthState
diagnostics.heartbeatC.timeoutCount
diagnostics.heartbeatC.recoveryCount
diagnostics.heartbeatC.lastRecoveryTimeMs
diagnostics.heartbeatC.healthState
diagnostics.emcyReportCount
diagnostics.emcyClearCount
diagnostics.nmtState
```

其中 B 使用 `heartbeatB`，C 使用 `heartbeatC`。只有 A 固件会监控 B/C；在 B、C 固件中，这两个监控项通常保持 `UNCONFIGURED`。
> Updated build flow: select `NodeA`, `NodeB`, or `NodeC` in `MDK-ARM/can00.uvprojx`; each target fixes the node role.
