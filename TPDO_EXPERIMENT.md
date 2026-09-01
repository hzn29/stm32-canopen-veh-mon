# TPDO 实验配置

本次在官方示例对象字典上增加了一个应用对象，并启用 TPDO1：

```text
0x2000:00：32 位 TPDO 序号
0x1800:01：TPDO1 默认 COB-ID 0x180，协议栈按 Node-ID 计算
0x1800:02：0xFE，事件触发传输
0x1800:05：100 ms 事件定时器
0x1A00:00：映射数量 1
0x1A00:01：0x20000020，表示 0x2000:00 的 32 bit
```

因此默认 TPDO1 ID 为：

```text
A：0x181
B：0x182
C：0x183
```

每 100 ms，`CanOpenPort_Process()` 会递增 `OD_RAM.x2000_tpdoSequence`，请求 TPDO1 发送。节点必须处于 Operational 状态，TPDO 才会发送。

Classic CAN 预期报文：

```text
ID      DLC  DATA
0x181   4    序号低字节在前
```

本次同时将 `CO_process_SYNC()`、`CO_process_RPDO()` 和 `CO_process_TPDO()` 接入 FreeRTOS 周期任务。
