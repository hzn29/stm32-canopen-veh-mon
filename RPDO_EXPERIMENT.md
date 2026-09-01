# RPDO 实验配置

已增加 `0x2001:00` 作为 RPDO1 控制命令：

```text
类型：UNSIGNED8
长度：1 字节
默认值：0
RPDO1 映射：0x2001:00，8 bit
```

RPDO1 使用默认 COB-ID `0x200 + Node-ID`：

```text
A：0x201
B：0x202
C：0x203
```

在 PCAN-View 中向节点 A 发送：

```text
ID：0x201
DLC：1
DATA：01
```

节点 A 收到后，`OD_RAM.x2001_rpdoCommand` 变为 `0x01`。发送 `00` 可以将其清零：

```text
ID：0x201
DLC：1
DATA：00
```

节点必须处于 Operational 状态。当前 `CanOpenPort_Process()` 已调用 `CO_process_RPDO()`，因此不需要手写 RPDO 解析代码。可通过 SDO 读取 `0x2001:00` 验证接收结果：节点 A 请求 ID 为 `0x601`，数据为 `40 01 20 00 00 00 00 00`。
