# CANopenNode 接入状态

## 已完成

- 已将官方 CANopenNode 源码放入 `Middlewares/CANopenNode`。
- 已复制官方对象字典示例 `OD.c`、`OD.h` 到 `Core/Src` 和 `Core/Inc`。
- 已建立 `CO_driver_target.h`，定义 STM32 目标类型和 Classic CAN 接收报文访问宏。
- 已建立 `CO_driver.c`，将 CANopenNode 的发送接口映射到 `HAL_FDCAN_AddMessageToTxFifoQ()`。
- 已建立 `canopen_port.c/.h`，提供 `CanOpenPort_Init(Node-ID)` 接口。
- 已将 CubeMX 和 `main.c` 改为 500 kbit/s Classic CAN、8 字节数据帧。
- 已将 A、B、C 映射为 CANopen Node-ID 1、2、3。
- 已为 Keil 工程增加 CANopenNode 头文件路径和底层源文件。
- 已禁用旧版自定义时间同步帧，为 CANopen SYNC COB-ID `0x080` 保留编号。

## 当前已启用

- CANopenNode 对象初始化已经在 `main.c` 中调用。
- `CO_process()` 已周期处理 NMT、Heartbeat、SDO、EMCY 等异步对象。
- 节点 A 的 `0x1016` 已配置监控 Node-ID 2（B）和 Node-ID 3（C），超时 1500 ms。
- 节点 B、C 的 `0x1016` 保持空配置，仅发送自身 Heartbeat。
- `CO_process_SYNC()`、`CO_process_RPDO()` 和 `CO_process_TPDO()` 已接入默认任务。
- 协议栈源文件和头文件路径已加入 Keil 工程。

## 仍需完善

- 已增加诊断快照，记录 B/C 的 Heartbeat 状态、超时次数、恢复次数和故障恢复时间。
- 已增加 Heartbeat EMCY 错误边沿和清除边沿统计，标准错误码为 `0x8130`。
- 已启用错误清除后自动返回 Operational 的 NMT 控制策略。
- 可选：使用 PCAN-View 记录超时前后的 Heartbeat 与 EMCY 报文，形成实验数据。

## 当前验证边界

本阶段只验证：

```text
Classic CAN 500 kbit/s
FDCAN HAL 底层初始化
CANopenNode 驱动对象初始化入口
CANopenNode 发送接口到 FDCAN TX FIFO 的映射
节点 A 监控 B/C 的 Heartbeat 超时、EMCY 和 NMT 状态恢复
```

下一步可使用 PCAN-View 对 B、C 分别断线，记录 Heartbeat 消失、EMCY 产生、NMT 退回和自动恢复的时间。
