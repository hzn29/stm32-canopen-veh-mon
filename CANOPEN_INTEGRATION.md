# CANopenNode 集成记录

本工程当前配置为 STM32G474RE、FDCAN1、500 kbit/s Classic CAN，CANopen Node-ID 由 `main.c` 中的 A/B/C 编译宏选择：A=1、B=2、C=3。

已接入的运行链路：

- `CanOpenPort_StackInit()` 创建并初始化 CANopenNode 顶层对象。
- `CanOpenPort_RxInterrupt()` 将 STM32 FDCAN FIFO0 报文转换为 `CO_CANrxMsg_t`。
- `CanOpenPort_Process(1000U)` 由 FreeRTOS 任务每 1 ms 调用。
- Keil 工程加入 CANopenNode 的 CANopen、NMT、Heartbeat、EMCY、SDO、SYNC、PDO、LED 和 LSS 源文件。

首次烧录后，使用 PCAN-View 观察启动报文和心跳报文：节点 A 为 `0x701`，节点 B 为 `0x702`，节点 C 为 `0x703`。心跳周期由对象字典 `0x1017` 决定，后续可通过 SDO 配置。
