# BH-182 CANopen TPDO 实验

## 节点分工

- 节点 A（Node-ID 1）：接收 GNSS TPDO，负责仪表盘和故障管理。
- 节点 B（Node-ID 2）：保留 ICM-42688 数据链路。
- 节点 C（Node-ID 3）：采集 BH-182，并发送 TPDO1/TPDO2。

## 对象字典

| 对象 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| 0x2100:00 | int32 | 1e-7 度 | 纬度 |
| 0x2101:00 | int32 | 1e-7 度 | 经度 |
| 0x2102:00 | int32 | mm | 海拔 |
| 0x2103:00 | uint16 | mm/s | 地面速度 |
| 0x2104:00 | uint8 | 个 | 卫星数量 |
| 0x2105:00 | uint8 | 0/1 | 定位有效 |

## TPDO 映射

- C TPDO1：COB-ID `0x183`，周期 100 ms，数据为纬度（字节 0~3）和经度（字节 4~7）。
- C TPDO2：COB-ID `0x283`，周期 100 ms，数据为海拔（字节 0~3）、速度（字节 4~5）、卫星数量（字节 6）和定位有效（字节 7）。

## 烧录与验证

1. 将 `CAN_NODE_ROLE` 设置为 `CAN_NODE_ROLE_C`，编译并烧录节点 C。
2. 将 `CAN_NODE_ROLE` 设置为 `CAN_NODE_ROLE_A`，编译并烧录节点 A。
3. 两块板接入同一 CAN 总线并使用 120 ohm 终端。
4. 在 PCAN-View 观察 `0x183` 和 `0x283`，预期约每 100 ms 各出现一帧。
5. 定位无效时，`0x2105` 对应的最后一个字节为 0；获得有效定位后为 1。
6. 按小端格式将 4 字节有符号数据还原为对象值，并与 A 节点对象字典或调试变量比较。

## 注意事项

- Classic CAN 单帧最多 8 字节，因此 GNSS 数据拆分为两个 TPDO。
- BH-182 必须实际输出 NMEA，且串口波特率必须与工程配置一致。
- 本次映射在编译期按节点角色选择；重新生成 OD 文件会覆盖手工修改。

## B 节点 ICM-42688 TPDO

- B TPDO1：CAN-ID `0x182`，周期 10 ms，依次发送 `accelX`、`accelY`、`accelZ`（各 16 位）、有效标志和 `0x2117` 的 8 位序号。
- B TPDO2：CAN-ID `0x282`，周期 10 ms，依次发送 `gyroX`、`gyroY`、`gyroZ`（各 16 位）和 `0x2118` 的 16 位序号。
- A RPDO3/RPDO4 分别接收上述两帧，数据写入 `0x2110~0x2118`。
- `0x2117` 和 `0x2118` 是两个独立的序号对象，分别服务于 TPDO1 和 TPDO2，不再使用同一个对象的不同位宽映射。

## A 节点数据汇总与故障管理

- A 分别记录 `0x182`、`0x282`、`0x183`、`0x283` 的最近接收时间和帧数。
- B 的两类 IMU 帧超过 30 ms 未更新时，`diagnostics.imuDataValid` 变为 0。
- C 的两类 GNSS 帧超过 300 ms 未更新时，`diagnostics.gnssDataValid` 变为 0。
- `diagnostics.nodeBHealthy` 和 `diagnostics.nodeCHealthy` 同时参考 Heartbeat 与应用 PDO 新鲜度。
- A 节点在 Heartbeat 正常但应用 PDO 超时时暂缓回到 Operational，数据恢复后再自动恢复。
- IMU 对象单位为传感器原始 ADC 计数，后续可在 A 节点按量程换算为物理单位。
