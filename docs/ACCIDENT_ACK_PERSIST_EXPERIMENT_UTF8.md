# 事故事件 ACK、超时重发与掉电保护

## 报文约定

- B 节点事故事件 TPDO3：标准 CAN-ID `0x382`，Classic CAN 8 字节。
- 字节 0：事件类型，`1` 为碰撞，`2` 为翻车。
- 字节 1：事件标志，当前为 `0`。
- 字节 2~3：事件编号 `event_id`，小端 16 位。
- 字节 4~5：加速度峰值，单位 mg。
- 字节 6~7：角速度峰值，单位 dps。
- A 节点 ACK：标准 CAN-ID `0x502`，Classic CAN 2 字节。
- ACK 字节 0~1：被确认的 `event_id`，小端 16 位。

## B 节点重发策略

1. B 首次发送事故 TPDO3 后保存待确认快照，并等待 `500 ms`。
2. 未收到相同编号 ACK 时重新发送同一事件编号，重试次数加一。
3. 连续重试达到 `10` 次后，重发间隔降为 `5 s`，避免故障线路长期占满总线。
4. ACK 编号匹配后清除 pending 标志和重试计数，停止该事件重发。
5. ACK 接收在 FDCAN 中断中只进入 FreeRTOS 队列，Flash 擦写在 CAN 处理任务中完成。

## 掉电保护内容

STM32G474RE 最后一个 2 KB Flash 页保存 40 字节记录：

- MQTT 发布周期；
- 已分配的最大事件编号；
- 待确认事件编号、事件类型；
- 加速度和角速度峰值；
- pending 标志和重试次数；
- CRC32 校验值。

B 节点启动时校验记录并恢复事件编号。若 pending 标志有效，CANopen 栈启动后约 100 ms 重新发送事故事件；收到 A ACK 后清除 pending 并再次保存。

## 实验步骤

1. A、B 正常连接，触发 B 的碰撞或翻车事件，PCAN-View 观察 `0x382` 后的 `0x502` ACK。
2. 临时断开 A 或拔掉 CANH/CANL，使 B 在 `500 ms` 后出现重复 `0x382`。
3. 记录 `diagnostics.accidentRetryCount` 和 `diagnostics.accidentAckTimeoutCount`，确认重发间隔由 500 ms 逐步变为 5 s。
4. 在 B 显示 pending 状态时复位或断电，再恢复供电；确认事件编号不回退且仍能重发。
5. 恢复总线后确认 A 回 ACK、B 停止重发，`diagnostics.accidentAckRxCount` 增加。
6. A 侧可观察 `accidentAckTxCount` 与 `accidentAckTxErrorCount`，区分 ACK 已入硬件队列和发送队列溢出。

## 注意事项

- 事件编号在 CAN 报文中传输低 16 位，掉电记录仍保存完整 32 位编号；实际部署应避免低 16 位在未确认期间回绕。
- Flash 每次状态变化都会擦除一个页，实验阶段可接受；量产版本应改为磨损均衡日志或外部 EEPROM/FRAM。
