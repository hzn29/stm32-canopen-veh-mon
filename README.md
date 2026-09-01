# STM32 CANopen Vehicle Monitor

基于 STM32G474RE、原生 FreeRTOS 和 CANopenNode 的多节点车载数据采集与事故监测实验平台。

本项目用于研究 STM32 + RTOS 下的 CAN/CAN FD 多节点通信、TPDO/RPDO 调度、Heartbeat 故障监控、事故事件可靠传输和物联网网关接入。它是实验室原型，不是经过汽车功能安全、EMC 或道路法规认证的量产产品。

## 功能概览

- A 节点（Node-ID 1）：汇总 B/C 数据、OLED 仪表显示、Heartbeat 监控、EMCY/NMT 故障管理和 MQTT 上报。
- B 节点（Node-ID 2）：ICM-42688 六轴 IMU 采集、本地碰撞/翻车检测、事故 TPDO/EMCY、ACK 超时重发。
- C 节点（Node-ID 3）：BH-182 GNSS UART DMA 接收、NMEA 解析和 GNSS TPDO 发布。
- A/B 事故可靠传输：B 使用 `0x382` 发布事故事件，A 使用 `0x502` 返回事件编号 ACK；未确认时 500 ms 重发，连续 10 次后降为 5 s。
- 掉电保护：B 将事件编号、待确认状态、峰值数据和重试次数写入 STM32G4 最后一页 Flash，并使用 CRC32 校验；重启后恢复 pending 事件。
- ESP32-12F：通过 USART2 使用 ESP-AT 连接 MQTT/OneNET，支持 TLS 配置、下行命令应答和事故可靠上报。

## 硬件

| 部件 | 数量 | 用途 |
| --- | ---: | --- |
| NUCLEO-G474RE | 3 | A/B/C 节点 MCU |
| TCAN1042HGV CAN FD 模块 | 3 | CAN 收发器 |
| PCAN-USB FD | 1 | 总线分析与报文记录 |
| ICM-42688 | 1 | B 节点 IMU |
| 北天 BH-182 | 1 | C 节点 GNSS |
| ESP32-12F | 1 | A 节点 MQTT 网关 |
| SSD1306 128x64 OLED | 1 | A 节点显示 |

### 关键引脚

- FDCAN1：`PA11 = RX`，`PA12 = TX`。
- SPI2 / ICM-42688：`PF1 = SCK`，`PB14 = MISO`，`PB15 = MOSI`，`PB12 = CS`。
- USART1 / BH-182：`PC4 = TX`，`PC5 = RX`，使用 DMA + IDLE 接收。
- USART2 / ESP32-12F：`PA2 = TX`，`PA3 = RX`。
- I2C1 / SSD1306：`PB8 = SCL`，`PB9 = SDA`。

CAN 总线两端各安装一个 120 ohm 终端电阻，断电测量 CANH-CANL 应约为 60 ohm。TCAN1042 的 VCC、VIO、STB 必须按模块原理图确认，所有节点共地。

## 软件环境

- STM32CubeIDE 和 STM32CubeG4
- Keil MDK-ARM（工程：`MDK-ARM/can00.uvprojx`）
- STM32 HAL
- 原生 FreeRTOS（`Middlewares/Third_Party/FreeRTOS`）
- CANopenNode（`Middlewares/CANopenNode`）
- PCAN-View（用于总线观测）

## 快速开始

1. 打开 `can00.ioc`，确认 FDCAN1 为 500 kbit/s Classic CAN，SPI2、USART1 DMA/IDLE、USART2 和 I2C1 配置与硬件一致。
2. 在 `Core/Inc/canopen_node_config.h` 选择节点角色：
   - `CAN_NODE_ROLE_A` 编译 A 节点；
   - `CAN_NODE_ROLE_B` 编译 B 节点；
   - `CAN_NODE_ROLE_C` 编译 C 节点。
3. 如需启用 OneNET，复制并编辑 `Core/Inc/esp32_mqtt_config.h` 中的 Wi-Fi、MQTT 主机、客户端和主题参数，并将 `ESP32_MQTT_CONFIGURED` 改为 `1U`。不要把真实密码、Token 或证书提交到公开仓库。
4. 用 Keil 打开 `MDK-ARM/can00.uvprojx`，分别编译并烧录三块 NUCLEO-G474RE。
5. 连接 CANH、CANL、GND 和两端终端电阻，用 PCAN-View 设置 500 kbit/s 观察 `0x701`、`0x702`、`0x703` Heartbeat。
6. 按实验文档逐项验证 TPDO/RPDO、GNSS、IMU、Heartbeat、EMCY、事故 ACK/重发、掉电恢复和 MQTT。

## 事故事件协议

### B -> A：事故 TPDO3（`0x382`）

| 字节 | 内容 |
| ---: | --- |
| 0 | 事件类型：`1` 碰撞，`2` 翻车 |
| 1 | 事件标志 |
| 2-3 | 16 位小端事件编号 |
| 4-5 | 加速度峰值，mg |
| 6-7 | 角速度峰值，dps |

### A -> B：事故 ACK（`0x502`）

ACK 为 2 字节 Classic CAN 报文，载荷是被确认事件编号的 16 位小端表示。B 只接受与当前 pending 事件匹配的 ACK。

## 目录说明

- `Core/Inc`、`Core/Src`：应用代码、驱动、CANopen 适配层和对象字典。
- `Middlewares/CANopenNode`：CANopenNode 协议栈及其 Apache License 2.0 文件。
- `Middlewares/Third_Party/FreeRTOS`：FreeRTOS 源码及其许可文件。
- `Drivers`：STM32G4 CMSIS/HAL 驱动。
- `MDK-ARM`：Keil 工程、启动文件和链接脚本。
- `*_EXPERIMENT.md`：硬件接线、实验步骤和观测指标。

## 已知限制

- 事故编号在 CAN 报文中传输低 16 位，长期运行时应设计回绕处理。
- 当前 Flash 保存策略每次状态变化擦除一个页，适合实验验证；量产应使用磨损均衡日志、EEPROM 或 FRAM。
- MQTT 参数和证书需要用户自行配置，仓库不包含任何云平台凭据。
- 尚未声明汽车级安全等级，也未替代安全气囊、EDR 或法规要求的事故记录设备。

## 开源许可

本项目新增应用代码以 MIT License 发布，第三方组件继续遵循其原始许可证。详见 `LICENSE` 和 `THIRD_PARTY_NOTICES.md`。

## 贡献

欢迎提交 Issue 和 Pull Request。提交前请确认：代码不包含个人凭据，新增代码有中文注释，硬件变化同步更新 `can00.ioc` 和实验文档，并完成至少一次三节点回归测试。
