# ESP32-12F + USART2 + OneNET MQTT 执行规划

## 1. 硬件连接

| NUCLEO-G474RE | ESP32-12F | 说明 |
|---|---|---|
| PA2 / USART2_TX | U0RXD / GPIO3 | STM32 发送给 ESP32 |
| PA3 / USART2_RX | U0TXD / GPIO1 | ESP32 返回 AT 响应 |
| GND | GND | 必须共地 |
| 3.3 V 稳压电源 | VCC | ESP32 发射峰值电流建议预留 500 mA 以上 |
| 3.3 V | EN/CH_PD | 拉高使能模块 |

USART2 当前配置为 115200、8N1、无硬件流控。ESP32-12F 的 UART IO 不能接 5 V 电平；不要把 NUCLEO 的 5 V 直接接到 ESP32 VCC 或 IO。

注意模块名称：市场上常见的 `ESP-12F` 实际为 ESP8266 模块，不等同于 ESP32-WROOM。烧录前先在串口终端发送 `AT+GMR`，确认固件是 Espressif ESP-AT，并确认支持 `AT+MQTTUSERCFG`、`AT+MQTTCONN` 和 `AT+MQTTPUB`；若是空白模块或非 ESP-AT 固件，需要先刷写对应固件。

NUCLEO-G474RE 的 `PA2/PA3` 通常还连接到板载 ST-LINK 虚拟串口。外接 ESP32 前，按照所用板卡原理图断开对应的 ST-LINK USART2 solder bridge（常见标号为 `SB13/SB14`），避免两个发送端同时驱动 `PA3`。断开后仍可使用 SWD 进行调试和烧录。

## 2. 软件分工

- `esp32_at.c`：单字节 USART2 中断接收、环形缓冲、AT 命令应答匹配、Wi-Fi/MQTT 状态机和 JSON 发布。
- `ESP32_MqttTask`：仅 A 节点创建，低于 CAN 接收任务，避免网络等待阻塞 CANopen 实时处理。
- `OD_RAM`：A 节点 RPDO 收到 B 的 IMU 和 C 的 GNSS 后作为遥测数据源。
- `CanOpenPortDiagnostics_t`：提供 B/C Heartbeat、PDO 新鲜度和健康状态。
- MQTT 下行只接受 `publish`、`status`、`set_period` 三类命令，并通过 `AT+MQTTSUB` 订阅控制主题。
- ESP-AT 的 `+MQTTSUBRECV` 通知先进入解析器，再进入 8 项 FreeRTOS 下行命令队列，由 MQTT 任务执行。

## 3. 连接状态机

```text
OFFLINE
  -> WAIT_CONFIG（配置头仍为占位符）
  -> AT + ATE0 + CWMODE
  -> CWJAP（Wi-Fi）
  -> MQTTUSERCFG
  -> MQTTCONN
  -> MQTTSUB（下行控制主题）
  -> MQTT_CONNECTED
  -> 每 1000 ms 发布 JSON
```

任一命令返回 `ERROR` 或超时，状态进入 `ERROR`，等待 5 s 后重新连接。发布失败不会重启 MCU，只会触发 ESP32 MQTT 重连。

## 4. OneNET 参数填写

编辑 `Core/Inc/esp32_mqtt_config.h`：

1. 填写 Wi-Fi SSID 和密码。
2. 按 OneNET 产品的 MQTT 接入文档填写服务域名、端口、客户端 ID、用户名、密码和发布主题。
3. 确认主题和鉴权字符串与 OneNET 产品类型匹配。
4. 最后将 `ESP32_MQTT_CONFIGURED` 改为 `1U`，再编译 A 固件。

凭据只放在本地配置头中，不要提交到公共仓库。当前配置默认使用 ESP-AT MQTT TLS scheme 2 和 8883 端口，数据链路已加密但不校验服务器证书。生产环境应将 `ESP32_MQTT_TLS_VERIFY_SERVER` 设为 `1U`，并按 ESP-AT 文档将 OneNET CA 证书写入 ESP-12F，再使用 scheme 3；证书未写入前不能开启校验。

## 5. 建议执行顺序

1. 不填真实参数，先烧录 A，确认 `esp32Status.state` 为 `ESP32_STATE_WAIT_CONFIG`，CANopen 仍正常收发。
2. 用 USB-TTL 或逻辑分析仪确认 USART2 端序、115200 波特率和 ESP32 上电输出。
3. 填写真实参数并将配置开关设为 1，烧录 A。
4. 用调试器观察 `commandCount`、`commandErrorCount`、`reconnectCount` 和 `publishCount`。
5. 在 OneNET 控制台确认每秒收到一条 JSON，并检查 `health.b`、`health.c` 与本地诊断一致。
6. 断开 B 或 C，确认对应健康位在 CANopen 超时后变为 0，云端仍能收到带故障状态的消息。
7. 恢复节点，确认 PDO 和 Heartbeat 恢复后健康位重新变为 1。

## 6. 当前 JSON 字段

```json
{
  "ts": 0,
  "imu": {"ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "valid": 0},
  "gnss": {"lat_e7": 0, "lon_e7": 0, "alt_mm": 0, "speed_mmps": 0, "sat": 0, "fix": 0},
  "health": {"b": 0, "c": 0}
}
```

## 7. MQTT 下行命令

OneNET 向 `ESP32_MQTT_SUB_TOPIC` 发布以下 JSON：

```json
{"cmd":"publish"}
```

立即发布一条当前遥测数据。

```json
{"cmd":"status"}
```

立即发布一条包含传感器和 B/C 健康状态的遥测数据。

```json
{"cmd":"set_period","value":5000}
```

把周期修改为 5000 ms。允许范围为 100～60000 ms；范围外、格式错误或未知命令都会增加 `downlinkRejectedCount`，不会执行任何 AT 命令。

收到下行时，ESP-AT 通常返回类似以下异步通知，实际主题和长度由模块填写：

```text
+MQTTSUBRECV:0,"YOUR_SUBSCRIBE_TOPIC",36,{"cmd":"set_period","value":5000}
```

调试器可观察：

```c
esp32Status.downlinkCount
esp32Status.downlinkAcceptedCount
esp32Status.downlinkRejectedCount
esp32Status.currentPublishPeriodMs
```

云端接入稳定后，再增加 MQTT 下行订阅，用于远程设置发布周期、故障阈值或请求设备重启；下行命令必须经过白名单校验，不能直接执行任意 AT 字符串。

## 8. 下行命令应答与可靠性测试

下行命令可以携带十进制 `cmd_id`，例如：

```json
{"cmd_id":1001,"cmd":"publish"}
```

设备会向 `ESP32_MQTT_ACK_TOPIC` 发布应答。应答格式如下：

```json
{"cmd_id":1001,"result":"accepted","error":0,"ts":123456}
{"cmd_id":1001,"result":"executed","error":0,"ts":123460}
```

`accepted` 表示命令通过白名单并进入 FreeRTOS 队列，`executed` 表示应用任务已经执行，`rejected` 表示格式、白名单或队列检查失败，`error` 为非零时表示执行错误。

## 9. 参数掉电保存与看门狗

`set_period` 执行成功后会把发布周期写入 STM32G474RE Flash 最后一页 `0x0807F800`，记录带 magic、版本和 CRC32。链接文件已经把应用 Flash 上限限制到 `0x0807F800`，因此该页不会被程序镜像覆盖。启动时读取有效记录，读取失败则回退到 `ESP32_MQTT_PUBLISH_PERIOD_MS`。

独立看门狗使用 LSI、64 分频和 3999 重载值，实际超时时间约 8 秒。高优先级 `IwdgMonitorTask` 每 500 ms 检查默认应用任务心跳并刷新 IWDG；应用任务停止运行超过超时时间时不再喂狗，由硬件自动复位。验证时可在调试器暂停默认任务，约 8 秒后观察 MCU 复位。

## 10. SSD1306 OLED 仪表盘

### 10.1 初始化发送修正

SSD1306 的 I2C 命令流必须以控制字节 `0x00` 开头。驱动初始化时会先发送该控制字节，再发送初始化命令序列；屏幕刷新数据仍使用 `0x40` 控制字节。该修正确保 OLED 上电后初始化流程符合 SSD1306 I2C 协议。

### 10.2 IMU 意外检测上报

A 节点使用 CANopen 汇总后的 B 节点 ICM-42688 数据执行意外检测。当前默认量程为加速度 +/-16 g、陀螺仪 +/-2000 dps：合成加速度达到 6000 mg 的单个采样点即可记录碰撞，避免漏掉短脉冲冲击；Z 轴重力分量低于 500 mg 且水平加速度超过 750 mg，或 Z 轴反向重力超过 700 mg，并持续确认 500 ms 时记录疑似翻车。事件确认后保持 10 s，便于 MQTT 上报；累计次数和峰值不会因保持窗口结束而清零。

MQTT 遥测 JSON 增加 `accident` 对象：

```json
"accident":{"active":1,"type":1,"name":"COLLISION","event_id":1,"collision_count":1,"rollover_count":0,"peak_accel_mg":6000,"peak_gyro_dps":320,"last_event_ms":123456}
```

其中 `type=1` 表示碰撞，`type=2` 表示翻车，`active=1` 表示仍在事件保持窗口内。阈值应根据车辆安装方向、传感器量程和实车道路试验重新标定，不能直接作为安全气囊或量产安全系统的唯一判据。

意外事件首次确认或事件类型变化时，A 节点调用 `ESP32_RequestImmediatePublish()` 设置即时发布请求；MQTT 任务在自己的上下文中执行发布，避免在 CAN/传感器任务中直接操作 USART。事件结束后恢复普通周期上报。

A 节点使用硬件 I2C1 连接 0.96 寸 128x64 SSD1306：

| OLED 模块 | NUCLEO-G474RE | 说明 |
|---|---|---|
| VCC | 3.3 V | 不建议接 5 V |
| GND | GND | 必须共地 |
| SCL | PB8 / I2C1_SCL | 100 kHz |
| SDA | PB9 / I2C1_SDA | 需要上拉电阻 |

驱动默认使用 7 位地址 `0x3C`（HAL 地址为 `0x78`）。A 节点 `OledDashboardTask` 每 200 ms 刷新一次，显示速度、GNSS 定位、卫星数、纬度、经度、IMU 六轴数据、B/C 健康状态和告警位 `AL`。B、C 节点不创建 OLED 任务。

OLED 现支持数据页和故障页。无告警时两页每 3 秒轮换一次；任意告警位有效时立即切换到故障页，显示 IMU、GNSS、Node B、Node C、MQTT 状态、告警位以及 B/C Heartbeat 累计超时次数。告警位清除后仍保留故障页约 3 秒，再恢复页面轮换。

## 11. 车辆告警状态机

`vehicle_alarm.c` 统一生成 OLED、MQTT 和调试器使用的告警状态。状态机每 100 ms 更新一次：

| 状态 | 条件 |
|---|---|
| `NORMAL` | 所有原始异常均已连续消失 5 个周期，或系统刚启动。 |
| `WARNING` | IMU、GNSS 或 MQTT 异常连续出现 3 个周期。 |
| `FAULT` | B/C 节点超时或 FDCAN Bus-Off，立即进入。 |
| `RECOVERING` | 非正常状态的所有原始异常已经消失，正在进行 5 个周期恢复确认。 |

云端 `alarm` 字段包含 `state`、已确认的 `active` 位图和实时的 `raw` 位图。Keil Watch 可查看 `vehicleAlarmStatus` 的状态、确认计数和状态转换次数。

可靠性测试观察以下字段：

```c
esp32Status.downlinkCount
esp32Status.downlinkAcceptedCount
esp32Status.downlinkRejectedCount
esp32Status.downlinkExecutedCount
esp32Status.downlinkAckCount
esp32Status.downlinkAckErrorCount
esp32Status.downlinkQueueFullCount
esp32Status.lastDownlinkLatencyMs
esp32Status.maxDownlinkLatencyMs
esp32Status.downlinkTestPassCount
esp32Status.downlinkTestFailCount
```

调试器中调用 `ESP32_RunDownlinkReliabilityTest()` 可注入两条合法命令和一条非法命令。预期接收数增加 3，接受数至少增加 2，拒绝数至少增加 1；真实 MQTT 连接建立后还应在应答主题看到 `accepted` 和 `executed` 两条结果。
# B 节点事故事件链路

## CANopen 事件帧

- B 节点使用 TPDO3 的标准 COB-ID `0x382` 发送事故事件。
- 数据长度固定为 8 字节：`type`、保留字节、16 位 `event_id`、16 位峰值加速度 mg、16 位峰值角速度 dps。
- `type=1` 表示碰撞，`type=2` 表示翻车；事件编号从 1 递增，重启后重新开始。
- B 节点仅在新事件确认时发送一次 TPDO，并调用 `CO_errorReport()` 产生厂商 EMCY；保持窗口结束后调用 `CO_errorReset()`。
- B 节点以 1 ms 周期读取 ICM-42688；碰撞单点确认，翻车必须连续满足姿态条件约 500 ms 才确认。

## A 节点可靠接收与上报

- FDCAN 接收回调只把 `0x382` 原始数据放入 FreeRTOS 队列，不在中断中执行检测或 UART 操作。
- A 节点默认任务按 `event_id` 去重，将有效事件应用到本地状态并放入 MQTT 事件队列。
- MQTT 离线或发布失败时事件保留在队首，下一次连接后重试；只有 ESP-AT 返回成功才出队。
- 事故事件使用独立主题 `ESP32_MQTT_ACCIDENT_TOPIC`，云端应按 `event_id` 做幂等处理。

## 建议验证步骤

1. 仅给 B 节点施加超过 6 g 的短冲击，观察 B 的 `accidentEventTxCount`、A 的 `accidentEventRxCount` 和 OLED 事故页。
2. 使用 PCAN-View 确认 `0x382` 数据字节 0 为 `01`，字节 2、3 为递增事件编号。
3. 暂停 ESP32 MQTT 连接，确认 A 的 `accidentEventQueuedCount` 增加而 `accidentEventSentCount` 不变；恢复连接后确认队列补发并计数增加。
4. 重复发送同一 `event_id` 的 `0x382` 帧，确认 A 的 `accidentEventDuplicateCount` 增加且不会产生第二条 MQTT 事件。
5. 观察 B 的厂商 EMCY 错误位和保持窗口结束后的 EMCY 清除报文。
## Implementation update

The current code defaults to TLS server certificate verification (`ESP32_MQTT_TLS_VERIFY_SERVER=1U`) and refuses to connect until `ESP32_MQTT_CA_CERT_CONFIGURED=1U`. The accident event and ACK protocols now carry a full 32-bit event ID; see `README.md` and `ACCIDENT_ACK_PERSIST_EXPERIMENT.md` for the authoritative frame layout.
