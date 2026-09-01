# BH-182 UART DMA NMEA 单节点验证记录

## 官方资料

- 产品页：https://www.beitian.com/sys-pd/1669.html
- BH-182 使用 TTL 串口电平，支持 NMEA 和 UBX 输出协议。
- 官方产品页标注默认波特率为 115200、默认更新率为 10 Hz。
- 本工程 USART1 设置为 9600、8N1；只有模块已保存为 9600 且输出 NMEA 时，才能直接解析。

## 当前硬件配置

- USART1_TX：PC4。
- USART1_RX：PC5。
- BH-182_TX 接 PC5。
- BH-182_RX 接 PC4。
- BH-182_GND 与 NUCLEO-G474RE GND 共地。
- 模块供电必须以实物标签和产品版本要求为准，UART 信号必须为 3.3 V TTL 电平。

## 软件实现

- `bh182.c` 使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 启动 DMA 循环接收。
- USART1 IDLE 中断进入 `USART1_IRQHandler()`，再由 HAL 调用 `HAL_UARTEx_RxEventCallback()`。
- 驱动按 `\r\n` 组装完整 NMEA 语句，验证 XOR 校验和。
- 驱动使用 DMA 写指针和软件读指针计算新增字节，支持写指针回绕和整缓冲区满传输。
- DMA 满传输事件通过 `HAL_UARTEx_GetRxEventType()` 识别，读写指针相同时可正确处理完整一圈数据。
- 解析 `GNRMC/GPRMC`：定位有效状态、经纬度和地面速度。
- 解析 `GNGGA/GPGGA`：定位质量、卫星数和海拔高度。

## Keil Watch

- `bh182Status`：预期为 `HAL_OK`，表示 UART DMA 空闲中断接收已启动。
- `bh182Data.fixValid`：有效定位后为 1。
- `bh182Data.latitudeE7`：纬度，除以 10000000 得到度。
- `bh182Data.longitudeE7`：经度，除以 10000000 得到度。
- `bh182Data.altitudeMm`：海拔，除以 1000 得到米。
- `bh182Data.speedMmps`：速度，除以 1000 得到米每秒。
- `bh182Data.satelliteCount`：参与定位卫星数量。
- `bh182Data.sentenceCount`：成功解析 NMEA 语句数量，应持续增加。
- `bh182Data.checksumErrorCount`：校验错误数，稳定后不应持续增加。
- `bh182Data.parseErrorCount`：格式错误数，稳定后不应持续增加。

## 排查顺序

1. 蓝色 TX 指示灯闪烁，说明模块正在输出数据。
2. `sentenceCount` 不变时，先互换 TX/RX，再确认 UART 波特率。
3. `checksumErrorCount` 持续增加时，检查波特率、电平和地线。
4. `sentenceCount` 增加但 `fixValid` 为 0 时，将天线移到开阔环境并等待首次定位。
5. 看到二进制乱码或没有 NMEA 语句时，用北天/UBX 配置工具切换为 NMEA 输出并保存 9600 波特率。
