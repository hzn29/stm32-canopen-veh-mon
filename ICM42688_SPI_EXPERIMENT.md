# ICM-42688 SPI 单节点验证记录

## 工程接入

- 目标 MCU：NUCLEO-G474RE。
- SPI 外设：SPI2，使用 Mode 0（CPOL=0、CPHA=0）。
- SPI2_SCK：PF1。
- SPI2_MISO：PB14。
- SPI2_MOSI：PB15。
- ICM-42688 CS：PB12，低电平有效，由 GPIO 软件控制。
- 传感器供电：3.3 V；传感器 GND 与开发板 GND 共地。

## 驱动行为

- `ICM42688_Init()` 读取 `WHO_AM_I` 寄存器 `0x75`。
- 预期 `WHO_AM_I` 返回值为 `0x47`。
- 初始化执行软件复位，并打开加速度计和陀螺仪低噪声模式。
- 加速度计和陀螺仪配置为默认量程、200 Hz 输出数据率。
- `StartDefaultTask()` 每 10 ms 读取一次 12 字节六轴原始数据。

## Keil Watch 观察量

- `icm42688Status`：预期为 `HAL_OK`。
- `icm42688WhoAmI`：预期为 `0x47`。
- `icm42688RawData.accelX/Y/Z`：静止时数值基本稳定，翻转开发板时发生变化。
- `icm42688RawData.gyroX/Y/Z`：静止时接近零，转动开发板时发生变化。

## 故障排查

1. `WHO_AM_I` 不是 `0x47` 时，先检查 CS、SCK、MISO、MOSI 是否接反。
2. 检查模块是否要求 3.3 V 供电，以及 VCC/VIO 是否都已供电。
3. 检查 PB12 是否确实连接到模块 CS，且没有被其他外设占用。
4. 若 SPI 读取超时，检查 SPI2 HAL 源文件是否已加入 Keil 工程。
