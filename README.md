# STM32 CANopen Vehicle Monitor

STM32G474RE multi-node vehicle monitoring prototype using native FreeRTOS, CANopenNode, ICM-42688 IMU, BH-182 GNSS, SSD1306 OLED, and an ESP-12F ESP-AT MQTT gateway.

This repository is a laboratory prototype. It is not automotive safety-certified and must not be used as a production restraint, braking, or emergency-control system.

## Features

- Node A (Node-ID 1): CANopen aggregation, B/C heartbeat supervision, EMCY/NMT policy, OLED status, and MQTT upload.
- Node B (Node-ID 2): ICM-42688 sampling, local collision/rollover detection, accident TPDO/EMCY, ACK retry, and persistent pending events.
- Node C (Node-ID 3): BH-182 UART DMA/IDLE NMEA reception and GNSS TPDO publication.
- CANopen driver: STM32 FDCAN RX messages are copied into a driver-owned context and dispatched by `CO_CANinterrupt()`; no template receive path is used.
- Persistence: two-page append journal with sequence number and CRC32. A page is erased only when the active page is full.
- TLS: server certificate verification is enabled by default. A CA certificate must be provisioned in the ESP-AT module before enabling MQTT credentials.
- Bus-Off recovery: the first three consecutive events use a 100 ms automatic restart, events 4-10 use exponential backoff (200 ms to 12.8 s), and a persistent failure after the tenth retry locks recovery and keeps the CAN fault alarm active until reset.

## Hardware

| Item | Quantity | Function |
| --- | ---: | --- |
| NUCLEO-G474RE | 3 | A/B/C MCU nodes |
| TCAN1042HGV CAN FD module | 3 | CAN transceivers |
| PCAN-USB FD | 1 | Bus monitor |
| ICM-42688 | 1 | Node B IMU |
| BH-182 | 1 | Node C GNSS |
| ESP-12F | 1 | Node A MQTT gateway |
| SSD1306 128x64 | 1 | Node A display |

Pin assignments are documented in `can00.ioc` and the experiment notes. Install 120 ohm termination at both physical bus ends; a powered-down resistance measurement should be about 60 ohm between CANH and CANL.

## Build

The verified build entry is Keil MDK-ARM:

1. Open `MDK-ARM/can00.uvprojx`.
2. Select one of the independent targets: `NodeA`, `NodeB`, or `NodeC`.
3. Build and flash the selected target. Each target fixes its own `CAN_NODE_ROLE_*` define and output directory.

`can00.ioc` is retained for STM32CubeMX/CubeIDE peripheral reference and regeneration. A CubeIDE `.project`/`.cproject` build project is not included; do not describe CubeIDE as a verified build path until those files are added.

## Accident protocol

The B-to-A accident event uses CANopen TPDO3 (`0x382`) with eight Classic CAN data bytes:

| Byte(s) | Meaning |
| --- | --- |
| 0 | Event type: `1` collision, `2` rollover |
| 1 | Event flags |
| 2..5 | Full 32-bit little-endian event ID |
| 6 | Peak acceleration in 100 mg units, saturated at 255 |
| 7 | Peak angular rate in 10 dps units, saturated at 255 |

A acknowledges the event on `0x502` with a four-byte little-endian event ID. B retries after 500 ms and changes to a 5 s retry period after ten unsuccessful retries. Duplicate events are acknowledged but not processed twice.

## MQTT/TLS configuration

Copy the values into `Core/Inc/esp32_mqtt_config.h` locally and never commit credentials, tokens, or certificates. Set `ESP32_MQTT_CONFIGURED` to `1U` only after provisioning the OneNET CA certificate in the ESP-AT module and setting `ESP32_MQTT_CA_CERT_CONFIGURED` to `1U`. The code fails closed when certificate verification is enabled but the CA is not configured.

## Third-party licenses

See `THIRD_PARTY_NOTICES.md`. New application code is released under MIT in `LICENSE`; CANopenNode, FreeRTOS, STM32 HAL, and CMSIS retain their original licenses.

## Status

The project is suitable for reproducible laboratory experiments and protocol research. Hardware-in-the-loop tests, fault injection, and conformance testing are still required before any safety-related claim.
