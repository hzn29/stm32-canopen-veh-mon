/* ESP32-12F 与 OneNET MQTT 参数配置。 */
#ifndef ESP32_MQTT_CONFIG_H
#define ESP32_MQTT_CONFIG_H

/* 配置为 1 后才会执行真实 Wi-Fi 和 MQTT 连接。 */
#define ESP32_MQTT_CONFIGURED 0U

/* 填写现场 Wi-Fi 名称。 */
#define ESP32_WIFI_SSID "YOUR_WIFI_SSID"
/* 填写现场 Wi-Fi 密码。 */
#define ESP32_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

/* 填写 OneNET MQTT 服务域名或 IP 地址。 */
#define ESP32_MQTT_HOST "YOUR_ONENET_MQTT_HOST"
/* 启用 MQTT TLS 加密，生产环境建议保持为 1。 */
#define ESP32_MQTT_TLS_ENABLED 1U
/* TLS 证书校验开关：0 加密但不校验服务端证书，1 校验 CA 证书。 */
#define ESP32_MQTT_TLS_VERIFY_SERVER 0U
/* 根据 TLS 和证书校验选项选择 ESP-AT MQTT scheme。 */
#if (ESP32_MQTT_TLS_ENABLED != 0U)
  #if (ESP32_MQTT_TLS_VERIFY_SERVER != 0U)
    /* scheme 3 表示 MQTT over TLS 并校验服务器证书。 */
    #define ESP32_MQTT_SCHEME 3U
  #else
    /* scheme 2 表示 MQTT over TLS，但不校验服务器证书。 */
    #define ESP32_MQTT_SCHEME 2U
  #endif
  /* TLS MQTT 默认端口为 8883。 */
  #define ESP32_MQTT_PORT 8883U
#else
  /* scheme 1 表示明文 MQTT over TCP。 */
  #define ESP32_MQTT_SCHEME 1U
  /* 明文 MQTT 默认端口为 1883。 */
  #define ESP32_MQTT_PORT 1883U
#endif
/* 填写 OneNET 产品要求的客户端 ID。 */
#define ESP32_MQTT_CLIENT_ID "YOUR_CLIENT_ID"
/* 填写 OneNET 产品要求的用户名。 */
#define ESP32_MQTT_USERNAME "YOUR_USERNAME"
/* 填写 OneNET 产品要求的密码或鉴权字符串。 */
#define ESP32_MQTT_PASSWORD "YOUR_MQTT_PASSWORD"
/* 填写 OneNET 属性上报主题。 */
#define ESP32_MQTT_PUB_TOPIC "YOUR_PUBLISH_TOPIC"
/* 填写 OneNET 下行控制主题，建议与上报主题分开。 */
#define ESP32_MQTT_SUB_TOPIC "YOUR_SUBSCRIBE_TOPIC"
/* 填写 OneNET 命令应答主题，建议与下行控制主题分开。 */
#define ESP32_MQTT_ACK_TOPIC "YOUR_ACK_TOPIC"
/* 配置事故事件专用 MQTT 主题，云端按 event_id 做幂等去重。 */
#define ESP32_MQTT_ACCIDENT_TOPIC "YOUR_ACCIDENT_TOPIC"

/* 配置 ESP-AT MQTT keepalive 秒数。 */
#define ESP32_MQTT_KEEPALIVE_SEC 60U
/* 配置遥测数据的发布周期。 */
#define ESP32_MQTT_PUBLISH_PERIOD_MS 1000U
/* 配置联网失败后的重试间隔。 */
#define ESP32_MQTT_RETRY_PERIOD_MS 5000U
/* 配置独立看门狗超时时间，单位为毫秒。 */
#define ESP32_IWDG_TIMEOUT_MS 8000U
/* 配置看门狗任务刷新周期，必须明显小于超时时间。 */
#define ESP32_IWDG_REFRESH_PERIOD_MS 500U

#endif /* ESP32_MQTT_CONFIG_H */
