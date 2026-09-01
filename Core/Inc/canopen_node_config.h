/* 定义三块开发板共用的 CANopen 节点编译配置。 */
#ifndef CANOPEN_NODE_CONFIG_H
#define CANOPEN_NODE_CONFIG_H

/* 引入标准整数类型，保证编码宏中的 uint32_t 在所有编译单元中可用。 */
#include <stdint.h>

/* 定义节点 A 的角色编号。 */
#define CAN_NODE_ROLE_A 0U
/* 定义节点 B 的角色编号。 */
#define CAN_NODE_ROLE_B 1U
/* 定义节点 C 的角色编号。 */
#define CAN_NODE_ROLE_C 2U

/* 定义所有节点发送自身 Heartbeat 的周期，单位为毫秒。 */
#define CANOPEN_HEARTBEAT_PRODUCER_PERIOD_MS 1000U
/* 定义节点 A 监控 B、C 时允许的最大 Heartbeat 间隔，单位为毫秒。 */
#define CANOPEN_HEARTBEAT_CONSUMER_TIMEOUT_MS 1500U
/* 累计出现三次独立 Heartbeat 超时后，将远端节点健康状态提升为 FAULT。 */
#define CANOPEN_HEALTH_TIMEOUT_EPISODE_LIMIT 3U
/* 进入超时后超过 5000 ms 仍未恢复，判定为恢复失败。 */
#define CANOPEN_HEALTH_RECOVERY_TIMEOUT_MS 5000U
/* 收到 Heartbeat 后保持 2000 ms 稳定，才将 RECOVERING 标记为 ACTIVE。 */
#define CANOPEN_HEALTH_STABLE_TIME_MS 2000U
/* A 节点判定 B 节点 IMU TPDO 数据失效的超时时间，单位为毫秒。 */
#define CANOPEN_IMU_PDO_TIMEOUT_MS 30U
/* A 节点判定 C 节点 GNSS TPDO 数据失效的超时时间，单位为毫秒。 */
#define CANOPEN_GNSS_PDO_TIMEOUT_MS 300U
/* 定义 B 等待 A 节点事故确认的最长时间，超时后重发同一事件 TPDO。 */
#define CANOPEN_ACCIDENT_ACK_TIMEOUT_MS 500U
/* 定义连续未确认次数达到该值后的告警门限，事件仍会继续低频重发。 */
#define CANOPEN_ACCIDENT_ACK_RETRY_WARNING_LIMIT 10U
/* 定义达到未确认告警门限后的重发间隔，降低长期断链时的总线占用。 */
#define CANOPEN_ACCIDENT_ACK_DEGRADED_RETRY_MS 5000U
/* 按 CANopen 规范编码 0x1016：高 16 位为 Node-ID，低 16 位为超时时间。 */
#define CANOPEN_HEARTBEAT_CONSUMER_ENTRY(node_id, timeout_ms) \
    ((((uint32_t)(node_id)) << 16U) | ((uint32_t)(timeout_ms) & 0xFFFFU)) /* 组合 Node-ID 和超时时间。 */

/* 选择当前编译固件对应的节点角色；烧录 B/C 时修改此宏。 */
#ifndef CAN_NODE_ROLE
#define CAN_NODE_ROLE CAN_NODE_ROLE_A
#endif

/* 将节点角色转换为 CANopen Node-ID。 */
#if (CAN_NODE_ROLE == CAN_NODE_ROLE_A)
#define CANOPEN_NODE_ID 1U
#elif (CAN_NODE_ROLE == CAN_NODE_ROLE_B)
#define CANOPEN_NODE_ID 2U
#elif (CAN_NODE_ROLE == CAN_NODE_ROLE_C)
#define CANOPEN_NODE_ID 3U
#else
#error "CAN_NODE_ROLE 必须设置为 CAN_NODE_ROLE_A、CAN_NODE_ROLE_B 或 CAN_NODE_ROLE_C"
#endif

#endif /* CANOPEN_NODE_CONFIG_H */
