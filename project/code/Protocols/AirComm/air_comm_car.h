/**
 * @file air_comm_car.h
 * @brief 车端空中通信模块（UART 串口协议）
 *
 * 功能：车端 MCU 与遥控端（手机/上位机）通过 UART 进行参数下发、
 *       函数调用、心跳保活和实时数据接收
 *
 * 帧格式（共 9 + payload + 2 = 11~259 字节）：
 *   [0:3]  帧头：0xAA 0xAA 0x55 0x55（四字节定帧头，抗干扰）
 *   [4]     类型：消息类型字节
 *   [5]     序号：帧序号（0~255 循环，用于 ACK 匹配）
 *   [6]     长度：payload 字节数（0~250）
 *   [7:N]   payload：数据载荷
 *   [N+1:N+2] CRC16-CCITT：覆盖帧头到 payload 末尾
 *
 * 消息类型：
 *   0x01 SET_PARAM  - 下发参数（需 ACK）    payload: [name_len][name...][float(4B)]
 *   0x02 ACK_PARAM  - 参数操作确认           payload: [status]
 *   0x03 EXEC_COMMAND - 执行远程命令（需 ACK）payload: [name_len][name...]
 *   0x04 ACK_COMMAND  - 远程命令确认         payload: [ACK文本]
 *   0x05 HEARTBEAT  - 心跳（不需 ACK）      payload: [reserved(2B)][tick_ms(4B)]
 *   0x06 RUN_DATA   - 双向实时数据（无 ACK） payload: [count][float0][float1]...
 *
 * ACK/重试策略：
 *   - SET_PARAM / EXEC_COMMAND 发送后启动 ACK 等待
 *   - ACK等待期间每200ms重发原帧
 *   - 普通操作总等待800ms，指定超时接口可为慢操作扩大窗口
 *   - 达到总等待时间仍无ACK → ACK_RESULT_TIMEOUT
 *   - 收到 ACK 但 status != OK → ACK_RESULT_ERROR
 *
 * 心跳/在线判断：
 *   - 车端每 200ms 发一次心跳
 *   - 收到对方心跳时更新 last_peer_ms
 *   - 超过 600ms 未收到对方心跳 → online_status=2（离线）
 *   - 在线 = online_status=1，离线 = online_status=2，初始 = 0
 *
 * 使用方式：
 *   1. air_comm_car_init() 初始化
 *   2. UART 中断中调 air_comm_car_rx_byte() 喂字节
 *   3. 主循环调 air_comm_car_poll() 解析帧 + 检查 ACK 超时
 *   4. 100Hz 调 air_comm_car_update_100HZ() 发心跳
 *   5. 上层通过 air_comm_car_is_online() 判断连接状态
 */

#ifndef AIR_COMM_CAR_H
#define AIR_COMM_CAR_H

#include "zf_common_headfile.h"

/* ===== 参数限制 ===== */
#define AIR_COMM_PARAM_NAME_MAX             (32U)   /* 参数名最大长度（字节） */
#define AIR_COMM_COMMAND_NAME_MAX           (32U)   /* 远程命令名最大长度，不含 '\0' */
#define AIR_COMM_ACK_TEXT_MAX               (96U)   /* 远程命令 ACK 文本最大长度，不含 '\0' */
#define AIR_COMM_RUN_DATA_MAX_FLOATS        (52U)   /* 实时数据最大 float 个数 */
#define AIR_COMM_BAUDRATE                   (1152000U) /* UART 波特率 1.152Mbps */

/* ===== ACK 状态码（对端返回的操作结果） ===== */
#define AIR_COMM_STATUS_OK                  (0U)    /* 操作成功 */
#define AIR_COMM_STATUS_NOT_FOUND           (1U)    /* 参数/函数未找到 */
#define AIR_COMM_STATUS_OUT_OF_RANGE        (2U)    /* 值超出范围 */
#define AIR_COMM_STATUS_ERROR               (3U)    /* 通用错误 */
#define AIR_COMM_STATUS_BUSY                (4U)    /* 远端事务忙 */
#define AIR_COMM_STATUS_REMOTE_TIMEOUT      (5U)    /* 远端下游通信超时 */
#define AIR_COMM_STATUS_REMOTE_MISMATCH     (6U)    /* 远端读回值不一致 */
#define AIR_COMM_STATUS_REMOTE_PARTIAL      (7U)    /* 多目标仅部分成功 */
#define AIR_COMM_STATUS_REMOTE_ROLLBACK_FAIL (8U)   /* 远端回滚失败 */

/* ===== 本地 ACK 结果（本端判断的传输结果） ===== */
#define AIR_COMM_ACK_RESULT_NONE            (0U)    /* 无待确认 ACK */
#define AIR_COMM_ACK_RESULT_OK              (1U)    /* 收到 ACK 且 status=OK */
#define AIR_COMM_ACK_RESULT_TIMEOUT         (2U)    /* 重试耗尽，超时 */
#define AIR_COMM_ACK_RESULT_ERROR           (3U)    /* 收到 ACK 但 status!=OK */

/**
 * @brief 实时数据回调函数类型
 * @param data float 数组指针
 * @param count float 个数
 * 谁注册：上层通过 air_comm_car_set_run_data_callback() 注册
 */
typedef void (*air_comm_run_data_fn)(const float *data, uint8 count);

/**
 * @brief 通信统计结构体
 * 谁用：调试/诊断，通过 air_comm_car_get_stats() 获取
 */
typedef struct
{
    uint32 tick_ms;                 /* 当前 tick（ms） */
    uint32 tx_frame_count;          /* 发送帧总数 */
    uint32 tx_byte_count;           /* 发送字节总数 */
    uint32 rx_frame_count;          /* 接收帧总数（CRC 校验通过） */
    uint32 rx_byte_count;           /* 接收字节总数 */
    uint32 rx_raw_byte_count;       /* 接收原始字节数（含无效帧） */
    uint32 crc_error_count;         /* CRC 校验失败次数 */
    uint32 rx_oversize_count;       /* payload 超限次数 */
    uint32 rx_queue_overflow_count; /* 接收队列溢出次数 */
    uint32 ack_ok_count;            /* ACK 成功次数 */
    uint32 ack_timeout_count;       /* ACK 超时次数 */
    uint32 ack_retry_count;         /* ACK 重试总次数 */
    uint32 heartbeat_tx_count;      /* 心跳发送次数 */
    uint32 heartbeat_rx_count;      /* 心跳接收次数 */
    uint8 online_status;            /* 在线状态：0=初始, 1=在线, 2=离线 */
    uint8 pending_ack;              /* 是否有待确认 ACK：1=有 */
    uint8 pending_ack_type;         /* 待确认的消息类型 */
    uint8 last_ack_status;          /* 最近一次 ACK 的状态码 */
    uint8 last_ack_type;            /* 最近一次 ACK 的消息类型 */
    uint8 last_ack_result;          /* 最近一次 ACK 的结果 */
    float last_ack_value;           /* 最近一次 ACK 返回的实际值 */
    char last_ack_name[AIR_COMM_PARAM_NAME_MAX + 1U];
    char last_command_ack_text[AIR_COMM_ACK_TEXT_MAX + 1U];
} air_comm_stats_t;

/**
 * @brief 初始化车端空中通信模块
 * 调用时机：系统启动时调一次
 * 内部：清零所有状态，初始化 UART3（1152000 波特率）
 */
void air_comm_car_init(void);

/**
 * @brief 1ms tick 计数器递增
 * 调用频率：1ms 定时器中断
 * 用途：驱动超时判断和心跳间隔
 */
void air_comm_car_tick_1MS(void);

/**
 * @brief 主循环轮询
 * 调用频率：主循环每次
 * 内部：从接收队列取字节 → 状态机解析帧 → 检查 ACK 超时和在线状态
 */
void air_comm_car_poll(void);

/**
 * @brief 100Hz 更新
 * 调用频率：100Hz（每 10ms）
 * 内部：检查是否需要发心跳（每 200ms），检查 ACK 超时
 */
void air_comm_car_update_100HZ(void);

/**
 * @brief UART 接收中断回调，喂入一个字节
 * @param byte 新收到的字节
 * 谁调用：UART3 RX 中断处理函数
 * 内部：写入环形接收队列（满则丢弃并计数）
 */
void air_comm_car_rx_byte(uint8 byte);

/**
 * @brief 判断对端是否在线
 * @return 1=在线，0=离线或初始
 * 在线条件：最近 600ms 内收到过对端心跳
 */
uint8 air_comm_car_is_online(void);

/**
 * @brief 获取在线状态码
 * @return 0=初始, 1=在线, 2=离线
 */
uint8 air_comm_car_get_online_status(void);
uint8 air_comm_car_is_run_data_fresh(void);

/**
 * @brief 获取当前 tick 计数
 * @return 毫秒计数
 */
uint32 air_comm_car_get_tick(void);

/**
 * @brief 下发参数到对端（需 ACK）
 * @param name 参数名（C 字符串，≤16 字节）
 * @param value 参数值（float）
 * @return 0=发送成功，1=失败（参数名无效/未初始化/有未完成 ACK）
 * 注意：同一时间只能有一个待确认的 ACK 帧
 */
uint8 air_comm_car_set_param(const char *name, float value);
/* timeout_ms为本次SET从首次发送开始计算的ACK总等待时间。 */
uint8 air_comm_car_set_param_with_timeout(const char *name, float value, uint32 timeout_ms);
uint8 air_comm_car_get_param(const char *name);
uint8 air_comm_car_get_param_with_timeout(const char *name, uint32 timeout_ms);

/**
 * @brief 执行 Air 端远程命令（需 ACK）
 * @param name 远程命令名
 * @return 0=发送成功，1=失败
 */
uint8 air_comm_car_exec_command(const char *name);

uint8 air_comm_send_run_data(const float *data, uint8 count);
void air_comm_set_run_data_callback(air_comm_run_data_fn callback);
uint8 air_comm_get_last_run_data(float *data, uint8 max_count, uint8 *count);

/**
 * @brief 是否有待确认的 ACK
 * @return 1=有，0=无
 */
void air_comm_car_cancel_pending_set_param(void);
void air_comm_car_cancel_pending_get_param(void);
void air_comm_car_cancel_pending_command(void);
void air_comm_car_clear_last_ack(void);
uint8 air_comm_car_has_pending_ack(void);

/**
 * @brief 获取最近一次 ACK 结果
 * @param type 输出消息类型（可选，传 NULL 跳过）
 * @param result 输出 ACK 结果（可选）
 * @param status 输出 ACK 状态码（可选）
 * @return 最近一次 ACK 的 result 值
 */
uint8 air_comm_car_get_last_ack(uint8 *type, uint8 *result, uint8 *status);
uint8 air_comm_car_get_last_ack_value(float *value);
uint8 air_comm_car_get_last_ack_name(char *name, uint8 size);
uint8 air_comm_car_get_last_command_ack_text(char *text, uint8 size);

/**
 * @brief 注册实时数据回调
 * @param callback 回调函数指针（NULL 取消注册）
 * 谁用：上层模块注册后，收到 RUN_DATA 帧时自动回调
 */
void air_comm_car_set_run_data_callback(air_comm_run_data_fn callback);

/**
 * @brief 获取通信统计
 * @param stats 输出统计结构体
 */
void air_comm_car_get_stats(air_comm_stats_t *stats);

#endif
