/**
 * @file air_comm_car.c
 * @brief 车端空中通信实现
 *
 * 接收状态机（4 状态，逐字节解析）：
 *   state=0: 帧头匹配（等待 0xAA 0xAA 0x55 0x55）
 *   state=1: 帧信息（type → seq → len）
 *   state=2: payload 接收
 *   state=3: CRC 接收（2 字节）→ 校验 → 处理帧
 *
 * 错误处理路径：
 *   - 帧头错字节：header_count 回退，不丢帧（容错重同步）
 *   - len > MAX_PAYLOAD：丢弃，state 回 0，计 rx_oversize_count
 *   - CRC 不匹配：丢弃，计 crc_error_count
 *   - 接收队列满：丢弃新字节，计 rx_queue_overflow_count
 *   - ACK等待期间每200ms重发原帧，达到本次总等待时间后ACK_RESULT_TIMEOUT
 */

#include "air_comm_car.h"

#define AIR_COMM_MSG_GET_PARAM             (0x07U)
#define AIR_COMM_MSG_ACK_GET_PARAM         (0x08U)

/* ===== 帧头定义（四字节双对，抗误判） ===== */
#define AIR_COMM_HEADER_0                  (0xAAU)
#define AIR_COMM_HEADER_1                  (0xAAU)
#define AIR_COMM_HEADER_2                  (0x55U)
#define AIR_COMM_HEADER_3                  (0x55U)

/* ===== 帧尺寸 ===== */
#define AIR_COMM_MAX_PAYLOAD               (250U)  /* payload 最大字节数 */
#define AIR_COMM_FRAME_OVERHEAD            (9U)    /* 帧开销：head(4)+type(1)+seq(1)+len(1)+crc(2) */
#define AIR_COMM_MAX_FRAME                 (AIR_COMM_MAX_PAYLOAD + AIR_COMM_FRAME_OVERHEAD)

/* ===== 接收队列 ===== */
#define AIR_COMM_RX_QUEUE_SIZE             (512U)  /* 环形接收缓冲区大小 */

/* ===== 消息类型 ===== */
#define AIR_COMM_MSG_SET_PARAM             (0x01U) /* 下发参数（需 ACK） */
#define AIR_COMM_MSG_ACK_PARAM             (0x02U) /* 参数操作确认 */
#define AIR_COMM_MSG_EXEC_COMMAND          (0x03U) /* 执行远程命令（需 ACK） */
#define AIR_COMM_MSG_ACK_COMMAND           (0x04U) /* 远程命令确认 */
#define AIR_COMM_MSG_HEARTBEAT             (0x05U) /* 心跳 */
#define AIR_COMM_MSG_RUN_DATA              (0x06U) /* 双向实时数据 */

/* ===== 超时策略 ===== */
#define COMM_ACK_RETRY_INTERVAL_MS         (200U)  /* ACK 重发间隔（ms） */
#define COMM_ACK_TIMEOUT_MS                (800U)  /* 普通ACK总等待时间（ms） */
#define COMM_HEARTBEAT_MS                  (200U)  /* 心跳发送间隔（ms） */
#define COMM_OFFLINE_MS                    (600U)  /* 对端离线判定阈值（ms） */

/* ===== 内部结构体 ===== */

/**
 * @brief 接收状态机
 * 逐字节解析帧，state 跟踪当前阶段
 */
#define COMM_RUN_DATA_TIMEOUT_MS           (100U)

typedef struct
{
    uint8 state;                        /* 状态机状态：0=帧头, 1=信息, 2=payload, 3=CRC */
    uint8 header_count;                 /* 帧头匹配进度 / 信息字段进度 */
    uint8 type;                         /* 消息类型 */
    uint8 seq;                          /* 帧序号 */
    uint8 len;                          /* payload 长度 */
    uint8 payload[AIR_COMM_MAX_PAYLOAD]; /* payload 缓冲区 */
    uint16 payload_count;               /* 已接收 payload 字节数 */
    uint16 crc;                         /* 收到的 CRC 值（小端） */
    uint8 crc_count;                    /* CRC 字节接收进度 */
} air_comm_rx_state_t;

/**
 * @brief ACK 等待状态
 * 保存最近一次需要 ACK 的发送帧，用于超时重发
 */
typedef struct
{
    uint8 active;                       /* 1=有 ACK 等待中 */
    uint8 type;                         /* 等待 ACK 的消息类型 */
    uint8 seq;                          /* 等待 ACK 的帧序号 */
    uint8 result;                       /* ACK 结果（NONE/OK/TIMEOUT/ERROR） */
    uint8 status;                       /* 对端返回的状态码 */
    uint32 start_time;                  /* 首次发送时间（ms） */
    uint32 send_time;                   /* 最近一次发送时间（ms） */
    uint32 timeout_ms;                  /* 本次ACK总等待时间（ms） */
    uint8 frame[AIR_COMM_MAX_FRAME];    /* 保存的帧副本（用于重发） */
    uint16 frame_len;                   /* 帧长度 */
} air_comm_ack_state_t;

/**
 * @brief 环形接收队列
 * UART 中断写入 head，主循环从 tail 读取
 * head/tail 用 volatile 保证中断/主循环可见性
 */
typedef struct
{
    uint8 data[AIR_COMM_RX_QUEUE_SIZE];
    volatile uint16 head;               /* 写入位置（中断修改） */
    volatile uint16 tail;               /* 读取位置（主循环修改） */
} air_comm_rx_queue_t;

/* ===== 静态状态 ===== */
static air_comm_rx_state_t s_air_comm_rx;
static air_comm_ack_state_t s_air_comm_ack;
static air_comm_rx_queue_t s_air_comm_rx_queue;
static air_comm_stats_t s_air_comm_stats;
static air_comm_run_data_fn s_air_comm_run_data_callback;
static float s_air_comm_last_run_data[AIR_COMM_RUN_DATA_MAX_FLOATS];
static uint8 s_air_comm_last_run_data_count;
static uint8 s_air_comm_last_run_data_valid;
static volatile uint32 s_air_comm_tick_ms;          /* 1ms tick（中断修改） */
static uint32 s_air_comm_last_peer_ms;              /* 最近一次收到对端心跳的时间 */
static uint32 s_air_comm_last_heartbeat_ms;         /* 最近一次发送心跳的时间 */
static uint8 s_air_comm_seq;                        /* 下一个发送帧的序号 */
static uint8 s_air_comm_initialized;                /* 初始化标志 */
static uint32 s_air_comm_last_run_data_ms;
static float s_air_comm_last_ack_value;
static char s_air_comm_last_ack_name[AIR_COMM_PARAM_NAME_MAX + 1U];
static char s_air_comm_last_command_ack_text[AIR_COMM_ACK_TEXT_MAX + 1U];

/* ===== CRC16-CCITT（多项式 0x1021，初始 0xFFFF） ===== */

/**
 * @brief CRC16-CCITT 校验
 * 覆盖范围：帧头到 payload 末尾（不含 CRC 自身）
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC16 值
 */
static uint16 air_comm_crc16(const uint8 *data, uint16 len)
{
    uint16 crc = 0xFFFFU;
    uint16 i;
    uint8 j;

    for(i = 0U; i < len; i++)
    {
        crc ^= (uint16)data[i] << 8;
        for(j = 0U; j < 8U; j++)
        {
            if((crc & 0x8000U) != 0U)
            {
                crc = (uint16)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

/* ===== 字节序工具 ===== */

/**
 * @brief 从字节缓冲区读取 float（按字节拷贝，避免对齐问题）
 */
static float air_comm_read_float(const uint8 *buffer)
{
    float value;
    uint8 *ptr = (uint8 *)&value;

    ptr[0] = buffer[0];
    ptr[1] = buffer[1];
    ptr[2] = buffer[2];
    ptr[3] = buffer[3];
    return value;
}

/**
 * @brief 将 float 写入字节缓冲区
 */
static void air_comm_write_float(uint8 *buffer, float value)
{
    uint8 *ptr = (uint8 *)&value;

    buffer[0] = ptr[0];
    buffer[1] = ptr[1];
    buffer[2] = ptr[2];
    buffer[3] = ptr[3];
}

/**
 * @brief 将 uint32 小端写入字节缓冲区
 */
static void air_comm_write_u32(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

/* ===== 发送层 ===== */

/**
 * @brief 通过 UART3 发送原始数据
 * @param data 数据指针
 * @param len 数据长度
 * @return 1=成功，0=失败
 */
static uint8 air_comm_send_uart(const uint8 *data, uint16 len)
{
    if((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    uart_write_buffer(UART_3, data, len);
    return 1U;
}

/**
 * @brief 查询请求类型对应的期望 ACK 类型
 * @param request_type 请求消息类型
 * @return 期望的 ACK 消息类型，0=不需要 ACK
 */
static uint8 air_comm_expected_ack_type(uint8 request_type)
{
    if(request_type == AIR_COMM_MSG_SET_PARAM)
    {
        return AIR_COMM_MSG_ACK_PARAM;
    }
    if(request_type == AIR_COMM_MSG_EXEC_COMMAND)
    {
        return AIR_COMM_MSG_ACK_COMMAND;
    }
    if(request_type == AIR_COMM_MSG_GET_PARAM)
    {
        return AIR_COMM_MSG_ACK_GET_PARAM;
    }
    return 0U;
}

static uint8 air_comm_text_starts_with(const char *text, const char *prefix);
static void air_comm_parse_command_ack_text(const uint8 *payload, uint8 len);

/**
 * @brief 从 ACK payload 解析状态码
 *
 * ACK_PARAM payload: [status(1B)] → 取 payload[0]
 * ACK_COMMAND payload: [ACK文本]，ACK_OK/ACK_EXIT_OK 视为成功
 *
 * @param ack_type ACK 消息类型
 * @param payload payload 指针
 * @param len payload 长度
 * @return 状态码，异常返回 AIR_COMM_STATUS_ERROR
 */
static uint8 air_comm_parse_ack_status(uint8 ack_type,
                                       const uint8 *payload,
                                       uint8 len)
{
    if(payload == NULL)
    {
        return AIR_COMM_STATUS_ERROR;
    }

    if((ack_type == AIR_COMM_MSG_ACK_PARAM) ||
       (ack_type == AIR_COMM_MSG_ACK_GET_PARAM))
    {
        return (len >= 2U) ? payload[0] : AIR_COMM_STATUS_ERROR;
    }

    if(ack_type == AIR_COMM_MSG_ACK_COMMAND)
    {
        air_comm_parse_command_ack_text(payload, len);
        if((air_comm_text_starts_with(s_air_comm_last_command_ack_text, "ACK_OK") != 0U) ||
           (air_comm_text_starts_with(s_air_comm_last_command_ack_text, "ACK_EXIT_OK") != 0U))
        {
            return AIR_COMM_STATUS_OK;
        }
        return AIR_COMM_STATUS_ERROR;
    }

    return AIR_COMM_STATUS_ERROR;
}

static float air_comm_parse_ack_value(uint8 ack_type,
                                      const uint8 *payload,
                                      uint8 len)
{
    if(payload == NULL)
    {
        return 0.0f;
    }

    if((ack_type == AIR_COMM_MSG_ACK_PARAM) ||
       (ack_type == AIR_COMM_MSG_ACK_GET_PARAM))
    {
        if((len < 6U) || ((uint16)len < (uint16)(2U + payload[1] + 4U)))
        {
            return 0.0f;
        }

        return air_comm_read_float(&payload[2U + payload[1]]);
    }

    if(ack_type == AIR_COMM_MSG_ACK_COMMAND)
    {
        return 0.0f;
    }

    return 0.0f;
}

static void air_comm_parse_ack_name(uint8 ack_type,
                                    const uint8 *payload,
                                    uint8 len)
{
    uint8 name_len;

    s_air_comm_last_ack_name[0] = '\0';
    if((payload == NULL) ||
       ((ack_type != AIR_COMM_MSG_ACK_PARAM) &&
        (ack_type != AIR_COMM_MSG_ACK_GET_PARAM)) ||
       (len < 2U))
    {
        return;
    }

    name_len = payload[1];
    if((name_len > AIR_COMM_PARAM_NAME_MAX) ||
       ((uint16)len < (uint16)(2U + name_len + 4U)))
    {
        return;
    }

    if(name_len > 0U)
    {
        memcpy(s_air_comm_last_ack_name, &payload[2], name_len);
    }
    s_air_comm_last_ack_name[name_len] = '\0';
}

static uint8 air_comm_text_starts_with(const char *text, const char *prefix)
{
    if((text == NULL) || (prefix == NULL))
    {
        return 0U;
    }

    return (strncmp(text, prefix, strlen(prefix)) == 0) ? 1U : 0U;
}

static void air_comm_parse_command_ack_text(const uint8 *payload, uint8 len)
{
    uint8 copy_len;

    s_air_comm_last_command_ack_text[0] = '\0';
    if((payload == NULL) || (len == 0U))
    {
        return;
    }

    copy_len = (len > AIR_COMM_ACK_TEXT_MAX) ? AIR_COMM_ACK_TEXT_MAX : len;
    memcpy(s_air_comm_last_command_ack_text, payload, copy_len);
    s_air_comm_last_command_ack_text[copy_len] = '\0';
}

/**
 * @brief 组装并发送一帧
 *
 * 帧布局：[AA AA 55 55] [type] [seq] [len] [payload...] [crc_lo crc_hi]
 *
 * @param type 消息类型
 * @param seq 帧序号
 * @param payload payload 数据
 * @param len payload 长度
 * @param save_for_ack 1=保存帧副本（用于超时重发）
 * @return 1=成功，0=失败
 */
static uint8 air_comm_send_frame(uint8 type,
                                 uint8 seq,
                                 const uint8 *payload,
                                 uint8 len,
                                 uint8 save_for_ack)
{
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 pos = 0U;
    uint16 crc;

    if((len > AIR_COMM_MAX_PAYLOAD) || ((len > 0U) && (payload == NULL)))
    {
        return 0U;
    }

    /* 帧头 */
    frame[pos++] = AIR_COMM_HEADER_0;
    frame[pos++] = AIR_COMM_HEADER_1;
    frame[pos++] = AIR_COMM_HEADER_2;
    frame[pos++] = AIR_COMM_HEADER_3;
    /* 帧信息 */
    frame[pos++] = type;
    frame[pos++] = seq;
    frame[pos++] = len;

    /* payload */
    if((len > 0U) && (payload != NULL))
    {
        memcpy(&frame[pos], payload, len);
        pos += len;
    }

    /* CRC 覆盖帧头到 payload 末尾 */
    crc = air_comm_crc16(frame, pos);
    frame[pos++] = (uint8)(crc & 0xFFU);
    frame[pos++] = (uint8)((crc >> 8) & 0xFFU);

    if(air_comm_send_uart(frame, pos) == 0U)
    {
        return 0U;
    }

    /* 需要 ACK 时保存帧副本 */
    if(save_for_ack != 0U)
    {
        memcpy(s_air_comm_ack.frame, frame, pos);
        s_air_comm_ack.frame_len = pos;
    }

    /* 统计 */
    s_air_comm_stats.tx_frame_count++;
    s_air_comm_stats.tx_byte_count += pos;
    if(type == AIR_COMM_MSG_HEARTBEAT)
    {
        s_air_comm_stats.heartbeat_tx_count++;
    }

    return 1U;
}

/**
 * @brief 发送消息（高层封装）
 *
 * @param type 消息类型
 * @param payload payload 数据
 * @param len payload 长度
 * @param need_ack 1=需要 ACK（会检查是否有未完成 ACK）
 * @return 1=成功，0=失败
 * 注意：need_ack=1 时，如果已有待确认 ACK，直接返回失败
 */
static uint8 air_comm_send(uint8 type,
                           const uint8 *payload,
                           uint8 len,
                           uint8 need_ack,
                           uint32 timeout_ms)
{
    uint8 seq;

    if((need_ack != 0U) && (s_air_comm_ack.active != 0U))
    {
        return 0U;
    }

    seq = s_air_comm_seq;
    if(air_comm_send_frame(type, seq, payload, len, need_ack) == 0U)
    {
        return 0U;
    }

    s_air_comm_seq++;
    if(need_ack != 0U)
    {
        s_air_comm_ack.active = 1U;
        s_air_comm_ack.type = type;
        s_air_comm_ack.seq = seq;
        s_air_comm_ack.start_time = s_air_comm_tick_ms;
        s_air_comm_ack.send_time = s_air_comm_tick_ms;
        s_air_comm_ack.timeout_ms = (timeout_ms > 0U) ? timeout_ms : COMM_ACK_TIMEOUT_MS;
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_NONE;
        s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
    }

    return 1U;
}

/**
 * @brief 发送心跳帧
 * payload: [reserved(2B)][tick_ms(4B)] 共 6 字节
 * 不需要 ACK，不保存帧副本
 */
static void air_comm_send_heartbeat(void)
{
    uint8 payload[6];

    payload[0] = 0U;
    payload[1] = 0U;
    air_comm_write_u32(&payload[2], s_air_comm_tick_ms);
    if(air_comm_send_frame(AIR_COMM_MSG_HEARTBEAT, s_air_comm_seq, payload, 6U, 0U) != 0U)
    {
        s_air_comm_seq++;
    }
}

/* ===== 接收处理 ===== */

/**
 * @brief 标记收到对端心跳
 * 更新 last_peer_ms 和 online_status=1
 */
static void air_comm_mark_peer_heartbeat(void)
{
    s_air_comm_last_peer_ms = s_air_comm_tick_ms;
    s_air_comm_stats.online_status = 1U;
}

/**
 * @brief 匹配 ACK 帧
 *
 * 匹配条件：
 *   1. 当前有待确认 ACK（active=1）
 *   2. ACK 类型匹配（ack_type == expected）
 *   3. 序号匹配（seq == 等待的 seq）
 *
 * 匹配成功后：清除 active，解析 status，更新统计
 *
 * @param ack_type 收到的 ACK 类型
 * @param seq 收到的序号
 * @param payload payload 数据
 * @param len payload 长度
 */
static void air_comm_match_ack(uint8 ack_type,
                               uint8 seq,
                               const uint8 *payload,
                               uint8 len)
{
    uint8 expected;
    uint8 status;
    float actual;

    if(s_air_comm_ack.active == 0U)
    {
        if(ack_type == AIR_COMM_MSG_ACK_COMMAND)
        {
            status = air_comm_parse_ack_status(ack_type, payload, len);
            actual = air_comm_parse_ack_value(ack_type, payload, len);
            s_air_comm_last_ack_value = actual;
            s_air_comm_stats.last_ack_type = AIR_COMM_MSG_EXEC_COMMAND;
            s_air_comm_stats.last_ack_status = status;
            s_air_comm_stats.last_ack_value = actual;
            memcpy(s_air_comm_stats.last_command_ack_text,
                   s_air_comm_last_command_ack_text,
                   sizeof(s_air_comm_stats.last_command_ack_text));
            if(status == AIR_COMM_STATUS_OK)
            {
                s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_OK;
                s_air_comm_stats.ack_ok_count++;
            }
            else
            {
                s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_ERROR;
            }
        }
        return;
    }

    expected = air_comm_expected_ack_type(s_air_comm_ack.type);
    if((ack_type != expected) || (seq != s_air_comm_ack.seq))
    {
        return;
    }

    status = air_comm_parse_ack_status(ack_type, payload, len);
    actual = air_comm_parse_ack_value(ack_type, payload, len);
    air_comm_parse_ack_name(ack_type, payload, len);
    if(ack_type == AIR_COMM_MSG_ACK_COMMAND)
    {
        air_comm_parse_command_ack_text(payload, len);
    }

    /* 清除等待状态 */
    s_air_comm_ack.active = 0U;
    s_air_comm_ack.status = status;
    s_air_comm_last_ack_value = actual;
    s_air_comm_stats.last_ack_type = s_air_comm_ack.type;
    s_air_comm_stats.last_ack_status = status;
    s_air_comm_stats.last_ack_value = actual;
    memcpy(s_air_comm_stats.last_ack_name,
           s_air_comm_last_ack_name,
           sizeof(s_air_comm_stats.last_ack_name));
    memcpy(s_air_comm_stats.last_command_ack_text,
           s_air_comm_last_command_ack_text,
           sizeof(s_air_comm_stats.last_command_ack_text));
    if(status == AIR_COMM_STATUS_OK)
    {
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_OK;
        s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_OK;
        s_air_comm_stats.ack_ok_count++;
    }
    else
    {
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_ERROR;
        s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_ERROR;
    }
}

/**
 * @brief 处理实时数据帧
 *
 * RUN_DATA payload 格式：[count(1B)][float0(4B)][float1(4B)]...
 * count = float 个数（≤32）
 *
 * @param payload payload 指针
 * @param len payload 长度
 */
static void air_comm_handle_run_data(const uint8 *payload, uint8 len)
{
    uint8 count;
    uint16 required_len;
    uint8 index;
    float data[AIR_COMM_RUN_DATA_MAX_FLOATS];

    if((payload == NULL) || (len < 1U))
    {
        return;
    }

    count = payload[0];
    if((count == 0U) || (count > AIR_COMM_RUN_DATA_MAX_FLOATS))
    {
        return;
    }

    required_len = (uint16)(1U + ((uint16)count * 4U));
    if((uint16)len != required_len)
    {
        return;
    }

    for(index = 0U; index < count; index++)
    {
        data[index] = air_comm_read_float(&payload[1U + ((uint16)index * 4U)]);
    }

    memcpy(s_air_comm_last_run_data, data, (size_t)count * sizeof(float));
    s_air_comm_last_run_data_count = count;
    s_air_comm_last_run_data_valid = 1U;
    s_air_comm_last_run_data_ms = s_air_comm_tick_ms;

    if(s_air_comm_run_data_callback != NULL)
    {
        s_air_comm_run_data_callback(data, count);
    }
}

/**
 * @brief 帧分发器（CRC 校验通过后调用）
 *
 * 分发逻辑：
 *   ACK_PARAM / ACK_COMMAND → 匹配 ACK
 *   HEARTBEAT → 标记对端在线
 *   RUN_DATA → 回调上层
 *   其他类型 → 忽略
 *
 * @param type 消息类型
 * @param seq 帧序号
 * @param payload payload
 * @param len payload 长度
 */
static void air_comm_handle_frame(uint8 type,
                                  uint8 seq,
                                  const uint8 *payload,
                                  uint8 len)
{
    switch(type)
    {
        case AIR_COMM_MSG_ACK_PARAM:
        case AIR_COMM_MSG_ACK_COMMAND:
        case AIR_COMM_MSG_ACK_GET_PARAM:
            air_comm_match_ack(type, seq, payload, len);
            break;

        case AIR_COMM_MSG_HEARTBEAT:
            s_air_comm_stats.heartbeat_rx_count++;
            air_comm_mark_peer_heartbeat();
            break;

        case AIR_COMM_MSG_RUN_DATA:
            air_comm_handle_run_data(payload, len);
            break;

        default:
            break;
    }
}

/**
 * @brief 处理一帧完整的接收数据
 *
 * 流程：
 *   1. 重建完整帧（用于 CRC 校验）
 *   2. 计算 CRC 并与收到的 CRC 比较
 *   3. 不匹配 → crc_error_count++，丢弃
 *   4. 匹配 → rx_frame_count++，分发处理
 */
static void air_comm_process_rx_frame(void)
{
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 pos = 0U;
    uint16 crc_calc;

    /* 重建帧 */
    frame[pos++] = AIR_COMM_HEADER_0;
    frame[pos++] = AIR_COMM_HEADER_1;
    frame[pos++] = AIR_COMM_HEADER_2;
    frame[pos++] = AIR_COMM_HEADER_3;
    frame[pos++] = s_air_comm_rx.type;
    frame[pos++] = s_air_comm_rx.seq;
    frame[pos++] = s_air_comm_rx.len;

    if(s_air_comm_rx.len > 0U)
    {
        memcpy(&frame[pos], s_air_comm_rx.payload, s_air_comm_rx.len);
        pos += s_air_comm_rx.len;
    }

    /* CRC 校验 */
    crc_calc = air_comm_crc16(frame, pos);
    if(crc_calc != s_air_comm_rx.crc)
    {
        s_air_comm_stats.crc_error_count++;
        return;
    }

    s_air_comm_stats.rx_frame_count++;
    s_air_comm_stats.rx_byte_count += (uint32)(pos + 2U);
    air_comm_handle_frame(s_air_comm_rx.type,
                          s_air_comm_rx.seq,
                          s_air_comm_rx.payload,
                          s_air_comm_rx.len);
}

/**
 * @brief 逐字节接收状态机
 *
 * state=0（帧头匹配）：
 *   按顺序匹配 AA AA 55 55，任何字节不匹配则重置
 *   header_count 用作帧头匹配进度计数器
 *
 * state=1（帧信息）：
 *   header_count 复用为信息字段进度：
 *     0 → type
 *     1 → seq
 *     2 → len → 根据 len 决定下一步
 *       len > MAX_PAYLOAD → 丢弃，回 state=0
 *       len == 0 → 直接跳 state=3 收 CRC
 *       len > 0 → 跳 state=2 收 payload
 *
 * state=2（payload）：
 *   逐字节写入 payload 缓冲区
 *   收满 len 字节后跳 state=3
 *
 * state=3（CRC）：
 *   收 2 字节 CRC → 调 process_rx_frame → 回 state=0
 *
 * @param byte 新收到的字节
 */
static void air_comm_rx_byte_parser(uint8 byte)
{
    s_air_comm_stats.rx_raw_byte_count++;

    switch(s_air_comm_rx.state)
    {
        case 0: /* 帧头匹配 */
            if((s_air_comm_rx.header_count == 0U) && (byte == AIR_COMM_HEADER_0))
            {
                s_air_comm_rx.header_count = 1U;
            }
            else if((s_air_comm_rx.header_count == 1U) && (byte == AIR_COMM_HEADER_1))
            {
                s_air_comm_rx.header_count = 2U;
            }
            else if((s_air_comm_rx.header_count == 2U) && (byte == AIR_COMM_HEADER_2))
            {
                s_air_comm_rx.header_count = 3U;
            }
            else if((s_air_comm_rx.header_count == 3U) && (byte == AIR_COMM_HEADER_3))
            {
                s_air_comm_rx.header_count = 0U;
                s_air_comm_rx.state = 1U;
            }
            else
            {
                /* 不匹配：如果当前字节恰好是帧头首字节，保留进度为 1 */
                s_air_comm_rx.header_count = (byte == AIR_COMM_HEADER_0) ? 1U : 0U;
            }
            break;

        case 1: /* 帧信息：type → seq → len */
            if(s_air_comm_rx.header_count == 0U)
            {
                s_air_comm_rx.type = byte;
                s_air_comm_rx.header_count = 1U;
            }
            else if(s_air_comm_rx.header_count == 1U)
            {
                s_air_comm_rx.seq = byte;
                s_air_comm_rx.header_count = 2U;
            }
            else
            {
                s_air_comm_rx.len = byte;
                s_air_comm_rx.header_count = 0U;
                s_air_comm_rx.payload_count = 0U;
                if(byte > AIR_COMM_MAX_PAYLOAD)
                {
                    /* payload 超限：丢弃 */
                    s_air_comm_stats.rx_oversize_count++;
                    s_air_comm_rx.state = 0U;
                }
                else if(byte == 0U)
                {
                    /* 无 payload：直接收 CRC */
                    s_air_comm_rx.state = 3U;
                    s_air_comm_rx.crc_count = 0U;
                }
                else
                {
                    /* 有 payload：收 payload */
                    s_air_comm_rx.state = 2U;
                }
            }
            break;

        case 2: /* payload 接收 */
            if(s_air_comm_rx.payload_count < AIR_COMM_MAX_PAYLOAD)
            {
                s_air_comm_rx.payload[s_air_comm_rx.payload_count++] = byte;
            }
            if(s_air_comm_rx.payload_count >= s_air_comm_rx.len)
            {
                s_air_comm_rx.state = 3U;
                s_air_comm_rx.crc_count = 0U;
            }
            break;

        case 3: /* CRC 接收 */
            if(s_air_comm_rx.crc_count == 0U)
            {
                s_air_comm_rx.crc = byte;
                s_air_comm_rx.crc_count = 1U;
            }
            else
            {
                s_air_comm_rx.crc |= (uint16)byte << 8;
                air_comm_process_rx_frame();
                s_air_comm_rx.state = 0U;
                s_air_comm_rx.header_count = 0U;
            }
            break;

        default: /* 异常状态：重置 */
            s_air_comm_rx.state = 0U;
            s_air_comm_rx.header_count = 0U;
            break;
    }
}

/* ===== 接收队列 ===== */

/**
 * @brief 从环形队列弹出一个字节
 * @param byte 输出字节
 * @return 1=有数据，0=队列空
 */
static uint8 air_comm_rx_queue_pop(uint8 *byte)
{
    uint16 tail;

    if(byte == NULL)
    {
        return 0U;
    }

    if(s_air_comm_rx_queue.head == s_air_comm_rx_queue.tail)
    {
        return 0U;
    }

    tail = s_air_comm_rx_queue.tail;
    *byte = s_air_comm_rx_queue.data[tail];
    tail++;
    if(tail >= AIR_COMM_RX_QUEUE_SIZE)
    {
        tail = 0U;
    }
    s_air_comm_rx_queue.tail = tail;
    return 1U;
}

/* ===== ACK 超时与在线检测 ===== */

/**
 * @brief 检查 ACK 超时和对端在线状态
 *
 * ACK 超时处理：
 *   - 每隔200ms无ACK时重发保存的帧
 *   - 达到本次总等待时间后active=0, result=TIMEOUT
 *
 * 在线检测：
 *   - 超过 600ms 未收到对端心跳 → online_status=2（离线）
 *   - last_peer_ms=0 表示已判定离线（避免重复判定）
 */
static void air_comm_task_ack_and_online(void)
{
    /* ACK 超时检查 */
    if(s_air_comm_ack.active != 0U)
    {
        if((s_air_comm_tick_ms - s_air_comm_ack.start_time) >= s_air_comm_ack.timeout_ms)
        {
            s_air_comm_ack.active = 0U;
            s_air_comm_ack.result = AIR_COMM_ACK_RESULT_TIMEOUT;
            s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
            s_air_comm_stats.last_ack_type = s_air_comm_ack.type;
            s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_TIMEOUT;
            s_air_comm_stats.last_ack_status = AIR_COMM_STATUS_ERROR;
            s_air_comm_stats.ack_timeout_count++;
        }
        else if((s_air_comm_tick_ms - s_air_comm_ack.send_time) >= COMM_ACK_RETRY_INTERVAL_MS)
        {
            /* 在总等待窗口内按固定间隔重发相同序号的原帧。 */
            if(air_comm_send_uart(s_air_comm_ack.frame, s_air_comm_ack.frame_len) != 0U)
            {
                s_air_comm_ack.send_time = s_air_comm_tick_ms;
                s_air_comm_stats.ack_retry_count++;
                s_air_comm_stats.tx_frame_count++;
                s_air_comm_stats.tx_byte_count += s_air_comm_ack.frame_len;
            }
        }
    }

    /* 对端在线检测 */
    if(s_air_comm_last_peer_ms != 0U)
    {
        if((s_air_comm_tick_ms - s_air_comm_last_peer_ms) >= COMM_OFFLINE_MS)
        {
            s_air_comm_stats.online_status = 2U;  /* 离线 */
            s_air_comm_last_peer_ms = 0U;
        }
    }
}

/* ===== 公共接口 ===== */

void air_comm_car_init(void)
{
    memset(&s_air_comm_rx, 0, sizeof(s_air_comm_rx));
    memset(&s_air_comm_ack, 0, sizeof(s_air_comm_ack));
    memset(&s_air_comm_rx_queue, 0, sizeof(s_air_comm_rx_queue));
    memset(&s_air_comm_stats, 0, sizeof(s_air_comm_stats));
    memset(s_air_comm_last_run_data, 0, sizeof(s_air_comm_last_run_data));
    s_air_comm_run_data_callback = NULL;
    s_air_comm_last_run_data_count = 0U;
    s_air_comm_last_run_data_valid = 0U;
    s_air_comm_tick_ms = 0U;
    s_air_comm_last_peer_ms = 0U;
    s_air_comm_last_heartbeat_ms = 0U;
    s_air_comm_last_run_data_ms = 0U;
    s_air_comm_seq = 0U;
    s_air_comm_initialized = 1U;
    s_air_comm_last_ack_value = 0.0f;
    memset(s_air_comm_last_ack_name, 0, sizeof(s_air_comm_last_ack_name));
    memset(s_air_comm_last_command_ack_text, 0, sizeof(s_air_comm_last_command_ack_text));

    uart_init(UART_3, AIR_COMM_BAUDRATE, UART3_TX_P17_2, UART3_RX_P17_1);
    uart_rx_interrupt(UART_3, 1U);
}

void air_comm_car_tick_1MS(void)
{
    s_air_comm_tick_ms++;
}

/**
 * @brief 主循环轮询入口
 * 处理量：最多处理 512 字节（一整轮队列），防止阻塞主循环
 * 流程：取字节 → 解析 → 检查 ACK 超时/在线
 */
void air_comm_car_poll(void)
{
    uint8 byte;
    uint16 guard = AIR_COMM_RX_QUEUE_SIZE;

    if(s_air_comm_initialized == 0U)
    {
        return;
    }

    while((guard > 0U) && (air_comm_rx_queue_pop(&byte) != 0U))
    {
        air_comm_rx_byte_parser(byte);
        guard--;
    }

    air_comm_task_ack_and_online();
}

/**
 * @brief 100Hz 更新入口
 * 检查是否需要发心跳（每 200ms 一次）
 * 同时检查 ACK 超时（冗余检查，确保及时）
 */
void air_comm_car_update_100HZ(void)
{
    if(s_air_comm_initialized == 0U)
    {
        return;
    }

    if((s_air_comm_tick_ms - s_air_comm_last_heartbeat_ms) >= COMM_HEARTBEAT_MS)
    {
        air_comm_send_heartbeat();
        s_air_comm_last_heartbeat_ms = s_air_comm_tick_ms;
    }

    air_comm_task_ack_and_online();
}

/**
 * @brief UART 接收中断回调
 * @param byte 接收到的字节
 * 写入环形队列，满则丢弃并计 overflow
 */
void air_comm_car_rx_byte(uint8 byte)
{
    uint16 next_head;

    next_head = s_air_comm_rx_queue.head + 1U;
    if(next_head >= AIR_COMM_RX_QUEUE_SIZE)
    {
        next_head = 0U;
    }

    if(next_head == s_air_comm_rx_queue.tail)
    {
        s_air_comm_stats.rx_queue_overflow_count++;
        return;
    }

    s_air_comm_rx_queue.data[s_air_comm_rx_queue.head] = byte;
    s_air_comm_rx_queue.head = next_head;
}

uint8 air_comm_car_is_online(void)
{
    return (s_air_comm_stats.online_status == 1U) ? 1U : 0U;
}

uint8 air_comm_car_get_online_status(void)
{
    return s_air_comm_stats.online_status;
}

uint8 air_comm_car_is_run_data_fresh(void)
{
    if((s_air_comm_initialized == 0U) || (s_air_comm_last_run_data_valid == 0U))
    {
        return 0U;
    }

    return ((s_air_comm_tick_ms - s_air_comm_last_run_data_ms) <= COMM_RUN_DATA_TIMEOUT_MS) ? 1U : 0U;
}

uint32 air_comm_car_get_tick(void)
{
    return s_air_comm_tick_ms;
}

/**
 * @brief 下发参数（需 ACK）
 *
 * payload 格式：[name_len(1B)][name...][value(4B float)]
 * 总长度 = 1 + name_len + 4
 *
 * @param name 参数名（C 字符串，≤16 字节）
 * @param value 参数值
 * @return 0=发送成功，1=失败
 */
uint8 air_comm_car_set_param_with_timeout(const char *name, float value, uint32 timeout_ms)
{
    uint8 payload[1U + AIR_COMM_PARAM_NAME_MAX + 4U];
    uint16 name_len;
    uint8 pos = 0U;

    if((name == NULL) || (s_air_comm_initialized == 0U))
    {
        return 1U;
    }

    name_len = (uint16)strlen(name);
    if((name_len == 0U) || (name_len > AIR_COMM_PARAM_NAME_MAX))
    {
        return 1U;
    }

    payload[pos++] = (uint8)name_len;
    memcpy(&payload[pos], name, name_len);
    pos = (uint8)(pos + (uint8)name_len);
    air_comm_write_float(&payload[pos], value);
    pos = (uint8)(pos + 4U);

    return (air_comm_send(AIR_COMM_MSG_SET_PARAM, payload, pos, 1U, timeout_ms) != 0U) ? 0U : 1U;
}

uint8 air_comm_car_set_param(const char *name, float value)
{
    return air_comm_car_set_param_with_timeout(name, value, COMM_ACK_TIMEOUT_MS);
}

uint8 air_comm_car_get_param_with_timeout(const char *name, uint32 timeout_ms)
{
    uint8 payload[1U + AIR_COMM_PARAM_NAME_MAX];
    uint16 name_len;
    uint8 pos = 0U;

    if((name == NULL) || (s_air_comm_initialized == 0U))
    {
        return 1U;
    }

    name_len = (uint16)strlen(name);
    if((name_len == 0U) || (name_len > AIR_COMM_PARAM_NAME_MAX))
    {
        return 1U;
    }

    payload[pos++] = (uint8)name_len;
    memcpy(&payload[pos], name, name_len);
    pos = (uint8)(pos + (uint8)name_len);

    return (air_comm_send(AIR_COMM_MSG_GET_PARAM,
                          payload,
                          pos,
                          1U,
                          timeout_ms) != 0U) ? 0U : 1U;
}

uint8 air_comm_car_get_param(const char *name)
{
    return air_comm_car_get_param_with_timeout(name, COMM_ACK_TIMEOUT_MS);
}

/**
 * @brief 执行 Air 端远程命令（需 ACK）
 *
 * payload 格式：[name_len(1B)][name...]
 *
 * @param name 远程命令名
 * @return 0=发送成功，1=失败
 */
uint8 air_comm_car_exec_command(const char *name)
{
    uint8 payload[1U + AIR_COMM_COMMAND_NAME_MAX];
    uint16 name_len;
    uint8 pos = 0U;

    if((name == NULL) || (s_air_comm_initialized == 0U))
    {
        return 1U;
    }

    name_len = (uint16)strlen(name);
    if((name_len == 0U) || (name_len > AIR_COMM_COMMAND_NAME_MAX))
    {
        return 1U;
    }

    payload[pos++] = (uint8)name_len;
    memcpy(&payload[pos], name, name_len);
    pos = (uint8)(pos + (uint8)name_len);

    return (air_comm_send(AIR_COMM_MSG_EXEC_COMMAND,
                          payload,
                          pos,
                          1U,
                          COMM_ACK_TIMEOUT_MS) != 0U) ? 0U : 1U;
}

uint8 air_comm_send_run_data(const float *data, uint8 count)
{
    uint8 payload[1U + (AIR_COMM_RUN_DATA_MAX_FLOATS * 4U)];
    uint8 pos = 0U;
    uint8 index;

    if((s_air_comm_initialized == 0U) ||
       (data == NULL) ||
       (count == 0U) ||
       (count > AIR_COMM_RUN_DATA_MAX_FLOATS))
    {
        return 0U;
    }

    payload[pos++] = count;
    for(index = 0U; index < count; index++)
    {
        air_comm_write_float(&payload[pos], data[index]);
        pos = (uint8)(pos + 4U);
    }

    return air_comm_send(AIR_COMM_MSG_RUN_DATA, payload, pos, 0U, 0U);
}

uint8 air_comm_car_has_pending_ack(void)
{
    return s_air_comm_ack.active;
}

void air_comm_car_cancel_pending_set_param(void)
{
    if((s_air_comm_ack.active != 0U) &&
       (s_air_comm_ack.type == AIR_COMM_MSG_SET_PARAM))
    {
        s_air_comm_ack.active = 0U;
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_TIMEOUT;
        s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
        s_air_comm_stats.last_ack_type = AIR_COMM_MSG_SET_PARAM;
        s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_TIMEOUT;
        s_air_comm_stats.last_ack_status = AIR_COMM_STATUS_ERROR;
        s_air_comm_stats.ack_timeout_count++;
    }
}

void air_comm_car_cancel_pending_get_param(void)
{
    if((s_air_comm_ack.active != 0U) &&
       (s_air_comm_ack.type == AIR_COMM_MSG_GET_PARAM))
    {
        s_air_comm_ack.active = 0U;
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_TIMEOUT;
        s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
        s_air_comm_stats.last_ack_type = AIR_COMM_MSG_GET_PARAM;
        s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_TIMEOUT;
        s_air_comm_stats.last_ack_status = AIR_COMM_STATUS_ERROR;
        s_air_comm_stats.ack_timeout_count++;
    }
}

void air_comm_car_cancel_pending_command(void)
{
    if((s_air_comm_ack.active != 0U) &&
       (s_air_comm_ack.type == AIR_COMM_MSG_EXEC_COMMAND))
    {
        s_air_comm_ack.active = 0U;
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_TIMEOUT;
        s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
        s_air_comm_stats.last_ack_type = AIR_COMM_MSG_EXEC_COMMAND;
        s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_TIMEOUT;
        s_air_comm_stats.last_ack_status = AIR_COMM_STATUS_ERROR;
        s_air_comm_stats.ack_timeout_count++;
    }
}

void air_comm_car_clear_last_ack(void)
{
    s_air_comm_ack.result = AIR_COMM_ACK_RESULT_NONE;
    s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
    s_air_comm_stats.last_ack_type = 0U;
    s_air_comm_stats.last_ack_result = AIR_COMM_ACK_RESULT_NONE;
    s_air_comm_stats.last_ack_status = AIR_COMM_STATUS_ERROR;
    s_air_comm_stats.last_ack_value = 0.0f;
    s_air_comm_last_ack_value = 0.0f;
    s_air_comm_last_ack_name[0] = '\0';
    s_air_comm_last_command_ack_text[0] = '\0';
    s_air_comm_stats.last_ack_name[0] = '\0';
    s_air_comm_stats.last_command_ack_text[0] = '\0';
}

uint8 air_comm_car_get_last_ack(uint8 *type, uint8 *result, uint8 *status)
{
    if(type != NULL)
    {
        *type = s_air_comm_stats.last_ack_type;
    }
    if(result != NULL)
    {
        *result = s_air_comm_stats.last_ack_result;
    }
    if(status != NULL)
    {
        *status = s_air_comm_stats.last_ack_status;
    }

    return s_air_comm_stats.last_ack_result;
}

uint8 air_comm_car_get_last_ack_value(float *value)
{
    if(value != NULL)
    {
        *value = s_air_comm_last_ack_value;
    }

    return s_air_comm_stats.last_ack_result;
}

uint8 air_comm_car_get_last_ack_name(char *name, uint8 size)
{
    if((name != NULL) && (size > 0U))
    {
        strncpy(name, s_air_comm_last_ack_name, (size_t)(size - 1U));
        name[size - 1U] = '\0';
    }

    return s_air_comm_stats.last_ack_result;
}

uint8 air_comm_car_get_last_command_ack_text(char *text, uint8 size)
{
    if((text != NULL) && (size > 0U))
    {
        strncpy(text, s_air_comm_last_command_ack_text, (size_t)(size - 1U));
        text[size - 1U] = '\0';
    }

    return s_air_comm_stats.last_ack_result;
}

void air_comm_car_set_run_data_callback(air_comm_run_data_fn callback)
{
    air_comm_set_run_data_callback(callback);
}

void air_comm_set_run_data_callback(air_comm_run_data_fn callback)
{
    s_air_comm_run_data_callback = callback;
}

uint8 air_comm_get_last_run_data(float *data, uint8 max_count, uint8 *count)
{
    if(count != NULL)
    {
        *count = 0U;
    }

    if((data == NULL) ||
       (count == NULL) ||
       (s_air_comm_last_run_data_valid == 0U) ||
       (max_count < s_air_comm_last_run_data_count))
    {
        return 0U;
    }

    if(s_air_comm_last_run_data_count > 0U)
    {
        memcpy(data,
               s_air_comm_last_run_data,
               (size_t)s_air_comm_last_run_data_count * sizeof(float));
    }
    *count = s_air_comm_last_run_data_count;

    return 1U;
}

void air_comm_car_get_stats(air_comm_stats_t *stats)
{
    if(stats == NULL)
    {
        return;
    }

    s_air_comm_stats.tick_ms = s_air_comm_tick_ms;
    s_air_comm_stats.pending_ack = s_air_comm_ack.active;
    s_air_comm_stats.pending_ack_type = s_air_comm_ack.type;
    memcpy(s_air_comm_stats.last_ack_name,
           s_air_comm_last_ack_name,
           sizeof(s_air_comm_stats.last_ack_name));
    memcpy(s_air_comm_stats.last_command_ack_text,
           s_air_comm_last_command_ack_text,
           sizeof(s_air_comm_stats.last_command_ack_text));
    *stats = s_air_comm_stats;
}
