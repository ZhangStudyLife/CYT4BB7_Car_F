#include "zf_common_headfile.h"
#ifndef _ALX_AOA_H_
#define _ALX_AOA_H_

/*
 * ALX AOA UWB 定位模块驱动
 * - UART 115200 接收基站发来的定位帧
 * - 帧格式：4 字节 0xFF 帧头 + 命令字 + 数据 + 异或校验
 * - 支持位置帧（0x2001）和心跳帧（0x2002）
 * - 内置 alpha-beta 滤波器 + 3 点中值滤波，输出平滑的 x/y 坐标（cm）
 * - 25Hz 更新（ALX_AOA_Update_25HZ），在定时中断或主循环调用
 */

/* UART 配置 */
#define ALX_AOA_UART_INDEX                 (UART_1)
#define ALX_AOA_UART_TX_PIN                (UART1_TX_P04_1)
#define ALX_AOA_UART_RX_PIN                (UART1_RX_P04_0)
#define ALX_AOA_UART_BAUDRATE              (115200)

/* 接收缓冲区和帧解析参数 */
#define ALX_AOA_RX_BUFFER_SIZE             (256)   /* 环形缓冲区大小（字节） */
#define ALX_AOA_MAX_FRAME_SIZE             (64)    /* 单帧最大长度 */

/* 帧头与命令字 */
#define ALX_AOA_FRAME_HEADER_BYTE          (0xFF)  /* 帧头标识（连续 4 个 0xFF） */
#define ALX_AOA_CMD_POSITION               (0x2001) /* 位置帧命令 */
#define ALX_AOA_CMD_HEARTBEAT              (0x2002) /* 心跳帧命令 */
#define ALX_AOA_POSITION_FRAME_SIZE        (37)    /* 位置帧总长度 */
#define ALX_AOA_HEARTBEAT_FRAME_SIZE       (16)    /* 心跳帧总长度 */

/* 数据有效性门限 */
#define ALX_AOA_DIST_MIN_CM                (20)    /* 最小有效距离（cm），低于此视为无效 */
#define ALX_AOA_DIST_MAX_CM                (500)   /* 最大有效距离（cm） */
#define ALX_AOA_EL_MAX_DEG                 (65)    /* 俯仰角有效范围（+/- 度） */

/*
 * alpha-beta 滤波器参数
 * ALPHA：位置修正增益，越大响应越快但噪声越大
 * BETA：速度修正增益，控制速度跟踪灵敏度
 * VEL_DECAY：无观测时速度衰减系数（0~1），防止失控
 * GATE：观测门限，残差超过此值视为野点，拒绝更新
 * REACQUIRE_COUNT：连续丢点 N 次后强制重新捕获
 */
#define ALX_AOA_XY_GATE_X_CM               (30.0f) /* x 方向观测门限（cm） */
#define ALX_AOA_XY_GATE_Y_CM               (30.0f) /* y 方向观测门限（cm） */
#define ALX_AOA_XY_GATE_2D_CM              (50.0f) /* 2D 合成门限（cm） */
#define ALX_AOA_AB_ALPHA                   (0.40f) /* alpha-beta 滤波 alpha */
#define ALX_AOA_AB_BETA                    (0.020f) /* alpha-beta 滤波 beta */
#define ALX_AOA_VEL_DECAY                  (0.90f) /* 无观测时速度衰减 */
#define ALX_AOA_MEDIAN_SIZE                (3U)    /* 中值滤波窗口大小 */
#define ALX_AOA_REACQUIRE_COUNT            (3U)    /* 连续丢点重新捕获阈值 */

/* 位置数据：基站解析出的原始角度/距离 + 计算出的 x/y 坐标（cm） */
typedef struct
{
    uint32 tag_id;              /* 标签 ID */
    uint32 base_id;             /* 基站 ID */
    uint32 distance_cm;         /* 斜距（cm） */
    int32  x_cm;                /* 水平投影 x（cm），由 azimuth + distance 算出 */
    int32  y_cm;                /* 水平投影 y（cm），由 elevation + distance 算出 */
    int32  z_cm;                /* z（当前未用，固定 0） */
    int16  azimuth_deg;         /* 方位角（度，signed） */
    int16  elevation_deg;       /* 俯仰角（度，signed） */
    uint32 last_position_ms;    /* 最后一次收到位置帧的时间戳（ms） */
    uint8  valid;               /* 1=数据有效 */
}ALX_AOA_Position_t;

typedef ALX_AOA_Position_t ALX_AOA_Data_t;

/* 心跳数据：基站周期性发送的心跳包 */
typedef struct
{
    uint32 base_id;
    uint32 sequence;            /* 心跳序列号 */
    uint32 last_heartbeat_ms;
    uint8  valid;
}ALX_AOA_Heartbeat_t;

/* 统计计数器：用于调试和诊断，记录收发/错误帧数 */
typedef struct
{
    uint32 rx_bytes;            /* 接收总字节数 */
    uint32 rx_overflow;         /* 环形缓冲区溢出次数 */
    uint8  rx_last_byte;        /* 最近收到的字节 */
    uint16 parser_frame_length; /* 当前帧缓冲区已累积长度 */
    uint16 last_packet_length;  /* 最近帧的 packet_length 字段 */
    uint16 last_command;        /* 最近帧的命令字 */
    uint32 frame_total;         /* 成功解析的总帧数 */
    uint32 position_count;      /* 位置帧数量 */
    uint32 heartbeat_count;     /* 心跳帧数量 */
    uint32 frame_bad_header;    /* 帧头错误（丢弃） */
    uint32 frame_bad_length;    /* 帧长错误 */
    uint32 frame_bad_xor;       /* 校验错误 */
    uint32 frame_unknown_cmd;   /* 未知命令 */
}ALX_AOA_Stats_t;

/* 滤波后的全局坐标（cm），上层直接读取即可 */
extern float alx_aoa_x_cm;
extern float alx_aoa_y_cm;

/* 初始化 UART + 清空缓冲区，上电调用一次 */
void  ALX_AOA_Init              (void);

/* 重置所有状态（含滤波器），切基站或异常恢复时调用 */
void  ALX_AOA_Reset             (void);

/* 串口接收中断入口：传入一个字节，写入环形缓冲区 */
void  ALX_AOA_InputByte         (uint8 dat);

/* 批量写入（可选），循环调用 InputByte */
void  ALX_AOA_InputBytes        (const uint8 *dat, uint16 length);

/*
 * 25Hz 帧解析入口
 * now_ms: 当前系统时间戳（ms），用于超时判断和滤波 dt 计算
 * 返回值：1=本轮解析到位置帧，0=无新位置
 */
uint8 ALX_AOA_Update_25HZ       (uint32 now_ms);

/* 获取最近一次原始位置数据，返回 1=有效，0=尚未收到有效帧 */
uint8 ALX_AOA_GetLatest         (ALX_AOA_Position_t *data);

/* 获取 alpha-beta 滤波后的 x/y（cm），返回 1=已初始化，0=尚未就绪 */
uint8 ALX_AOA_GetFilteredXY     (float *x_cm, float *y_cm);

/* 获取心跳数据，返回 1=有效 */
uint8 ALX_AOA_GetHeartbeat      (ALX_AOA_Heartbeat_t *data);

/* 判断标签是否在线：最近一次位置帧在 timeout_ms 内则在线 */
uint8 ALX_AOA_IsTagOnline       (uint32 now_ms, uint32 timeout_ms);

/* 读取统计计数器 */
void  ALX_AOA_GetStats          (ALX_AOA_Stats_t *stats);

/* 清零统计计数器 */
void  ALX_AOA_ResetStats        (void);

#endif
