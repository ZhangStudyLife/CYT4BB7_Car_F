#include "ALX_AOA.h"

/*
 * 帧格式（大端字节序）：
 * FF FF FF FF | PacketLength(2) | SequenceID(2) | RequestCommand(2) | VersionID(2) |
 * AnchorID(4) | TagID(4) | Distance(4) | Azimuth(2s) | Elevation(2s) | TagStatus |
 * BatchSn | Reserve | Xor
 * 注意：Command 在 byte offset 8，不是 6
 */

#define ALX_AOA_OFFSET_PACKET_LENGTH        (4)
#define ALX_AOA_OFFSET_COMMAND              (8)
#define ALX_AOA_OFFSET_ANCHOR_ID            (12)
#define ALX_AOA_OFFSET_TAG_ID               (16)
#define ALX_AOA_OFFSET_DISTANCE             (20)
#define ALX_AOA_OFFSET_AZIMUTH              (24)
#define ALX_AOA_OFFSET_ELEVATION            (26)
#define ALX_AOA_COMMAND_FIELD_SIZE          (2)
#define ALX_AOA_ANGLE_SCALE                 (1.0f)
#define ALX_AOA_PI                          (3.14159265358979323846f)
#define ALX_AOA_DEG_TO_RAD                  (ALX_AOA_PI / 180.0f)

static volatile uint16 alx_aoa_rx_head = 0;
static volatile uint16 alx_aoa_rx_tail = 0;
static volatile uint8  alx_aoa_rx_buffer[ALX_AOA_RX_BUFFER_SIZE];

static uint8  alx_aoa_frame_buffer[ALX_AOA_MAX_FRAME_SIZE];
static uint16 alx_aoa_frame_length = 0;

static ALX_AOA_Position_t  alx_aoa_latest_position;
static ALX_AOA_Heartbeat_t alx_aoa_latest_heartbeat;
static ALX_AOA_Stats_t     alx_aoa_stats;

float alx_aoa_x_cm = 0.0f;
float alx_aoa_y_cm = 0.0f;

static float  alx_aoa_filt_x_cm     = 0.0f;
static float  alx_aoa_filt_y_cm     = 0.0f;
static float  alx_aoa_filt_vx       = 0.0f;
static float  alx_aoa_filt_vy       = 0.0f;
static float  alx_aoa_raw_x_hist[ALX_AOA_MEDIAN_SIZE] = {0.0f};
static float  alx_aoa_raw_y_hist[ALX_AOA_MEDIAN_SIZE] = {0.0f};
static uint8  alx_aoa_raw_hist_count = 0;
static uint8  alx_aoa_raw_hist_index = 0;
static uint8  alx_aoa_filt_init     = 0;
static uint8  alx_aoa_outlier_count = 0;
static uint32 alx_aoa_last_filt_ms  = 0;

/* 环形缓冲区辅助：索引递增（到尾则回绕） */
static uint16 alx_aoa_next_index (uint16 index)
{
    index ++;
    if(ALX_AOA_RX_BUFFER_SIZE <= index)
    {
        index = 0;
    }
    return index;
}

/* 返回环形缓冲区中待处理字节数 */
static uint16 alx_aoa_rx_count (void)
{
    uint16 head = alx_aoa_rx_head;
    uint16 tail = alx_aoa_rx_tail;

    if(head >= tail)
    {
        return (uint16)(head - tail);
    }

    return (uint16)(ALX_AOA_RX_BUFFER_SIZE - tail + head);
}

/* 从环形缓冲区弹出一个字节，返回 1=成功，0=空 */
static uint8 alx_aoa_pop_byte (uint8 *dat)
{
    if(alx_aoa_rx_head == alx_aoa_rx_tail)
    {
        return 0;
    }

    *dat = alx_aoa_rx_buffer[alx_aoa_rx_tail];
    alx_aoa_rx_tail = alx_aoa_next_index(alx_aoa_rx_tail);

    return 1;
}

/* 大端 16 位无符号 */
static uint16 alx_aoa_read_be16 (const uint8 *dat)
{
    return (uint16)(((uint16)dat[0] << 8) | dat[1]);
}

/* 大端 32 位无符号 */
static uint32 alx_aoa_read_be32 (const uint8 *dat)
{
    return (((uint32)dat[0] << 24) |
            ((uint32)dat[1] << 16) |
            ((uint32)dat[2] << 8 ) |
            ((uint32)dat[3]));
}

/* 大端 16 位有符号（补码转换） */
static int16 alx_aoa_read_s16 (const uint8 *dat)
{
    uint16 value = alx_aoa_read_be16(dat);

    if(0 != (value & 0x8000))
    {
        return (int16)(-1 - (int16)(0xFFFF - value));
    }

    return (int16)value;
}

static int32 alx_aoa_float_to_s32 (float value)
{
    if(value >= 0.0f)
    {
        return (int32)(value + 0.5f);
    }

    return (int32)(value - 0.5f);
}

/* 角度（整数度）-> 弧度 */
static float alx_aoa_angle_to_rad (int16 angle)
{
    return (((float)angle) / ALX_AOA_ANGLE_SCALE) * ALX_AOA_DEG_TO_RAD;
}

/*
 * 球坐标 -> 平面 xy
 * x = distance * sin(azimuth)
 * y = -distance * sin(elevation)
 */
static void alx_aoa_calculate_xy (ALX_AOA_Position_t *position)
{
    float azimuth_rad = alx_aoa_angle_to_rad(position->azimuth_deg);
    float elevation_rad = alx_aoa_angle_to_rad(position->elevation_deg);
    float distance = (float)position->distance_cm;

    position->x_cm = alx_aoa_float_to_s32(distance * sinf(azimuth_rad));
    position->y_cm = alx_aoa_float_to_s32(-distance * sinf(elevation_rad));
    position->z_cm = 0;
}

/* 原始数据有效性检查：距离、角度、范围过滤 */
static uint8 alx_aoa_raw_valid (const ALX_AOA_Position_t *p)
{
    if(0 == p->distance_cm)
    {
        return 0;
    }

    if(255 == p->azimuth_deg)
    {
        return 0;
    }

    if(p->elevation_deg > ALX_AOA_EL_MAX_DEG || p->elevation_deg < -ALX_AOA_EL_MAX_DEG)
    {
        return 0;
    }

    if(p->distance_cm < ALX_AOA_DIST_MIN_CM || p->distance_cm > ALX_AOA_DIST_MAX_CM)
    {
        return 0;
    }

    return 1;
}

static void alx_aoa_raw_history_reset (float raw_x, float raw_y)
{
    uint8 i;

    for(i = 0; i < ALX_AOA_MEDIAN_SIZE; i ++)
    {
        alx_aoa_raw_x_hist[i] = raw_x;
        alx_aoa_raw_y_hist[i] = raw_y;
    }

    alx_aoa_raw_hist_count = ALX_AOA_MEDIAN_SIZE;
    alx_aoa_raw_hist_index = 0;
}

static void alx_aoa_raw_history_push (float raw_x, float raw_y)
{
    alx_aoa_raw_x_hist[alx_aoa_raw_hist_index] = raw_x;
    alx_aoa_raw_y_hist[alx_aoa_raw_hist_index] = raw_y;

    alx_aoa_raw_hist_index ++;
    if(alx_aoa_raw_hist_index >= ALX_AOA_MEDIAN_SIZE)
    {
        alx_aoa_raw_hist_index = 0;
    }

    if(alx_aoa_raw_hist_count < ALX_AOA_MEDIAN_SIZE)
    {
        alx_aoa_raw_hist_count ++;
    }
}

static void alx_aoa_raw_history_get_median (float *x_cm, float *y_cm)
{
    if(alx_aoa_raw_hist_count >= ALX_AOA_MEDIAN_SIZE)
    {
        *x_cm = car_filter_median3f(alx_aoa_raw_x_hist[0],
                                    alx_aoa_raw_x_hist[1],
                                    alx_aoa_raw_x_hist[2]);
        *y_cm = car_filter_median3f(alx_aoa_raw_y_hist[0],
                                    alx_aoa_raw_y_hist[1],
                                    alx_aoa_raw_y_hist[2]);
    }
    else
    {
        uint8 latest;

        if(0 == alx_aoa_raw_hist_count)
        {
            *x_cm = 0.0f;
            *y_cm = 0.0f;
            return;
        }

        latest = (0 == alx_aoa_raw_hist_index) ?
                 (uint8)(ALX_AOA_MEDIAN_SIZE - 1U) :
                 (uint8)(alx_aoa_raw_hist_index - 1U);
        *x_cm = alx_aoa_raw_x_hist[latest];
        *y_cm = alx_aoa_raw_y_hist[latest];
    }
}

static uint8 alx_aoa_observation_gate_ok (float rx, float ry)
{
    float jump_2d = sqrtf(rx * rx + ry * ry);

    if((fabsf(rx) > ALX_AOA_XY_GATE_X_CM) ||
       (fabsf(ry) > ALX_AOA_XY_GATE_Y_CM) ||
       (jump_2d > ALX_AOA_XY_GATE_2D_CM))
    {
        return 0;
    }

    return 1;
}

/*
 * alpha-beta 滤波 + 中值滤波 + 观测门限
 * 三种状态：
 *   1) 观测有效且在门限内 -> 正常 alpha-beta 更新
 *   2) 观测无效且连续丢点 >= REACQUIRE_COUNT -> 强制重捕获
 *   3) 其他 -> 仅预测，速度衰减
 */
static void alx_aoa_filter_xy (const ALX_AOA_Position_t *p, uint32 now_ms)
{
    float raw_x;
    float raw_y;
    float obs_x;
    float obs_y;
    float dt;
    float rx;
    float ry;
    float x_pred;
    float y_pred;
    uint8 obs_ok;

    raw_x = (float)p->x_cm;
    raw_y = (float)p->y_cm;

    obs_ok = alx_aoa_raw_valid(p);

    if(!alx_aoa_filt_init)
    {
        if(obs_ok)
        {
            alx_aoa_raw_history_reset(raw_x, raw_y);
            alx_aoa_filt_x_cm     = raw_x;
            alx_aoa_filt_y_cm     = raw_y;
            alx_aoa_filt_vx       = 0.0f;
            alx_aoa_filt_vy       = 0.0f;
            alx_aoa_filt_init     = 1;
            alx_aoa_outlier_count = 0;
            alx_aoa_last_filt_ms  = now_ms;
        }

        alx_aoa_x_cm = alx_aoa_filt_x_cm;
        alx_aoa_y_cm = alx_aoa_filt_y_cm;
        return;
    }

    if(0 != alx_aoa_last_filt_ms)
    {
        dt = (float)((uint32)(now_ms - alx_aoa_last_filt_ms)) * 0.001f;
        if(dt < 0.005f)
        {
            dt = 0.005f;
        }
        if(dt > 0.100f)
        {
            dt = 0.100f;
        }
    }
    else
    {
        dt = 0.02f;
    }

    alx_aoa_last_filt_ms = now_ms;

    if(obs_ok)
    {
        alx_aoa_raw_history_push(raw_x, raw_y);
    }

    alx_aoa_raw_history_get_median(&obs_x, &obs_y);

    x_pred = alx_aoa_filt_x_cm + alx_aoa_filt_vx * dt;
    y_pred = alx_aoa_filt_y_cm + alx_aoa_filt_vy * dt;
    rx = obs_x - x_pred;
    ry = obs_y - y_pred;

    if((0U != obs_ok) && (0U != alx_aoa_observation_gate_ok(rx, ry)))
    {
        alx_aoa_outlier_count = 0;

        alx_aoa_filt_x_cm = x_pred + ALX_AOA_AB_ALPHA * rx;
        alx_aoa_filt_y_cm = y_pred + ALX_AOA_AB_ALPHA * ry;
        alx_aoa_filt_vx  += (ALX_AOA_AB_BETA * rx) / dt;
        alx_aoa_filt_vy  += (ALX_AOA_AB_BETA * ry) / dt;
    }
    else if((0U != obs_ok) && (alx_aoa_outlier_count >= (ALX_AOA_REACQUIRE_COUNT - 1U)))
    {
        alx_aoa_filt_x_cm = obs_x;
        alx_aoa_filt_y_cm = obs_y;
        alx_aoa_filt_vx = 0.0f;
        alx_aoa_filt_vy = 0.0f;
        alx_aoa_outlier_count = 0;
    }
    else
    {
        alx_aoa_filt_x_cm = x_pred;
        alx_aoa_filt_y_cm = y_pred;
        alx_aoa_filt_vx  *= ALX_AOA_VEL_DECAY;
        alx_aoa_filt_vy  *= ALX_AOA_VEL_DECAY;

        if(0U != obs_ok)
        {
            alx_aoa_outlier_count ++;
        }
    }

    alx_aoa_x_cm = alx_aoa_filt_x_cm;
    alx_aoa_y_cm = alx_aoa_filt_y_cm;
}

/* 丢弃帧缓冲区前 length 字节（已处理） */
static void alx_aoa_drop_frame_bytes (uint16 length)
{
    if(length >= alx_aoa_frame_length)
    {
        alx_aoa_frame_length = 0;
        return;
    }

    memmove(&alx_aoa_frame_buffer[0],
            &alx_aoa_frame_buffer[length],
            (uint32)(alx_aoa_frame_length - length));
    alx_aoa_frame_length = (uint16)(alx_aoa_frame_length - length);
}

static void alx_aoa_keep_header_prefix (void)
{
    uint16 keep = 0;
    uint16 i;

    while((keep < alx_aoa_frame_length) &&
          (keep < 3) &&
          (ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[alx_aoa_frame_length - 1 - keep]))
    {
        keep ++;
    }

    if(keep != alx_aoa_frame_length)
    {
        for(i = 0; i < keep; i ++)
        {
            alx_aoa_frame_buffer[i] = alx_aoa_frame_buffer[alx_aoa_frame_length - keep + i];
        }
        alx_aoa_frame_length = keep;
    }
}

static uint8 alx_aoa_has_header_at (uint16 index)
{
    return ((ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[index]) &&
            (ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[index + 1]) &&
            (ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[index + 2]) &&
            (ALX_AOA_FRAME_HEADER_BYTE == alx_aoa_frame_buffer[index + 3]));
}

static uint8 alx_aoa_seek_header (void)
{
    uint16 index;

    if(alx_aoa_frame_length < 4)
    {
        alx_aoa_keep_header_prefix();
        return 0;
    }

    for(index = 0; (uint16)(index + 3) < alx_aoa_frame_length; index ++)
    {
        if(alx_aoa_has_header_at(index))
        {
            if(0 != index)
            {
                alx_aoa_drop_frame_bytes(index);
                alx_aoa_stats.frame_bad_header ++;
            }
            return 1;
        }
    }

    alx_aoa_keep_header_prefix();
    alx_aoa_stats.frame_bad_header ++;
    return 0;
}

/* 解析位置帧：提取角度/距离 -> 计算 xy -> alpha-beta 滤波 */
static void alx_aoa_parse_position (const uint8 *frame, uint32 now_ms)
{
    alx_aoa_latest_position.base_id        = alx_aoa_read_be32(&frame[ALX_AOA_OFFSET_ANCHOR_ID]);
    alx_aoa_latest_position.tag_id         = alx_aoa_read_be32(&frame[ALX_AOA_OFFSET_TAG_ID]);
    alx_aoa_latest_position.distance_cm    = alx_aoa_read_be32(&frame[ALX_AOA_OFFSET_DISTANCE]);
    alx_aoa_latest_position.azimuth_deg    = alx_aoa_read_s16(&frame[ALX_AOA_OFFSET_AZIMUTH]);
    alx_aoa_latest_position.elevation_deg  = alx_aoa_read_s16(&frame[ALX_AOA_OFFSET_ELEVATION]);
    alx_aoa_calculate_xy(&alx_aoa_latest_position);
    alx_aoa_filter_xy(&alx_aoa_latest_position, now_ms);
    alx_aoa_latest_position.last_position_ms = now_ms;
    alx_aoa_latest_position.valid            = 1;

    alx_aoa_stats.position_count ++;
    alx_aoa_stats.frame_total ++;
}

/* 从帧缓冲区循环解析：找帧头 -> 读命令 -> 按类型处理 */
static uint8 alx_aoa_parse_stream (uint32 now_ms)
{
    uint16 packet_length;
    uint16 command;
    uint8 parsed_position = 0;

    while(1)
    {
        if(!alx_aoa_seek_header())
        {
            break;
        }

        if(alx_aoa_frame_length < (ALX_AOA_OFFSET_COMMAND + ALX_AOA_COMMAND_FIELD_SIZE))
        {
            break;
        }

        packet_length = alx_aoa_read_be16(&alx_aoa_frame_buffer[ALX_AOA_OFFSET_PACKET_LENGTH]);
        alx_aoa_stats.last_packet_length = packet_length;
        command = alx_aoa_read_be16(&alx_aoa_frame_buffer[ALX_AOA_OFFSET_COMMAND]);
        alx_aoa_stats.last_command = command;

        if(ALX_AOA_CMD_POSITION == command)
        {
            if(alx_aoa_frame_length < ALX_AOA_POSITION_FRAME_SIZE)
            {
                break;
            }

            if(ALX_AOA_POSITION_FRAME_SIZE != packet_length)
            {
                alx_aoa_stats.frame_bad_length ++;
            }

            alx_aoa_parse_position(alx_aoa_frame_buffer, now_ms);
            parsed_position = 1;
            alx_aoa_drop_frame_bytes(ALX_AOA_POSITION_FRAME_SIZE);
        }
        else
        {
            alx_aoa_stats.frame_unknown_cmd ++;
            alx_aoa_drop_frame_bytes(1);
        }
    }

    return parsed_position;
}

static uint8 alx_aoa_feed_parser (uint8 dat, uint32 now_ms)
{
    if(ALX_AOA_MAX_FRAME_SIZE <= alx_aoa_frame_length)
    {
        alx_aoa_stats.frame_bad_length ++;
        alx_aoa_drop_frame_bytes(1);
    }

    alx_aoa_frame_buffer[alx_aoa_frame_length] = dat;
    alx_aoa_frame_length ++;
    alx_aoa_stats.parser_frame_length = alx_aoa_frame_length;

    return alx_aoa_parse_stream(now_ms);
}

void ALX_AOA_Reset (void)
{
    alx_aoa_rx_head = 0;
    alx_aoa_rx_tail = 0;
    alx_aoa_frame_length = 0;
    memset((void *)alx_aoa_rx_buffer, 0, sizeof(alx_aoa_rx_buffer));
    memset(&alx_aoa_frame_buffer[0], 0, sizeof(alx_aoa_frame_buffer));
    memset(&alx_aoa_latest_position, 0, sizeof(alx_aoa_latest_position));
    memset(&alx_aoa_latest_heartbeat, 0, sizeof(alx_aoa_latest_heartbeat));
    memset(&alx_aoa_stats, 0, sizeof(alx_aoa_stats));

    alx_aoa_x_cm          = 0.0f;
    alx_aoa_y_cm          = 0.0f;
    alx_aoa_filt_x_cm     = 0.0f;
    alx_aoa_filt_y_cm     = 0.0f;
    alx_aoa_filt_vx       = 0.0f;
    alx_aoa_filt_vy       = 0.0f;
    memset(&alx_aoa_raw_x_hist[0], 0, sizeof(alx_aoa_raw_x_hist));
    memset(&alx_aoa_raw_y_hist[0], 0, sizeof(alx_aoa_raw_y_hist));
    alx_aoa_raw_hist_count = 0;
    alx_aoa_raw_hist_index = 0;
    alx_aoa_filt_init     = 0;
    alx_aoa_outlier_count = 0;
    alx_aoa_last_filt_ms  = 0;
}

void ALX_AOA_Init (void)
{
    ALX_AOA_Reset();
    uart_init(ALX_AOA_UART_INDEX, ALX_AOA_UART_BAUDRATE, ALX_AOA_UART_TX_PIN, ALX_AOA_UART_RX_PIN);
    uart_rx_interrupt(ALX_AOA_UART_INDEX, 1);
}

void ALX_AOA_InputByte (uint8 dat)
{
    uint16 next = alx_aoa_next_index(alx_aoa_rx_head);

    alx_aoa_stats.rx_bytes ++;
    alx_aoa_stats.rx_last_byte = dat;

    if(next == alx_aoa_rx_tail)
    {
        alx_aoa_stats.rx_overflow ++;
        return;
    }

    alx_aoa_rx_buffer[alx_aoa_rx_head] = dat;
    alx_aoa_rx_head = next;
}

void ALX_AOA_InputBytes (const uint8 *dat, uint16 length)
{
    uint16 i;

    if(0 == dat)
    {
        return;
    }

    for(i = 0; i < length; i ++)
    {
        ALX_AOA_InputByte(dat[i]);
    }
}

uint8 ALX_AOA_Update_25HZ (uint32 now_ms)
{
    uint16 pending = alx_aoa_rx_count();
    uint8 dat;
    uint8 parsed_position = 0;

    while(0 != pending)
    {
        if(!alx_aoa_pop_byte(&dat))
        {
            break;
        }

        if(alx_aoa_feed_parser(dat, now_ms))
        {
            parsed_position = 1;
        }

        pending --;
    }

    alx_aoa_stats.parser_frame_length = alx_aoa_frame_length;

    return parsed_position;
}

uint8 ALX_AOA_GetLatest (ALX_AOA_Position_t *data)
{
    if((0 == data) || (!alx_aoa_latest_position.valid))
    {
        return 0;
    }

    *data = alx_aoa_latest_position;
    return 1;
}

uint8 ALX_AOA_GetFilteredXY (float *x_cm, float *y_cm)
{
    if((0 == x_cm) || (0 == y_cm))
    {
        return 0;
    }

    *x_cm = alx_aoa_x_cm;
    *y_cm = alx_aoa_y_cm;

    return alx_aoa_filt_init;
}

uint8 ALX_AOA_GetHeartbeat (ALX_AOA_Heartbeat_t *data)
{
    if((0 == data) || (!alx_aoa_latest_heartbeat.valid))
    {
        return 0;
    }

    *data = alx_aoa_latest_heartbeat;
    return 1;
}

uint8 ALX_AOA_IsTagOnline (uint32 now_ms, uint32 timeout_ms)
{
    if(!alx_aoa_latest_position.valid)
    {
        return 0;
    }

    return (((uint32)(now_ms - alx_aoa_latest_position.last_position_ms)) <= timeout_ms);
}

void ALX_AOA_GetStats (ALX_AOA_Stats_t *stats)
{
    if(0 == stats)
    {
        return;
    }

    *stats = alx_aoa_stats;
}

void ALX_AOA_ResetStats (void)
{
    memset(&alx_aoa_stats, 0, sizeof(alx_aoa_stats));
}
