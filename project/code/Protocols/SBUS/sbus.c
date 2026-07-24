/**
 * @file sbus.c
 * @brief SBUS 遥控器解码实现
 *
 * 处理路径：
 *   正常帧 → uart_receiver.finsh_flag=1 → 清零超时计数器 → 标准化映射 → 输出
 *   无帧超时 → 超时计数器递增到阈值 → receiver_online=0 → 上层强制急停
 *   开关值异常 → channel_valid[i]=0 → 上层按无效处理（强制急停）
 */

#include "sbus.h"

/* 中心死区：摇杆在中心附近 ±50 范围内输出 0，防止漂移导致误动作 */
#define SBUS_CENTER_DEADZONE            (50U)

/* 开关容差：原始值与目标挡位差值在此范围内认为拨到位了 */
#define SBUS_SWITCH_TOLERANCE           (50U)

/* 帧超时阈值：底层连续这么长时间没收到新帧就判离线 */
#define SBUS_FRAME_TIMEOUT_MS           (100U)

/* 更新周期：sbus_update_25HZ 的调用间隔 */
#define SBUS_PERIOD_MS                  (40U)

/* 超时计数上限 = ceil(140ms / 40ms) = 4 个周期无帧则离线 */
#define SBUS_FRAME_TIMEOUT_CYCLES       ((SBUS_FRAME_TIMEOUT_MS + SBUS_PERIOD_MS - 1U) / SBUS_PERIOD_MS)

/**
 * @brief 各通道原始值下限（出厂校准值）
 * 含义：对应通道摇杆推到最下/最左时的 ADC 值
 * 注意：不同通道的下限不同（硬件差异），CH5/CH6 是开关，没有中心值
 */
static const uint16 s_sbus_channel_min[SBUS_USED_CHANNEL_COUNT] =
{
    322U,   /* CH1: 摇杆最小值 */
    204U,   /* CH2 */
    306U,   /* CH3 */
    306U,   /* CH4 */
    204U,   /* CH5: 开关拨下位值 */
    240U    /* CH6: 开关拨下位值 */
};

/**
 * @brief 各通道中心值（仅 CH1~CH4 有效）
 * 含义：摇杆居中时的 ADC 值
 * CH5/CH6 是开关，中心值无意义填 0
 */
static const uint16 s_sbus_channel_center[SBUS_USED_CHANNEL_COUNT] =
{
    1028U,  /* CH1 */
    1025U,  /* CH2 */
    1040U,  /* CH3 */
    1028U,  /* CH4 */
    0U,     /* CH5: 开关，无中心 */
    0U      /* CH6: 开关，无中心 */
};

/**
 * @brief 各通道原始值上限（出厂校准值）
 * 含义：对应通道摇杆推到最上/最右时的 ADC 值
 */
static const uint16 s_sbus_channel_max[SBUS_USED_CHANNEL_COUNT] =
{
    1807U,  /* CH1: 摇杆最大值 */
    1805U,  /* CH2 */
    1777U,  /* CH3 */
    1754U,  /* CH4 */
    1807U,  /* CH5: 开关拨上位值 */
    1807U   /* CH6: 开关拨上位值 */
};

sbus_state_t g_sbus_state = {0};           /* 全局 SBUS 状态 */
static uint8 s_frame_timeout_counter = SBUS_FRAME_TIMEOUT_CYCLES;  /* 超时计数器，初始值=超限（离线态） */

/**
 * @brief 将原始值钳位到 [min_value, max_value]
 * @param value 原始值
 * @param min_value 下限
 * @param max_value 上限
 * @return 钳位后的值
 */
static uint16 sbus_clamp_raw(uint16 value, uint16 min_value, uint16 max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief 将摇杆原始值映射为标准化轴值（-1000~+1000）
 *
 * 映射逻辑：
 *   - 中心死区 ±50 内输出 0
 *   - 死区以下线性映射到 -1000~0
 *   - 死区以上线性映射到 0~+1000
 *   - 单位：无量纲，1000 代表满量程
 *
 * @param raw_value 原始 ADC 值
 * @param min_value 校准下限
 * @param center_value 校准中心
 * @param max_value 校准上限
 * @return 标准化值 -1000~+1000
 */
static int16 sbus_map_axis(uint16 raw_value,
                           uint16 min_value,
                           uint16 center_value,
                           uint16 max_value)
{
    uint16 value;
    uint16 lower_deadzone_edge;
    uint16 upper_deadzone_edge;
    int32 mapped;

    value = sbus_clamp_raw(raw_value, min_value, max_value);
    lower_deadzone_edge = center_value - SBUS_CENTER_DEADZONE;
    upper_deadzone_edge = center_value + SBUS_CENTER_DEADZONE;

    /* 死区以下：负方向线性映射 */
    if(value < lower_deadzone_edge)
    {
        mapped = -((int32)(lower_deadzone_edge - value) * 1000) /
                 (int32)(lower_deadzone_edge - min_value);
        return (int16)mapped;
    }

    /* 死区以上：正方向线性映射 */
    if(value > upper_deadzone_edge)
    {
        mapped = ((int32)(value - upper_deadzone_edge) * 1000) /
                 (int32)(max_value - upper_deadzone_edge);
        return (int16)mapped;
    }

    /* 死区内 */
    return 0;
}

/**
 * @brief 将开关原始值判断为上/下/无效
 *
 * 判定逻辑：原始值与目标挡位的差值 ≤ SBUS_SWITCH_TOLERANCE(50) 即认为到位
 * 如果两个挡位都不在容差内，返回 SBUS_STD_SWITCH_INVALID(-1)
 *
 * @param raw_value 开关原始 ADC 值
 * @param up_value 开关拨到"上"时的期望值
 * @param down_value 开关拨到"下"时的期望值
 * @return 0=上, 1=下, -1=无效
 */
static int16 sbus_map_switch_2pos(uint16 raw_value, uint16 up_value, uint16 down_value)
{
    int32 delta_up;
    int32 delta_down;

    delta_up = (int32)raw_value - (int32)up_value;
    delta_down = (int32)raw_value - (int32)down_value;

    if(delta_up < 0)
    {
        delta_up = -delta_up;
    }

    if(delta_down < 0)
    {
        delta_down = -delta_down;
    }

    if(delta_up <= (int32)SBUS_SWITCH_TOLERANCE)
    {
        return SBUS_STD_SWITCH_UP;
    }

    if(delta_down <= (int32)SBUS_SWITCH_TOLERANCE)
    {
        return SBUS_STD_SWITCH_DOWN;
    }

    return SBUS_STD_SWITCH_INVALID;
}

/**
 * @brief 检查底层是否有新帧到达，更新超时计数器和在线状态
 *
 * 超时机制：
 *   - 每次调用时检查 uart_receiver.finsh_flag
 *   - 有新帧：清零计数器，置 frame_updated=1
 *   - 无新帧：计数器递增（上限 = SBUS_FRAME_TIMEOUT_CYCLES）
 *   - 计数器 ≥ 阈值 且 receiver.state != 1 → receiver_online=0（离线）
 */
static void sbus_refresh_receiver_watchdog(void)
{
    if(1U == uart_receiver.finsh_flag)
    {
        /* 有新帧到达：重置超时计数器 */
        s_frame_timeout_counter = 0U;
        g_sbus_state.frame_updated = 1U;
        uart_receiver.finsh_flag = 0U;
    }
    else
    {
        /* 本轮无新帧 */
        g_sbus_state.frame_updated = 0U;

        if(s_frame_timeout_counter < SBUS_FRAME_TIMEOUT_CYCLES)
        {
            s_frame_timeout_counter++;
        }
    }

    /* 在线条件：底层串口状态正常 且 未超过超时阈值 */
    g_sbus_state.receiver_online =
        ((1U == uart_receiver.state) && (s_frame_timeout_counter < SBUS_FRAME_TIMEOUT_CYCLES)) ? 1U : 0U;
}

/**
 * @brief 从底层 uart_receiver 快照通道值并做标准化映射
 *
 * 映射规则：
 *   - CH1: 摇杆轴，取反（适配硬件方向）
 *   - CH2~CH4: 摇杆轴，正常方向
 *   - CH5~CH6: 二挡开关
 *   - CH1~CH4 默认 channel_valid=1，CH5/CH6 只有开关值有效时才置 1
 */
static void sbus_snapshot_channels(void)
{
    uint8 index;

    /* 先把底层原始值拷贝过来 */
    for(index = 0U; index < SBUS_CHANNEL_COUNT; index++)
    {
        g_sbus_state.raw_channel[index] = uart_receiver.channel[index];
        g_sbus_state.std_channel[index] = 0;
        g_sbus_state.channel_valid[index] = 0U;
    }

    /* CH1: 摇杆轴，取反适配 */
    g_sbus_state.std_channel[SBUS_CH1] =
        (int16)-sbus_map_axis(g_sbus_state.raw_channel[SBUS_CH1],
                              s_sbus_channel_min[SBUS_CH1],
                              s_sbus_channel_center[SBUS_CH1],
                              s_sbus_channel_max[SBUS_CH1]);
    /* CH2: 摇杆轴 */
    g_sbus_state.std_channel[SBUS_CH2] =
        sbus_map_axis(g_sbus_state.raw_channel[SBUS_CH2],
                      s_sbus_channel_min[SBUS_CH2],
                      s_sbus_channel_center[SBUS_CH2],
                      s_sbus_channel_max[SBUS_CH2]);
    /* CH3: 摇杆轴 */
    g_sbus_state.std_channel[SBUS_CH3] =
        sbus_map_axis(g_sbus_state.raw_channel[SBUS_CH3],
                      s_sbus_channel_min[SBUS_CH3],
                      s_sbus_channel_center[SBUS_CH3],
                      s_sbus_channel_max[SBUS_CH3]);
    /* CH4: 摇杆轴 */
    g_sbus_state.std_channel[SBUS_CH4] =
        sbus_map_axis(g_sbus_state.raw_channel[SBUS_CH4],
                      s_sbus_channel_min[SBUS_CH4],
                      s_sbus_channel_center[SBUS_CH4],
                      s_sbus_channel_max[SBUS_CH4]);
    /* CH5: 二挡开关（总使能） */
    g_sbus_state.std_channel[SBUS_CH5] =
        sbus_map_switch_2pos(g_sbus_state.raw_channel[SBUS_CH5],
                             s_sbus_channel_min[SBUS_CH5],
                             s_sbus_channel_max[SBUS_CH5]);
    /* CH6: 二挡开关（模式选择） */
    g_sbus_state.std_channel[SBUS_CH6] =
        sbus_map_switch_2pos(g_sbus_state.raw_channel[SBUS_CH6],
                             s_sbus_channel_min[SBUS_CH6],
                             s_sbus_channel_max[SBUS_CH6]);

    /* 标记通道有效性 */
    for(index = 0U; index < SBUS_USED_CHANNEL_COUNT; index++)
    {
        if((index == SBUS_CH5) || (index == SBUS_CH6))
        {
            /* 开关通道：值必须是有效挡位 */
            g_sbus_state.channel_valid[index] =
                (SBUS_STD_SWITCH_INVALID != g_sbus_state.std_channel[index]) ? 1U : 0U;
        }
        else
        {
            /* 摇杆轴：映射后总是有效 */
            g_sbus_state.channel_valid[index] = 1U;
        }
    }
}

/**
 * @brief SBUS 模块初始化
 * 重置所有通道为 0，接收器状态为离线
 * 调用时机：系统启动时调一次
 */
void sbus_init(void)
{
    uint8 index;

    s_frame_timeout_counter = SBUS_FRAME_TIMEOUT_CYCLES;
    g_sbus_state.receiver_online = 0U;
    g_sbus_state.frame_updated = 0U;

    for(index = 0U; index < SBUS_CHANNEL_COUNT; index++)
    {
        g_sbus_state.raw_channel[index] = 0U;
        g_sbus_state.std_channel[index] = 0;
        g_sbus_state.channel_valid[index] = 0U;
    }
}

/**
 * @brief 25Hz 更新入口
 * 调用频率：25Hz（每 40ms），由主循环或定时器驱动
 * 内部流程：刷新看门狗 → 快照通道并标准化
 */
void sbus_update_25HZ(void)
{
    sbus_refresh_receiver_watchdog();
    sbus_snapshot_channels();
}

/**
 * @brief 获取当前 SBUS 状态
 * 返回值：指向 g_sbus_state 的只读指针
 * 谁用：WirelessControl 模块每 25Hz 读一次
 */
const sbus_state_t *sbus_get_state(void)
{
    return &g_sbus_state;
}
