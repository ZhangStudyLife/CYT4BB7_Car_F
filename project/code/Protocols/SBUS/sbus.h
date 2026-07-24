/**
 * @file sbus.h
 * @brief SBUS 遥控接收器解码模块
 *
 * 帧格式（底层由 uart_receiver 驱动，本层只做标准化映射）：
 *   - SBUS 原始帧 25 字节，波特率 100000，8E2，每帧包含 16 个通道（11bit 精度）
 *   - 本模块只取前 6 个通道（CH1~CH6），其中 CH1~CH4 为摇杆轴（-1000~+1000），
 *     CH5~CH6 为二挡开关（上=0 / 下=1 / 无效=-1）
 *
 * 超时策略：
 *   - 底层每 ~14ms 收一帧，本模块每 40ms 调一次 sbus_update_25HZ()
 *   - 超过 SBUS_FRAME_TIMEOUT_MS(100ms) + SBUS_PERIOD_MS(40ms) = 3 个周期未收到帧
 *     判定 receiver_online = 0
 *
 * 使用方式：
 *   1. 系统启动时调 sbus_init()
 *   2. 25Hz 主循环调 sbus_update_25HZ()
 *   3. 上层通过 sbus_get_state() 获取最新状态
 */

#ifndef SBUS_H
#define SBUS_H

#include "zf_common_headfile.h"

#define SBUS_CHANNEL_COUNT              (10U)   /* SBUS 原始通道总数（硬件帧） */
#define SBUS_USED_CHANNEL_COUNT         (6U)    /* 本模块实际使用的通道数 */

/* 标准化后的轴范围定义 */
#define SBUS_STD_AXIS_MIN               (-1000) /* 摇杆轴最小值 */
#define SBUS_STD_AXIS_MAX               (1000)  /* 摇杆轴最大值 */

/* 二挡开关的标准值 */
#define SBUS_STD_SWITCH_UP              (0)     /* 开关拨到上方 */
#define SBUS_STD_SWITCH_DOWN            (1)     /* 开关拨到下方 */
#define SBUS_STD_SWITCH_INVALID         (-1)    /* 开关值异常，不在两个挡位容差内 */

/* 通道索引（映射到标准功能） */
#define SBUS_CH1                        (0U)    /* CH1: 左右平移（strafe） */
#define SBUS_CH2                        (1U)    /* CH2: 前后移动（forward） */
#define SBUS_CH3                        (2U)    /* CH3: 油门/速度 */
#define SBUS_CH4                        (3U)    /* CH4: 旋转（rotate） */
#define SBUS_CH5                        (4U)    /* CH5: 总使能开关 */
#define SBUS_CH6                        (5U)    /* CH6: 模式选择（遥控/UWB跟随） */

/**
 * @brief SBUS 解码后的状态结构体
 *
 * 谁用：WirelessControl 模块通过 sbus_get_state() 读取
 * 何时更新：sbus_update_25HZ() 每 40ms 刷新一次
 */
typedef struct
{
    uint16 raw_channel[SBUS_CHANNEL_COUNT];    /* 原始通道值（0~2047 级） */
    int16 std_channel[SBUS_CHANNEL_COUNT];    /* 标准化通道值：轴=-1000~+1000，开关=0/1/-1 */
    uint8 channel_valid[SBUS_CHANNEL_COUNT];  /* 通道有效标志：1=有效，0=异常 */
    uint8 receiver_online;                     /* 接收器在线：1=在线，0=超时离线 */
    uint8 frame_updated;                       /* 本轮是否收到新帧：1=有新数据 */
} sbus_state_t;

extern sbus_state_t g_sbus_state;  /* 全局 SBUS 状态，上层可直接读取 */

/**
 * @brief 初始化 SBUS 模块
 * 重置所有通道值为 0，接收器状态为离线
 */
void sbus_init(void);

/**
 * @brief 25Hz 更新 SBUS 状态
 * 调用频率：25Hz（每 40ms）
 * 内部操作：
 *   1. 检查底层 uart_receiver 是否有新帧 → 更新超时计数器
 *   2. 将原始通道值标准化为 -1000~+1000 或开关状态
 */
void sbus_update_25HZ(void);

/**
 * @brief 获取当前 SBUS 状态指针
 * 返回值：指向全局 g_sbus_state 的只读指针
 * 注意：返回的指针在下次 sbus_update_25HZ() 调用后可能被覆盖
 */
const sbus_state_t *sbus_get_state(void);

#endif /* SBUS_H */
