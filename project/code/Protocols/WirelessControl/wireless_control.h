/**
 * @file wireless_control.h
 * @brief 无线遥控控制层
 *
 * 功能：将 SBUS 通道值转换为小车运动控制指令
 * 上游：SBUS 模块提供通道状态
 * 下游：运动控制模块读取 forward/strafe/rotate 速度
 *
 * 安全机制：
 *   - 接收器离线 → 强制急停
 *   - CH5 使能开关未拨下 → 强制急停
 *   - CH6 模式开关不在有效位置 → 强制急停
 *   - mode2 占位模式下清除所有目标值
 */

#ifndef _WIRELESS_CONTROL_H_
#define _WIRELESS_CONTROL_H_

#include "zf_common_headfile.h"

/* 控制参数 */
#define MAX_CONTROL_SPEED                   (600)     /* 最大平移速度（单位：mm/s 或编码器脉冲/s） */
#define MAX_ANGULAR_SPEED                   (2.0f)    /* 最大旋转角速度（单位：rad/s） */

#define WIRELESS_CONTROL_PERIOD_MS          (40U)     /* 更新周期 40ms = 25Hz */

#define WIRELESS_CONTROL_CHANNEL_COUNT      (6U)      /* 使用的 SBUS 通道数 */
#define WIRELESS_CONTROL_WHEEL_COUNT        (4U)      /* 轮子数量（四轮） */

/**
 * @brief 无线控制状态结构体
 *
 * 使用方式：
 *   1. 25Hz 调 wireless_control_update_25HZ() 刷新
 *   2. 运动控制模块通过 wireless_control_get_state() 读取速度指令
 */
typedef struct
{
    uint16_t raw_channel[WIRELESS_CONTROL_CHANNEL_COUNT];  /* SBUS 原始通道值（透传） */
    int16_t std_channel[WIRELESS_CONTROL_CHANNEL_COUNT];   /* SBUS 标准化通道值（透传） */
    int16_t forward_speed;     /* 前后速度，正值=前进，单位同 MAX_CONTROL_SPEED */
    int16_t strafe_speed;      /* 左右平移速度，正值=右移 */
    float rotate_speed;        /* 旋转角速度，正值=逆时针，单位 rad/s */
    int16_t wheel_target[WIRELESS_CONTROL_WHEEL_COUNT];   /* 各轮目标值（预留） */
    int16_t wheel_pwm[WIRELESS_CONTROL_WHEEL_COUNT];       /* 各轮 PWM（预留） */
    uint8_t receiver_online;   /* SBUS 接收器是否在线：1=在线 */
    uint8_t control_enabled;   /* 控制是否使能：1=使能，0=急停 */
    uint8_t emergency_stop_active;  /* 急停是否激活：1=急停中 */
    uint8_t remote_mode_requested;  /* CH6 请求遥控模式：1=是 */
    uint8_t mode2_requested;        /* CH6 requests the reserved mode2 slot. */
    uint8_t mode_request_valid;     /* 模式请求是否有效：1=有效（CH6 在有效挡位） */
} wireless_control_state_t;

extern wireless_control_state_t g_wireless_control_state;  /* 全局状态 */

/**
 * @brief 初始化无线控制模块
 * 初始化时默认进入急停状态
 */
void wireless_control_init(void);

/**
 * @brief 25Hz 更新无线控制状态
 * 调用频率：25Hz（每 40ms）
 * 内部流程：
 *   1. 读取 SBUS 状态
 *   2. 检查安全条件（CH5 使能 + CH6 模式 + receiver_online）
 *   3. 不满足安全条件 → 强制急停
 *   4. 满足条件 → 解析摇杆值为 forward/strafe/rotate 速度
 */
void wireless_control_update_25HZ(void);

/**
 * @brief 获取当前控制状态指针
 * 返回值：指向 g_wireless_control_state 的只读指针
 */
const wireless_control_state_t *wireless_control_get_state(void);

#endif
