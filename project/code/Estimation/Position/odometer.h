#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#ifndef ODOMETER_BEACON_FIXATOR_ENABLE
#define ODOMETER_BEACON_FIXATOR_ENABLE (1U)
#endif

/**
 * ============================================================================
 *  里程计模块 (Odometer) —— 麦克纳姆轮正运动学 + 机体→水平坐标变换
 * ============================================================================
 *
 *  坐标系约定（水平面输出）：
 *      X 轴：水平面内，指向初始航向的正右方
 *      Y 轴：水平面内，指向初始航向的正前方（X 轴逆时针旋转 90°）
 *      偏航角 yaw：逆时针为正（从上方俯视），符合右手定则
 *
 *  极性速查表（以水平坐标系输出为准）：
 *  ┌──────────┬────────────────────┬─────────────────────────┐
 *  │  运动方向 │  g_odometer.vel    │  g_odometer.position    │
 *  ├──────────┼────────────────────┼─────────────────────────┤
 *  │  右移    │  vel[x] > 0        │  position[x] 增大       │
 *  │  左移    │  vel[x] < 0        │  position[x] 减小       │
 *  │  前进    │  vel[y] > 0        │  position[y] 增大       │
 *  │  后退    │  vel[y] < 0        │  position[y] 减小       │
 *  │  逆时针转 │  yaw 增大（正）     │  （不影响平移位置）      │
 *  │  顺时针转 │  yaw 减小（负）     │  （不影响平移位置）      │
 *  └──────────┴────────────────────┴─────────────────────────┘
 *
 *  推导依据：
 *    1. 编码器层（encoder_control.c）：各轮按实际编码器极性配置 invert，确保
 *       编码器 count_raw > 0 等价于轮子物理正转（车体前进方向）。
 *    2. 正运动学（odometer.c）：
 *         body_vel[x] = ( LF - RF - LR + RR) / 4   ← 右移为正
 *         body_vel[y] = ( LF + RF + LR + RR) / 4   ← 前进为正
 *    3. 控制层（control.c）逆运动学与正运动学自洽：
 *         LF = forward - strafe - rot
 *       底层 strafe 命令沿用既有轮速解算符号，Mode1 会把右正 m/s 目标转换成对应命令。
 *    4. 机体→水平旋转（odometer_body_to_horizontal）：
 *         水平[x] =  cos(yaw)*body[x] - sin(yaw)*body[y]
 *         水平[y] =  sin(yaw)*body[x] + cos(yaw)*body[y]
 *       yaw=0 时水平系 = 机体系；yaw 增大（逆时针）时坐标系跟随旋转。
 *
 *  ODOMETER_BEACON_FIXATOR_ENABLE 只控制运行期信标位置修正是否应用。
 *  初始全局坐标来自 beacon_config，属于地图坐标配置，关闭修正时仍然生效。
 *
 *  注意：g_euler.yaw 由 Mahony AHRS 解算，yaw=0 对应上电时的初始航向。
 *        当前安装/解算链路中 g_euler.yaw 顺时针为正，里程计内部取反后按逆时针为正使用。
 *        odometer_reset() 会重置航向基准，使当前朝向变为 Y 轴正方向。
 * ============================================================================
 */

#include "zf_common_headfile.h"

/* ---- 周期与标定常量 ---- */

#define ODOMETER_UPDATE_DT_S                (0.01f)     /* 更新周期 10ms（100Hz 调用） */
#define ODOMETER_IMU_UPDATE_DT_S            (0.001f)
#define ODOMETER_FORWARD_COUNT_PER_METER    (14000.0f)  /* 惯导日志标定：前向稳健拟合 */
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (15000.0f)  /* 惯导日志标定：横向左右合并保守拟合 */
                                                        /* 前向与横向分轴标定，避免用单一比例覆盖麦轮滑移差异 */

/* ---- 鲁棒滤波与静态门限 ---- */

#define ODOMETER_STATIC_ENCODER_SPEED_MPS   (0.03f)     /* 静态死区：合速度 < 0.03 m/s 时视为静止，输出归零 */
#define ODOMETER_STATIC_ACCEL_MPS2          (0.25f)
#define ODOMETER_ACCEL_BIAS_ALPHA_STATIC    (0.005f)
#define ODOMETER_ENCODER_BLEND_ALPHA        (1.00f)
#define ODOMETER_SLIP_BLEND_ALPHA           (0.15f)
#define ODOMETER_SLIP_INNOVATION_THRESH     (0.50f)
#define ODOMETER_SLIP_ACCEL_DIFF_THRESH     (3.00f)
#define ODOMETER_SLIP_HOLD_TICKS            (8U)
#define ODOMETER_CROSSTALK_Y_FROM_X_GAIN    (0.25f)
#define ODOMETER_CROSSTALK_Y_CORR_MIN_MPS   (0.08f)
#define ODOMETER_CROSSTALK_Y_CORR_RATIO_MIN (0.30f)

/* ---- 轴索引 ---- */

enum
{
    x = 0,                          /* X 轴索引（右方为正） */
    y = 1,                          /* Y 轴索引（前进方向） */
    ODOMETER_X = x,
    ODOMETER_Y = y,
    ODOMETER_AXIS_NUM = 2           /* 二维平面，X/Y 两轴 */
};

/**
 * 里程计输出数据结构
 *
 * position 和 vel 均为水平坐标系下的二维向量：
 *   [x] = 横向分量（正 = 右移，负 = 左移）
 *   [y] = 前进方向分量（正 = 前进，负 = 后退）
 *
 * position 由 vel 对时间积分得到，代表相对于 odometer_reset() 时刻的累计位移。
 * vel 经过静态死区滤波，静止时输出 0。
 */
typedef struct
{
    float position[ODOMETER_AXIS_NUM];  /* 累计位置 [m]，水平坐标系 */
    float vel[ODOMETER_AXIS_NUM];       /* 瞬时速度 [m/s]，水平坐标系 */
    float acc[ODOMETER_AXIS_NUM];       /* 水平坐标系加速度 [m/s^2] */
    float body_vel[ODOMETER_AXIS_NUM];  /* 车体系速度 [m/s]，x右移，y前进 */
} odometer_data_t;

/**
 * 全局里程计实例
 *
 * 使用方式：
 *   g_odometer.vel[x]      → 当前横移速度（m/s），正=右移，负=左移
 *   g_odometer.vel[y]      → 当前前进速度（m/s），正=前进，负=后退
 *   g_odometer.position[x] → 累计横移位移（m），正=在起点右侧
 *   g_odometer.position[y] → 累计前进位移（m），正=在起点前方
 *
 * 原理链路：
 *   编码器脉冲 → 一阶低通滤波 → 麦轮正运动学（机体坐标）
 *   → 鲁棒门控（打滑检测+中值替换） → 机体→水平坐标旋转 → 速度/位置积分
 */
extern odometer_data_t g_odometer;

/* 初始化里程计模块（上电调用一次，内部调用 odometer_reset） */
void odometer_init(void);

/* 重置里程计：清零速度/位置/偏航基准，当前朝向变为 Y 轴正方向 */
void odometer_reset(void);

/* 100Hz 周期更新：读取四轮编码器 → 正运动学 → 坐标变换 → 积分 */
void odometer_update_100HZ(void);
void odometer_update_1000HZ(void);

/* 获取相对于最近一次odometer_reset()航向零点的当前实际航向，单位rad。 */
float odometer_get_heading_rad(void);

/**
 * @brief 立即应用fixator生成的待修正位置，使后续模块读取修正后的全局坐标。
 * @param 无。
 * @return 无。
 */
void odometer_apply_pending_fix(void);

#endif
