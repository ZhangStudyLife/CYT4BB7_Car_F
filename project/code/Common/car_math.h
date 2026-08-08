/**
 * @file car_math.h
 * @brief 通用浮点数学工具，给控制器/估计器/菜单等模块使用
 *
 * 全部 float 版本，单位由调用方保证一致。
 */

#ifndef CAR_MATH_H
#define CAR_MATH_H

#include "zf_common_headfile.h"

/** 取绝对值（float 版），PID 误差计算、死区判断等场景常用 */
float car_math_absf(float value);

/** 返回两数中的较小值 */
float car_math_minf(float a, float b);

/** 返回两数中的较大值 */
float car_math_maxf(float a, float b);

/** 限幅：将 value 钳制到 [min_value, max_value]，PID 输出、参数校验常用 */
float car_math_clampf(float value, float min_value, float max_value);

/** 对称限幅：将 value 钳制到 [-limit, limit]，limit 通常为正数 */
float car_math_limit_absf(float value, float limit);

/**
 * 硬死区：|value| <= deadband 时返回 0，否则原样返回
 * 用于遥控器摇杆、UWB 误差等噪声过滤
 */
float car_math_deadband(float value, float deadband);

/**
 * 软死区（去死区平移）：去掉 deadband 后的剩余量映射到原点开始
 * 例：value=3, deadband=1 → 返回 2；value=0.5, deadband=1 → 返回 0
 * 适用于需要"死区外从零开始线性输出"的场景，如编码器速度
 */
float car_math_soft_deadband(float value, float deadband);

/**
 * 线性映射：将 value 从 [in_min, in_max] 线性映射到 [out_min, out_max]
 * 注意：in_max == in_min 时直接返回 out_min 避免除零
 * 用于遥控器通道值映射、传感器标定等
 */
float car_math_map_linear(float value,
                          float in_min,
                          float in_max,
                          float out_min,
                          float out_max);

float car_speed_mps_to_encoder_cnt(float speed_mps);

float car_speed_encoder_cnt_to_mps(float speed_cnt);

#endif /* CAR_MATH_H */
