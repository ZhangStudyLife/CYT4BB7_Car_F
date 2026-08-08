/**
 * @file car_math.c
 * @brief 通用浮点数学工具实现
 */

#include "car_math.h"

#define CAR_SPEED_ENCODER_CNT_PER_MPS (115.0f)

float car_math_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

float car_math_minf(float a, float b)
{
    return (a < b) ? a : b;
}

float car_math_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

float car_math_clampf(float value, float min_value, float max_value)
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

float car_math_limit_absf(float value, float limit)
{
    if(value > limit)
    {
        return limit;
    }

    if(value < -limit)
    {
        return -limit;
    }

    return value;
}

/* 硬死区：落在 deadband 范围内直接归零 */
float car_math_deadband(float value, float deadband)
{
    if(car_math_absf(value) <= deadband)
    {
        return 0.0f;
    }

    return value;
}

/* 软死区：去掉死区宽度后，剩余量从零开始输出，保留符号 */
float car_math_soft_deadband(float value, float deadband)
{
    float magnitude;

    magnitude = car_math_absf(value) - deadband;
    if(magnitude <= 0.0f)
    {
        return 0.0f;
    }

    return (value >= 0.0f) ? magnitude : -magnitude;
}

/* 线性映射，in_max == in_min 时返回 out_min 避免除零 */
float car_math_map_linear(float value,
                          float in_min,
                          float in_max,
                          float out_min,
                          float out_max)
{
    if(in_max == in_min)
    {
        return out_min;
    }

    return out_min + ((value - in_min) * (out_max - out_min) / (in_max - in_min));
}

float car_speed_mps_to_encoder_cnt(float speed_mps)
{
    return speed_mps * CAR_SPEED_ENCODER_CNT_PER_MPS;
}

float car_speed_encoder_cnt_to_mps(float speed_cnt)
{
    return speed_cnt / CAR_SPEED_ENCODER_CNT_PER_MPS;
}
