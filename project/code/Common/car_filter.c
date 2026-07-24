/**
 * @file car_filter.c
 * @brief 滤波工具实现
 */

#include "car_filter.h"


void car_filter_lpf1_reset(car_filter_lpf1_t *filter, float value)
{
    if(0 == filter)
    {
        return;
    }

    filter->value = value;
    filter->ready = 1U;
}

/* 有状态低通：alpha 自动 clamp 到 [0,1]，首次输入自动初始化 */
float car_filter_lpf1_update(car_filter_lpf1_t *filter, float input, float alpha)
{
    if(0 == filter)
    {
        return input;
    }

    if(alpha < 0.0f)
    {
        alpha = 0.0f;
    }

    if(alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    if(0U == filter->ready)
    {
        car_filter_lpf1_reset(filter, input);
        return input;
    }

    filter->value += alpha * (input - filter->value);
    return filter->value;
}

/* 无状态低通：参数非法时直通返回 input，避免错误滤波 */
float car_filter_lpf1_apply(float previous, float input, float dt_s, float tau_s)
{
    float alpha;

    if((tau_s <= 0.0f) || (dt_s <= 0.0f))
    {
        return input;
    }

    alpha = dt_s / (tau_s + dt_s);
    return previous + (alpha * (input - previous));
}

/* 三值中值：三次比较交换排序后取中间值 */
float car_filter_median3f(float a, float b, float c)
{
    float t;

    if(a > b)
    {
        t = a;
        a = b;
        b = t;
    }

    if(b > c)
    {
        t = b;
        b = c;
        c = t;
    }

    if(a > b)
    {
        b = a;
    }

    return b;
}
