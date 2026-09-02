/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
/**
 * @file car_filter.h
 * @brief 通用滤波工具：一阶低通 + 三值中值
 *
 * 用于传感器数据平滑（编码器速度、IMU、UWB 等）。
 * 两个 LPF1 接口区别：
 *   - lpf1_update：有状态版，内部记住上一次输出，适合周期调用
 *   - lpf1_apply：  无状态版，调用方自己传入 previous，适合一次性计算
 */

#ifndef CAR_FILTER_H
#define CAR_FILTER_H

#include "zf_common_headfile.h"

/** 一阶低通滤波器状态 */
typedef struct
{
    float value;  /**< 上一次滤波输出 */
    uint8 ready;  /**< 1=已初始化，0=首次输入时自动初始化 */
} car_filter_lpf1_t;

/**
 * 重置滤波器：直接把输出设为 value，ready 置 1
 * 在切换模式、重新标定后调用
 */
void car_filter_lpf1_reset(car_filter_lpf1_t *filter, float value);

/**
 * 有状态一阶低通：y += alpha * (input - y)
 * @param filter  滤波器状态，首次调用会自动 reset
 * @param input   当前采样值
 * @param alpha   平滑系数 [0,1]，越大跟踪越快、越不平滑；会被自动 clamp
 * @return 滤波后值
 */
float car_filter_lpf1_update(car_filter_lpf1_t *filter, float input, float alpha);

/**
 * 无状态一阶低通：alpha = dt_s / (tau_s + dt_s)
 * @param previous 上一次滤波输出
 * @param input    当前采样值
 * @param dt_s     采样间隔（秒），<=0 时直接返回 input
 * @param tau_s    时间常数（秒），越大越平滑；<=0 时直接返回 input
 * @return 滤波后值
 */
float car_filter_lpf1_apply(float previous, float input, float dt_s, float tau_s);

/**
 * 三值中值滤波：取 a/b/c 的中间值，用于去脉冲噪声
 * 编码器多采样、传感器异常值过滤等场景
 */
float car_filter_median3f(float a, float b, float c);

#endif /* CAR_FILTER_H */
