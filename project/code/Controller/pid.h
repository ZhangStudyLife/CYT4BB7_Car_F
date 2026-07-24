/*********************************************************************************************************************
* PID控制器模块 - 头文件
*
* 位置式PID：用于航向角控制（外环）
*   输入/输出单位：角度(°) → 角速度(rad/s)
*
* 增量式PID：用于轮速控制（内环）
*   输入/输出单位：编码器计数(pulse) → PWM增量
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef __PID_H__
#define __PID_H__


/* 位置式PID结构体
 * 用于航向角控制（角度环、角速度环）
 * 输入：target-current的误差
 * 输出：控制量（角度环输出角速度，角速度环输出编码器计数）
 * 特点：p项使用二次+线性组合，误差正负时公式不同
 */
typedef struct
{
    float kp_2;         // 二次比例系数（误差平方项权重）
    float kp_1;         // 线性比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分累积值
    float prev_err;     // 上一次误差
    float p_term;       // 最近一次P项输出（调试用）
    float i_term;       // 最近一次I项输出（调试用）
    float d_term;       // 最近一次D项输出（调试用）
    float output;       // 最近一次总输出（调试用）
    float i_limit;      // 积分限幅值（防止积分饱和）
    float output_limit; // 输出限幅值
} PositionalPID;

/* 增量式PID结构体
 * 用于四轮速度环控制
 * 输入：目标编码器计数 - 实际编码器计数
 * 输出：PWM增量（累加到output上，范围±output_limit）
 * 优点：不会积分饱和，切换时输出变化平滑
 */
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float last_error;   // 上一次误差
    float prev_error;   // 上上次误差
    float p_term;       // 最近一次P项增量（调试用）
    float i_term;       // 最近一次I项增量（调试用）
    float d_term;       // 最近一次D项增量（调试用）
    float increment;    // 最近一次总增量（调试用）
    float output;       // 当前输出（PWM值，累加型）
    float output_limit; // 输出限幅（绝对值上限）
} IncrementPID;

/* 初始化位置式PID
 * i_limit: 积分限幅（积分累积值超过此值会被钳位）
 * output_limit: 输出限幅（输出超过此值会被钳位）
 */
void PositionalPID_Init(PositionalPID* pid, float kp_2, float kp_1, float ki, float kd,
                        float i_limit, float output_limit);

/* 更新位置式PID
 * target: 目标值
 * current: 当前值
 * 返回: 控制输出（已限幅）
 * 注意：p项公式为 kp_2*err^2 + kp_1*err，误差负时二次项取反
 */
float PositionalPID_Update(PositionalPID* pid, float target, float current);

/* 初始化增量式PID
 * output_limit: 输出限幅（PWM绝对值上限）
 */
void IncrementPID_Init(IncrementPID* pid, float kp, float ki, float kd,
                       float output_limit);

/* 更新增量式PID
 * target: 目标编码器计数（周期内的脉冲数）
 * current: 实际编码器计数（周期内的脉冲数）
 * 返回: 累加后的PWM输出（已限幅）
 * 调用频率：100HZ（由Control_100Hz调用）
 */
float IncrementPID_Update(IncrementPID *pid, float target, float current);

#endif
