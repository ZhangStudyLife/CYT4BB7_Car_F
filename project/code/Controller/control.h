/* 两轮差速底盘控制模块
 *
 * 控制链路：
 *   Control_WheelSpeed100Hz：左右轮目标脉冲/10ms -> 左右轮速度PID -> PWM
 *   Control_Twist100Hz：线速度/角速度命令 -> 两轮差速解算 -> 左右轮速度闭环
 *   Control_YawRate100Hz：角速度PID -> 差速解算 -> 左右轮速度闭环
 *   Control_Heading100Hz：航向角PID -> 角速度PID -> 差速解算 -> 左右轮速度闭环
 */

#include "zf_common_headfile.h"
#ifndef __CONTROL_H__
#define __CONTROL_H__

typedef enum
{
    CAR_CONTROL_COMMAND_STOP = 0,
    CAR_CONTROL_COMMAND_PWM,
    CAR_CONTROL_COMMAND_WHEEL_SPEED,
    CAR_CONTROL_COMMAND_TWIST,
    CAR_CONTROL_COMMAND_YAW_RATE,
    CAR_CONTROL_COMMAND_HEADING
} car_control_command_mode_t;

/*
 * 遥控层、任务层和菜单调试层共用的控制命令。
 * 每种模式只使用对应字段，其余字段应保持为0。
 */
typedef struct
{
    car_control_command_mode_t mode;
    uint8 enabled;
    float left_pwm;
    float right_pwm;
    float left_target_count;
    float right_target_count;
    float linear_mps;
    float yaw_rate_rad_s;
    float heading_target_rad;
} car_control_command_t;

/* 左右驱动轮位置式PID */
extern PositionalPID wheel_left_pid;
extern PositionalPID wheel_right_pid;

/* 航向PID */
extern PositionalPID yaw_angle_pid;
extern PositionalPID yaw_rate_pid;

/* 调试中间变量 */
extern float control_yaw_angle_current;
extern float control_yaw_angle_output;
extern float control_yaw_rate_target;
extern float control_yaw_rate_current;
extern float control_yaw_rate_output;
extern float control_heading_target;
extern float control_heading_error;

/* 左右轮闭环调试量：目标/反馈单位为脉冲/10ms，输出单位为PWM计数。 */
extern float control_left_wheel_target_count;
extern float control_right_wheel_target_count;
extern float control_left_wheel_feedback_count;
extern float control_right_wheel_feedback_count;
extern float control_left_wheel_feedforward_pwm;
extern float control_right_wheel_feedforward_pwm;
extern float control_left_wheel_output_pwm;
extern float control_right_wheel_output_pwm;

void  Control_Init(void);
void  Control_Reset(void);
void  Control_Stop(void);

/*
 * 左右轮速度闭环。
 * left/right_target_count：每个100Hz控制周期的编码器脉冲目标，正值表示车辆前进。
 */
void Control_WheelSpeed100Hz(float left_target_count, float right_target_count);

/*
 * 两轮差速解算。
 * linear_mps：车体前向线速度，前进为正，单位m/s。
 * yaw_rate_rad_s：俯视逆时针角速度为正，单位rad/s。
 */
void Control_Twist100Hz(float linear_mps, float yaw_rate_rad_s);

/*
 * 两轮角速度闭环。
 * linear_mps：车体前向线速度，单位m/s。
 * yaw_rate_target_rad_s：目标角速度，俯视逆时针为正，单位rad/s。
 */
void Control_YawRate100Hz(float linear_mps, float yaw_rate_target_rad_s);

/*
 * 两轮航向串级闭环。
 * heading_target_rad：目标航向角，内部按最短路径归一化到[-pi, pi]。
 */
void Control_Heading100Hz(float linear_mps, float heading_target_rad);

/* 100Hz唯一输出执行器：所有输入源必须通过该接口到达电机。 */
void Control_ExecuteCommand100Hz(const car_control_command_t *command);

float Control_GetYawAngle(void);

#endif
