/* 串级PID控制模块
 *
 * 架构：
 *   100Hz -> yaw角度环(目标0) + yaw角速度环 + 四轮速度环
 */

#include "zf_common_headfile.h"
#ifndef __CONTROL_H__
#define __CONTROL_H__

/* 四轮位置式PID */
extern PositionalPID wheel_left_front_pid;
extern PositionalPID wheel_right_front_pid;
extern PositionalPID wheel_left_rear_pid;
extern PositionalPID wheel_right_rear_pid;

/* 航向PID */
extern PositionalPID yaw_angle_pid;
extern PositionalPID yaw_rate_pid;

/* 调试中间变量 */
extern float control_yaw_angle_current;
extern float control_yaw_angle_output;
extern float control_yaw_rate_target;
extern float control_yaw_rate_current;
extern float control_yaw_rate_output;

void  Control_Init(void);
void  Control_Reset(void);
void  Control_Stop(void);
/* strafe 是底层麦轮解算命令符号，Mode1 会从右正速度目标转换到该符号。 */
void  Control_100Hz(float forward, float strafe, float yaw_target_rad);
float Control_GetYawAngle(void);

#endif
