#include "zf_common_headfile.h"
#ifndef __MOTOR_H
#define __MOTOR_H

/*
 * 电机引脚映射
 * M1/M2 按两轮底盘逻辑编号：M1=左轮，M2=右轮；M3/M4 保留原麦轮硬件映射。
 * DIR 引脚控制方向（GPIO），PWM 引脚输出占空比（TCPWM）。
 * INVERT：机械装配导致正转方向与期望相反时置 1，内部会自动取反 speed。
 * PWM 频率 17kHz（见 motor.c pwm_init），占空比范围 0~MOTOR_PWM_MAX。
 */

// 电机1 - 左轮 (Left)
#define MOTOR_M1_DIR        (P10_3)
#define MOTOR_M1_PWM        (TCPWM_CH30_P10_2)
#define MOTOR_M1_INVERT     0       // 0=不反转, 1=反转

// 电机2 - 右轮 (Right)
#define MOTOR_M2_DIR        (P09_1)
#define MOTOR_M2_PWM        (TCPWM_CH24_P09_0)
#define MOTOR_M2_INVERT     0       // 0=不反转, 1=反转

// 电机3 - 左后 (Left Rear)  -> 宏名 M4
#define MOTOR_M4_DIR        (P05_3)
#define MOTOR_M4_PWM        (TCPWM_CH11_P05_2)
#define MOTOR_M4_INVERT     1       // 0=不反转, 1=反转

// 电机4 - 右后 (Right Rear) -> 宏名 M3
#define MOTOR_M3_DIR        (P05_1)
#define MOTOR_M3_PWM        (TCPWM_CH09_P05_0)
#define MOTOR_M3_INVERT     0       // 0=不反转, 1=反转

#define MOTOR_PWM_MAX           6000        // 软件PWM限幅；底层PWM_DUTY_MAX=10000，因此对应60%占空比

/*
 * 两轮底盘主接口。
 * 逻辑约定：M1=左轮、M2=右轮，正 speed 表示车辆前进，负 speed 表示车辆后退。
 */
void two_wheel_motor_init(void);
void two_wheel_motor_set(int16_t left_speed, int16_t right_speed);
void two_wheel_motor_stop(void);

/* 旧四轮麦轮兼容接口；两轮底盘正常控制流程不再使用。 */
void mecanum_motor_init(void);

/* PWM 限幅：将 speed 限制在 [-MOTOR_PWM_MAX, MOTOR_PWM_MAX] */
int16_t speed_limit(int16_t speed);

/*
 * 设置单个电机速度（底层接口，一般不直接调用）
 * speed: 正值正转，负值反转，单位 = PWM 占空比计数值（0~5000）
 * invert: 为 1 时内部取反 speed，用于校正机械装配方向
 */
void motor_set_single(gpio_pin_enum dir_pin, pwm_channel_enum pwm_ch, int16_t speed, uint8_t invert);

/* 设置电机1（物理左轮）速度，speed 单位同上 */
void motor_m1_set_speed(int16_t speed);

/* 设置电机2（物理右轮）速度 */
void motor_m2_set_speed(int16_t speed);

/* 旧麦轮接口：设置电机3（物理右后轮）速度 */
void motor_m3_set_speed(int16_t speed);

/* 旧麦轮接口：设置电机4（物理左后轮）速度 */
void motor_m4_set_speed(int16_t speed);

/* 同时设置四轮速度（麦轮解算结果直接传入），上层控制用这个 */
void mecanum_motor_set_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/* 紧急停车：四轮 speed 全部置 0 */
void mecanum_motor_stop(void);

#endif

