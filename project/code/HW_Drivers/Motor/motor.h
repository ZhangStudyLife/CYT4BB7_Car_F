#ifndef MOTOR_H
#define MOTOR_H

#include "zf_common_headfile.h"

#define MOTOR_LEFT_DIR       (P10_3)
#define MOTOR_LEFT_PWM       (TCPWM_CH30_P10_2)
#define MOTOR_LEFT_INVERT    0

#define MOTOR_RIGHT_DIR      (P09_1)
#define MOTOR_RIGHT_PWM      (TCPWM_CH24_P09_0)
#define MOTOR_RIGHT_INVERT   0

#define MOTOR_PWM_MAX        6000

void motor_init(void);
void motor_left_set_speed(int16_t speed);
void motor_right_set_speed(int16_t speed);
void motor_stop(void);

#endif
