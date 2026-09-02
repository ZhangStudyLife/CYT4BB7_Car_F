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
#ifndef MOTOR_H
#define MOTOR_H

#include "zf_common_headfile.h"

#define MOTOR_RIGHT_DIR       (P10_3)
#define MOTOR_RIGHT_PWM       (TCPWM_CH30_P10_2)
#define MOTOR_RIGHT_INVERT   1

#define MOTOR_LEFT_DIR      (P09_1)
#define MOTOR_LEFT_PWM      (TCPWM_CH24_P09_0)
#define MOTOR_LEFT_INVERT   0

#define MOTOR_PWM_MAX        9000

void motor_init(void);
void motor_left_set_speed(int16_t speed);
void motor_right_set_speed(int16_t speed);
void motor_stop(void);

#endif
