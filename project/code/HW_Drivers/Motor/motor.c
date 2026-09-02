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
#include "motor.h"

static int16_t motor_limit_speed(int16_t speed)
{
    if (speed > MOTOR_PWM_MAX)
    {
        return MOTOR_PWM_MAX;
    }

    if (speed < -MOTOR_PWM_MAX)
    {
        return -MOTOR_PWM_MAX;
    }

    return speed;
}

static void motor_set(gpio_pin_enum dir_pin, pwm_channel_enum pwm_ch, int16_t speed, uint8_t invert)
{
    speed = motor_limit_speed(speed);

    if (invert)
    {
        speed = -speed;
    }

    if (speed >= 0)
    {
        gpio_set_level(dir_pin, GPIO_HIGH);
        pwm_set_duty(pwm_ch, speed);
    }
    else
    {
        gpio_set_level(dir_pin, GPIO_LOW);
        pwm_set_duty(pwm_ch, -speed);
    }
}

void motor_init(void)
{
    gpio_init(MOTOR_LEFT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_LEFT_PWM, 17000, 0);

    gpio_init(MOTOR_RIGHT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_RIGHT_PWM, 17000, 0);

    motor_stop();
}

void motor_left_set_speed(int16_t speed)
{
    motor_set(MOTOR_LEFT_DIR, MOTOR_LEFT_PWM, speed, MOTOR_LEFT_INVERT);
}

void motor_right_set_speed(int16_t speed)
{
    motor_set(MOTOR_RIGHT_DIR, MOTOR_RIGHT_PWM, speed, MOTOR_RIGHT_INVERT);
}

void motor_stop(void)
{
    motor_left_set_speed(0);
    motor_right_set_speed(0);
}
