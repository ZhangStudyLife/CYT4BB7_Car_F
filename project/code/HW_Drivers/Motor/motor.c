#include "motor.h"

/* 初始化四轮电机 GPIO + PWM，频率 17kHz */
void mecanum_motor_init(void)
{
    // 左前
    gpio_init(MOTOR_M1_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M1_PWM, 17000, 0);

    // 右前
    gpio_init(MOTOR_M2_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M2_PWM, 17000, 0);

    // 左后
    gpio_init(MOTOR_M3_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M3_PWM, 17000, 0);

    // 右后
    gpio_init(MOTOR_M4_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M4_PWM, 17000, 0);
}

/* PWM 占空比限幅 */
int16_t speed_limit(int16_t speed)
{
    if(speed > MOTOR_PWM_MAX)
    {
        speed = MOTOR_PWM_MAX;
    }
    else if(speed < (-MOTOR_PWM_MAX))
    {
        speed = -MOTOR_PWM_MAX;
    }
    return speed;
}

/*
 * 设置单个电机速度
 * 流程：限幅 -> invert 取反 -> speed>=0 则 DIR=LOW 正转，否则 DIR=HIGH 反转
 * PWM 占空比始终取绝对值传入
 */
void motor_set_single(gpio_pin_enum dir_pin, pwm_channel_enum pwm_ch, int16_t speed, uint8_t invert)
{
    speed = speed_limit(speed);

    if (invert)
    {
        speed = -speed;         // 机械装配反转：取反 speed
    }

    if (speed >= 0)
    {
        gpio_set_level(dir_pin, GPIO_LOW);      // 正转：DIR 拉低
        pwm_set_duty(pwm_ch, speed);
    }
    else
    {
        gpio_set_level(dir_pin, GPIO_HIGH);     // 反转：DIR 拉高
        pwm_set_duty(pwm_ch, -speed);
    }
}

/**
 * @brief  设置电机1（左前）速度
 */
void motor_m1_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M1_DIR, MOTOR_M1_PWM, speed, MOTOR_M1_INVERT);
}

/**
 * @brief  设置电机2（右前）速度
 */
void motor_m2_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M2_DIR, MOTOR_M2_PWM, speed, MOTOR_M2_INVERT);
}

/**
 * @brief  设置电机3（左后）速度
 */
void motor_m3_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M3_DIR, MOTOR_M3_PWM, speed, MOTOR_M3_INVERT);
}

/**
 * @brief  设置电机4（右后）速度
 */
void motor_m4_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M4_DIR, MOTOR_M4_PWM, speed, MOTOR_M4_INVERT);
}

/**
 * @brief  同时设置四轮电机速度
 */
void mecanum_motor_set_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    motor_m1_set_speed(m1);
    motor_m2_set_speed(m2);
    motor_m3_set_speed(m3);
    motor_m4_set_speed(m4);
}

/**
 * @brief  停止所有电机
 */
void mecanum_motor_stop(void)
{
    mecanum_motor_set_all(0, 0, 0, 0);
}
