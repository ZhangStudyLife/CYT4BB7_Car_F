#include "motor.h"

/*
 * 初始化两轮差速底盘的驱动电机，频率 17kHz。
 * M1 固定对应物理左轮，M2 固定对应物理右轮；初始化占空比均为 0。
 */
void two_wheel_motor_init(void)
{
    // 电机1：左轮
    gpio_init(MOTOR_M1_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M1_PWM, 17000, 0);

    // 电机2：右轮
    gpio_init(MOTOR_M2_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M2_PWM, 17000, 0);
}

/*
 * 旧四轮麦轮初始化接口。
 * 先初始化两轮底盘使用的 M1/M2，再初始化仅供旧麦轮代码兼容的 M3/M4。
 */
void mecanum_motor_init(void)
{
    two_wheel_motor_init();

    // 旧麦轮电机3：右后轮
    gpio_init(MOTOR_M3_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M3_PWM, 17000, 0);

    // 旧麦轮电机4：左后轮
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
 * @brief  设置电机1（物理左轮）速度
 */
void motor_m1_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M1_DIR, MOTOR_M1_PWM, speed, MOTOR_M1_INVERT);
}

/**
 * @brief  设置电机2（物理右轮）速度
 */
void motor_m2_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M2_DIR, MOTOR_M2_PWM, speed, MOTOR_M2_INVERT);
}

/**
 * @brief  设置旧麦轮电机3（物理右后轮）速度
 */
void motor_m3_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M3_DIR, MOTOR_M3_PWM, speed, MOTOR_M3_INVERT);
}

/**
 * @brief  设置旧麦轮电机4（物理左后轮）速度
 */
void motor_m4_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M4_DIR, MOTOR_M4_PWM, speed, MOTOR_M4_INVERT);
}

/**
 * @brief  设置两轮差速底盘左右轮速度
 * @param  left_speed  左轮PWM指令，正值表示车辆前进
 * @param  right_speed 右轮PWM指令，正值表示车辆前进
 */
void two_wheel_motor_set(int16_t left_speed, int16_t right_speed)
{
    motor_m1_set_speed(left_speed);
    motor_m2_set_speed(right_speed);
}

/**
 * @brief  停止两轮差速底盘的左右驱动轮
 */
void two_wheel_motor_stop(void)
{
    two_wheel_motor_set(0, 0);
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
