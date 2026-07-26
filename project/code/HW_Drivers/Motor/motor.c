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
        gpio_set_level(dir_pin, GPIO_LOW);
        pwm_set_duty(pwm_ch, speed);
    }
    else
    {
        gpio_set_level(dir_pin, GPIO_HIGH);
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
