#include "negative_pressure_motor.h"
#include "zf_driver_delay.h"
#include "zf_driver_pwm.h"

#define NEGATIVE_PRESSURE_DUTY_MIN    (4000U)
#define NEGATIVE_PRESSURE_DUTY_MAX    (8000U)

/* P05_0（P50）为左电调，P05_2（P52）为右电调。 */
static const pwm_channel_enum s_negative_pressure_pwm[NEGATIVE_PRESSURE_MOTOR_COUNT] =
{
    TCPWM_CH09_P05_0,
    TCPWM_CH11_P05_2
};

static uint8 s_negative_pressure_enabled = 0U;

static uint16 negative_pressure_limit_throttle(uint16 throttle)
{
    if (throttle > NEGATIVE_PRESSURE_THROTTLE_LIMIT_MAX)
    {
        return NEGATIVE_PRESSURE_THROTTLE_LIMIT_MAX;
    }

    return throttle;
}

static uint32 negative_pressure_throttle_to_duty(uint16 throttle)
{
    uint32 duty_range = NEGATIVE_PRESSURE_DUTY_MAX - NEGATIVE_PRESSURE_DUTY_MIN;

    throttle = negative_pressure_limit_throttle(throttle);
    return NEGATIVE_PRESSURE_DUTY_MIN +
           ((uint32)throttle * duty_range) / NEGATIVE_PRESSURE_INPUT_MAX;
}

static void negative_pressure_write(negative_pressure_motor_e motor, uint16 throttle)
{
    if (motor >= NEGATIVE_PRESSURE_MOTOR_COUNT)
    {
        return;
    }

    pwm_set_duty(s_negative_pressure_pwm[motor],
                 negative_pressure_throttle_to_duty(throttle));
}

void negative_pressure_init(void)
{
    s_negative_pressure_enabled = 0U;

    for (uint8 i = 0U; i < NEGATIVE_PRESSURE_MOTOR_COUNT; i++)
    {
        pwm_init(s_negative_pressure_pwm[i],
                 NEGATIVE_PRESSURE_PWM_FREQ,
                 NEGATIVE_PRESSURE_DUTY_MIN);
    }

    /* 电调上电后需要持续接收一段时间的最低油门信号。 */
    system_delay_ms(NEGATIVE_PRESSURE_ESC_ARM_DELAY_MS);
}

void negative_pressure_enable(void)
{
    s_negative_pressure_enabled = 1U;
}

void negative_pressure_disable(void)
{
    s_negative_pressure_enabled = 0U;
    negative_pressure_write(NEGATIVE_PRESSURE_MOTOR_LEFT, 0U);
    negative_pressure_write(NEGATIVE_PRESSURE_MOTOR_RIGHT, 0U);
}

uint8 negative_pressure_is_enabled(void)
{
    return s_negative_pressure_enabled;
}

void negative_pressure_set_motor(negative_pressure_motor_e motor, uint16 throttle)
{
    if (motor >= NEGATIVE_PRESSURE_MOTOR_COUNT)
    {
        return;
    }

    if (s_negative_pressure_enabled == 0U)
    {
        negative_pressure_write(motor, 0U);
        return;
    }

    negative_pressure_write(motor, throttle);
}

void negative_pressure_set_throttle(uint16 left_throttle, uint16 right_throttle)
{
    if (s_negative_pressure_enabled == 0U)
    {
        negative_pressure_disable();
        return;
    }

    negative_pressure_write(NEGATIVE_PRESSURE_MOTOR_LEFT, left_throttle);
    negative_pressure_write(NEGATIVE_PRESSURE_MOTOR_RIGHT, right_throttle);
}
