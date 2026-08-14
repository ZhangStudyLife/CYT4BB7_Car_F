#include "car_mode.h"
#include "pid_core.h"

#define MODE8_STICK_MAX (1000.0f)

volatile float mode8_speed_left_kp = 10.80f;
volatile float mode8_speed_left_ki = 0.52f;
volatile float mode8_speed_left_kd = 1.00f;
volatile float mode8_speed_right_kp = 10.80f;
volatile float mode8_speed_right_ki = 0.52f;
volatile float mode8_speed_right_kd = 1.00f;
volatile float mode8_speed_filter_alpha = 0.557f;
volatile float mode8_speed_ff_static = 800.0f;
volatile float mode8_target_speed_mps = 2.5f;
volatile float mode8_speed_i_limit = 2000.0f;
volatile float mode8_speed_ff_deadband = 10.0f;
volatile float mode8_speed_ff_transition = 100.0f;
volatile float mode8_speed_increment_limit = 6000.0f;

static pid_t s_mode8_left_speed_pid;
static pid_t s_mode8_right_speed_pid;

static int16 mode8_speed_pid_update(pid_t *pid,
                                    float target,
                                    float feedback,
                                    float kp,
                                    float ki,
                                    float kd)
{
    float output;
    float ff_deadband = car_math_clampf(mode8_speed_ff_deadband,
                                        0.0f, 200.0f);
    float ff_transition = (mode8_speed_ff_transition > ff_deadband)
                              ? mode8_speed_ff_transition
                              : ff_deadband;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = 0.0f;
    pid->i_limit = mode8_speed_i_limit;
    PID_SetStaticFeedforward(pid, mode8_speed_ff_static);
    PID_SetFeedforwardTransition(pid, ff_deadband, ff_transition);
    PID_SetExternalFeedforward(pid, 0.0f);
    PID_SetIncrementLimit(pid, mode8_speed_increment_limit);
    PID_SetOutputLimits(pid, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    output = PID_UpdateIncremental(pid, target, feedback);
    return (int16)car_math_clampf(output,
                                  -(float)MOTOR_PWM_MAX,
                                   (float)MOTOR_PWM_MAX);
}

void car_mode8_init(void)
{
    PID_Init(&s_mode8_left_speed_pid,
             mode8_speed_left_kp, mode8_speed_left_ki, mode8_speed_left_kd,
             0.0f, mode8_speed_i_limit);
    PID_Init(&s_mode8_right_speed_pid,
             mode8_speed_right_kp, mode8_speed_right_ki, mode8_speed_right_kd,
             0.0f, mode8_speed_i_limit);
    car_mode8_reset();
}

void car_mode8_reset(void)
{
    PID_Reset(&s_mode8_left_speed_pid);
    PID_Reset(&s_mode8_right_speed_pid);
    g_car_base_speed_command = 0.0f;
    g_car_base_speed_target = 0.0f;
    Left_Target_Speed = 0.0f;
    Right_Target_Speed = 0.0f;
    g_car_speed_left_motor_output = 0.0f;
    g_car_speed_right_motor_output = 0.0f;
}

void car_mode8_update_25HZ(uint32 now_ms)
{
    (void)now_ms;
}

void car_mode8_update_100HZ(uint32 now_ms)
{
    float target_speed_mps;
    int16 left_output;
    int16 right_output;

    (void)now_ms;
    target_speed_mps = car_math_clampf(g_air_std_ch1,
                                       -MODE8_STICK_MAX,
                                        MODE8_STICK_MAX) *
                       car_math_clampf(mode8_target_speed_mps, 0.0f, 5.0f) /
                       MODE8_STICK_MAX;
    Left_Target_Speed = car_speed_mps_to_encoder_cnt(target_speed_mps);
    Right_Target_Speed = Left_Target_Speed;
    g_car_base_speed_command = Left_Target_Speed;
    g_car_base_speed_target = Left_Target_Speed;

    left_output = mode8_speed_pid_update(
        &s_mode8_left_speed_pid,
        Left_Target_Speed, g_car_speed_left_filtered,
        mode8_speed_left_kp, mode8_speed_left_ki, mode8_speed_left_kd);
    right_output = mode8_speed_pid_update(
        &s_mode8_right_speed_pid,
        Right_Target_Speed, g_car_speed_right_filtered,
        mode8_speed_right_kp, mode8_speed_right_ki, mode8_speed_right_kd);
    g_car_speed_left_motor_output = (float)left_output;
    g_car_speed_right_motor_output = (float)right_output;
    motor_left_set_speed(left_output);
    motor_right_set_speed(right_output);
}
