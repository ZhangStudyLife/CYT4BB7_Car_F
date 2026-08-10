#include "car_mode.h"
#include "pid_core.h"

#define MODE8_TARGET_SPEED_MAX_MPS      (2.5f)
#define MODE8_STICK_MAX                 (1000.0f)
#define MODE8_SPEED_I_LIMIT             (2000.0f)
#define MODE8_SPEED_FF_DEADBAND         (10.0f)
#define MODE8_SPEED_FF_TRANSITION       (100.0f)
#define MODE8_SPEED_INCREMENT_LIMIT     (6000.0f)

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

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = 0.0f;
    pid->i_limit = MODE8_SPEED_I_LIMIT;
    PID_SetStaticFeedforward(pid, car_speed_ff_static);
    PID_SetFeedforwardTransition(pid,
                                 MODE8_SPEED_FF_DEADBAND,
                                 MODE8_SPEED_FF_TRANSITION);
    PID_SetExternalFeedforward(pid, 0.0f);
    PID_SetIncrementLimit(pid, MODE8_SPEED_INCREMENT_LIMIT);
    PID_SetOutputLimits(pid, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    output = PID_UpdateIncremental(pid, target, feedback);
    return (int16)car_math_clampf(output,
                                  -(float)MOTOR_PWM_MAX,
                                   (float)MOTOR_PWM_MAX);
}

void car_mode8_init(void)
{
    PID_Init(&s_mode8_left_speed_pid,
             car_speed_left_kp, car_speed_left_ki, car_speed_left_kd,
             0.0f, MODE8_SPEED_I_LIMIT);
    PID_Init(&s_mode8_right_speed_pid,
             car_speed_right_kp, car_speed_right_ki, car_speed_right_kd,
             0.0f, MODE8_SPEED_I_LIMIT);
    car_mode8_reset();
}

void car_mode8_reset(void)
{
    PID_Reset(&s_mode8_left_speed_pid);
    PID_Reset(&s_mode8_right_speed_pid);
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
                       MODE8_TARGET_SPEED_MAX_MPS / MODE8_STICK_MAX;
    Left_Target_Speed = car_speed_mps_to_encoder_cnt(target_speed_mps);
    Right_Target_Speed = Left_Target_Speed;

    left_output = mode8_speed_pid_update(
        &s_mode8_left_speed_pid,
        Left_Target_Speed, g_car_speed_left_filtered,
        car_speed_left_kp, car_speed_left_ki, car_speed_left_kd);
    right_output = mode8_speed_pid_update(
        &s_mode8_right_speed_pid,
        Right_Target_Speed, g_car_speed_right_filtered,
        car_speed_right_kp, car_speed_right_ki, car_speed_right_kd);
    g_car_speed_left_motor_output = (float)left_output;
    g_car_speed_right_motor_output = (float)right_output;
    motor_left_set_speed(left_output);
    motor_right_set_speed(right_output);
}
