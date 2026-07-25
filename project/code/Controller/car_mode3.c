/* Mode3：车端规划修正后的车体系速度模式。
 * 两轮底盘保持Air目标航向，只执行前向速度分量；
 * carplanfix产生的横移分量暂不进入电机输出。
 */
#include "car_mode.h"
#include "car_loop.h"

#define MODE3_MAX_VELOCITY_MPS       (3.0f)
#define MODE3_MIN_OUTPUT_LIMIT       (0.0f)

car_mode3_state_t g_car_mode3_state = {0};

static PositionalPID s_mode3_forward_pid;
static PositionalPID s_mode3_strafe_pid;

static void car_mode3_pid_init(void)
{
    PositionalPID_Init(&s_mode3_forward_pid,
                       0.0f,
                       mode3_velocity_forward_kp,
                       mode3_velocity_forward_ki,
                       mode3_velocity_forward_kd,
                       mode3_velocity_i_limit,
                       mode3_velocity_pid_output_limit);
    PositionalPID_Init(&s_mode3_strafe_pid,
                       0.0f,
                       mode3_velocity_strafe_kp,
                       mode3_velocity_strafe_ki,
                       mode3_velocity_strafe_kd,
                       mode3_velocity_i_limit,
                       mode3_velocity_pid_output_limit);
}
static void car_mode3_pid_apply_params(void)
{
    s_mode3_forward_pid.kp_2 = 0.0f;
    s_mode3_forward_pid.kp_1 = mode3_velocity_forward_kp;
    s_mode3_forward_pid.ki = mode3_velocity_forward_ki;
    s_mode3_forward_pid.kd = mode3_velocity_forward_kd;
    s_mode3_forward_pid.i_limit = mode3_velocity_i_limit;
    s_mode3_forward_pid.output_limit = mode3_velocity_pid_output_limit;

    s_mode3_strafe_pid.kp_2 = 0.0f;
    s_mode3_strafe_pid.kp_1 = mode3_velocity_strafe_kp;
    s_mode3_strafe_pid.ki = mode3_velocity_strafe_ki;
    s_mode3_strafe_pid.kd = mode3_velocity_strafe_kd;
    s_mode3_strafe_pid.i_limit = mode3_velocity_i_limit;
    s_mode3_strafe_pid.output_limit = mode3_velocity_pid_output_limit;
}

static float car_mode3_limit_output(float value)
{
    float limit = mode3_velocity_output_limit;

    if(limit < MODE3_MIN_OUTPUT_LIMIT)
    {
        limit = MODE3_MIN_OUTPUT_LIMIT;
    }

    return car_math_limit_absf(value, limit);
}

void car_mode3_init(void)
{
    car_mode3_reset();
}

void car_mode3_reset(void)
{
    car_mode3_pid_init();

    g_car_mode3_state.raw_forward_mps = 0.0f;
    g_car_mode3_state.raw_strafe_mps = 0.0f;
    g_car_mode3_state.velocity_forward_target_mps = 0.0f;
    g_car_mode3_state.velocity_strafe_target_mps = 0.0f;
    g_car_mode3_state.velocity_forward_feedback_mps = 0.0f;
    g_car_mode3_state.velocity_strafe_feedback_mps = 0.0f;
    g_car_mode3_state.forward_feedforward = 0.0f;
    g_car_mode3_state.strafe_feedforward = 0.0f;
    g_car_mode3_state.forward_pid_output = 0.0f;
    g_car_mode3_state.strafe_pid_output = 0.0f;
    g_car_mode3_state.forward_target = 0.0f;
    g_car_mode3_state.strafe_target = 0.0f;
    g_car_mode3_state.forward_pid_p_term = 0.0f;
    g_car_mode3_state.forward_pid_i_term = 0.0f;
    g_car_mode3_state.forward_pid_d_term = 0.0f;
    g_car_mode3_state.strafe_pid_p_term = 0.0f;
    g_car_mode3_state.strafe_pid_i_term = 0.0f;
    g_car_mode3_state.strafe_pid_d_term = 0.0f;
    g_car_mode3_state.output_valid = 0U;
}

void car_mode3_update_25HZ(uint32 now_ms)
{
    (void)now_ms;
}

void car_mode3_update_100HZ(uint32 now_ms)
{
    (void)now_ms;

    car_mode3_pid_apply_params();

    if (g_air_car_plan_valid > 0.5f)
    {
        g_car_mode3_state.raw_forward_mps = car_math_limit_absf(g_car_plan_forward_mps,
                                                                MODE3_MAX_VELOCITY_MPS);
        g_car_mode3_state.raw_strafe_mps = car_math_limit_absf(g_car_plan_strafe_mps,
                                                               MODE3_MAX_VELOCITY_MPS);
    }
    else
    {
        g_car_mode3_state.raw_forward_mps = 0.0f;
        g_car_mode3_state.raw_strafe_mps = 0.0f;
    }

    g_car_mode3_state.velocity_forward_target_mps = g_car_mode3_state.raw_forward_mps;
    g_car_mode3_state.velocity_strafe_target_mps = g_car_mode3_state.raw_strafe_mps;

    g_car_mode3_state.velocity_forward_feedback_mps = g_odometer.body_vel[y];
    g_car_mode3_state.velocity_strafe_feedback_mps = g_odometer.body_vel[x];

    g_car_mode3_state.forward_feedforward =
        g_car_mode3_state.velocity_forward_target_mps *
        ODOMETER_FORWARD_COUNT_PER_METER *
        ODOMETER_UPDATE_DT_S;
    g_car_mode3_state.strafe_feedforward =
        -g_car_mode3_state.velocity_strafe_target_mps *
        ODOMETER_STRAFE_COUNT_PER_METER_ABS *
        ODOMETER_UPDATE_DT_S;

    g_car_mode3_state.forward_pid_output =
        PositionalPID_Update(&s_mode3_forward_pid,
                             g_car_mode3_state.velocity_forward_target_mps,
                             g_car_mode3_state.velocity_forward_feedback_mps);
    g_car_mode3_state.strafe_pid_output =
        PositionalPID_Update(&s_mode3_strafe_pid,
                             g_car_mode3_state.velocity_strafe_target_mps,
                             g_car_mode3_state.velocity_strafe_feedback_mps);

    g_car_mode3_state.forward_target =
        car_mode3_limit_output(g_car_mode3_state.forward_feedforward +
                               g_car_mode3_state.forward_pid_output);
    g_car_mode3_state.strafe_target =
        car_mode3_limit_output(g_car_mode3_state.strafe_feedforward -
                               g_car_mode3_state.strafe_pid_output);

    g_car_mode3_state.forward_pid_p_term = s_mode3_forward_pid.p_term;
    g_car_mode3_state.forward_pid_i_term = s_mode3_forward_pid.i_term;
    g_car_mode3_state.forward_pid_d_term = s_mode3_forward_pid.d_term;
    g_car_mode3_state.strafe_pid_p_term = s_mode3_strafe_pid.p_term;
    g_car_mode3_state.strafe_pid_i_term = s_mode3_strafe_pid.i_term;
    g_car_mode3_state.strafe_pid_d_term = s_mode3_strafe_pid.d_term;
    g_car_mode3_state.output_valid = 1U;

    /* 两轮任务模式输出物理线速度，方向由统一航向串级环控制。 */
    car_forward_target = g_car_mode3_state.velocity_forward_target_mps;
    car_yaw_rate_target = 0.0f;
}
