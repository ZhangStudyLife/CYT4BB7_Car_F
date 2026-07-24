/* Mode2: remote body velocity closed loop.
 * Remote gives body forward/right velocity targets in m/s.
 * Odometer gives body velocity feedback in m/s, X positive means right, Y positive means forward.
 * Output converts right-positive velocity to the wheel-space strafe command.
 */
#include "car_mode.h"
#include "car_loop.h"

#define MODE2_MAX_VELOCITY_MPS       (1.5f)
#define MODE2_MIN_OUTPUT_LIMIT       (0.0f)

car_mode2_state_t g_car_mode2_state = {0};

static PositionalPID s_mode2_forward_pid;
static PositionalPID s_mode2_strafe_pid;

static void car_mode2_pid_init(void)
{
    PositionalPID_Init(&s_mode2_forward_pid,
                       0.0f,
                       mode2_velocity_forward_kp,
                       mode2_velocity_forward_ki,
                       mode2_velocity_forward_kd,
                       mode2_velocity_i_limit,
                       mode2_velocity_pid_output_limit);
    PositionalPID_Init(&s_mode2_strafe_pid,
                       0.0f,
                       mode2_velocity_strafe_kp,
                       mode2_velocity_strafe_ki,
                       mode2_velocity_strafe_kd,
                       mode2_velocity_i_limit,
                       mode2_velocity_pid_output_limit);
}

static void car_mode2_pid_apply_params(void)
{
    s_mode2_forward_pid.kp_2 = 0.0f;
    s_mode2_forward_pid.kp_1 = mode2_velocity_forward_kp;
    s_mode2_forward_pid.ki = mode2_velocity_forward_ki;
    s_mode2_forward_pid.kd = mode2_velocity_forward_kd;
    s_mode2_forward_pid.i_limit = mode2_velocity_i_limit;
    s_mode2_forward_pid.output_limit = mode2_velocity_pid_output_limit;

    s_mode2_strafe_pid.kp_2 = 0.0f;
    s_mode2_strafe_pid.kp_1 = mode2_velocity_strafe_kp;
    s_mode2_strafe_pid.ki = mode2_velocity_strafe_ki;
    s_mode2_strafe_pid.kd = mode2_velocity_strafe_kd;
    s_mode2_strafe_pid.i_limit = mode2_velocity_i_limit;
    s_mode2_strafe_pid.output_limit = mode2_velocity_pid_output_limit;
}

static float car_mode2_limit_output(float value)
{
    float limit = mode2_velocity_output_limit;

    if(limit < MODE2_MIN_OUTPUT_LIMIT)
    {
        limit = MODE2_MIN_OUTPUT_LIMIT;
    }

    return car_math_limit_absf(value, limit);
}

void car_mode2_init(void)
{
    car_mode2_reset();
}

void car_mode2_reset(void)
{
    car_mode2_pid_init();

    g_car_mode2_state.raw_forward_mps = 0.0f;
    g_car_mode2_state.raw_strafe_mps = 0.0f;
    g_car_mode2_state.velocity_forward_target_mps = 0.0f;
    g_car_mode2_state.velocity_strafe_target_mps = 0.0f;
    g_car_mode2_state.velocity_forward_feedback_mps = 0.0f;
    g_car_mode2_state.velocity_strafe_feedback_mps = 0.0f;
    g_car_mode2_state.forward_feedforward = 0.0f;
    g_car_mode2_state.strafe_feedforward = 0.0f;
    g_car_mode2_state.forward_pid_output = 0.0f;
    g_car_mode2_state.strafe_pid_output = 0.0f;
    g_car_mode2_state.forward_target = 0.0f;
    g_car_mode2_state.strafe_target = 0.0f;
    g_car_mode2_state.forward_pid_p_term = 0.0f;
    g_car_mode2_state.forward_pid_i_term = 0.0f;
    g_car_mode2_state.forward_pid_d_term = 0.0f;
    g_car_mode2_state.strafe_pid_p_term = 0.0f;
    g_car_mode2_state.strafe_pid_i_term = 0.0f;
    g_car_mode2_state.strafe_pid_d_term = 0.0f;
    g_car_mode2_state.output_valid = 0U;
}

void car_mode2_update_25HZ(uint32 now_ms)
{
    (void)now_ms;
}

void car_mode2_update_100HZ(uint32 now_ms)
{
    (void)now_ms;

    car_mode2_pid_apply_params();

    if (g_air_car_plan_valid > 0.5f)
    {
        g_car_mode2_state.raw_forward_mps = car_math_limit_absf(g_air_car_plan_forward_mps,
                                                                MODE2_MAX_VELOCITY_MPS);
        g_car_mode2_state.raw_strafe_mps = car_math_limit_absf(g_air_car_plan_strafe_mps,
                                                               MODE2_MAX_VELOCITY_MPS);
    }
    else
    {
        g_car_mode2_state.raw_forward_mps = 0.0f;
        g_car_mode2_state.raw_strafe_mps = 0.0f;
    }

    g_car_mode2_state.velocity_forward_target_mps =
        car_filter_lpf1_apply(g_car_mode2_state.velocity_forward_target_mps,
                              g_car_mode2_state.raw_forward_mps,
                              ODOMETER_UPDATE_DT_S,
                              mode2_velocity_smooth_tau_s);
    g_car_mode2_state.velocity_strafe_target_mps =
        car_filter_lpf1_apply(g_car_mode2_state.velocity_strafe_target_mps,
                              g_car_mode2_state.raw_strafe_mps,
                              ODOMETER_UPDATE_DT_S,
                              mode2_velocity_smooth_tau_s);

    g_car_mode2_state.velocity_forward_feedback_mps = g_odometer.body_vel[y];
    g_car_mode2_state.velocity_strafe_feedback_mps = g_odometer.body_vel[x];

    g_car_mode2_state.forward_feedforward =
        g_car_mode2_state.velocity_forward_target_mps *
        ODOMETER_FORWARD_COUNT_PER_METER *
        ODOMETER_UPDATE_DT_S;
    g_car_mode2_state.strafe_feedforward =
        -g_car_mode2_state.velocity_strafe_target_mps *
        ODOMETER_STRAFE_COUNT_PER_METER_ABS *
        ODOMETER_UPDATE_DT_S;

    g_car_mode2_state.forward_pid_output =
        PositionalPID_Update(&s_mode2_forward_pid,
                             g_car_mode2_state.velocity_forward_target_mps,
                             g_car_mode2_state.velocity_forward_feedback_mps);
    g_car_mode2_state.strafe_pid_output =
        PositionalPID_Update(&s_mode2_strafe_pid,
                             g_car_mode2_state.velocity_strafe_target_mps,
                             g_car_mode2_state.velocity_strafe_feedback_mps);

    g_car_mode2_state.forward_target =
        car_mode2_limit_output(g_car_mode2_state.forward_feedforward +
                               g_car_mode2_state.forward_pid_output);
    g_car_mode2_state.strafe_target =
        car_mode2_limit_output(g_car_mode2_state.strafe_feedforward -
                               g_car_mode2_state.strafe_pid_output);

    g_car_mode2_state.forward_pid_p_term = s_mode2_forward_pid.p_term;
    g_car_mode2_state.forward_pid_i_term = s_mode2_forward_pid.i_term;
    g_car_mode2_state.forward_pid_d_term = s_mode2_forward_pid.d_term;
    g_car_mode2_state.strafe_pid_p_term = s_mode2_strafe_pid.p_term;
    g_car_mode2_state.strafe_pid_i_term = s_mode2_strafe_pid.i_term;
    g_car_mode2_state.strafe_pid_d_term = s_mode2_strafe_pid.d_term;
    g_car_mode2_state.output_valid = 1U;

    car_forward_target = g_car_mode2_state.forward_target;
    car_strafe_target = g_car_mode2_state.strafe_target;
}
