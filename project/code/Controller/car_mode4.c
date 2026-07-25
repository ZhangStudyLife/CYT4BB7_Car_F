/* Mode4：遥控水平速度目标模式。
 * 水平速度向量先转换到车体系；两轮底盘保持Air目标航向，只执行转换后的
 * 前向分量，无法实现的横移分量不进入电机输出。
 */
#include "car_mode.h"
#include "car_loop.h"

#define MODE4_FIXED_VELOCITY_MPS     (1.5f)   /* Mode4 遥控非零档固定合速度，单位 m/s */
#define MODE4_STICK_MAX              (1000.0f)
#define MODE4_STICK_ZONE_WIDTH       (200.0f)
#define MODE4_MIN_OUTPUT_LIMIT       (0.0f)

car_mode4_state_t g_car_mode4_state = {0};

static PositionalPID s_mode4_forward_pid;
static PositionalPID s_mode4_strafe_pid;

static void car_mode4_pid_init(void)
{
    PositionalPID_Init(&s_mode4_forward_pid,
                       0.0f,
                       mode4_velocity_forward_kp,
                       mode4_velocity_forward_ki,
                       mode4_velocity_forward_kd,
                       mode4_velocity_i_limit,
                       mode4_velocity_pid_output_limit);
    PositionalPID_Init(&s_mode4_strafe_pid,
                       0.0f,
                       mode4_velocity_strafe_kp,
                       mode4_velocity_strafe_ki,
                       mode4_velocity_strafe_kd,
                       mode4_velocity_i_limit,
                       mode4_velocity_pid_output_limit);
}
static void car_mode4_pid_apply_params(void)
{
    s_mode4_forward_pid.kp_2 = 0.0f;
    s_mode4_forward_pid.kp_1 = mode4_velocity_forward_kp;
    s_mode4_forward_pid.ki = mode4_velocity_forward_ki;
    s_mode4_forward_pid.kd = mode4_velocity_forward_kd;
    s_mode4_forward_pid.i_limit = mode4_velocity_i_limit;
    s_mode4_forward_pid.output_limit = mode4_velocity_pid_output_limit;

    s_mode4_strafe_pid.kp_2 = 0.0f;
    s_mode4_strafe_pid.kp_1 = mode4_velocity_strafe_kp;
    s_mode4_strafe_pid.ki = mode4_velocity_strafe_ki;
    s_mode4_strafe_pid.kd = mode4_velocity_strafe_kd;
    s_mode4_strafe_pid.i_limit = mode4_velocity_i_limit;
    s_mode4_strafe_pid.output_limit = mode4_velocity_pid_output_limit;
}

static void car_mode4_stick_to_velocity(float strafe_stick,
                                        float forward_stick,
                                        float *forward_mps,
                                        float *strafe_mps)
{
    float radius;

    strafe_stick = car_math_limit_absf(strafe_stick, MODE4_STICK_MAX);
    forward_stick = car_math_limit_absf(forward_stick, MODE4_STICK_MAX);
    radius = sqrtf((strafe_stick * strafe_stick) +
                   (forward_stick * forward_stick));

    if(radius < MODE4_STICK_ZONE_WIDTH)
    {
        *forward_mps = 0.0f;
        *strafe_mps = 0.0f;
        return;
    }

    *forward_mps = forward_stick * MODE4_FIXED_VELOCITY_MPS / radius;
    *strafe_mps = strafe_stick * MODE4_FIXED_VELOCITY_MPS / radius;
}

static float car_mode4_limit_output(float value)
{
    float limit = mode4_velocity_output_limit;

    if(limit < MODE4_MIN_OUTPUT_LIMIT)
    {
        limit = MODE4_MIN_OUTPUT_LIMIT;
    }

    return car_math_limit_absf(value, limit);
}

void car_mode4_init(void)
{
    car_mode4_reset();
}

void car_mode4_reset(void)
{
    car_mode4_pid_init();

    g_car_mode4_state.raw_forward_mps = 0.0f;
    g_car_mode4_state.raw_strafe_mps = 0.0f;
    g_car_mode4_state.velocity_forward_target_mps = 0.0f;
    g_car_mode4_state.velocity_strafe_target_mps = 0.0f;
    g_car_mode4_state.velocity_forward_feedback_mps = 0.0f;
    g_car_mode4_state.velocity_strafe_feedback_mps = 0.0f;
    g_car_mode4_state.forward_feedforward = 0.0f;
    g_car_mode4_state.strafe_feedforward = 0.0f;
    g_car_mode4_state.forward_pid_output = 0.0f;
    g_car_mode4_state.strafe_pid_output = 0.0f;
    g_car_mode4_state.forward_target = 0.0f;
    g_car_mode4_state.strafe_target = 0.0f;
    g_car_mode4_state.forward_pid_p_term = 0.0f;
    g_car_mode4_state.forward_pid_i_term = 0.0f;
    g_car_mode4_state.forward_pid_d_term = 0.0f;
    g_car_mode4_state.strafe_pid_p_term = 0.0f;
    g_car_mode4_state.strafe_pid_i_term = 0.0f;
    g_car_mode4_state.strafe_pid_d_term = 0.0f;
    g_car_mode4_state.output_valid = 0U;
}

void car_mode4_update_25HZ(uint32 now_ms)
{
    (void)now_ms;
}

void car_mode4_update_100HZ(uint32 now_ms)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float forward_target_mps;
    float strafe_target_mps;

    (void)now_ms;

    car_mode4_pid_apply_params();

    car_mode4_stick_to_velocity(g_air_crsf_std_ch0,
                                g_air_crsf_std_ch1,
                                &g_car_mode4_state.raw_forward_mps,
                                &g_car_mode4_state.raw_strafe_mps);

    yaw_rad = Control_GetYawAngle();
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    forward_target_mps =
        (cos_yaw * g_car_mode4_state.raw_forward_mps) -
        (sin_yaw * g_car_mode4_state.raw_strafe_mps);
    strafe_target_mps =
        (sin_yaw * g_car_mode4_state.raw_forward_mps) +
        (cos_yaw * g_car_mode4_state.raw_strafe_mps);

    g_car_mode4_state.velocity_forward_target_mps =
        car_filter_lpf1_apply(g_car_mode4_state.velocity_forward_target_mps,
                              forward_target_mps,
                              ODOMETER_UPDATE_DT_S,
                              mode4_velocity_smooth_tau_s);
    g_car_mode4_state.velocity_strafe_target_mps =
        car_filter_lpf1_apply(g_car_mode4_state.velocity_strafe_target_mps,
                              strafe_target_mps,
                              ODOMETER_UPDATE_DT_S,
                              mode4_velocity_smooth_tau_s);

    g_car_mode4_state.velocity_forward_feedback_mps = g_odometer.body_vel[y];
    g_car_mode4_state.velocity_strafe_feedback_mps = g_odometer.body_vel[x];

    g_car_mode4_state.forward_feedforward =
        g_car_mode4_state.velocity_forward_target_mps *
        ODOMETER_FORWARD_COUNT_PER_METER *
        ODOMETER_UPDATE_DT_S;
    g_car_mode4_state.strafe_feedforward =
        -g_car_mode4_state.velocity_strafe_target_mps *
        ODOMETER_STRAFE_COUNT_PER_METER_ABS *
        ODOMETER_UPDATE_DT_S;

    g_car_mode4_state.forward_pid_output =
        PositionalPID_Update(&s_mode4_forward_pid,
                             g_car_mode4_state.velocity_forward_target_mps,
                             g_car_mode4_state.velocity_forward_feedback_mps);
    g_car_mode4_state.strafe_pid_output =
        PositionalPID_Update(&s_mode4_strafe_pid,
                             g_car_mode4_state.velocity_strafe_target_mps,
                             g_car_mode4_state.velocity_strafe_feedback_mps);

    g_car_mode4_state.forward_target =
        car_mode4_limit_output(g_car_mode4_state.forward_feedforward +
                               g_car_mode4_state.forward_pid_output);
    g_car_mode4_state.strafe_target =
        car_mode4_limit_output(g_car_mode4_state.strafe_feedforward -
                               g_car_mode4_state.strafe_pid_output);

    g_car_mode4_state.forward_pid_p_term = s_mode4_forward_pid.p_term;
    g_car_mode4_state.forward_pid_i_term = s_mode4_forward_pid.i_term;
    g_car_mode4_state.forward_pid_d_term = s_mode4_forward_pid.d_term;
    g_car_mode4_state.strafe_pid_p_term = s_mode4_strafe_pid.p_term;
    g_car_mode4_state.strafe_pid_i_term = s_mode4_strafe_pid.i_term;
    g_car_mode4_state.strafe_pid_d_term = s_mode4_strafe_pid.d_term;
    g_car_mode4_state.output_valid = 1U;

    /* 两轮任务模式不再执行横移，输出车体前向物理速度。 */
    car_forward_target = g_car_mode4_state.velocity_forward_target_mps;
    car_yaw_rate_target = 0.0f;
}
