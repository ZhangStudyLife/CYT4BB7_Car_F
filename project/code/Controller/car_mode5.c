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
#include "car_mode.h"
#include "car_loop.h"
#include "pid_core.h"
#include <math.h>

#define MODE5_DEG_TO_RAD                 (0.017453292519943295f)
#define MODE5_RAD_TO_DEG                 (57.29577951308232f)

#define MODE5_LARGE_TURN_NORMAL          (0U)
#define MODE5_LARGE_TURN_BRAKE           (1U)
#define MODE5_LARGE_TURN_PIVOT           (2U)
volatile float mode5_speed_left_kp = 10.80f;
volatile float mode5_speed_left_ki = 0.26f;
volatile float mode5_speed_left_kd = 1.00f;
volatile float mode5_speed_right_kp = 10.80f;
volatile float mode5_speed_right_ki = 0.26f;
volatile float mode5_speed_right_kd = 1.00f;
volatile float mode5_speed_filter_alpha = 0.557f;
volatile float mode5_gyroz_kp = 1.27f;
volatile float mode5_gyroz_ki = 0.022f;
volatile float mode5_gyroz_kff = 0.12f;
volatile float mode5_gyroz_k_turn = 1.0f;
volatile float mode5_yaw_kp = 6.00f;
volatile float mode5_yaw_kd = 0.50f;
volatile float mode5_target_speed_mps = 2.5f;
volatile float mode5_input_deadzone = 100.0f;
volatile float mode5_alignment_stop_deg = 90.0f;
volatile float mode5_wheel_target_limit = 1000.0f;
volatile float mode5_speed_i_limit = 2000.0f;
volatile float mode5_speed_ff_deadband = 10.0f;
volatile float mode5_speed_ff_transition = 100.0f;
volatile float mode5_speed_pwm_step_limit = 6000.0f;
volatile float mode5_speed_decel_step = 600.0f;
volatile float mode5_gyroz_output_limit = 600.0f;
volatile float mode5_yaw_rate_limit_dps = 1000.0f;
volatile float mode5_wheel_stop_speed = 8.0f;
volatile float mode5_large_turn_brake_speed = 80.0f;
volatile float mode5_large_turn_brake_target = 20.0f;
volatile float mode5_large_turn_brake_ff = 1200.0f;
volatile float mode5_large_turn_brake_rate_dps = 300.0f;
volatile float mode5_large_turn_enter_deg = 90.0f;
volatile float mode5_large_turn_pivot_exit_deg = 45.0f;
volatile float mode5_large_turn_exit_start_deg = 35.0f;
volatile float mode5_large_turn_finish_deg = 3.0f;
volatile float mode5_large_turn_trigger_cycles = 5.0f;
volatile float mode5_large_turn_finish_cycles = 2.0f;
volatile float mode5_large_turn_timeout_cycles = 500.0f;
volatile float mode5_exit_command_match_deg = 80.0f;
volatile float mode5_brake_target_margin = 5.0f;
volatile float mode5_brake_ff_fade_span = 40.0f;

/* 纵向力模型系数：a = b0*pwm_norm + b1*v + b2*sign(v) + b4。 */
static const float mode5_ff_fan_table[8] =
    {0.0f, 2000.0f, 2500.0f, 3000.0f, 3500.0f, 4000.0f, 4500.0f, 5000.0f};
static const float mode5_ff_model[8][4] = {
    {12.766022f, -0.8075469f, -0.3274675f, 0.4309255f},
    {12.461290f, -0.5966385f, -0.7292719f, 0.0420895f},
    {12.377394f, -0.7135110f, -0.2277669f, 0.1232141f},
    {13.180540f, -0.6681932f, -0.7527403f, 0.1125604f},
    {12.100573f, -0.6336953f, -0.7125850f, 0.1641450f},
    {14.248403f, -1.0152786f, -0.4983454f, 0.6730683f},
    {13.342708f, -0.7303908f, -0.8071498f, -0.2283004f},
    {14.170924f, -0.7917450f, -0.8738459f, -0.2449317f},
};

static pid_t s_mode5_left_speed_pid;
static pid_t s_mode5_right_speed_pid;
static pid_t s_mode5_yaw_pid;
static pid_t s_mode5_gyroz_pid;
static float s_mode5_yaw_target_deg;
static float s_mode5_gyroz_target_dps;
static uint8 s_mode5_large_turn_state;
static uint8 s_mode5_large_turn_rearm_required;
static int8 s_mode5_large_turn_direction;
static uint16 s_mode5_large_turn_trigger_cycles;
static uint16 s_mode5_large_turn_elapsed_cycles;
static float s_mode5_large_turn_target_yaw_deg;

static float mode5_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint16 mode5_cycle_count(float value, uint16 min_value,
                                uint16 max_value)
{
    value = mode5_clampf(value, (float)min_value, (float)max_value);
    return (uint16)(value + 0.5f);
}

static float mode5_wrap_deg(float angle_deg)
{
    if (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    else if (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float mode5_yaw_error_deg(void)
{
    return mode5_wrap_deg(s_mode5_yaw_target_deg - g_car_yaw_feedback_deg);
}

static void mode5_large_turn_reset(void)
{
    s_mode5_large_turn_state = MODE5_LARGE_TURN_NORMAL;
    s_mode5_large_turn_rearm_required = 0U;
    s_mode5_large_turn_direction = 0;
    s_mode5_large_turn_trigger_cycles = 0U;
    s_mode5_large_turn_elapsed_cycles = 0U;
    s_mode5_large_turn_target_yaw_deg = 0.0f;
}

static void mode5_large_turn_set(uint8 state)
{
    s_mode5_large_turn_state = state;
    if (state == MODE5_LARGE_TURN_NORMAL)
    {
        s_mode5_large_turn_direction = 0;
        s_mode5_large_turn_trigger_cycles = 0U;
        s_mode5_large_turn_elapsed_cycles = 0U;
        s_mode5_large_turn_target_yaw_deg = 0.0f;
    }
}

static float mode5_speed_feedforward(float target)
{
    float fan = mode5_clampf(g_car_negative_pressure_throttle,
                             mode5_ff_fan_table[0], mode5_ff_fan_table[7]);
    float speed_cnt = fabsf(target);
    float speed_mps = target / 115.0f;
    float direction = (target < 0.0f) ? -1.0f : 1.0f;
    float beta[4];
    float model_ff;
    float scale = 1.0f;
    uint8 index = 0U;

    while ((index < 7U) && (fan > mode5_ff_fan_table[index + 1U]))
    {
        index++;
    }
    if (index >= 7U)
    {
        index = 6U;
    }
    {
        float span = mode5_ff_fan_table[index + 1U] -
                     mode5_ff_fan_table[index];
        float ratio = (span > 0.0f)
                          ? (fan - mode5_ff_fan_table[index]) / span
                          : 0.0f;
        uint8 k;
        for (k = 0U; k < 4U; k++)
        {
            beta[k] = mode5_ff_model[index][k] +
                      ratio * (mode5_ff_model[index + 1U][k] -
                               mode5_ff_model[index][k]);
        }
    }

    if (speed_cnt <= mode5_speed_ff_deadband)
    {
        return 0.0f;
    }
    if (mode5_speed_ff_transition > mode5_speed_ff_deadband &&
        speed_cnt < mode5_speed_ff_transition)
    {
        scale = (speed_cnt - mode5_speed_ff_deadband) /
                (mode5_speed_ff_transition - mode5_speed_ff_deadband);
    }
    model_ff = -(beta[1] * speed_mps + beta[2] * direction + beta[3]) /
               beta[0] * (float)MOTOR_PWM_MAX;
    model_ff *= scale;
    return (target > 0.0f)
               ? mode5_clampf(model_ff, 0.0f, 5000.0f)
               : mode5_clampf(model_ff, -5000.0f, 0.0f);
}

static uint8 mode5_command_update(float heading_deg,
                                  float speed_mps,
                                  uint8 command_active)
{
    if (s_mode5_large_turn_state != MODE5_LARGE_TURN_NORMAL)
    {
        s_mode5_yaw_target_deg = s_mode5_large_turn_target_yaw_deg;
        g_car_base_speed_command = 0.0f;
        return command_active;
    }

    if (s_mode5_large_turn_rearm_required != 0U)
    {
        if (command_active == 0U)
        {
            s_mode5_large_turn_rearm_required = 0U;
        }
    }

    if (command_active == 0U)
    {
        g_car_base_speed_command = 0.0f;
        return 0U;
    }

    s_mode5_yaw_target_deg = mode5_wrap_deg(heading_deg);
    g_car_base_speed_command = car_speed_mps_to_encoder_cnt(speed_mps);
    return 1U;
}

static uint8 mode5_world_command_update(float *speed_mps)
{
    float ch0 = g_air_std_ch0;
    float ch1 = g_air_std_ch1;
    float magnitude = sqrtf(ch0 * ch0 + ch1 * ch1);
    float deadzone = mode5_clampf(mode5_input_deadzone, 0.0f, 500.0f);

    *speed_mps = mode5_clampf(mode5_target_speed_mps, 0.0f, 5.0f);

    return mode5_command_update(
        atan2f(ch0, ch1) * MODE5_RAD_TO_DEG,
        *speed_mps,
        (magnitude >= deadzone) ? 1U : 0U);
}

static void mode5_large_turn_timeout(void)
{
    mode5_large_turn_reset();
    s_mode5_large_turn_rearm_required = 1U;
    g_car_base_speed_command = 0.0f;
    s_mode5_yaw_target_deg = g_car_yaw_feedback_deg;
    PID_Reset(&s_mode5_yaw_pid);
    PID_Reset(&s_mode5_gyroz_pid);
}

static void mode5_large_turn_update(uint8 command_active,
                                    float command_speed_mps)
{
    float yaw_error = mode5_yaw_error_deg();
    float yaw_error_abs = fabsf(yaw_error);
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    uint8 translation_slow =
        (fabsf(body_speed) <= mode5_large_turn_brake_speed) ? 1U : 0U;
    uint16 trigger_cycles = mode5_cycle_count(
        mode5_large_turn_trigger_cycles, 1U, 100U);
    uint16 timeout_cycles = mode5_cycle_count(
        mode5_large_turn_timeout_cycles, 10U, 5000U);

    if (s_mode5_large_turn_state != MODE5_LARGE_TURN_NORMAL)
    {
        if (++s_mode5_large_turn_elapsed_cycles >=
            timeout_cycles)
        {
            mode5_large_turn_timeout();
            return;
        }
    }

    if (s_mode5_large_turn_state == MODE5_LARGE_TURN_NORMAL)
    {
        if ((s_mode5_large_turn_rearm_required == 0U) &&
            (command_active != 0U) &&
            (yaw_error_abs >= mode5_large_turn_enter_deg))
        {
            if (s_mode5_large_turn_trigger_cycles <
                trigger_cycles)
            {
                s_mode5_large_turn_trigger_cycles++;
            }

            if (s_mode5_large_turn_trigger_cycles >=
                trigger_cycles)
            {
                s_mode5_large_turn_direction = (yaw_error >= 0.0f) ? 1 : -1;
                s_mode5_large_turn_target_yaw_deg = s_mode5_yaw_target_deg;
                s_mode5_large_turn_elapsed_cycles = 0U;
                s_mode5_large_turn_trigger_cycles = 0U;
                mode5_large_turn_set((translation_slow != 0U)
                                         ? MODE5_LARGE_TURN_PIVOT
                                         : MODE5_LARGE_TURN_BRAKE);
            }
        }
        else
        {
            s_mode5_large_turn_trigger_cycles = 0U;
        }
    }
    else if (s_mode5_large_turn_state == MODE5_LARGE_TURN_BRAKE)
    {
        if (yaw_error_abs <= mode5_large_turn_pivot_exit_deg)
        {
            mode5_large_turn_set(MODE5_LARGE_TURN_NORMAL);
            g_car_base_speed_command =
                (command_active != 0U)
                    ? car_speed_mps_to_encoder_cnt(command_speed_mps)
                    : 0.0f;
        }
        else if (translation_slow != 0U)
        {
            mode5_large_turn_set(MODE5_LARGE_TURN_PIVOT);
        }
    }
    else if (s_mode5_large_turn_state == MODE5_LARGE_TURN_PIVOT)
    {
        if (yaw_error_abs <= mode5_large_turn_pivot_exit_deg)
        {
            mode5_large_turn_set(MODE5_LARGE_TURN_NORMAL);
            g_car_base_speed_command =
                (command_active != 0U)
                    ? car_speed_mps_to_encoder_cnt(command_speed_mps)
                    : 0.0f;
        }
    }

    if ((s_mode5_large_turn_state == MODE5_LARGE_TURN_BRAKE) ||
        (s_mode5_large_turn_state == MODE5_LARGE_TURN_PIVOT))
    {
        g_car_base_speed_command = 0.0f;
    }
}

static void mode5_speed_plan_update(void)
{
    float command;
    float delta;

    if (s_mode5_large_turn_state != MODE5_LARGE_TURN_BRAKE)
    {
        g_car_base_speed_target = g_car_base_speed_command;
        return;
    }

    command = mode5_clampf(
        mode5_large_turn_brake_target,
        0.0f,
        fmaxf(0.0f,
              mode5_large_turn_brake_speed - mode5_brake_target_margin));
    delta = mode5_clampf(command - g_car_base_speed_target,
                         -mode5_speed_decel_step,
                          mode5_speed_decel_step);
    g_car_base_speed_target += delta;
}

static float mode5_yaw_control(void)
{
    float yaw_error = mode5_yaw_error_deg();
    float desired_rate;

    if (((s_mode5_large_turn_state == MODE5_LARGE_TURN_BRAKE) ||
         (s_mode5_large_turn_state == MODE5_LARGE_TURN_PIVOT)) &&
        (s_mode5_large_turn_direction != 0))
    {
        yaw_error = (float)s_mode5_large_turn_direction * fabsf(yaw_error);
    }

    s_mode5_yaw_pid.kp = mode5_yaw_kp;
    s_mode5_yaw_pid.ki = 0.0f;
    s_mode5_yaw_pid.kd = mode5_yaw_kd;
    PID_SetOutputLimits(&s_mode5_yaw_pid,
                        -mode5_yaw_rate_limit_dps,
                         mode5_yaw_rate_limit_dps);
    desired_rate = PID_Update(&s_mode5_yaw_pid, yaw_error, 0.0f);
    if (s_mode5_large_turn_state == MODE5_LARGE_TURN_BRAKE)
    {
        desired_rate = mode5_clampf(
            desired_rate,
            -mode5_large_turn_brake_rate_dps,
             mode5_large_turn_brake_rate_dps);
    }
    return -desired_rate;
}

static void mode5_gyroz_control(float gyroz_target)
{
    float output;
    float left_abs;
    float right_abs;
    float wheel_peak;

    s_mode5_gyroz_pid.kp = mode5_gyroz_kp;
    s_mode5_gyroz_pid.ki = mode5_gyroz_ki;
    s_mode5_gyroz_pid.kd = 0.0f;
    s_mode5_gyroz_pid.kff = mode5_gyroz_kff;
    s_mode5_gyroz_pid.i_limit = mode5_gyroz_output_limit;
    PID_SetOutputLimits(&s_mode5_gyroz_pid,
                        -mode5_gyroz_output_limit,
                         mode5_gyroz_output_limit);
    output = PID_Update(&s_mode5_gyroz_pid,
                        -gyroz_target,
                        g_car_gyroz_feedback_dps);

    Left_Target_Speed = g_car_base_speed_target +
                        mode5_gyroz_k_turn * output;
    Right_Target_Speed = g_car_base_speed_target -
                         mode5_gyroz_k_turn * output;
    left_abs = fabsf(Left_Target_Speed);
    right_abs = fabsf(Right_Target_Speed);
    wheel_peak = (left_abs > right_abs) ? left_abs : right_abs;
    if (wheel_peak > mode5_wheel_target_limit)
    {
        Left_Target_Speed *= mode5_wheel_target_limit / wheel_peak;
        Right_Target_Speed *= mode5_wheel_target_limit / wheel_peak;
    }
}

static float mode5_speed_direction(float feedback, float fallback)
{
    if (fabsf(feedback) > mode5_wheel_stop_speed)
    {
        return (feedback > 0.0f) ? 1.0f : -1.0f;
    }
    if (fallback > 0.0f)
    {
        return 1.0f;
    }
    if (fallback < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

static float mode5_large_turn_brake_feedforward(void)
{
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    float speed_abs = fabsf(body_speed);
    float scale;

    if ((s_mode5_large_turn_state != MODE5_LARGE_TURN_BRAKE) ||
        (speed_abs <= mode5_large_turn_brake_speed))
    {
        return 0.0f;
    }
    scale = mode5_clampf(
        (speed_abs - mode5_large_turn_brake_speed) /
            fmaxf(mode5_brake_ff_fade_span, 1.0f),
        0.0f, 1.0f);
    return -mode5_speed_direction(body_speed, 0.0f) *
           mode5_large_turn_brake_ff * scale;
}

static int16 mode5_speed_pid_update(pid_t *pid,
                                    float target,
                                    float feedback,
                                    float kp,
                                    float ki,
                                    float kd,
                                    float brake_ff)
{
    float output;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = 0.0f;
    pid->i_limit = mode5_speed_i_limit;
    PID_SetExternalFeedforward(pid, mode5_speed_feedforward(target));
    PID_SetIncrementLimit(pid, mode5_speed_pwm_step_limit);
    PID_SetOutputLimits(pid, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    output = PID_UpdatePositionLimited(pid, target, feedback) + brake_ff;
    return (int16)mode5_clampf(output,
                              -(float)MOTOR_PWM_MAX,
                               (float)MOTOR_PWM_MAX);
}

void car_mode5_init(void)
{
    PID_Init(&s_mode5_left_speed_pid, mode5_speed_left_kp, mode5_speed_left_ki,
             mode5_speed_left_kd, 0.0f, mode5_speed_i_limit);
    PID_Init(&s_mode5_right_speed_pid, mode5_speed_right_kp, mode5_speed_right_ki,
             mode5_speed_right_kd, 0.0f, mode5_speed_i_limit);
    PID_Init(&s_mode5_yaw_pid, mode5_yaw_kp, 0.0f,
             mode5_yaw_kd, 0.0f, 0.0f);
    PID_Init(&s_mode5_gyroz_pid, mode5_gyroz_kp, mode5_gyroz_ki, 0.0f,
             mode5_gyroz_kff, mode5_gyroz_output_limit);
    car_mode5_reset();
}

void car_mode5_reset(void)
{
    PID_Reset(&s_mode5_left_speed_pid);
    PID_Reset(&s_mode5_right_speed_pid);
    PID_Reset(&s_mode5_yaw_pid);
    PID_Reset(&s_mode5_gyroz_pid);
    s_mode5_yaw_target_deg = g_car_yaw_feedback_deg;
    s_mode5_gyroz_target_dps = 0.0f;
    mode5_large_turn_reset();
    g_car_base_speed_command = 0.0f;
    g_car_base_speed_target = 0.0f;
    Left_Target_Speed = 0.0f;
    Right_Target_Speed = 0.0f;
    g_car_speed_left_motor_output = 0.0f;
    g_car_speed_right_motor_output = 0.0f;
}

static void mode5_control_update(float command_speed_mps)
{
    float gyroz_target;
    float brake_ff;
    int16 left_output;
    int16 right_output;

    /* Temporary pure-yaw test: keep the large-turn state machine inactive. */
    mode5_large_turn_update(0U, command_speed_mps);
    mode5_speed_plan_update();
    gyroz_target = mode5_yaw_control();
    s_mode5_gyroz_target_dps = -gyroz_target;
    mode5_gyroz_control(gyroz_target);

    brake_ff = mode5_large_turn_brake_feedforward();

    left_output = mode5_speed_pid_update(
        &s_mode5_left_speed_pid,
        Left_Target_Speed, g_car_speed_left_filtered,
        mode5_speed_left_kp, mode5_speed_left_ki, mode5_speed_left_kd,
        brake_ff);
    right_output = mode5_speed_pid_update(
        &s_mode5_right_speed_pid,
        Right_Target_Speed, g_car_speed_right_filtered,
        mode5_speed_right_kp, mode5_speed_right_ki, mode5_speed_right_kd,
        brake_ff);
    g_car_speed_left_motor_output = (float)left_output;
    g_car_speed_right_motor_output = (float)right_output;
    motor_left_set_speed(left_output);
    motor_right_set_speed(right_output);
}

void car_mode5_update_100HZ(uint32 now_ms)
{
    float speed_mps;

    (void)now_ms;
    (void)mode5_world_command_update(&speed_mps);
    mode5_control_update(speed_mps);
}

void car_mode5_get_diag(car_drive_diag_t *diag)
{
    diag->yaw_target_deg = s_mode5_yaw_target_deg;
    diag->yaw_error_deg = mode5_yaw_error_deg();
    diag->gyroz_target_dps = s_mode5_gyroz_target_dps;
    diag->gyroz_output = s_mode5_gyroz_pid.output;
    diag->yaw_p_term = s_mode5_yaw_pid.p_term;
    diag->yaw_d_term = s_mode5_yaw_pid.d_term;
    diag->yaw_output = s_mode5_yaw_pid.output;
    diag->gyroz_p_term = s_mode5_gyroz_pid.p_term;
    diag->gyroz_i_term = s_mode5_gyroz_pid.i_term;
    diag->gyroz_ff_term = s_mode5_gyroz_pid.ff_term;
    diag->left_speed_p_term = s_mode5_left_speed_pid.p_term;
    diag->left_speed_i_term = s_mode5_left_speed_pid.i_term;
    diag->left_speed_d_term = s_mode5_left_speed_pid.d_term;
    diag->left_speed_ff_term = s_mode5_left_speed_pid.ff_term;
    diag->left_brake_ff = g_car_speed_left_motor_output -
                          s_mode5_left_speed_pid.output;
    diag->right_speed_p_term = s_mode5_right_speed_pid.p_term;
    diag->right_speed_i_term = s_mode5_right_speed_pid.i_term;
    diag->right_speed_d_term = s_mode5_right_speed_pid.d_term;
    diag->right_speed_ff_term = s_mode5_right_speed_pid.ff_term;
    diag->right_brake_ff = g_car_speed_right_motor_output -
                           s_mode5_right_speed_pid.output;
    diag->large_turn_target_yaw_deg = s_mode5_large_turn_target_yaw_deg;
    diag->large_turn_target_speed_mps = 0.0f;
    diag->large_turn_trigger_cycles = s_mode5_large_turn_trigger_cycles;
    diag->large_turn_finish_cycles = 0U;
    diag->large_turn_elapsed_cycles = s_mode5_large_turn_elapsed_cycles;
    diag->large_turn_direction = s_mode5_large_turn_direction;
    diag->large_turn_state = s_mode5_large_turn_state;
    diag->large_turn_rearm_required = s_mode5_large_turn_rearm_required;
    diag->speed_brake_active = 0U;
}
