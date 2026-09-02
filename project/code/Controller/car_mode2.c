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
#include "pid_core.h"
#include <math.h>

#define MODE2_DEG_TO_RAD                 (0.017453292519943295f)
#define MODE2_RAD_TO_DEG                 (57.29577951308232f)

#define MODE2_LARGE_TURN_NORMAL          (0U)
#define MODE2_LARGE_TURN_BRAKE           (1U)
#define MODE2_LARGE_TURN_PIVOT           (2U)
#define MODE2_LARGE_TURN_EXIT            (3U)
volatile float mode2_speed_left_kp = 10.80f;
volatile float mode2_speed_left_ki = 0.52f;
volatile float mode2_speed_left_kd = 1.00f;
volatile float mode2_speed_right_kp = 10.80f;
volatile float mode2_speed_right_ki = 0.52f;
volatile float mode2_speed_right_kd = 1.00f;
volatile float mode2_speed_filter_alpha = 0.557f;
volatile float mode2_speed_ff_static = 800.0f;
volatile float mode2_gyroz_kp = 1.27f;
volatile float mode2_gyroz_ki = 0.022f;
volatile float mode2_gyroz_kff = 0.12f;
volatile float mode2_gyroz_k_turn = 1.0f;
volatile float mode2_yaw_kp = 6.00f;
volatile float mode2_yaw_kd = 0.50f;
volatile float mode2_target_speed_mps = 2.5f;
volatile float mode2_input_deadzone = 100.0f;
volatile float mode2_alignment_stop_deg = 90.0f;
volatile float mode2_wheel_target_limit = 1000.0f;
volatile float mode2_speed_i_limit = 2000.0f;
volatile float mode2_speed_ff_deadband = 10.0f;
volatile float mode2_speed_ff_transition = 100.0f;
volatile float mode2_speed_increment_limit = 6000.0f;
volatile float mode2_speed_brake_static = 800.0f;
volatile float mode2_speed_decel_step = 600.0f;
volatile float mode2_gyroz_output_limit = 600.0f;
volatile float mode2_yaw_rate_limit_dps = 1000.0f;
volatile float mode2_gyroz_stop_target_dps = 2.0f;
volatile float mode2_wheel_stop_speed = 8.0f;
volatile float mode2_large_turn_brake_speed = 80.0f;
volatile float mode2_large_turn_brake_target = 20.0f;
volatile float mode2_large_turn_brake_ff = 1200.0f;
volatile float mode2_large_turn_brake_rate_dps = 300.0f;
volatile float mode2_large_turn_enter_deg = 90.0f;
volatile float mode2_large_turn_pivot_exit_deg = 35.0f;
volatile float mode2_large_turn_exit_start_deg = 35.0f;
volatile float mode2_large_turn_finish_deg = 3.0f;
volatile float mode2_large_turn_trigger_cycles = 5.0f;
volatile float mode2_large_turn_finish_cycles = 2.0f;
volatile float mode2_large_turn_timeout_cycles = 500.0f;
volatile float mode2_exit_command_match_deg = 80.0f;
volatile float mode2_brake_target_margin = 5.0f;
volatile float mode2_brake_ff_fade_span = 40.0f;

static pid_t s_mode2_left_speed_pid;
static pid_t s_mode2_right_speed_pid;
static pid_t s_mode2_yaw_pid;
static pid_t s_mode2_gyroz_pid;
static float s_mode2_yaw_target_deg;
static float s_mode2_gyroz_target_dps;
static float s_mode2_previous_base_target;
static uint8 s_mode2_speed_brake_active;
static float s_mode2_left_brake_direction;
static float s_mode2_right_brake_direction;
static uint8 s_mode2_large_turn_state;
static uint8 s_mode2_large_turn_rearm_required;
static int8 s_mode2_large_turn_direction;
static uint16 s_mode2_large_turn_trigger_cycles;
static uint16 s_mode2_large_turn_finish_cycles;
static uint16 s_mode2_large_turn_elapsed_cycles;
static float s_mode2_large_turn_target_yaw_deg;
static float s_mode2_large_turn_target_speed_mps;

static float mode2_clampf(float value, float min_value, float max_value)
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

static uint16 mode2_cycle_count(float value, uint16 min_value,
                                uint16 max_value)
{
    value = mode2_clampf(value, (float)min_value, (float)max_value);
    return (uint16)(value + 0.5f);
}

static float mode2_wrap_deg(float angle_deg)
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

static float mode2_yaw_error_deg(void)
{
    return mode2_wrap_deg(s_mode2_yaw_target_deg - g_car_yaw_feedback_deg);
}

static void mode2_large_turn_reset(void)
{
    s_mode2_large_turn_state = MODE2_LARGE_TURN_NORMAL;
    s_mode2_large_turn_rearm_required = 0U;
    s_mode2_large_turn_direction = 0;
    s_mode2_large_turn_trigger_cycles = 0U;
    s_mode2_large_turn_finish_cycles = 0U;
    s_mode2_large_turn_elapsed_cycles = 0U;
    s_mode2_large_turn_target_yaw_deg = 0.0f;
    s_mode2_large_turn_target_speed_mps = 0.0f;
}

static void mode2_large_turn_set(uint8 state)
{
    s_mode2_large_turn_state = state;
    if (state == MODE2_LARGE_TURN_NORMAL)
    {
        s_mode2_large_turn_direction = 0;
        s_mode2_large_turn_trigger_cycles = 0U;
        s_mode2_large_turn_finish_cycles = 0U;
        s_mode2_large_turn_elapsed_cycles = 0U;
        s_mode2_large_turn_target_yaw_deg = 0.0f;
        s_mode2_large_turn_target_speed_mps = 0.0f;
    }
}

static uint8 mode2_exit_command_matches(float heading_deg,
                                        uint8 command_active)
{
    if (command_active == 0U)
    {
        return 0U;
    }
    return (fabsf(mode2_wrap_deg(
                heading_deg - s_mode2_large_turn_target_yaw_deg)) <=
            mode2_exit_command_match_deg) ? 1U : 0U;
}

static void mode2_apply_latched_command(float heading_deg,
                                        uint8 command_active)
{
    float yaw_error = mode2_wrap_deg(
        s_mode2_large_turn_target_yaw_deg - g_car_yaw_feedback_deg);
    float exit_start_deg = fmaxf(mode2_large_turn_exit_start_deg,
                                 mode2_large_turn_finish_deg + 0.5f);
    float alignment = 0.0f;

    if ((s_mode2_large_turn_state == MODE2_LARGE_TURN_EXIT) &&
        (mode2_exit_command_matches(heading_deg, command_active) != 0U) &&
        (fabsf(yaw_error) < exit_start_deg))
    {
        alignment = (exit_start_deg - fabsf(yaw_error)) /
                    (exit_start_deg -
                     mode2_large_turn_finish_deg);
        alignment = mode2_clampf(alignment, 0.0f, 1.0f);
    }

    s_mode2_yaw_target_deg = s_mode2_large_turn_target_yaw_deg;
    g_car_base_speed_command =
        car_speed_mps_to_encoder_cnt(s_mode2_large_turn_target_speed_mps) *
        alignment;
}

static uint8 mode2_command_update(float heading_deg,
                                  float speed_mps,
                                  uint8 command_active)
{
    float yaw_error;

    if (s_mode2_large_turn_state != MODE2_LARGE_TURN_NORMAL)
    {
        if ((s_mode2_large_turn_state == MODE2_LARGE_TURN_EXIT) &&
            (command_active != 0U) &&
            (mode2_exit_command_matches(heading_deg, command_active) == 0U))
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_NORMAL);
        }
        else
        {
            mode2_apply_latched_command(heading_deg, command_active);
            return 1U;
        }
    }

    if (s_mode2_large_turn_rearm_required != 0U)
    {
        if (command_active == 0U)
        {
            s_mode2_large_turn_rearm_required = 0U;
        }
    }

    if (command_active == 0U)
    {
        g_car_base_speed_command = 0.0f;
        return 0U;
    }

    s_mode2_yaw_target_deg = mode2_wrap_deg(heading_deg);
    yaw_error = mode2_yaw_error_deg();
    g_car_base_speed_command =
        (fabsf(yaw_error) >= mode2_alignment_stop_deg)
            ? 0.0f
            : car_speed_mps_to_encoder_cnt(speed_mps) *
                  cosf(yaw_error * MODE2_DEG_TO_RAD);
    return 1U;
}

static uint8 mode2_world_command_update(float *speed_mps)
{
    float ch0 = g_air_std_ch0;
    float ch1 = g_air_std_ch1;
    float magnitude = sqrtf(ch0 * ch0 + ch1 * ch1);
    float deadzone = mode2_clampf(mode2_input_deadzone, 0.0f, 500.0f);

    *speed_mps = mode2_clampf(mode2_target_speed_mps, 0.0f, 5.0f);

    return mode2_command_update(
        atan2f(ch0, ch1) * MODE2_RAD_TO_DEG,
        *speed_mps,
        (magnitude >= deadzone) ? 1U : 0U);
}

static void mode2_large_turn_timeout(void)
{
    mode2_large_turn_reset();
    s_mode2_large_turn_rearm_required = 1U;
    g_car_base_speed_command = 0.0f;
    s_mode2_yaw_target_deg = g_car_yaw_feedback_deg;
    PID_Reset(&s_mode2_yaw_pid);
    PID_Reset(&s_mode2_gyroz_pid);
}

static void mode2_large_turn_update(uint8 command_active,
                                    float command_speed_mps)
{
    float yaw_error = mode2_yaw_error_deg();
    float yaw_error_abs = fabsf(yaw_error);
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    uint8 translation_slow =
        (fabsf(body_speed) <= mode2_large_turn_brake_speed) ? 1U : 0U;
    uint16 trigger_cycles = mode2_cycle_count(
        mode2_large_turn_trigger_cycles, 1U, 100U);
    uint16 finish_cycles = mode2_cycle_count(
        mode2_large_turn_finish_cycles, 1U, 100U);
    uint16 timeout_cycles = mode2_cycle_count(
        mode2_large_turn_timeout_cycles, 10U, 5000U);

    if (s_mode2_large_turn_state != MODE2_LARGE_TURN_NORMAL)
    {
        if (++s_mode2_large_turn_elapsed_cycles >=
            timeout_cycles)
        {
            mode2_large_turn_timeout();
            return;
        }
    }

    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_NORMAL)
    {
        if ((s_mode2_large_turn_rearm_required == 0U) &&
            (command_active != 0U) &&
            (yaw_error_abs >= mode2_large_turn_enter_deg))
        {
            if (s_mode2_large_turn_trigger_cycles <
                trigger_cycles)
            {
                s_mode2_large_turn_trigger_cycles++;
            }

            if (s_mode2_large_turn_trigger_cycles >=
                trigger_cycles)
            {
                s_mode2_large_turn_direction = (yaw_error >= 0.0f) ? 1 : -1;
                s_mode2_large_turn_target_yaw_deg = s_mode2_yaw_target_deg;
                s_mode2_large_turn_target_speed_mps = command_speed_mps;
                s_mode2_large_turn_elapsed_cycles = 0U;
                s_mode2_large_turn_trigger_cycles = 0U;
                mode2_large_turn_set((translation_slow != 0U)
                                         ? MODE2_LARGE_TURN_PIVOT
                                         : MODE2_LARGE_TURN_BRAKE);
            }
        }
        else
        {
            s_mode2_large_turn_trigger_cycles = 0U;
        }
    }
    else if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        if (yaw_error_abs <= mode2_large_turn_pivot_exit_deg)
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_EXIT);
        }
        else if (translation_slow != 0U)
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_PIVOT);
        }
    }
    else if (s_mode2_large_turn_state == MODE2_LARGE_TURN_PIVOT)
    {
        if (yaw_error_abs <= mode2_large_turn_pivot_exit_deg)
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_EXIT);
        }
    }
    else if (yaw_error_abs <= mode2_large_turn_finish_deg)
    {
        if (++s_mode2_large_turn_finish_cycles >=
            finish_cycles)
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_NORMAL);
        }
    }
    else
    {
        s_mode2_large_turn_finish_cycles = 0U;
    }

    if ((s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE) ||
        (s_mode2_large_turn_state == MODE2_LARGE_TURN_PIVOT))
    {
        g_car_base_speed_command = 0.0f;
    }
}

static void mode2_speed_plan_update(void)
{
    float command;
    float delta;

    s_mode2_previous_base_target = g_car_base_speed_target;
    if (s_mode2_large_turn_state != MODE2_LARGE_TURN_BRAKE)
    {
        g_car_base_speed_target = g_car_base_speed_command;
        return;
    }

    command = mode2_clampf(
        mode2_large_turn_brake_target,
        0.0f,
        fmaxf(0.0f,
              mode2_large_turn_brake_speed - mode2_brake_target_margin));
    delta = mode2_clampf(command - g_car_base_speed_target,
                         -mode2_speed_decel_step,
                          mode2_speed_decel_step);
    g_car_base_speed_target += delta;
}

static float mode2_yaw_control(void)
{
    float yaw_error = mode2_yaw_error_deg();
    float desired_rate;

    if (((s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE) ||
         (s_mode2_large_turn_state == MODE2_LARGE_TURN_PIVOT)) &&
        (s_mode2_large_turn_direction != 0))
    {
        yaw_error = (float)s_mode2_large_turn_direction * fabsf(yaw_error);
    }

    s_mode2_yaw_pid.kp = mode2_yaw_kp;
    s_mode2_yaw_pid.ki = 0.0f;
    s_mode2_yaw_pid.kd = mode2_yaw_kd;
    PID_SetOutputLimits(&s_mode2_yaw_pid,
                        -mode2_yaw_rate_limit_dps,
                         mode2_yaw_rate_limit_dps);
    desired_rate = PID_Update(&s_mode2_yaw_pid, yaw_error, 0.0f);
    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        desired_rate = mode2_clampf(
            desired_rate,
            -mode2_large_turn_brake_rate_dps,
             mode2_large_turn_brake_rate_dps);
    }
    return -desired_rate;
}

static void mode2_gyroz_control(float gyroz_target)
{
    float output;
    float left_abs;
    float right_abs;
    float wheel_peak;

    s_mode2_gyroz_pid.kp = mode2_gyroz_kp;
    s_mode2_gyroz_pid.ki = mode2_gyroz_ki;
    s_mode2_gyroz_pid.kd = 0.0f;
    s_mode2_gyroz_pid.kff = mode2_gyroz_kff;
    s_mode2_gyroz_pid.i_limit = mode2_gyroz_output_limit;
    PID_SetOutputLimits(&s_mode2_gyroz_pid,
                        -mode2_gyroz_output_limit,
                         mode2_gyroz_output_limit);
    output = PID_Update(&s_mode2_gyroz_pid,
                        -gyroz_target,
                        g_car_gyroz_feedback_dps);

    Left_Target_Speed = g_car_base_speed_target +
                        mode2_gyroz_k_turn * output;
    Right_Target_Speed = g_car_base_speed_target -
                         mode2_gyroz_k_turn * output;
    left_abs = fabsf(Left_Target_Speed);
    right_abs = fabsf(Right_Target_Speed);
    wheel_peak = (left_abs > right_abs) ? left_abs : right_abs;
    if (wheel_peak > mode2_wheel_target_limit)
    {
        Left_Target_Speed *= mode2_wheel_target_limit / wheel_peak;
        Right_Target_Speed *= mode2_wheel_target_limit / wheel_peak;
    }
}

static float mode2_speed_direction(float feedback, float fallback)
{
    if (fabsf(feedback) > mode2_wheel_stop_speed)
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

static void mode2_brake_update(uint8 command_active, float gyroz_target)
{
    if ((command_active != 0U) ||
        (fabsf(gyroz_target) >= mode2_gyroz_stop_target_dps))
    {
        s_mode2_speed_brake_active = 0U;
        return;
    }
    if ((s_mode2_speed_brake_active == 0U) &&
        ((fabsf(g_car_speed_left_filtered) > mode2_wheel_stop_speed) ||
         (fabsf(g_car_speed_right_filtered) > mode2_wheel_stop_speed)))
    {
        s_mode2_left_brake_direction = mode2_speed_direction(
            g_car_speed_left_filtered, s_mode2_previous_base_target);
        s_mode2_right_brake_direction = mode2_speed_direction(
            g_car_speed_right_filtered, s_mode2_previous_base_target);
        s_mode2_speed_brake_active = 1U;
    }
}

static float mode2_brake_feedforward(float feedback, float direction)
{
    float speed_abs = fabsf(feedback);
    float ff_deadband = fmaxf(mode2_speed_ff_deadband,
                              mode2_wheel_stop_speed + 1.0f);
    float scale;

    if ((s_mode2_speed_brake_active == 0U) || (direction == 0.0f) ||
        (feedback * direction <= 0.0f) ||
        (speed_abs <= mode2_wheel_stop_speed))
    {
        return 0.0f;
    }
    scale = (speed_abs >= ff_deadband)
                ? 1.0f
                : (speed_abs - mode2_wheel_stop_speed) /
                      (ff_deadband - mode2_wheel_stop_speed);
    return -direction * mode2_speed_brake_static * scale;
}

static float mode2_large_turn_brake_feedforward(void)
{
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    float speed_abs = fabsf(body_speed);
    float scale;

    if ((s_mode2_large_turn_state != MODE2_LARGE_TURN_BRAKE) ||
        (speed_abs <= mode2_large_turn_brake_speed))
    {
        return 0.0f;
    }
    scale = mode2_clampf(
        (speed_abs - mode2_large_turn_brake_speed) /
            fmaxf(mode2_brake_ff_fade_span, 1.0f),
        0.0f, 1.0f);
    return -mode2_speed_direction(body_speed, 0.0f) *
           mode2_large_turn_brake_ff * scale;
}

static int16 mode2_speed_pid_update(pid_t *pid,
                                    float target,
                                    float feedback,
                                    float kp,
                                    float ki,
                                    float kd,
                                    float brake_ff)
{
    float output;
    float ff_deadband = fmaxf(mode2_speed_ff_deadband,
                              mode2_wheel_stop_speed + 1.0f);
    float ff_transition = fmaxf(mode2_speed_ff_transition, ff_deadband);

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = 0.0f;
    pid->i_limit = mode2_speed_i_limit;
    PID_SetStaticFeedforward(pid, mode2_speed_ff_static);
    PID_SetFeedforwardTransition(pid, ff_deadband, ff_transition);
    PID_SetExternalFeedforward(pid, 0.0f);
    PID_SetIncrementLimit(pid, mode2_speed_increment_limit);
    PID_SetOutputLimits(pid, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    output = PID_UpdateIncremental(pid, target, feedback) + brake_ff;
    return (int16)mode2_clampf(output,
                              -(float)MOTOR_PWM_MAX,
                               (float)MOTOR_PWM_MAX);
}

void car_mode2_init(void)
{
    PID_Init(&s_mode2_left_speed_pid, mode2_speed_left_kp, mode2_speed_left_ki,
             mode2_speed_left_kd, 0.0f, mode2_speed_i_limit);
    PID_Init(&s_mode2_right_speed_pid, mode2_speed_right_kp, mode2_speed_right_ki,
             mode2_speed_right_kd, 0.0f, mode2_speed_i_limit);
    PID_Init(&s_mode2_yaw_pid, mode2_yaw_kp, 0.0f,
             mode2_yaw_kd, 0.0f, 0.0f);
    PID_Init(&s_mode2_gyroz_pid, mode2_gyroz_kp, mode2_gyroz_ki, 0.0f,
             mode2_gyroz_kff, mode2_gyroz_output_limit);
    car_mode2_reset();
}

void car_mode2_reset(void)
{
    PID_Reset(&s_mode2_left_speed_pid);
    PID_Reset(&s_mode2_right_speed_pid);
    PID_Reset(&s_mode2_yaw_pid);
    PID_Reset(&s_mode2_gyroz_pid);
    s_mode2_yaw_target_deg = g_car_yaw_feedback_deg;
    s_mode2_gyroz_target_dps = 0.0f;
    s_mode2_previous_base_target = 0.0f;
    s_mode2_speed_brake_active = 0U;
    s_mode2_left_brake_direction = 0.0f;
    s_mode2_right_brake_direction = 0.0f;
    mode2_large_turn_reset();
    g_car_base_speed_command = 0.0f;
    g_car_base_speed_target = 0.0f;
    Left_Target_Speed = 0.0f;
    Right_Target_Speed = 0.0f;
    g_car_speed_left_motor_output = 0.0f;
    g_car_speed_right_motor_output = 0.0f;
}

static void mode2_control_update(uint8 command_active,
                                 float command_speed_mps)
{
    float gyroz_target;
    float left_brake_ff;
    float right_brake_ff;
    float large_turn_brake_ff;
    int16 left_output;
    int16 right_output;

    mode2_large_turn_update(command_active, command_speed_mps);
    mode2_speed_plan_update();
    gyroz_target = mode2_yaw_control();
    s_mode2_gyroz_target_dps = -gyroz_target;
    mode2_gyroz_control(gyroz_target);

    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        s_mode2_speed_brake_active = 0U;
    }
    else
    {
        mode2_brake_update(command_active, gyroz_target);
    }

    large_turn_brake_ff = mode2_large_turn_brake_feedforward();
    left_brake_ff = (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
                        ? large_turn_brake_ff
                        : mode2_brake_feedforward(
                              g_car_speed_left_filtered,
                              s_mode2_left_brake_direction);
    right_brake_ff = (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
                         ? large_turn_brake_ff
                         : mode2_brake_feedforward(
                               g_car_speed_right_filtered,
                               s_mode2_right_brake_direction);

    left_output = mode2_speed_pid_update(
        &s_mode2_left_speed_pid,
        Left_Target_Speed, g_car_speed_left_filtered,
        mode2_speed_left_kp, mode2_speed_left_ki, mode2_speed_left_kd,
        left_brake_ff);
    right_output = mode2_speed_pid_update(
        &s_mode2_right_speed_pid,
        Right_Target_Speed, g_car_speed_right_filtered,
        mode2_speed_right_kp, mode2_speed_right_ki, mode2_speed_right_kd,
        right_brake_ff);
    g_car_speed_left_motor_output = (float)left_output;
    g_car_speed_right_motor_output = (float)right_output;
    motor_left_set_speed(left_output);
    motor_right_set_speed(right_output);
}

void car_mode2_update_100HZ(uint32 now_ms)
{
    float speed_mps;
    uint8 command_active;

    (void)now_ms;
    command_active = mode2_world_command_update(&speed_mps);
    mode2_control_update(command_active, speed_mps);
}

void car_mode2_get_diag(car_drive_diag_t *diag)
{
    diag->yaw_target_deg = s_mode2_yaw_target_deg;
    diag->yaw_error_deg = mode2_yaw_error_deg();
    diag->gyroz_target_dps = s_mode2_gyroz_target_dps;
    diag->gyroz_output = s_mode2_gyroz_pid.output;
    diag->yaw_p_term = s_mode2_yaw_pid.p_term;
    diag->yaw_d_term = s_mode2_yaw_pid.d_term;
    diag->yaw_output = s_mode2_yaw_pid.output;
    diag->gyroz_p_term = s_mode2_gyroz_pid.p_term;
    diag->gyroz_i_term = s_mode2_gyroz_pid.i_term;
    diag->gyroz_ff_term = s_mode2_gyroz_pid.ff_term;
    diag->left_speed_p_term = s_mode2_left_speed_pid.p_term;
    diag->left_speed_i_term = s_mode2_left_speed_pid.i_term;
    diag->left_speed_d_term = s_mode2_left_speed_pid.d_term;
    diag->left_speed_ff_term = s_mode2_left_speed_pid.ff_term;
    diag->left_brake_ff = g_car_speed_left_motor_output -
                          s_mode2_left_speed_pid.output;
    diag->right_speed_p_term = s_mode2_right_speed_pid.p_term;
    diag->right_speed_i_term = s_mode2_right_speed_pid.i_term;
    diag->right_speed_d_term = s_mode2_right_speed_pid.d_term;
    diag->right_speed_ff_term = s_mode2_right_speed_pid.ff_term;
    diag->right_brake_ff = g_car_speed_right_motor_output -
                           s_mode2_right_speed_pid.output;
    diag->large_turn_target_yaw_deg = s_mode2_large_turn_target_yaw_deg;
    diag->large_turn_target_speed_mps = s_mode2_large_turn_target_speed_mps;
    diag->large_turn_trigger_cycles = s_mode2_large_turn_trigger_cycles;
    diag->large_turn_finish_cycles = s_mode2_large_turn_finish_cycles;
    diag->large_turn_elapsed_cycles = s_mode2_large_turn_elapsed_cycles;
    diag->large_turn_direction = s_mode2_large_turn_direction;
    diag->large_turn_state = s_mode2_large_turn_state;
    diag->large_turn_rearm_required = s_mode2_large_turn_rearm_required;
    diag->speed_brake_active = s_mode2_speed_brake_active;
}
