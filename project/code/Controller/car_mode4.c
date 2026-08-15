#include "car_mode.h"
#include "pid_core.h"
#include <math.h>

#define MODE4_DEG_TO_RAD                 (0.017453292519943295f)
#define MODE4_RAD_TO_DEG                 (57.29577951308232f)

#define MODE4_LARGE_TURN_NORMAL          (0U)
#define MODE4_LARGE_TURN_BRAKE           (1U)
#define MODE4_LARGE_TURN_PIVOT           (2U)
volatile float mode4_speed_left_kp = 10.80f;
volatile float mode4_speed_left_ki = 0.52f;
volatile float mode4_speed_left_kd = 1.00f;
volatile float mode4_speed_right_kp = 10.80f;
volatile float mode4_speed_right_ki = 0.52f;
volatile float mode4_speed_right_kd = 1.00f;
volatile float mode4_speed_filter_alpha = 0.557f;
volatile float mode4_speed_ff_static = 800.0f;
volatile float mode4_gyroz_kp = 1.27f;
volatile float mode4_gyroz_ki = 0.022f;
volatile float mode4_gyroz_kff = 0.12f;
volatile float mode4_gyroz_k_turn = 1.0f;
volatile float mode4_yaw_kp = 6.00f;
volatile float mode4_yaw_kd = 0.50f;
volatile float mode4_plan_speed_limit_mps = 3.0f;
volatile float mode4_alignment_stop_deg = 90.0f;
volatile float mode4_wheel_target_limit = 1000.0f;
volatile float mode4_speed_i_limit = 2000.0f;
volatile float mode4_speed_ff_deadband = 10.0f;
volatile float mode4_speed_ff_transition = 100.0f;
volatile float mode4_speed_increment_limit = 6000.0f;
volatile float mode4_speed_brake_static = 800.0f;
volatile float mode4_speed_decel_step = 600.0f;
volatile float mode4_gyroz_output_limit = 600.0f;
volatile float mode4_yaw_rate_limit_dps = 1000.0f;
volatile float mode4_gyroz_stop_target_dps = 2.0f;
volatile float mode4_wheel_stop_speed = 8.0f;
volatile float mode4_large_turn_brake_speed = 80.0f;
volatile float mode4_large_turn_brake_target = 20.0f;
volatile float mode4_large_turn_brake_ff = 1200.0f;
volatile float mode4_large_turn_brake_rate_dps = 300.0f;
volatile float mode4_large_turn_enter_deg = 90.0f;
volatile float mode4_large_turn_pivot_exit_deg = 45.0f;
volatile float mode4_large_turn_exit_start_deg = 35.0f;
volatile float mode4_large_turn_finish_deg = 3.0f;
volatile float mode4_large_turn_trigger_cycles = 5.0f;
volatile float mode4_large_turn_finish_cycles = 2.0f;
volatile float mode4_large_turn_timeout_cycles = 500.0f;
volatile float mode4_exit_command_match_deg = 80.0f;
volatile float mode4_brake_target_margin = 5.0f;
volatile float mode4_brake_ff_fade_span = 40.0f;

static pid_t s_mode4_left_speed_pid;
static pid_t s_mode4_right_speed_pid;
static pid_t s_mode4_yaw_pid;
static pid_t s_mode4_gyroz_pid;
static float s_mode4_yaw_target_deg;
static float s_mode4_gyroz_target_dps;
static float s_mode4_previous_base_target;
static uint8 s_mode4_speed_brake_active;
static float s_mode4_left_brake_direction;
static float s_mode4_right_brake_direction;
static uint8 s_mode4_large_turn_state;
static uint8 s_mode4_large_turn_rearm_required;
static int8 s_mode4_large_turn_direction;
static uint16 s_mode4_large_turn_trigger_cycles;
static uint16 s_mode4_large_turn_elapsed_cycles;
static float s_mode4_large_turn_target_yaw_deg;

static float mode4_clampf(float value, float min_value, float max_value)
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

static uint16 mode4_cycle_count(float value, uint16 min_value,
                                uint16 max_value)
{
    value = mode4_clampf(value, (float)min_value, (float)max_value);
    return (uint16)(value + 0.5f);
}

static float mode4_wrap_deg(float angle_deg)
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

static float mode4_yaw_error_deg(void)
{
    return mode4_wrap_deg(s_mode4_yaw_target_deg - g_car_yaw_feedback_deg);
}

static void mode4_large_turn_reset(void)
{
    s_mode4_large_turn_state = MODE4_LARGE_TURN_NORMAL;
    s_mode4_large_turn_rearm_required = 0U;
    s_mode4_large_turn_direction = 0;
    s_mode4_large_turn_trigger_cycles = 0U;
    s_mode4_large_turn_elapsed_cycles = 0U;
    s_mode4_large_turn_target_yaw_deg = 0.0f;
}

static void mode4_large_turn_set(uint8 state)
{
    s_mode4_large_turn_state = state;
    if (state == MODE4_LARGE_TURN_NORMAL)
    {
        s_mode4_large_turn_direction = 0;
        s_mode4_large_turn_trigger_cycles = 0U;
        s_mode4_large_turn_elapsed_cycles = 0U;
        s_mode4_large_turn_target_yaw_deg = 0.0f;
    }
}

static uint8 mode4_command_update(float heading_deg,
                                  float speed_mps,
                                  uint8 command_active)
{
    if (s_mode4_large_turn_state != MODE4_LARGE_TURN_NORMAL)
    {
        s_mode4_yaw_target_deg = s_mode4_large_turn_target_yaw_deg;
        g_car_base_speed_command = 0.0f;
        return command_active;
    }

    if (s_mode4_large_turn_rearm_required != 0U)
    {
        if (command_active == 0U)
        {
            s_mode4_large_turn_rearm_required = 0U;
        }
    }

    if (command_active == 0U)
    {
        g_car_base_speed_command = 0.0f;
        return 0U;
    }

    s_mode4_yaw_target_deg = mode4_wrap_deg(heading_deg);
    g_car_base_speed_command = car_speed_mps_to_encoder_cnt(speed_mps);
    return 1U;
}

static void mode4_abort_command(void)
{
    mode4_large_turn_reset();
    g_car_base_speed_command = 0.0f;
    s_mode4_yaw_target_deg = g_car_yaw_feedback_deg;
    PID_Reset(&s_mode4_yaw_pid);
    PID_Reset(&s_mode4_gyroz_pid);
}

static uint8 mode4_plan_command_update(float *speed_mps)
{
    float strafe_mps = g_air_car_plan_strafe_mps;
    float forward_mps = g_air_car_plan_forward_mps;
    float speed = sqrtf(strafe_mps * strafe_mps +
                        forward_mps * forward_mps);
    float speed_limit = mode4_clampf(mode4_plan_speed_limit_mps, 0.0f, 5.0f);

    if ((g_air_car_plan_valid < 0.5f) ||
        (speed <= 0.0f) || (speed_limit <= 0.0f))
    {
        mode4_abort_command();
        *speed_mps = 0.0f;
        return 0U;
    }
    if (speed > speed_limit)
    {
        strafe_mps *= speed_limit / speed;
        forward_mps *= speed_limit / speed;
        speed = speed_limit;
    }

    *speed_mps = speed;
    return mode4_command_update(
        g_car_yaw_feedback_deg +
            atan2f(strafe_mps, forward_mps) * MODE4_RAD_TO_DEG,
        speed, 1U);
}

static void mode4_large_turn_timeout(void)
{
    mode4_large_turn_reset();
    s_mode4_large_turn_rearm_required = 1U;
    g_car_base_speed_command = 0.0f;
    s_mode4_yaw_target_deg = g_car_yaw_feedback_deg;
    PID_Reset(&s_mode4_yaw_pid);
    PID_Reset(&s_mode4_gyroz_pid);
}

static void mode4_large_turn_update(uint8 command_active,
                                    float command_speed_mps)
{
    float yaw_error = mode4_yaw_error_deg();
    float yaw_error_abs = fabsf(yaw_error);
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    uint8 translation_slow =
        (fabsf(body_speed) <= mode4_large_turn_brake_speed) ? 1U : 0U;
    uint16 trigger_cycles = mode4_cycle_count(
        mode4_large_turn_trigger_cycles, 1U, 100U);
    uint16 timeout_cycles = mode4_cycle_count(
        mode4_large_turn_timeout_cycles, 10U, 5000U);

    if (s_mode4_large_turn_state != MODE4_LARGE_TURN_NORMAL)
    {
        if (++s_mode4_large_turn_elapsed_cycles >=
            timeout_cycles)
        {
            mode4_large_turn_timeout();
            return;
        }
    }

    if (s_mode4_large_turn_state == MODE4_LARGE_TURN_NORMAL)
    {
        if ((s_mode4_large_turn_rearm_required == 0U) &&
            (command_active != 0U) &&
            (yaw_error_abs >= mode4_large_turn_enter_deg))
        {
            if (s_mode4_large_turn_trigger_cycles <
                trigger_cycles)
            {
                s_mode4_large_turn_trigger_cycles++;
            }

            if (s_mode4_large_turn_trigger_cycles >=
                trigger_cycles)
            {
                s_mode4_large_turn_direction = (yaw_error >= 0.0f) ? 1 : -1;
                s_mode4_large_turn_target_yaw_deg = s_mode4_yaw_target_deg;
                s_mode4_large_turn_elapsed_cycles = 0U;
                s_mode4_large_turn_trigger_cycles = 0U;
                mode4_large_turn_set((translation_slow != 0U)
                                         ? MODE4_LARGE_TURN_PIVOT
                                         : MODE4_LARGE_TURN_BRAKE);
            }
        }
        else
        {
            s_mode4_large_turn_trigger_cycles = 0U;
        }
    }
    else if (s_mode4_large_turn_state == MODE4_LARGE_TURN_BRAKE)
    {
        if (yaw_error_abs <= mode4_large_turn_pivot_exit_deg)
        {
            mode4_large_turn_set(MODE4_LARGE_TURN_NORMAL);
            g_car_base_speed_command =
                (command_active != 0U)
                    ? car_speed_mps_to_encoder_cnt(command_speed_mps)
                    : 0.0f;
        }
        else if (translation_slow != 0U)
        {
            mode4_large_turn_set(MODE4_LARGE_TURN_PIVOT);
        }
    }
    else if (s_mode4_large_turn_state == MODE4_LARGE_TURN_PIVOT)
    {
        if (yaw_error_abs <= mode4_large_turn_pivot_exit_deg)
        {
            mode4_large_turn_set(MODE4_LARGE_TURN_NORMAL);
            g_car_base_speed_command =
                (command_active != 0U)
                    ? car_speed_mps_to_encoder_cnt(command_speed_mps)
                    : 0.0f;
        }
    }

    if ((s_mode4_large_turn_state == MODE4_LARGE_TURN_BRAKE) ||
        (s_mode4_large_turn_state == MODE4_LARGE_TURN_PIVOT))
    {
        g_car_base_speed_command = 0.0f;
    }
}

static void mode4_speed_plan_update(void)
{
    float command;
    float delta;

    s_mode4_previous_base_target = g_car_base_speed_target;
    if (s_mode4_large_turn_state != MODE4_LARGE_TURN_BRAKE)
    {
        g_car_base_speed_target = g_car_base_speed_command;
        return;
    }

    command = mode4_clampf(
        mode4_large_turn_brake_target,
        0.0f,
        fmaxf(0.0f,
              mode4_large_turn_brake_speed - mode4_brake_target_margin));
    delta = mode4_clampf(command - g_car_base_speed_target,
                         -mode4_speed_decel_step,
                          mode4_speed_decel_step);
    g_car_base_speed_target += delta;
}

static float mode4_yaw_control(void)
{
    float yaw_error = mode4_yaw_error_deg();
    float desired_rate;

    if (((s_mode4_large_turn_state == MODE4_LARGE_TURN_BRAKE) ||
         (s_mode4_large_turn_state == MODE4_LARGE_TURN_PIVOT)) &&
        (s_mode4_large_turn_direction != 0))
    {
        yaw_error = (float)s_mode4_large_turn_direction * fabsf(yaw_error);
    }

    s_mode4_yaw_pid.kp = mode4_yaw_kp;
    s_mode4_yaw_pid.ki = 0.0f;
    s_mode4_yaw_pid.kd = mode4_yaw_kd;
    PID_SetOutputLimits(&s_mode4_yaw_pid,
                        -mode4_yaw_rate_limit_dps,
                         mode4_yaw_rate_limit_dps);
    desired_rate = PID_Update(&s_mode4_yaw_pid, yaw_error, 0.0f);
    if (s_mode4_large_turn_state == MODE4_LARGE_TURN_BRAKE)
    {
        desired_rate = mode4_clampf(
            desired_rate,
            -mode4_large_turn_brake_rate_dps,
             mode4_large_turn_brake_rate_dps);
    }
    return -desired_rate;
}

static void mode4_gyroz_control(float gyroz_target)
{
    float output;
    float left_abs;
    float right_abs;
    float wheel_peak;

    s_mode4_gyroz_pid.kp = mode4_gyroz_kp;
    s_mode4_gyroz_pid.ki = mode4_gyroz_ki;
    s_mode4_gyroz_pid.kd = 0.0f;
    s_mode4_gyroz_pid.kff = mode4_gyroz_kff;
    s_mode4_gyroz_pid.i_limit = mode4_gyroz_output_limit;
    PID_SetOutputLimits(&s_mode4_gyroz_pid,
                        -mode4_gyroz_output_limit,
                         mode4_gyroz_output_limit);
    output = PID_Update(&s_mode4_gyroz_pid,
                        -gyroz_target,
                        g_car_gyroz_feedback_dps);

    Left_Target_Speed = g_car_base_speed_target +
                        mode4_gyroz_k_turn * output;
    Right_Target_Speed = g_car_base_speed_target -
                         mode4_gyroz_k_turn * output;
    left_abs = fabsf(Left_Target_Speed);
    right_abs = fabsf(Right_Target_Speed);
    wheel_peak = (left_abs > right_abs) ? left_abs : right_abs;
    if (wheel_peak > mode4_wheel_target_limit)
    {
        Left_Target_Speed *= mode4_wheel_target_limit / wheel_peak;
        Right_Target_Speed *= mode4_wheel_target_limit / wheel_peak;
    }
}

static float mode4_speed_direction(float feedback, float fallback)
{
    if (fabsf(feedback) > mode4_wheel_stop_speed)
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

static void mode4_brake_update(uint8 command_active, float gyroz_target)
{
    if ((command_active != 0U) ||
        (fabsf(gyroz_target) >= mode4_gyroz_stop_target_dps))
    {
        s_mode4_speed_brake_active = 0U;
        return;
    }
    if ((s_mode4_speed_brake_active == 0U) &&
        ((fabsf(g_car_speed_left_filtered) > mode4_wheel_stop_speed) ||
         (fabsf(g_car_speed_right_filtered) > mode4_wheel_stop_speed)))
    {
        s_mode4_left_brake_direction = mode4_speed_direction(
            g_car_speed_left_filtered, s_mode4_previous_base_target);
        s_mode4_right_brake_direction = mode4_speed_direction(
            g_car_speed_right_filtered, s_mode4_previous_base_target);
        s_mode4_speed_brake_active = 1U;
    }
}

static float mode4_brake_feedforward(float feedback, float direction)
{
    float speed_abs = fabsf(feedback);
    float ff_deadband = fmaxf(mode4_speed_ff_deadband,
                              mode4_wheel_stop_speed + 1.0f);
    float scale;

    if ((s_mode4_speed_brake_active == 0U) || (direction == 0.0f) ||
        (feedback * direction <= 0.0f) ||
        (speed_abs <= mode4_wheel_stop_speed))
    {
        return 0.0f;
    }
    scale = (speed_abs >= ff_deadband)
                ? 1.0f
                : (speed_abs - mode4_wheel_stop_speed) /
                      (ff_deadband - mode4_wheel_stop_speed);
    return -direction * mode4_speed_brake_static * scale;
}

static float mode4_large_turn_brake_feedforward(void)
{
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    float speed_abs = fabsf(body_speed);
    float scale;

    if ((s_mode4_large_turn_state != MODE4_LARGE_TURN_BRAKE) ||
        (speed_abs <= mode4_large_turn_brake_speed))
    {
        return 0.0f;
    }
    scale = mode4_clampf(
        (speed_abs - mode4_large_turn_brake_speed) /
            fmaxf(mode4_brake_ff_fade_span, 1.0f),
        0.0f, 1.0f);
    return -mode4_speed_direction(body_speed, 0.0f) *
           mode4_large_turn_brake_ff * scale;
}

static int16 mode4_speed_pid_update(pid_t *pid,
                                    float target,
                                    float feedback,
                                    float kp,
                                    float ki,
                                    float kd,
                                    float brake_ff)
{
    float output;
    float ff_deadband = fmaxf(mode4_speed_ff_deadband,
                              mode4_wheel_stop_speed + 1.0f);
    float ff_transition = fmaxf(mode4_speed_ff_transition, ff_deadband);

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = 0.0f;
    pid->i_limit = mode4_speed_i_limit;
    PID_SetStaticFeedforward(pid, mode4_speed_ff_static);
    PID_SetFeedforwardTransition(pid, ff_deadband, ff_transition);
    PID_SetExternalFeedforward(pid, 0.0f);
    PID_SetIncrementLimit(pid, mode4_speed_increment_limit);
    PID_SetOutputLimits(pid, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    output = PID_UpdateIncremental(pid, target, feedback) + brake_ff;
    return (int16)mode4_clampf(output,
                              -(float)MOTOR_PWM_MAX,
                               (float)MOTOR_PWM_MAX);
}

void car_mode4_init(void)
{
    PID_Init(&s_mode4_left_speed_pid, mode4_speed_left_kp, mode4_speed_left_ki,
             mode4_speed_left_kd, 0.0f, mode4_speed_i_limit);
    PID_Init(&s_mode4_right_speed_pid, mode4_speed_right_kp, mode4_speed_right_ki,
             mode4_speed_right_kd, 0.0f, mode4_speed_i_limit);
    PID_Init(&s_mode4_yaw_pid, mode4_yaw_kp, 0.0f,
             mode4_yaw_kd, 0.0f, 0.0f);
    PID_Init(&s_mode4_gyroz_pid, mode4_gyroz_kp, mode4_gyroz_ki, 0.0f,
             mode4_gyroz_kff, mode4_gyroz_output_limit);
    car_mode4_reset();
}

void car_mode4_reset(void)
{
    PID_Reset(&s_mode4_left_speed_pid);
    PID_Reset(&s_mode4_right_speed_pid);
    PID_Reset(&s_mode4_yaw_pid);
    PID_Reset(&s_mode4_gyroz_pid);
    s_mode4_yaw_target_deg = g_car_yaw_feedback_deg;
    s_mode4_gyroz_target_dps = 0.0f;
    s_mode4_previous_base_target = 0.0f;
    s_mode4_speed_brake_active = 0U;
    s_mode4_left_brake_direction = 0.0f;
    s_mode4_right_brake_direction = 0.0f;
    mode4_large_turn_reset();
    g_car_base_speed_command = 0.0f;
    g_car_base_speed_target = 0.0f;
    Left_Target_Speed = 0.0f;
    Right_Target_Speed = 0.0f;
    g_car_speed_left_motor_output = 0.0f;
    g_car_speed_right_motor_output = 0.0f;
}

static void mode4_control_update(uint8 command_active,
                                 float command_speed_mps)
{
    float gyroz_target;
    float left_brake_ff;
    float right_brake_ff;
    float large_turn_brake_ff;
    int16 left_output;
    int16 right_output;

    /* Temporary pure-yaw test: keep the large-turn state machine inactive. */
    mode4_large_turn_update(0U, command_speed_mps);
    mode4_speed_plan_update();
    gyroz_target = mode4_yaw_control();
    s_mode4_gyroz_target_dps = -gyroz_target;
    mode4_gyroz_control(gyroz_target);

    if (s_mode4_large_turn_state == MODE4_LARGE_TURN_BRAKE)
    {
        s_mode4_speed_brake_active = 0U;
    }
    else
    {
        mode4_brake_update(command_active, gyroz_target);
    }

    large_turn_brake_ff = mode4_large_turn_brake_feedforward();
    left_brake_ff = (s_mode4_large_turn_state == MODE4_LARGE_TURN_BRAKE)
                        ? large_turn_brake_ff
                        : mode4_brake_feedforward(
                              g_car_speed_left_filtered,
                              s_mode4_left_brake_direction);
    right_brake_ff = (s_mode4_large_turn_state == MODE4_LARGE_TURN_BRAKE)
                         ? large_turn_brake_ff
                         : mode4_brake_feedforward(
                               g_car_speed_right_filtered,
                               s_mode4_right_brake_direction);

    left_output = mode4_speed_pid_update(
        &s_mode4_left_speed_pid,
        Left_Target_Speed, g_car_speed_left_filtered,
        mode4_speed_left_kp, mode4_speed_left_ki, mode4_speed_left_kd,
        left_brake_ff);
    right_output = mode4_speed_pid_update(
        &s_mode4_right_speed_pid,
        Right_Target_Speed, g_car_speed_right_filtered,
        mode4_speed_right_kp, mode4_speed_right_ki, mode4_speed_right_kd,
        right_brake_ff);
    g_car_speed_left_motor_output = (float)left_output;
    g_car_speed_right_motor_output = (float)right_output;
    motor_left_set_speed(left_output);
    motor_right_set_speed(right_output);
}

void car_mode4_update_100HZ(uint32 now_ms)
{
    float speed_mps;
    uint8 command_active;

    (void)now_ms;
    command_active = mode4_plan_command_update(&speed_mps);
    mode4_control_update(command_active, speed_mps);
}

void car_mode4_get_diag(car_drive_diag_t *diag)
{
    diag->yaw_target_deg = s_mode4_yaw_target_deg;
    diag->yaw_error_deg = mode4_yaw_error_deg();
    diag->gyroz_target_dps = s_mode4_gyroz_target_dps;
    diag->gyroz_output = s_mode4_gyroz_pid.output;
    diag->yaw_p_term = s_mode4_yaw_pid.p_term;
    diag->yaw_d_term = s_mode4_yaw_pid.d_term;
    diag->yaw_output = s_mode4_yaw_pid.output;
    diag->gyroz_p_term = s_mode4_gyroz_pid.p_term;
    diag->gyroz_i_term = s_mode4_gyroz_pid.i_term;
    diag->gyroz_ff_term = s_mode4_gyroz_pid.ff_term;
    diag->left_speed_p_term = s_mode4_left_speed_pid.p_term;
    diag->left_speed_i_term = s_mode4_left_speed_pid.i_term;
    diag->left_speed_d_term = s_mode4_left_speed_pid.d_term;
    diag->left_speed_ff_term = s_mode4_left_speed_pid.ff_term;
    diag->left_brake_ff = g_car_speed_left_motor_output -
                          s_mode4_left_speed_pid.output;
    diag->right_speed_p_term = s_mode4_right_speed_pid.p_term;
    diag->right_speed_i_term = s_mode4_right_speed_pid.i_term;
    diag->right_speed_d_term = s_mode4_right_speed_pid.d_term;
    diag->right_speed_ff_term = s_mode4_right_speed_pid.ff_term;
    diag->right_brake_ff = g_car_speed_right_motor_output -
                           s_mode4_right_speed_pid.output;
    diag->large_turn_target_yaw_deg = s_mode4_large_turn_target_yaw_deg;
    diag->large_turn_target_speed_mps = 0.0f;
    diag->large_turn_trigger_cycles = s_mode4_large_turn_trigger_cycles;
    diag->large_turn_finish_cycles = 0U;
    diag->large_turn_elapsed_cycles = s_mode4_large_turn_elapsed_cycles;
    diag->large_turn_direction = s_mode4_large_turn_direction;
    diag->large_turn_state = s_mode4_large_turn_state;
    diag->large_turn_rearm_required = s_mode4_large_turn_rearm_required;
    diag->speed_brake_active = s_mode4_speed_brake_active;
}
