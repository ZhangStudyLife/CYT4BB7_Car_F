#include "car_mode.h"
#include "pid_core.h"
#include <math.h>

#define MODE2_TARGET_SPEED_MPS           (2.3f)
#define MODE2_INPUT_DEADZONE             (100.0f)
#define MODE2_ALIGNMENT_STOP_DEG         (90.0f)
#define MODE2_WHEEL_TARGET_LIMIT         (1000.0f)
#define MODE2_SPEED_I_LIMIT              (2000.0f)
#define MODE2_SPEED_FF_DEADBAND          (10.0f)
#define MODE2_SPEED_FF_TRANSITION        (100.0f)
#define MODE2_SPEED_INCREMENT_LIMIT      (6000.0f)
#define MODE2_SPEED_BRAKE_STATIC         (800.0f)
#define MODE2_SPEED_DECEL_STEP           (600.0f)
#define MODE2_GYROZ_OUTPUT_LIMIT         (600.0f)
#define MODE2_YAW_RATE_LIMIT_DPS         (1000.0f)
#define MODE2_GYROZ_STOP_TARGET_DPS      (2.0f)
#define MODE2_WHEEL_STOP_SPEED           (8.0f)
#define MODE2_DEG_TO_RAD                 (0.017453292519943295f)
#define MODE2_RAD_TO_DEG                 (57.29577951308232f)

#define MODE2_LARGE_TURN_NORMAL          (0U)
#define MODE2_LARGE_TURN_BRAKE           (1U)
#define MODE2_LARGE_TURN_PIVOT           (2U)
#define MODE2_LARGE_TURN_EXIT            (3U)
#define MODE2_LARGE_TURN_BRAKE_SPEED     (80.0f)
#define MODE2_LARGE_TURN_BRAKE_TARGET    (20.0f)
#define MODE2_LARGE_TURN_BRAKE_FF        (1200.0f)
#define MODE2_LARGE_TURN_BRAKE_RATE_DPS  (300.0f)
#define MODE2_LARGE_TURN_ENTER_DEG       (90.0f)
#define MODE2_LARGE_TURN_PIVOT_EXIT_DEG  (35.0f)
#define MODE2_LARGE_TURN_EXIT_START_DEG  (35.0f)
#define MODE2_LARGE_TURN_FINISH_DEG      (3.0f)
#define MODE2_LARGE_TURN_FINISH_CYCLES   (2U)
#define MODE2_LARGE_TURN_TIMEOUT_CYCLES  (500U)
#define MODE2_EXIT_COMMAND_MATCH_DEG     (80.0f)
#define MODE2_BRAKE_TARGET_MARGIN        (5.0f)
#define MODE2_BRAKE_FF_FADE_SPAN         (40.0f)

volatile float car_speed_left_kp = 10.80f;
volatile float car_speed_left_ki = 0.52f;
volatile float car_speed_left_kd = 1.00f;
volatile float car_speed_right_kp = 10.80f;
volatile float car_speed_right_ki = 0.52f;
volatile float car_speed_right_kd = 1.00f;
volatile float car_speed_ff_static = 800.0f;
volatile float car_gyroz_kff = 0.12f;
volatile float car_gyroz_kp = 1.27f;
volatile float car_gyroz_ki = 0.022f;
volatile float car_gyroz_k_turn = 1.0f;
volatile float car_yaw_kp = 6.00f;
volatile float car_yaw_kd = 0.50f;

volatile float g_car_base_speed_command = 0.0f;
volatile float g_car_base_speed_target = 0.0f;
volatile float g_car_speed_left_motor_output = 0.0f;
volatile float g_car_speed_right_motor_output = 0.0f;
volatile float Left_Target_Speed = 0.0f;
volatile float Right_Target_Speed = 0.0f;

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
static uint16 s_mode2_large_turn_finish_cycles;
static uint16 s_mode2_large_turn_elapsed_cycles;
static float s_mode2_large_turn_target_yaw_deg;

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
    s_mode2_large_turn_finish_cycles = 0U;
    s_mode2_large_turn_elapsed_cycles = 0U;
    s_mode2_large_turn_target_yaw_deg = 0.0f;
}

static void mode2_large_turn_set(uint8 state)
{
    s_mode2_large_turn_state = state;
    if (state == MODE2_LARGE_TURN_NORMAL)
    {
        s_mode2_large_turn_direction = 0;
        s_mode2_large_turn_finish_cycles = 0U;
        s_mode2_large_turn_elapsed_cycles = 0U;
        s_mode2_large_turn_target_yaw_deg = 0.0f;
    }
}

static uint8 mode2_exit_command_matches(void)
{
    float ch0 = g_air_std_ch0;
    float ch1 = g_air_std_ch1;
    float heading_deg;

    if (sqrtf(ch0 * ch0 + ch1 * ch1) < MODE2_INPUT_DEADZONE)
    {
        return 0U;
    }
    heading_deg = atan2f(ch0, ch1) * MODE2_RAD_TO_DEG;
    return (fabsf(mode2_wrap_deg(
                heading_deg - s_mode2_large_turn_target_yaw_deg)) <=
            MODE2_EXIT_COMMAND_MATCH_DEG) ? 1U : 0U;
}

static void mode2_apply_latched_command(void)
{
    float yaw_error = mode2_wrap_deg(
        s_mode2_large_turn_target_yaw_deg - g_car_yaw_feedback_deg);
    float alignment = 0.0f;

    if ((s_mode2_large_turn_state == MODE2_LARGE_TURN_EXIT) &&
        (mode2_exit_command_matches() != 0U) &&
        (fabsf(yaw_error) < MODE2_LARGE_TURN_EXIT_START_DEG))
    {
        alignment = (MODE2_LARGE_TURN_EXIT_START_DEG - fabsf(yaw_error)) /
                    (MODE2_LARGE_TURN_EXIT_START_DEG -
                     MODE2_LARGE_TURN_FINISH_DEG);
        alignment = mode2_clampf(alignment, 0.0f, 1.0f);
    }

    s_mode2_yaw_target_deg = s_mode2_large_turn_target_yaw_deg;
    g_car_base_speed_command =
        car_speed_mps_to_encoder_cnt(MODE2_TARGET_SPEED_MPS) * alignment;
}

static uint8 mode2_world_command_update(void)
{
    float ch0 = g_air_std_ch0;
    float ch1 = g_air_std_ch1;
    float magnitude = sqrtf(ch0 * ch0 + ch1 * ch1);
    float yaw_error;

    if (s_mode2_large_turn_state != MODE2_LARGE_TURN_NORMAL)
    {
        if ((s_mode2_large_turn_state == MODE2_LARGE_TURN_EXIT) &&
            (magnitude >= MODE2_INPUT_DEADZONE) &&
            (mode2_exit_command_matches() == 0U))
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_NORMAL);
        }
        else
        {
            mode2_apply_latched_command();
            return 1U;
        }
    }

    if (s_mode2_large_turn_rearm_required != 0U)
    {
        if (magnitude < MODE2_INPUT_DEADZONE)
        {
            s_mode2_large_turn_rearm_required = 0U;
        }
        else
        {
            g_car_base_speed_command = 0.0f;
            return 0U;
        }
    }

    if (magnitude < MODE2_INPUT_DEADZONE)
    {
        g_car_base_speed_command = 0.0f;
        return 0U;
    }

    s_mode2_yaw_target_deg = atan2f(ch0, ch1) * MODE2_RAD_TO_DEG;
    yaw_error = mode2_yaw_error_deg();
    g_car_base_speed_command =
        (fabsf(yaw_error) >= MODE2_ALIGNMENT_STOP_DEG)
            ? 0.0f
            : car_speed_mps_to_encoder_cnt(MODE2_TARGET_SPEED_MPS) *
                  cosf(yaw_error * MODE2_DEG_TO_RAD);
    return 1U;
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

static void mode2_large_turn_update(uint8 command_active)
{
    float yaw_error = mode2_yaw_error_deg();
    float yaw_error_abs = fabsf(yaw_error);
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    uint8 translation_slow =
        (fabsf(body_speed) <= MODE2_LARGE_TURN_BRAKE_SPEED) ? 1U : 0U;

    if (s_mode2_large_turn_state != MODE2_LARGE_TURN_NORMAL)
    {
        if (++s_mode2_large_turn_elapsed_cycles >=
            MODE2_LARGE_TURN_TIMEOUT_CYCLES)
        {
            mode2_large_turn_timeout();
            return;
        }
    }

    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_NORMAL)
    {
        if ((command_active != 0U) &&
            (yaw_error_abs >= MODE2_LARGE_TURN_ENTER_DEG))
        {
            s_mode2_large_turn_direction = (yaw_error >= 0.0f) ? 1 : -1;
            s_mode2_large_turn_target_yaw_deg = s_mode2_yaw_target_deg;
            s_mode2_large_turn_elapsed_cycles = 0U;
            mode2_large_turn_set((translation_slow != 0U)
                                     ? MODE2_LARGE_TURN_PIVOT
                                     : MODE2_LARGE_TURN_BRAKE);
        }
    }
    else if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        if (yaw_error_abs <= MODE2_LARGE_TURN_PIVOT_EXIT_DEG)
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
        if (yaw_error_abs <= MODE2_LARGE_TURN_PIVOT_EXIT_DEG)
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_EXIT);
        }
    }
    else if (yaw_error_abs <= MODE2_LARGE_TURN_FINISH_DEG)
    {
        if (++s_mode2_large_turn_finish_cycles >=
            MODE2_LARGE_TURN_FINISH_CYCLES)
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
        MODE2_LARGE_TURN_BRAKE_TARGET,
        0.0f,
        MODE2_LARGE_TURN_BRAKE_SPEED - MODE2_BRAKE_TARGET_MARGIN);
    delta = mode2_clampf(command - g_car_base_speed_target,
                         -MODE2_SPEED_DECEL_STEP,
                          MODE2_SPEED_DECEL_STEP);
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

    s_mode2_yaw_pid.kp = car_yaw_kp;
    s_mode2_yaw_pid.ki = 0.0f;
    s_mode2_yaw_pid.kd = car_yaw_kd;
    PID_SetOutputLimits(&s_mode2_yaw_pid,
                        -MODE2_YAW_RATE_LIMIT_DPS,
                         MODE2_YAW_RATE_LIMIT_DPS);
    desired_rate = PID_Update(&s_mode2_yaw_pid, yaw_error, 0.0f);
    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        desired_rate = mode2_clampf(
            desired_rate,
            -MODE2_LARGE_TURN_BRAKE_RATE_DPS,
             MODE2_LARGE_TURN_BRAKE_RATE_DPS);
    }
    return -desired_rate;
}

static void mode2_gyroz_control(float gyroz_target)
{
    float output;
    float left_abs;
    float right_abs;
    float wheel_peak;

    s_mode2_gyroz_pid.kp = car_gyroz_kp;
    s_mode2_gyroz_pid.ki = car_gyroz_ki;
    s_mode2_gyroz_pid.kd = 0.0f;
    s_mode2_gyroz_pid.kff = car_gyroz_kff;
    s_mode2_gyroz_pid.i_limit = MODE2_GYROZ_OUTPUT_LIMIT;
    PID_SetOutputLimits(&s_mode2_gyroz_pid,
                        -MODE2_GYROZ_OUTPUT_LIMIT,
                         MODE2_GYROZ_OUTPUT_LIMIT);
    output = PID_Update(&s_mode2_gyroz_pid,
                        -gyroz_target,
                        g_car_gyroz_feedback_dps);

    Left_Target_Speed = g_car_base_speed_target +
                        car_gyroz_k_turn * output;
    Right_Target_Speed = g_car_base_speed_target -
                         car_gyroz_k_turn * output;
    left_abs = fabsf(Left_Target_Speed);
    right_abs = fabsf(Right_Target_Speed);
    wheel_peak = (left_abs > right_abs) ? left_abs : right_abs;
    if (wheel_peak > MODE2_WHEEL_TARGET_LIMIT)
    {
        Left_Target_Speed *= MODE2_WHEEL_TARGET_LIMIT / wheel_peak;
        Right_Target_Speed *= MODE2_WHEEL_TARGET_LIMIT / wheel_peak;
    }
}

static float mode2_speed_direction(float feedback, float fallback)
{
    if (fabsf(feedback) > MODE2_WHEEL_STOP_SPEED)
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
        (fabsf(gyroz_target) >= MODE2_GYROZ_STOP_TARGET_DPS))
    {
        s_mode2_speed_brake_active = 0U;
        return;
    }
    if ((s_mode2_speed_brake_active == 0U) &&
        ((fabsf(g_car_speed_left_filtered) > MODE2_WHEEL_STOP_SPEED) ||
         (fabsf(g_car_speed_right_filtered) > MODE2_WHEEL_STOP_SPEED)))
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
    float scale;

    if ((s_mode2_speed_brake_active == 0U) || (direction == 0.0f) ||
        (feedback * direction <= 0.0f) ||
        (speed_abs <= MODE2_WHEEL_STOP_SPEED))
    {
        return 0.0f;
    }
    scale = (speed_abs >= MODE2_SPEED_FF_DEADBAND)
                ? 1.0f
                : (speed_abs - MODE2_WHEEL_STOP_SPEED) /
                      (MODE2_SPEED_FF_DEADBAND - MODE2_WHEEL_STOP_SPEED);
    return -direction * MODE2_SPEED_BRAKE_STATIC * scale;
}

static float mode2_large_turn_brake_feedforward(void)
{
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    float speed_abs = fabsf(body_speed);
    float scale;

    if ((s_mode2_large_turn_state != MODE2_LARGE_TURN_BRAKE) ||
        (speed_abs <= MODE2_LARGE_TURN_BRAKE_SPEED))
    {
        return 0.0f;
    }
    scale = mode2_clampf(
        (speed_abs - MODE2_LARGE_TURN_BRAKE_SPEED) /
            MODE2_BRAKE_FF_FADE_SPAN,
        0.0f, 1.0f);
    return -mode2_speed_direction(body_speed, 0.0f) *
           MODE2_LARGE_TURN_BRAKE_FF * scale;
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

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = 0.0f;
    pid->i_limit = MODE2_SPEED_I_LIMIT;
    PID_SetStaticFeedforward(pid, car_speed_ff_static);
    PID_SetFeedforwardTransition(pid,
                                 MODE2_SPEED_FF_DEADBAND,
                                 MODE2_SPEED_FF_TRANSITION);
    PID_SetExternalFeedforward(pid, 0.0f);
    PID_SetIncrementLimit(pid, MODE2_SPEED_INCREMENT_LIMIT);
    PID_SetOutputLimits(pid, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    output = PID_UpdateIncremental(pid, target, feedback) + brake_ff;
    return (int16)mode2_clampf(output,
                              -(float)MOTOR_PWM_MAX,
                               (float)MOTOR_PWM_MAX);
}

void car_mode2_init(void)
{
    PID_Init(&s_mode2_left_speed_pid, car_speed_left_kp, car_speed_left_ki,
             car_speed_left_kd, 0.0f, MODE2_SPEED_I_LIMIT);
    PID_Init(&s_mode2_right_speed_pid, car_speed_right_kp, car_speed_right_ki,
             car_speed_right_kd, 0.0f, MODE2_SPEED_I_LIMIT);
    PID_Init(&s_mode2_yaw_pid, car_yaw_kp, 0.0f,
             car_yaw_kd, 0.0f, 0.0f);
    PID_Init(&s_mode2_gyroz_pid, car_gyroz_kp, car_gyroz_ki, 0.0f,
             car_gyroz_kff, MODE2_GYROZ_OUTPUT_LIMIT);
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

void car_mode2_update_100HZ(uint32 now_ms)
{
    uint8 command_active;
    float gyroz_target;
    float left_brake_ff;
    float right_brake_ff;
    float large_turn_brake_ff;
    int16 left_output;
    int16 right_output;

    (void)now_ms;
    command_active = mode2_world_command_update();
    mode2_large_turn_update(command_active);
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
        car_speed_left_kp, car_speed_left_ki, car_speed_left_kd,
        left_brake_ff);
    right_output = mode2_speed_pid_update(
        &s_mode2_right_speed_pid,
        Right_Target_Speed, g_car_speed_right_filtered,
        car_speed_right_kp, car_speed_right_ki, car_speed_right_kd,
        right_brake_ff);
    g_car_speed_left_motor_output = (float)left_output;
    g_car_speed_right_motor_output = (float)right_output;
    motor_left_set_speed(left_output);
    motor_right_set_speed(right_output);
}

void car_mode2_get_diag(car_mode2_diag_t *diag)
{
    diag->yaw_target_deg = s_mode2_yaw_target_deg;
    diag->yaw_error_deg = mode2_yaw_error_deg();
    diag->gyroz_target_dps = s_mode2_gyroz_target_dps;
    diag->gyroz_output = s_mode2_gyroz_pid.output;
    diag->large_turn_state = s_mode2_large_turn_state;
    diag->large_turn_rearm_required = s_mode2_large_turn_rearm_required;
    diag->speed_brake_active = s_mode2_speed_brake_active;
}
