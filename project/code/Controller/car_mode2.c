#include "car_mode.h"
#include "pid_core.h"
#include <math.h>

#define MODE2_TARGET_SPEED                 (300.0f) /* Mode2固定轮速目标幅值 */
#define MODE2_SPEED_I_LIMIT               (2000.0f) /* 左右轮速度环积分限幅 */
#define MODE2_SPEED_PLAN_MIN_STEP          (1.0f) /* 速度规划最小单周期变化量 */
#define MODE2_ALIGNMENT_STOP_DEG           (90.0f) /* 超过该航向误差时停止平移 */
#define MODE2_WHEEL_TARGET_ABS_LIMIT       (1000.0f) /* 单轮目标绝对值上限 */
#define MODE2_DEG_TO_RAD                   (0.017453292519943295f) /* 角度转弧度 */
#define MODE2_RAD_TO_DEG                   (57.29577951308232f) /* 弧度转角度 */
#define MODE2_GYROZ_EQUIVALENT_SCALE       (28.1448005f) /* 保留原角速度等效比例 */
#define MODE2_GYROZ_OUTPUT_LIMIT           (600.0f) /* 角速度环输出限幅 */
#define MODE2_YAW_STOP_ANGLE_DEG           (2.0f) /* 静止判定航向误差 */
#define MODE2_YAW_WAKE_ANGLE_DEG           (1.0f) /* 静止状态唤醒航向误差 */
#define MODE2_GYROZ_STOP_TARGET_DPS        (2.0f) /* 静止判定目标角速度 */
#define MODE2_GYROZ_WAKE_TARGET_DPS        (4.0f) /* 静止状态唤醒目标角速度 */
#define MODE2_YAW_STOP_RATE_DPS            (9.0f) /* 静止判定实际角速度 */
#define MODE2_WHEEL_STOP_SPEED             (8.0f) /* 车轮静止判定速度 */
#define MODE2_LARGE_TURN_NORMAL            (0U) /* 正常行驶阶段 */
#define MODE2_LARGE_TURN_BRAKE             (1U) /* 大角度转向减速阶段 */
#define MODE2_LARGE_TURN_PIVOT             (2U) /* 原地转向阶段 */
#define MODE2_LARGE_TURN_EXIT              (3U) /* 转向完成恢复阶段 */
#define MODE2_LARGE_TURN_TRIGGER_CYCLES    (1U) /* 大角度转向触发确认周期 */
#define MODE2_LARGE_TURN_FINISH_CYCLES     (2U) /* 转向完成确认周期 */
#define MODE2_LARGE_TURN_TIMEOUT_CYCLES    (500U) /* 单次大角度转向超时周期 */
#define MODE2_EXIT_COMMAND_MATCH_DEG       (80.0f) /* EXIT阶段有效指令夹角 */
#define MODE2_BRAKE_TARGET_MARGIN          (5.0f) /* BRAKE目标与切换速度最小差值 */
#define MODE2_BRAKE_FF_FADE_SPAN           (40.0f) /* BRAKE前馈衰减速度跨度 */
#define MODE2_EXIT_RECOVERY_MIN_SPAN_DEG   (1.0f) /* EXIT恢复最小角度跨度 */
#define MODE2_ACCEL_TURN_FULL_RATIO        (0.20f) /* 完整保留加速前馈的转向比例 */
#define MODE2_ACCEL_TURN_DISABLE_RATIO     (0.50f) /* 禁用加速前馈的转向比例 */

volatile float car_speed_left_kp = 10.80f;
volatile float car_speed_left_ki = 0.52f;
volatile float car_speed_left_kd = 1.00f;
volatile float car_speed_right_kp = 10.80f;
volatile float car_speed_right_ki = 0.52f;
volatile float car_speed_right_kd = 1.00f;
volatile float car_speed_ff_slope = 0.00f;
volatile float car_speed_ff_static = 800.0f;
volatile float car_speed_ff_deadband = 10.0f;
volatile float car_speed_ff_transition = 100.0f;
volatile float car_speed_brake_static = 800.0f;
volatile float car_speed_delta_output_limit = 6000.0f;
volatile float car_speed_accel_kff = 10.0f;
volatile float car_speed_accel_step_limit = 40.0f;
volatile float car_speed_decel_step_limit = 600.0f;
volatile float car_speed_accel_ff_limit = 800.0f;
volatile float car_gyroz_kff = 0.12f;
volatile float car_gyroz_kp = 1.27f;
volatile float car_gyroz_ki = 0.022f;
volatile float car_gyroz_k_turn = 1.0f;
volatile float car_yaw_kp = 6.00f;
volatile float car_yaw_kd = 0.50f;
volatile float car_yaw_rate_limit_dps = 1000.0f;
volatile float car_yaw_control_mode = 1.0f;
volatile float car_large_turn_brake_speed = 80.0f;
volatile float car_large_turn_brake_target_speed = 20.0f;
volatile float car_large_turn_brake_ff = 1200.0f;
volatile float car_large_turn_brake_rate_limit_dps = 300.0f;
volatile float car_large_turn_enter_angle_deg = 90.0f;
volatile float car_large_turn_pivot_exit_angle_deg = 35.0f;
volatile float car_large_turn_exit_speed_start_angle_deg = 35.0f;
volatile float car_large_turn_finish_angle_deg = 3.0f;
volatile float car_large_turn_brake_stable_cycles = 1.0f;

volatile float g_car_base_speed_command = 0.0f;
volatile float g_car_base_speed_target = 0.0f;
volatile float g_car_base_speed_delta = 0.0f;
volatile float g_car_speed_accel_ff = 0.0f;
volatile float g_car_speed_left_brake_ff = 0.0f;
volatile float g_car_speed_right_brake_ff = 0.0f;
volatile float g_car_speed_left_motor_output = 0.0f;
volatile float g_car_speed_right_motor_output = 0.0f;
volatile float g_car_speed_brake_active = 0.0f;
volatile float Left_Target_Speed = 0.0f;
volatile float Right_Target_Speed = 0.0f;
volatile float g_car_gyroz_target_dps = 0.0f;
volatile float g_car_gyroz_feedback_equivalent = 0.0f;
volatile float g_car_gyroz_error = 0.0f;
volatile float g_car_gyroz_ff_term = 0.0f;
volatile float g_car_gyroz_p_term = 0.0f;
volatile float g_car_gyroz_i_term = 0.0f;
volatile float g_car_gyroz_output = 0.0f;
volatile float g_car_yaw_target_deg = 0.0f;
volatile float g_car_yaw_p_term = 0.0f;
volatile float g_car_yaw_d_term = 0.0f;
volatile float g_car_world_velocity_x_command = 0.0f;
volatile float g_car_world_velocity_y_command = 0.0f;
volatile float g_car_world_speed_magnitude = 0.0f;
volatile float g_car_world_speed_limit = 0.0f;
volatile float g_car_world_heading_target_deg = 0.0f;
volatile float g_car_world_heading_error_deg = 0.0f;
volatile float g_car_world_alignment_scale = 0.0f;
volatile float g_car_world_body_speed_feedback = 0.0f;
volatile float g_car_world_reverse_active = 0.0f;
volatile float g_car_large_turn_state = 0.0f;

static pid_t s_mode2_left_speed_pid;
static pid_t s_mode2_right_speed_pid;
static pid_t s_mode2_yaw_pid;
static pid_t s_mode2_gyroz_pid;
static uint8 s_mode2_world_command_active;
static uint8 s_mode2_input_confirm_cycles;
static uint8 s_mode2_yaw_stopped;
static uint8 s_mode2_speed_brake_active;
static float s_mode2_left_brake_direction;
static float s_mode2_right_brake_direction;
static uint8 s_mode2_large_turn_state;
static uint8 s_mode2_large_turn_latched;
static uint8 s_mode2_large_turn_rearm_required;
static int8 s_mode2_large_turn_direction;
static int8 s_mode2_large_turn_trigger_direction;
static uint16 s_mode2_large_turn_brake_cycles;
static uint16 s_mode2_large_turn_trigger_cycles;
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
    return mode2_wrap_deg(g_car_yaw_target_deg - g_car_yaw_feedback_deg);
}

static void mode2_large_turn_reset(void)
{
    s_mode2_large_turn_state = MODE2_LARGE_TURN_NORMAL;
    s_mode2_large_turn_latched = 0U;
    s_mode2_large_turn_rearm_required = 0U;
    s_mode2_large_turn_direction = 0;
    s_mode2_large_turn_trigger_direction = 0;
    s_mode2_large_turn_brake_cycles = 0U;
    s_mode2_large_turn_trigger_cycles = 0U;
    s_mode2_large_turn_finish_cycles = 0U;
    s_mode2_large_turn_elapsed_cycles = 0U;
    s_mode2_large_turn_target_yaw_deg = 0.0f;
    g_car_large_turn_state = (float)MODE2_LARGE_TURN_NORMAL;
}

static void mode2_large_turn_set(uint8 state)
{
    if (state == s_mode2_large_turn_state)
    {
        return;
    }
    s_mode2_large_turn_state = state;
    s_mode2_large_turn_brake_cycles = 0U;
    g_car_large_turn_state = (float)state;
    if (state == MODE2_LARGE_TURN_NORMAL)
    {
        s_mode2_large_turn_latched = 0U;
        s_mode2_large_turn_direction = 0;
        s_mode2_large_turn_finish_cycles = 0U;
        s_mode2_large_turn_elapsed_cycles = 0U;
        s_mode2_large_turn_target_yaw_deg = 0.0f;
    }
}

static void mode2_clear_diagnostics(void)
{
    g_car_base_speed_command = 0.0f;
    g_car_base_speed_target = 0.0f;
    g_car_base_speed_delta = 0.0f;
    g_car_speed_accel_ff = 0.0f;
    g_car_speed_left_brake_ff = 0.0f;
    g_car_speed_right_brake_ff = 0.0f;
    g_car_speed_left_motor_output = 0.0f;
    g_car_speed_right_motor_output = 0.0f;
    g_car_speed_brake_active = 0.0f;
    Left_Target_Speed = 0.0f;
    Right_Target_Speed = 0.0f;
    g_car_gyroz_target_dps = 0.0f;
    g_car_gyroz_feedback_equivalent = 0.0f;
    g_car_gyroz_error = 0.0f;
    g_car_gyroz_ff_term = 0.0f;
    g_car_gyroz_p_term = 0.0f;
    g_car_gyroz_i_term = 0.0f;
    g_car_gyroz_output = 0.0f;
    g_car_yaw_p_term = 0.0f;
    g_car_yaw_d_term = 0.0f;
    g_car_world_velocity_x_command = 0.0f;
    g_car_world_velocity_y_command = 0.0f;
    g_car_world_speed_magnitude = 0.0f;
    g_car_world_speed_limit = 0.0f;
    g_car_world_heading_target_deg = g_car_yaw_feedback_deg;
    g_car_world_heading_error_deg = 0.0f;
    g_car_world_alignment_scale = 0.0f;
    g_car_world_body_speed_feedback = 0.0f;
    g_car_world_reverse_active = 0.0f;
}

static uint8 mode2_exit_command_matches(void)
{
    float raw_x = g_air_std_ch1;
    float raw_y = g_air_std_ch0;
    float heading_deg;

    if (sqrtf(raw_x * raw_x + raw_y * raw_y) < CAR_WORLD_INPUT_ENTER_DEADZONE)
    {
        return 0U;
    }
    heading_deg = atan2f(raw_y, raw_x) * MODE2_RAD_TO_DEG;
    return (fabsf(mode2_wrap_deg(
                heading_deg - s_mode2_large_turn_target_yaw_deg)) <=
            MODE2_EXIT_COMMAND_MATCH_DEG) ? 1U : 0U;
}

static void mode2_apply_latched_command(void)
{
    float yaw_error_deg = mode2_wrap_deg(
        s_mode2_large_turn_target_yaw_deg - g_car_yaw_feedback_deg);
    float yaw_error_abs = fabsf(yaw_error_deg);
    float recovery_start = fabsf(car_large_turn_exit_speed_start_angle_deg);
    float pivot_exit = fabsf(car_large_turn_pivot_exit_angle_deg);
    float finish_angle = fabsf(car_large_turn_finish_angle_deg);
    float alignment_scale = 0.0f;
    float recovery_span;
    float target_rad;

    if (finish_angle > pivot_exit)
    {
        finish_angle = pivot_exit;
    }
    if (recovery_start > pivot_exit)
    {
        recovery_start = pivot_exit;
    }
    if (recovery_start < finish_angle + MODE2_EXIT_RECOVERY_MIN_SPAN_DEG)
    {
        recovery_start = finish_angle + MODE2_EXIT_RECOVERY_MIN_SPAN_DEG;
    }
    if ((s_mode2_large_turn_state == MODE2_LARGE_TURN_EXIT) &&
        (mode2_exit_command_matches() != 0U) &&
        (yaw_error_abs < recovery_start))
    {
        recovery_span = recovery_start - finish_angle;
        alignment_scale = (recovery_start - yaw_error_abs) / recovery_span;
        alignment_scale = mode2_clampf(alignment_scale, 0.0f, 1.0f);
    }

    target_rad = s_mode2_large_turn_target_yaw_deg * MODE2_DEG_TO_RAD;
    s_mode2_world_command_active = 1U;
    g_car_yaw_target_deg = s_mode2_large_turn_target_yaw_deg;
    g_car_base_speed_command = MODE2_TARGET_SPEED * alignment_scale;
    g_car_world_velocity_x_command = cosf(target_rad) * MODE2_TARGET_SPEED;
    g_car_world_velocity_y_command = sinf(target_rad) * MODE2_TARGET_SPEED;
    g_car_world_speed_magnitude = MODE2_TARGET_SPEED;
    g_car_world_speed_limit = MODE2_TARGET_SPEED;
    g_car_world_heading_target_deg = s_mode2_large_turn_target_yaw_deg;
    g_car_world_heading_error_deg = yaw_error_deg;
    g_car_world_alignment_scale = alignment_scale;
}

static void mode2_world_command_update(void)
{
    float raw_x = g_air_std_ch1;
    float raw_y = g_air_std_ch0;
    float magnitude = sqrtf(raw_x * raw_x + raw_y * raw_y);
    float heading_deg;
    float heading_error_deg;
    float alignment_scale;
    float target_rad;
    uint8 input_ready = 0U;
    uint8 update_heading = 0U;

    g_car_world_speed_limit = MODE2_TARGET_SPEED;
    g_car_world_body_speed_feedback =
        0.5f * (g_car_speed_left_filtered + g_car_speed_right_filtered);

    if (s_mode2_large_turn_latched != 0U)
    {
        if ((s_mode2_large_turn_state == MODE2_LARGE_TURN_EXIT) &&
            (magnitude >= CAR_WORLD_INPUT_ENTER_DEADZONE) &&
            (mode2_exit_command_matches() == 0U))
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_NORMAL);
        }
        else
        {
            mode2_apply_latched_command();
            return;
        }
    }
    if (s_mode2_large_turn_rearm_required != 0U)
    {
        if (magnitude <= CAR_WORLD_INPUT_EXIT_DEADZONE)
        {
            s_mode2_large_turn_rearm_required = 0U;
        }
        else
        {
            s_mode2_world_command_active = 0U;
            g_car_base_speed_command = 0.0f;
            return;
        }
    }

    if (s_mode2_world_command_active != 0U)
    {
        if (magnitude > CAR_WORLD_INPUT_EXIT_DEADZONE)
        {
            input_ready = 1U;
            update_heading =
                (magnitude >= CAR_WORLD_INPUT_ENTER_DEADZONE) ? 1U : 0U;
        }
    }
    else if (magnitude >= CAR_WORLD_INPUT_ENTER_DEADZONE)
    {
        if (s_mode2_input_confirm_cycles < CAR_WORLD_INPUT_CONFIRM_CYCLES)
        {
            s_mode2_input_confirm_cycles++;
        }
        if (s_mode2_input_confirm_cycles >= CAR_WORLD_INPUT_CONFIRM_CYCLES)
        {
            input_ready = 1U;
            update_heading = 1U;
        }
    }
    else
    {
        s_mode2_input_confirm_cycles = 0U;
    }

    if (input_ready == 0U)
    {
        s_mode2_world_command_active = 0U;
        s_mode2_input_confirm_cycles = 0U;
        g_car_yaw_target_deg = g_car_yaw_feedback_deg;
        g_car_base_speed_command = 0.0f;
        g_car_world_velocity_x_command = 0.0f;
        g_car_world_velocity_y_command = 0.0f;
        g_car_world_speed_magnitude = 0.0f;
        g_car_world_heading_target_deg = g_car_yaw_target_deg;
        g_car_world_heading_error_deg = 0.0f;
        g_car_world_alignment_scale = 0.0f;
        PID_Reset(&s_mode2_yaw_pid);
        return;
    }

    if (s_mode2_world_command_active == 0U)
    {
        s_mode2_world_command_active = 1U;
        PID_Reset(&s_mode2_yaw_pid);
    }
    s_mode2_input_confirm_cycles = 0U;
    heading_deg = (update_heading != 0U)
                      ? atan2f(raw_y, raw_x) * MODE2_RAD_TO_DEG
                      : g_car_yaw_target_deg;
    heading_error_deg = mode2_wrap_deg(heading_deg - g_car_yaw_feedback_deg);
    alignment_scale = (fabsf(heading_error_deg) >= MODE2_ALIGNMENT_STOP_DEG)
                          ? 0.0f
                          : cosf(heading_error_deg * MODE2_DEG_TO_RAD);
    alignment_scale = mode2_clampf(alignment_scale, 0.0f, 1.0f);
    target_rad = heading_deg * MODE2_DEG_TO_RAD;

    g_car_yaw_target_deg = heading_deg;
    g_car_base_speed_command = MODE2_TARGET_SPEED * alignment_scale;
    g_car_world_velocity_x_command = cosf(target_rad) * MODE2_TARGET_SPEED;
    g_car_world_velocity_y_command = sinf(target_rad) * MODE2_TARGET_SPEED;
    g_car_world_speed_magnitude = MODE2_TARGET_SPEED;
    g_car_world_heading_target_deg = heading_deg;
    g_car_world_heading_error_deg = heading_error_deg;
    g_car_world_alignment_scale = alignment_scale;
    g_car_world_reverse_active = 0.0f;
}

static uint16 mode2_brake_stable_cycles(void)
{
    float value = mode2_clampf(car_large_turn_brake_stable_cycles,
                               1.0f, 100.0f);
    return (uint16)(value + 0.5f);
}

static void mode2_large_turn_timeout(void)
{
    mode2_large_turn_reset();
    s_mode2_large_turn_rearm_required = 1U;
    s_mode2_world_command_active = 0U;
    g_car_base_speed_command = 0.0f;
    g_car_yaw_target_deg = g_car_yaw_feedback_deg;
    PID_Reset(&s_mode2_yaw_pid);
    PID_Reset(&s_mode2_gyroz_pid);
}

static void mode2_large_turn_update(void)
{
    float yaw_error_deg = mode2_yaw_error_deg();
    float yaw_error_abs = fabsf(yaw_error_deg);
    float enter_angle = fabsf(car_large_turn_enter_angle_deg);
    float pivot_exit_angle = fabsf(car_large_turn_pivot_exit_angle_deg);
    float finish_angle = fabsf(car_large_turn_finish_angle_deg);
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    uint8 translation_slow =
        (fabsf(body_speed) <= fabsf(car_large_turn_brake_speed)) ? 1U : 0U;
    uint16 stable_cycles = mode2_brake_stable_cycles();
    int8 direction;

    if (enter_angle < pivot_exit_angle)
    {
        enter_angle = pivot_exit_angle;
    }
    if (finish_angle > pivot_exit_angle)
    {
        finish_angle = pivot_exit_angle;
    }

    if (s_mode2_large_turn_latched != 0U)
    {
        if (s_mode2_large_turn_elapsed_cycles < MODE2_LARGE_TURN_TIMEOUT_CYCLES)
        {
            s_mode2_large_turn_elapsed_cycles++;
        }
        if (s_mode2_large_turn_elapsed_cycles >= MODE2_LARGE_TURN_TIMEOUT_CYCLES)
        {
            mode2_large_turn_timeout();
            return;
        }
    }

    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_NORMAL)
    {
        if ((s_mode2_world_command_active != 0U) &&
            (yaw_error_abs >= enter_angle))
        {
            direction = (yaw_error_deg >= 0.0f) ? 1 : -1;
            if (direction != s_mode2_large_turn_trigger_direction)
            {
                s_mode2_large_turn_trigger_direction = direction;
                s_mode2_large_turn_trigger_cycles = 1U;
            }
            else if (s_mode2_large_turn_trigger_cycles <
                     MODE2_LARGE_TURN_TRIGGER_CYCLES)
            {
                s_mode2_large_turn_trigger_cycles++;
            }
            if (s_mode2_large_turn_trigger_cycles >=
                MODE2_LARGE_TURN_TRIGGER_CYCLES)
            {
                s_mode2_large_turn_latched = 1U;
                s_mode2_large_turn_direction = direction;
                s_mode2_large_turn_target_yaw_deg = g_car_yaw_target_deg;
                s_mode2_large_turn_elapsed_cycles = 0U;
                mode2_large_turn_set((translation_slow != 0U)
                                         ? MODE2_LARGE_TURN_PIVOT
                                         : MODE2_LARGE_TURN_BRAKE);
            }
        }
        else
        {
            s_mode2_large_turn_trigger_cycles = 0U;
            s_mode2_large_turn_trigger_direction = 0;
        }
    }
    else if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        if (yaw_error_abs <= pivot_exit_angle)
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_EXIT);
        }
        else if (translation_slow != 0U)
        {
            if (s_mode2_large_turn_brake_cycles < stable_cycles)
            {
                s_mode2_large_turn_brake_cycles++;
            }
            if (s_mode2_large_turn_brake_cycles >= stable_cycles)
            {
                mode2_large_turn_set(MODE2_LARGE_TURN_PIVOT);
            }
        }
        else
        {
            s_mode2_large_turn_brake_cycles = 0U;
        }
    }
    else if (s_mode2_large_turn_state == MODE2_LARGE_TURN_PIVOT)
    {
        if (yaw_error_abs <= pivot_exit_angle)
        {
            mode2_large_turn_set(MODE2_LARGE_TURN_EXIT);
        }
    }
    else if (yaw_error_abs <= finish_angle)
    {
        if (++s_mode2_large_turn_finish_cycles >= MODE2_LARGE_TURN_FINISH_CYCLES)
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

static float mode2_plan_step(float value)
{
    value = fabsf(value);
    return (value >= MODE2_SPEED_PLAN_MIN_STEP)
               ? value : MODE2_SPEED_PLAN_MIN_STEP;
}

static void mode2_speed_plan_update(void)
{
    float command = g_car_base_speed_command;
    float previous = g_car_base_speed_target;
    float brake_target_max;
    float step;
    float delta;
    float accel_limit = fabsf(car_speed_accel_ff_limit);

    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        command = fabsf(car_large_turn_brake_target_speed);
        brake_target_max = fabsf(car_large_turn_brake_speed) -
                           MODE2_BRAKE_TARGET_MARGIN;
        if (brake_target_max < 0.0f)
        {
            brake_target_max = 0.0f;
        }
        command = mode2_clampf(command, 0.0f, brake_target_max);
    }
    step = (fabsf(command) > fabsf(previous))
               ? mode2_plan_step(car_speed_accel_step_limit)
               : mode2_plan_step(car_speed_decel_step_limit);
    delta = mode2_clampf(command - previous, -step, step);
    g_car_base_speed_target = previous + delta;
    g_car_base_speed_delta = delta;
    g_car_speed_accel_ff = (car_speed_accel_kff > 0.0f)
                               ? mode2_clampf(car_speed_accel_kff * delta,
                                              -accel_limit, accel_limit)
                               : 0.0f;
}

static void mode2_yaw_control(void)
{
    float yaw_error_deg = mode2_yaw_error_deg();
    float desired_rate_dps;

    if (((s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE) ||
         (s_mode2_large_turn_state == MODE2_LARGE_TURN_PIVOT)) &&
        (s_mode2_large_turn_direction != 0))
    {
        yaw_error_deg = (float)s_mode2_large_turn_direction *
                        fabsf(yaw_error_deg);
    }
    s_mode2_yaw_pid.kp = car_yaw_kp;
    s_mode2_yaw_pid.ki = 0.0f;
    s_mode2_yaw_pid.kd = car_yaw_kd;
    PID_SetOutputLimits(&s_mode2_yaw_pid,
                        -car_yaw_rate_limit_dps,
                        car_yaw_rate_limit_dps);
    desired_rate_dps = PID_Update(&s_mode2_yaw_pid, yaw_error_deg, 0.0f);
    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        desired_rate_dps = mode2_clampf(
            desired_rate_dps,
            -fabsf(car_large_turn_brake_rate_limit_dps),
            fabsf(car_large_turn_brake_rate_limit_dps));
    }
    g_car_yaw_p_term = s_mode2_yaw_pid.p_term;
    g_car_yaw_d_term = s_mode2_yaw_pid.d_term;
    g_car_gyroz_target_dps = -desired_rate_dps;
}

static void mode2_gyroz_control(void)
{
    float left_abs;
    float right_abs;
    float wheel_peak;
    float wheel_scale;

    g_car_gyroz_feedback_equivalent = g_car_gyroz_feedback_dps *
                                      MODE2_DEG_TO_RAD *
                                      MODE2_GYROZ_EQUIVALENT_SCALE;
    s_mode2_gyroz_pid.kp = car_gyroz_kp;
    s_mode2_gyroz_pid.ki = car_gyroz_ki;
    s_mode2_gyroz_pid.kd = 0.0f;
    s_mode2_gyroz_pid.kff = car_gyroz_kff;
    s_mode2_gyroz_pid.i_limit = MODE2_GYROZ_OUTPUT_LIMIT;
    PID_SetOutputLimits(&s_mode2_gyroz_pid,
                        -MODE2_GYROZ_OUTPUT_LIMIT,
                        MODE2_GYROZ_OUTPUT_LIMIT);
    g_car_gyroz_output = PID_Update(&s_mode2_gyroz_pid,
                                    -g_car_gyroz_target_dps,
                                    g_car_gyroz_feedback_dps);
    g_car_gyroz_error = s_mode2_gyroz_pid.error;
    g_car_gyroz_ff_term = s_mode2_gyroz_pid.ff_term;
    g_car_gyroz_p_term = s_mode2_gyroz_pid.p_term;
    g_car_gyroz_i_term = s_mode2_gyroz_pid.i_term;

    Left_Target_Speed = g_car_base_speed_target +
                        car_gyroz_k_turn * g_car_gyroz_output;
    Right_Target_Speed = g_car_base_speed_target -
                         car_gyroz_k_turn * g_car_gyroz_output;
    left_abs = fabsf(Left_Target_Speed);
    right_abs = fabsf(Right_Target_Speed);
    wheel_peak = (left_abs > right_abs) ? left_abs : right_abs;
    if (wheel_peak > MODE2_WHEEL_TARGET_ABS_LIMIT)
    {
        wheel_scale = MODE2_WHEEL_TARGET_ABS_LIMIT / wheel_peak;
        Left_Target_Speed *= wheel_scale;
        Right_Target_Speed *= wheel_scale;
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

static void mode2_brake_update(void)
{
    float previous_target;

    if ((s_mode2_world_command_active != 0U) ||
        (fabsf(g_car_gyroz_target_dps) >= MODE2_GYROZ_STOP_TARGET_DPS))
    {
        s_mode2_speed_brake_active = 0U;
        g_car_speed_brake_active = 0.0f;
        return;
    }
    if ((s_mode2_speed_brake_active == 0U) &&
        ((fabsf(g_car_speed_left_filtered) > MODE2_WHEEL_STOP_SPEED) ||
         (fabsf(g_car_speed_right_filtered) > MODE2_WHEEL_STOP_SPEED)))
    {
        previous_target = g_car_base_speed_target - g_car_base_speed_delta;
        s_mode2_left_brake_direction = mode2_speed_direction(
            g_car_speed_left_filtered, previous_target);
        s_mode2_right_brake_direction = mode2_speed_direction(
            g_car_speed_right_filtered, previous_target);
        s_mode2_speed_brake_active = 1U;
        g_car_speed_brake_active = 1.0f;
    }
}

static float mode2_brake_feedforward(float feedback, float direction)
{
    float speed_abs = fabsf(feedback);
    float full_speed = fabsf(car_speed_ff_deadband);
    float scale;

    if ((s_mode2_speed_brake_active == 0U) || (direction == 0.0f) ||
        (feedback * direction <= 0.0f) ||
        (speed_abs <= MODE2_WHEEL_STOP_SPEED))
    {
        return 0.0f;
    }
    if (full_speed <= MODE2_WHEEL_STOP_SPEED)
    {
        full_speed = MODE2_WHEEL_STOP_SPEED + 1.0f;
    }
    scale = (speed_abs >= full_speed)
                ? 1.0f
                : (speed_abs - MODE2_WHEEL_STOP_SPEED) /
                      (full_speed - MODE2_WHEEL_STOP_SPEED);
    return -direction * fabsf(car_speed_brake_static) * scale;
}

static float mode2_large_turn_brake_feedforward(void)
{
    float body_speed = 0.5f *
        (g_car_speed_left_filtered + g_car_speed_right_filtered);
    float release_speed = fabsf(car_large_turn_brake_speed);
    float speed_abs = fabsf(body_speed);
    float scale;

    if ((s_mode2_large_turn_state != MODE2_LARGE_TURN_BRAKE) ||
        (speed_abs <= release_speed))
    {
        return 0.0f;
    }
    scale = (speed_abs - release_speed) / MODE2_BRAKE_FF_FADE_SPAN;
    scale = mode2_clampf(scale, 0.0f, 1.0f);
    return -mode2_speed_direction(body_speed, 0.0f) *
           fabsf(car_large_turn_brake_ff) * scale;
}

static float mode2_accel_feedforward(void)
{
    float reference = fabsf(g_car_base_speed_target);
    float turn_target;
    float turn_ratio;
    float scale;

    if (reference < MODE2_SPEED_PLAN_MIN_STEP)
    {
        return 0.0f;
    }
    turn_target = 0.5f * (Left_Target_Speed - Right_Target_Speed);
    turn_ratio = fabsf(turn_target) / reference;
    if (turn_ratio <= MODE2_ACCEL_TURN_FULL_RATIO)
    {
        scale = 1.0f;
    }
    else if (turn_ratio >= MODE2_ACCEL_TURN_DISABLE_RATIO)
    {
        scale = 0.0f;
    }
    else
    {
        scale = (MODE2_ACCEL_TURN_DISABLE_RATIO - turn_ratio) /
                (MODE2_ACCEL_TURN_DISABLE_RATIO -
                 MODE2_ACCEL_TURN_FULL_RATIO);
    }
    return g_car_speed_accel_ff * scale;
}

static int16 mode2_speed_pid_update(pid_t *pid,
                                    float target,
                                    float feedback,
                                    float kp,
                                    float ki,
                                    float kd,
                                    float external_ff,
                                    float brake_ff)
{
    float output;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = car_speed_ff_slope;
    pid->i_limit = MODE2_SPEED_I_LIMIT;
    PID_SetStaticFeedforward(pid, car_speed_ff_static);
    PID_SetFeedforwardTransition(pid, car_speed_ff_deadband,
                                 car_speed_ff_transition);
    PID_SetExternalFeedforward(pid, external_ff);
    PID_SetIncrementLimit(pid, car_speed_delta_output_limit);
    PID_SetOutputLimits(pid, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);
    output = PID_UpdateIncremental(pid, target, feedback) + brake_ff;
    return (int16)mode2_clampf(output,
                              -(float)MOTOR_PWM_MAX,
                              (float)MOTOR_PWM_MAX);
}

static uint8 mode2_stopped(float angle_limit_deg,
                           float target_rate_limit_dps,
                           float feedback_rate_limit_dps,
                           float wheel_speed_limit)
{
    return ((fabsf(mode2_yaw_error_deg()) < angle_limit_deg) &&
            (fabsf(g_car_gyroz_target_dps) < target_rate_limit_dps) &&
            (fabsf(g_car_gyroz_feedback_dps) < feedback_rate_limit_dps) &&
            (fabsf(g_car_speed_left_filtered) < wheel_speed_limit) &&
            (fabsf(g_car_speed_right_filtered) < wheel_speed_limit))
               ? 1U : 0U;
}

void car_mode2_init(void)
{
    PID_Init(&s_mode2_left_speed_pid, car_speed_left_kp, car_speed_left_ki,
             car_speed_left_kd, car_speed_ff_slope, MODE2_SPEED_I_LIMIT);
    PID_Init(&s_mode2_right_speed_pid, car_speed_right_kp, car_speed_right_ki,
             car_speed_right_kd, car_speed_ff_slope, MODE2_SPEED_I_LIMIT);
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
    s_mode2_world_command_active = 0U;
    s_mode2_input_confirm_cycles = 0U;
    s_mode2_yaw_stopped = 0U;
    s_mode2_speed_brake_active = 0U;
    s_mode2_left_brake_direction = 0.0f;
    s_mode2_right_brake_direction = 0.0f;
    g_car_yaw_target_deg = g_car_yaw_feedback_deg;
    mode2_large_turn_reset();
    mode2_clear_diagnostics();
}

void car_mode2_update_25HZ(uint32 now_ms)
{
    (void)now_ms;
}

void car_mode2_update_100HZ(uint32 now_ms)
{
    float accel_ff;
    float left_brake_ff;
    float right_brake_ff;
    float large_turn_brake_ff;
    int16 left_output;
    int16 right_output;

    (void)now_ms;
    mode2_world_command_update();
    mode2_large_turn_update();
    mode2_speed_plan_update();
    mode2_yaw_control();
    mode2_gyroz_control();

    if (s_mode2_yaw_stopped != 0U)
    {
        if ((Left_Target_Speed == 0.0f) &&
            (Right_Target_Speed == 0.0f) &&
            (mode2_stopped(MODE2_YAW_WAKE_ANGLE_DEG,
                           MODE2_GYROZ_WAKE_TARGET_DPS,
                           2.0f * MODE2_YAW_STOP_RATE_DPS,
                           2.0f * MODE2_WHEEL_STOP_SPEED) != 0U))
        {
            g_car_speed_left_motor_output = 0.0f;
            g_car_speed_right_motor_output = 0.0f;
            motor_stop();
            return;
        }
        s_mode2_yaw_stopped = 0U;
    }

    if ((Left_Target_Speed == 0.0f) &&
        (Right_Target_Speed == 0.0f) &&
        (mode2_stopped(MODE2_YAW_STOP_ANGLE_DEG,
                       MODE2_GYROZ_STOP_TARGET_DPS,
                       MODE2_YAW_STOP_RATE_DPS,
                       MODE2_WHEEL_STOP_SPEED) != 0U))
    {
        PID_Reset(&s_mode2_left_speed_pid);
        PID_Reset(&s_mode2_right_speed_pid);
        PID_Reset(&s_mode2_yaw_pid);
        PID_Reset(&s_mode2_gyroz_pid);
        s_mode2_yaw_stopped = 1U;
        g_car_speed_left_motor_output = 0.0f;
        g_car_speed_right_motor_output = 0.0f;
        motor_stop();
        return;
    }

    if (s_mode2_large_turn_state == MODE2_LARGE_TURN_BRAKE)
    {
        s_mode2_speed_brake_active = 0U;
        g_car_speed_brake_active = 0.0f;
    }
    else
    {
        mode2_brake_update();
    }
    accel_ff = mode2_accel_feedforward();
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
    g_car_speed_left_brake_ff = left_brake_ff;
    g_car_speed_right_brake_ff = right_brake_ff;

    left_output = mode2_speed_pid_update(
        &s_mode2_left_speed_pid,
        Left_Target_Speed, g_car_speed_left_filtered,
        car_speed_left_kp, car_speed_left_ki, car_speed_left_kd,
        accel_ff, left_brake_ff);
    right_output = mode2_speed_pid_update(
        &s_mode2_right_speed_pid,
        Right_Target_Speed, g_car_speed_right_filtered,
        car_speed_right_kp, car_speed_right_ki, car_speed_right_kd,
        accel_ff, right_brake_ff);
    g_car_speed_left_motor_output = (float)left_output;
    g_car_speed_right_motor_output = (float)right_output;
    motor_left_set_speed(left_output);
    motor_right_set_speed(right_output);
}
