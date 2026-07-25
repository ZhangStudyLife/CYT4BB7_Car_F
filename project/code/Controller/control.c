#include "control.h"

#include <math.h>

#define CONTROL_DEG_TO_RAD (0.017453292519943295f)
#define CONTROL_PI         (3.14159265358979323846f)
#define CONTROL_TWO_PI     (6.28318530717958647692f)
#define CONTROL_UPDATE_DT_S            (0.01f)
#define CONTROL_WHEEL_FF_KS_LEFT       (280.0f)
#define CONTROL_WHEEL_FF_KS_RIGHT      (260.0f)
#define CONTROL_WHEEL_FF_KV_LEFT       (4.20f)
#define CONTROL_WHEEL_FF_KV_RIGHT      (4.20f)
#define CONTROL_WHEEL_FF_KSTART_LEFT   (450.0f)
#define CONTROL_WHEEL_FF_KSTART_RIGHT  (380.0f)
#define CONTROL_WHEEL_FF_KS_FULL_SPEED (100.0f)
#define CONTROL_WHEEL_FF_START_FULL_SPEED (15.0f)
#define CONTROL_WHEEL_FF_START_TARGET_MIN (3.0f)
#define CONTROL_WHEEL_FF_START_FEEDBACK_MAX (2.0f)
#define CONTROL_GEOMETRY_MIN_VALUE     (0.0001f)
#define CONTROL_YAW_RATE_ABS_MAX_RAD_S (3.0f)

/* PID 实例 */
PositionalPID wheel_left_pid;
PositionalPID wheel_right_pid;
PositionalPID yaw_angle_pid;
PositionalPID yaw_rate_pid;

/* 调试中间变量 */
float control_yaw_angle_current = 0.0f;
float control_yaw_angle_output = 0.0f;
float control_yaw_rate_target = 0.0f;
float control_yaw_rate_current = 0.0f;
float control_yaw_rate_output = 0.0f;
float control_heading_target = 0.0f;
float control_heading_error = 0.0f;

float control_left_wheel_target_count = 0.0f;
float control_right_wheel_target_count = 0.0f;
float control_left_wheel_feedback_count = 0.0f;
float control_right_wheel_feedback_count = 0.0f;
float control_left_wheel_feedforward_pwm = 0.0f;
float control_right_wheel_feedforward_pwm = 0.0f;
float control_left_wheel_output_pwm = 0.0f;
float control_right_wheel_output_pwm = 0.0f;

static car_control_command_mode_t s_last_command_mode = CAR_CONTROL_COMMAND_STOP;

static float control_wheel_ff(float target, float feedback, float ks, float kv, float kstart)
{
    float abs_target = target;
    float abs_feedback = feedback;
    float ks_scale;
    float start_scale;
    float min_ff;
    float ff;

    if(target == 0.0f) return 0.0f;
    if(abs_target < 0.0f) abs_target = -abs_target;
    if(abs_feedback < 0.0f) abs_feedback = -abs_feedback;

    ks_scale = abs_target / CONTROL_WHEEL_FF_KS_FULL_SPEED;
    if(ks_scale > 1.0f) ks_scale = 1.0f;
    ks_scale *= ks_scale;

    if(target > 0.0f) ff = kv * target + ks * ks_scale;
    else ff = kv * target - ks * ks_scale;

    if((abs_target > CONTROL_WHEEL_FF_START_TARGET_MIN) &&
       (abs_feedback < CONTROL_WHEEL_FF_START_FEEDBACK_MAX))
    {
        start_scale = abs_target / CONTROL_WHEEL_FF_START_FULL_SPEED;
        if(start_scale > 1.0f) start_scale = 1.0f;

        min_ff = kstart * start_scale;
        if((target > 0.0f) && (ff < min_ff)) ff = min_ff;
        else if((target < 0.0f) && (ff > -min_ff)) ff = -min_ff;
    }

    return ff;
}

static float control_wrap_pi(float angle)
{
    if(!isfinite(angle))
    {
        return 0.0f;
    }

    angle = fmodf(angle, CONTROL_TWO_PI);
    if(angle > CONTROL_PI)
    {
        angle -= CONTROL_TWO_PI;
    }
    else if(angle < -CONTROL_PI)
    {
        angle += CONTROL_TWO_PI;
    }

    return angle;
}

/* 将PID与前馈叠加后的浮点输出限制到电机驱动允许范围。 */
static float control_limit_motor_pwm(float pwm)
{
    if(!isfinite(pwm))
    {
        return 0.0f;
    }

    if(pwm > (float)MOTOR_PWM_MAX)
    {
        return (float)MOTOR_PWM_MAX;
    }
    if(pwm < -(float)MOTOR_PWM_MAX)
    {
        return -(float)MOTOR_PWM_MAX;
    }

    return pwm;
}

static float control_limit_yaw_rate(float yaw_rate_rad_s)
{
    if(!isfinite(yaw_rate_rad_s))
    {
        return 0.0f;
    }

    if(yaw_rate_rad_s > CONTROL_YAW_RATE_ABS_MAX_RAD_S)
    {
        return CONTROL_YAW_RATE_ABS_MAX_RAD_S;
    }
    if(yaw_rate_rad_s < -CONTROL_YAW_RATE_ABS_MAX_RAD_S)
    {
        return -CONTROL_YAW_RATE_ABS_MAX_RAD_S;
    }

    return yaw_rate_rad_s;
}

static void control_pid_init_all(void)
{
    PositionalPID_Init(&wheel_left_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&wheel_right_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&yaw_angle_pid, 0.0f, yaw_angle_kp, yaw_angle_ki, yaw_angle_kd, yaw_angle_i_limit, yaw_angle_output_limit);
    PositionalPID_Init(&yaw_rate_pid, 0.0f, yaw_rate_kp, yaw_rate_ki, yaw_rate_kd, yaw_rate_i_limit, yaw_rate_output_limit);
}

static void control_wheel_pid_apply_params(PositionalPID *pid)
{
    pid->kp_2 = 0.0f;
    pid->kp_1 = wheel_kp;
    pid->ki = wheel_ki;
    pid->kd = wheel_kd;
    pid->i_limit = wheel_i_limit;
    pid->output_limit = wheel_output_limit;
}

static void control_pid_apply_all(void)
{
    control_wheel_pid_apply_params(&wheel_left_pid);
    control_wheel_pid_apply_params(&wheel_right_pid);

    yaw_angle_pid.kp_1 = yaw_angle_kp;     yaw_angle_pid.ki = yaw_angle_ki;
    yaw_angle_pid.kd = yaw_angle_kd;       yaw_angle_pid.i_limit = yaw_angle_i_limit;
    yaw_angle_pid.output_limit = yaw_angle_output_limit;

    yaw_rate_pid.kp_1 = yaw_rate_kp;       yaw_rate_pid.ki = yaw_rate_ki;
    yaw_rate_pid.kd = yaw_rate_kd;         yaw_rate_pid.i_limit = yaw_rate_i_limit;
    yaw_rate_pid.output_limit = yaw_rate_output_limit;
}

/* ========== 公共接口 ========== */

void Control_Init(void)
{
    control_pid_init_all();
}

void Control_Reset(void)
{
    control_pid_init_all();
    control_yaw_angle_current = control_yaw_angle_output = 0.0f;
    control_yaw_rate_target = control_yaw_rate_current = control_yaw_rate_output = 0.0f;
    control_heading_target = control_heading_error = 0.0f;
    control_left_wheel_target_count = control_right_wheel_target_count = 0.0f;
    control_left_wheel_feedback_count = control_right_wheel_feedback_count = 0.0f;
    control_left_wheel_feedforward_pwm = control_right_wheel_feedforward_pwm = 0.0f;
    control_left_wheel_output_pwm = control_right_wheel_output_pwm = 0.0f;
}

void Control_Stop(void)
{
    Control_Reset();
    /* 当前底盘仅使用 M1 左轮和 M2 右轮，停车时不访问未初始化的旧麦轮通道。 */
    two_wheel_motor_stop();
}

float Control_GetYawAngle(void)
{
    float yaw = -g_euler.yaw * CONTROL_DEG_TO_RAD;

    return control_wrap_pi(yaw);
}

/*
 * 100Hz左右轮速度闭环。
 * 目标与反馈统一使用“脉冲/10ms”，避免在PID内部混用m/s和编码器计数。
 */
void Control_WheelSpeed100Hz(float left_target_count, float right_target_count)
{
    if((!isfinite(left_target_count)) || (!isfinite(right_target_count)))
    {
        Control_Stop();
        return;
    }

    control_pid_apply_all();

    control_left_wheel_target_count = left_target_count;
    control_right_wheel_target_count = right_target_count;
    control_left_wheel_feedback_count = encoder_get_left_filtered_count();
    control_right_wheel_feedback_count = encoder_get_right_filtered_count();

    control_left_wheel_feedforward_pwm =
        control_wheel_ff(control_left_wheel_target_count,
                         control_left_wheel_feedback_count,
                         CONTROL_WHEEL_FF_KS_LEFT,
                         CONTROL_WHEEL_FF_KV_LEFT,
                         CONTROL_WHEEL_FF_KSTART_LEFT);
    control_right_wheel_feedforward_pwm =
        control_wheel_ff(control_right_wheel_target_count,
                         control_right_wheel_feedback_count,
                         CONTROL_WHEEL_FF_KS_RIGHT,
                         CONTROL_WHEEL_FF_KV_RIGHT,
                         CONTROL_WHEEL_FF_KSTART_RIGHT);

    control_left_wheel_output_pwm = control_limit_motor_pwm(
        control_left_wheel_feedforward_pwm +
        PositionalPID_Update(&wheel_left_pid,
                             control_left_wheel_target_count,
                             control_left_wheel_feedback_count));
    control_right_wheel_output_pwm = control_limit_motor_pwm(
        control_right_wheel_feedforward_pwm +
        PositionalPID_Update(&wheel_right_pid,
                             control_right_wheel_target_count,
                             control_right_wheel_feedback_count));

    two_wheel_motor_set((int16_t)control_left_wheel_output_pwm,
                        (int16_t)control_right_wheel_output_pwm);
}

/*
 * 两轮差速逆运动学：
 *   v_left  = v - omega * track / 2
 *   v_right = v + omega * track / 2
 * 再按左右轮各自的每米脉冲数换算为100Hz轮速闭环目标。
 */
static void control_apply_twist_100hz(float linear_mps, float yaw_rate_rad_s)
{
    float half_track_m;
    float left_mps;
    float right_mps;
    float left_target_count;
    float right_target_count;

    if((!isfinite(linear_mps)) ||
       (!isfinite(yaw_rate_rad_s)) ||
       (!isfinite(wheel_left_count_per_meter)) ||
       (!isfinite(wheel_right_count_per_meter)) ||
       (!isfinite(wheel_track_m)) ||
       (wheel_left_count_per_meter <= CONTROL_GEOMETRY_MIN_VALUE) ||
       (wheel_right_count_per_meter <= CONTROL_GEOMETRY_MIN_VALUE) ||
       (wheel_track_m <= CONTROL_GEOMETRY_MIN_VALUE))
    {
        Control_Stop();
        return;
    }

    yaw_rate_rad_s = control_limit_yaw_rate(yaw_rate_rad_s);
    half_track_m = wheel_track_m * 0.5f;
    left_mps = linear_mps - yaw_rate_rad_s * half_track_m;
    right_mps = linear_mps + yaw_rate_rad_s * half_track_m;

    left_target_count =
        left_mps * wheel_left_count_per_meter * CONTROL_UPDATE_DT_S;
    right_target_count =
        right_mps * wheel_right_count_per_meter * CONTROL_UPDATE_DT_S;

    Control_WheelSpeed100Hz(left_target_count, right_target_count);
}

void Control_Twist100Hz(float linear_mps, float yaw_rate_rad_s)
{
    if((!isfinite(linear_mps)) || (!isfinite(yaw_rate_rad_s)))
    {
        Control_Stop();
        return;
    }

    control_yaw_rate_target = control_limit_yaw_rate(yaw_rate_rad_s);
    control_yaw_rate_current =
        -g_imufilter_1000hz.gyroz * CONTROL_DEG_TO_RAD;
    control_yaw_rate_output = control_yaw_rate_target;

    control_apply_twist_100hz(linear_mps, control_yaw_rate_target);
}

/*
 * 100Hz角速度内环。
 *
 * yaw_rate_pid输出统一定义为rad/s，并直接作为两轮差速解算的角速度命令。
 * 该结构与原麦轮串级方向环一致：PID输出完整旋转命令，不额外叠加目标角速度
 * 前馈。待标准串级实车调通后，再根据响应情况决定是否增加前馈。
 */
void Control_YawRate100Hz(float linear_mps, float yaw_rate_target_rad_s)
{
    if((!isfinite(linear_mps)) || (!isfinite(yaw_rate_target_rad_s)))
    {
        Control_Stop();
        return;
    }

    control_pid_apply_all();

    control_yaw_rate_target = control_limit_yaw_rate(yaw_rate_target_rad_s);
    control_yaw_rate_current =
        -g_imufilter_1000hz.gyroz * CONTROL_DEG_TO_RAD;
    control_yaw_rate_output =
        PositionalPID_Update(&yaw_rate_pid,
                             control_yaw_rate_target,
                             control_yaw_rate_current);
    control_yaw_rate_output =
        control_limit_yaw_rate(control_yaw_rate_output);

    control_apply_twist_100hz(linear_mps, control_yaw_rate_output);
}

/*
 * 100Hz航向角外环 + 角速度内环。
 *
 * PositionalPID_Update()内部直接计算target-current，因此这里先将误差归一化到
 * [-pi, pi]，再构造当前角度附近的等效目标，避免跨越+/-pi时选择错误长路径。
 */
void Control_Heading100Hz(float linear_mps, float heading_target_rad)
{
    float equivalent_target_rad;

    if((!isfinite(linear_mps)) || (!isfinite(heading_target_rad)))
    {
        Control_Stop();
        return;
    }

    control_pid_apply_all();

    control_heading_target = control_wrap_pi(heading_target_rad);
    control_yaw_angle_current = Control_GetYawAngle();
    control_heading_error =
        control_wrap_pi(control_heading_target - control_yaw_angle_current);
    equivalent_target_rad =
        control_yaw_angle_current + control_heading_error;

    control_yaw_angle_output =
        PositionalPID_Update(&yaw_angle_pid,
                             equivalent_target_rad,
                             control_yaw_angle_current);
    control_yaw_angle_output =
        control_limit_yaw_rate(control_yaw_angle_output);

    Control_YawRate100Hz(linear_mps, control_yaw_angle_output);
}

void Control_ExecuteCommand100Hz(const car_control_command_t *command)
{
    if((command == NULL) ||
       (command->enabled == 0U) ||
       (command->mode == CAR_CONTROL_COMMAND_STOP))
    {
        Control_Stop();
        s_last_command_mode = CAR_CONTROL_COMMAND_STOP;
        return;
    }

    if(command->mode != s_last_command_mode)
    {
        /*
         * 切换PWM/轮速/角速度/航向模式时清除上一个模式的积分与微分状态，
         * 防止调试模式或遥控模式切换后继承旧控制器输出。
         */
        Control_Reset();
        s_last_command_mode = command->mode;
    }

    switch(command->mode)
    {
    case CAR_CONTROL_COMMAND_PWM:
        if((!isfinite(command->left_pwm)) ||
           (!isfinite(command->right_pwm)))
        {
            Control_Stop();
            s_last_command_mode = CAR_CONTROL_COMMAND_STOP;
            return;
        }
        two_wheel_motor_set(
            (int16_t)control_limit_motor_pwm(command->left_pwm),
            (int16_t)control_limit_motor_pwm(command->right_pwm));
        break;

    case CAR_CONTROL_COMMAND_WHEEL_SPEED:
        Control_WheelSpeed100Hz(command->left_target_count,
                                command->right_target_count);
        break;

    case CAR_CONTROL_COMMAND_TWIST:
        Control_Twist100Hz(command->linear_mps,
                           command->yaw_rate_rad_s);
        break;

    case CAR_CONTROL_COMMAND_YAW_RATE:
        Control_YawRate100Hz(command->linear_mps,
                             command->yaw_rate_rad_s);
        break;

    case CAR_CONTROL_COMMAND_HEADING:
        Control_Heading100Hz(command->linear_mps,
                             command->heading_target_rad);
        break;

    default:
        Control_Stop();
        s_last_command_mode = CAR_CONTROL_COMMAND_STOP;
        break;
    }
}
