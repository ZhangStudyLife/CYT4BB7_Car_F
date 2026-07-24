#include "control.h"

#include <math.h>

#define CONTROL_DEG_TO_RAD (0.017453292519943295f)
#define CONTROL_PI         (3.14159265358979323846f)
#define CONTROL_TWO_PI     (6.28318530717958647692f)
#define CONTROL_WHEEL_FF_KS_LF         (270.0f)
#define CONTROL_WHEEL_FF_KS_RF         (390.0f)
#define CONTROL_WHEEL_FF_KS_LR         (430.0f)
#define CONTROL_WHEEL_FF_KS_RR         (370.0f)
#define CONTROL_WHEEL_FF_KV_LF         (8.20f)
#define CONTROL_WHEEL_FF_KV_RF         (6.25f)
#define CONTROL_WHEEL_FF_KV_LR         (7.05f)
#define CONTROL_WHEEL_FF_KV_RR         (8.55f)
#define CONTROL_WHEEL_FF_KSTART_LF     (240.0f)
#define CONTROL_WHEEL_FF_KSTART_RF     (300.0f)
#define CONTROL_WHEEL_FF_KSTART_LR     (325.0f)
#define CONTROL_WHEEL_FF_KSTART_RR     (300.0f)
#define CONTROL_WHEEL_FF_KS_FULL_SPEED (100.0f)
#define CONTROL_WHEEL_FF_START_FULL_SPEED (15.0f)
#define CONTROL_WHEEL_FF_START_TARGET_MIN (3.0f)
#define CONTROL_WHEEL_FF_START_FEEDBACK_MAX (2.0f)

/* PID 实例 */
PositionalPID wheel_left_front_pid;
PositionalPID wheel_right_front_pid;
PositionalPID wheel_left_rear_pid;
PositionalPID wheel_right_rear_pid;
PositionalPID yaw_angle_pid;
PositionalPID yaw_rate_pid;

/* 调试中间变量 */
float control_yaw_angle_current = 0.0f;
float control_yaw_angle_output = 0.0f;
float control_yaw_rate_target = 0.0f;
float control_yaw_rate_current = 0.0f;
float control_yaw_rate_output = 0.0f;

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

static void control_pid_init_all(void)
{
    PositionalPID_Init(&wheel_left_front_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&wheel_right_front_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&wheel_left_rear_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
    PositionalPID_Init(&wheel_right_rear_pid, 0.0f, wheel_kp, wheel_ki, wheel_kd, wheel_i_limit, wheel_output_limit);
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
    control_wheel_pid_apply_params(&wheel_left_front_pid);
    control_wheel_pid_apply_params(&wheel_right_front_pid);
    control_wheel_pid_apply_params(&wheel_left_rear_pid);
    control_wheel_pid_apply_params(&wheel_right_rear_pid);

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
}

void Control_Stop(void)
{
    Control_Reset();
    mecanum_motor_stop();
}

float Control_GetYawAngle(void)
{
    float yaw = -g_euler.yaw * CONTROL_DEG_TO_RAD;

    return control_wrap_pi(yaw);
}

/* 100Hz：yaw目标 + 麦克纳姆解算 + 四轮速度环 + 电机输出 */
void Control_100Hz(float forward, float strafe, float yaw_target_rad)
{
    float rot, lf, rf, lr, rr;
    float lf_feedback, rf_feedback, lr_feedback, rr_feedback;
    float lf_ff, rf_ff, lr_ff, rr_ff;
    float yaw_error;

    control_pid_apply_all();

    control_yaw_angle_current = Control_GetYawAngle();
    yaw_error = control_wrap_pi(yaw_target_rad - control_yaw_angle_current);
    control_yaw_angle_output = PositionalPID_Update(&yaw_angle_pid, yaw_error, 0.0f);
    control_yaw_rate_target = control_yaw_angle_output;

    control_yaw_rate_current = -g_imufilter_1000hz.gyroz * CONTROL_DEG_TO_RAD;
    control_yaw_rate_output = PositionalPID_Update(&yaw_rate_pid, control_yaw_rate_target, control_yaw_rate_current);

    rot = control_yaw_rate_output;
    lf = forward - strafe - rot;
    rf = forward + strafe + rot;
    lr = forward + strafe - rot;
    rr = forward - strafe + rot;
    lf_feedback = encoder_get_left_front_filtered_count();
    rf_feedback = encoder_get_right_front_filtered_count();
    lr_feedback = encoder_get_left_rear_filtered_count();
    rr_feedback = encoder_get_right_rear_filtered_count();
    lf_ff = control_wheel_ff(lf, lf_feedback, CONTROL_WHEEL_FF_KS_LF, CONTROL_WHEEL_FF_KV_LF, CONTROL_WHEEL_FF_KSTART_LF);
    rf_ff = control_wheel_ff(rf, rf_feedback, CONTROL_WHEEL_FF_KS_RF, CONTROL_WHEEL_FF_KV_RF, CONTROL_WHEEL_FF_KSTART_RF);
    lr_ff = control_wheel_ff(lr, lr_feedback, CONTROL_WHEEL_FF_KS_LR, CONTROL_WHEEL_FF_KV_LR, CONTROL_WHEEL_FF_KSTART_LR);
    rr_ff = control_wheel_ff(rr, rr_feedback, CONTROL_WHEEL_FF_KS_RR, CONTROL_WHEEL_FF_KV_RR, CONTROL_WHEEL_FF_KSTART_RR);

    mecanum_motor_set_all(
        (int16_t)(lf_ff + PositionalPID_Update(&wheel_left_front_pid, lf, lf_feedback)),
        (int16_t)(rf_ff + PositionalPID_Update(&wheel_right_front_pid, rf, rf_feedback)),
        (int16_t)(lr_ff + PositionalPID_Update(&wheel_left_rear_pid, lr, lr_feedback)),
        (int16_t)(rr_ff + PositionalPID_Update(&wheel_right_rear_pid, rr, rr_feedback)));

    // wifi_justfloat(lf, lf_feedback, lf_ff, wheel_left_front_pid.p_term, wheel_left_front_pid.i_term,
    //                rf, rf_feedback, rf_ff, wheel_right_front_pid.p_term, wheel_right_front_pid.i_term,
    //                lr, lr_feedback, lr_ff, wheel_left_rear_pid.p_term, wheel_left_rear_pid.i_term,
    //                rr, rr_feedback, rr_ff, wheel_right_rear_pid.p_term, wheel_right_rear_pid.i_term,
    //                g_euler.roll, g_euler.pitch, g_euler.yaw,
    //                g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,
    //                g_imufilter_1000hz.accx, g_imufilter_1000hz.accy);



}
