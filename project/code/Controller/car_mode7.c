/*
 * Mode7：两轮差速遥控速度模式。
 *
 * 与Mode6使用相同的物理量控制链路，区别是Mode7在100Hz对遥控目标做一阶
 * 低通，适合调试“遥控目标平滑 -> 角速度环 -> 轮速环”的完整闭环。
 *
 * ch1(Pitch)：前后线速度，单位m/s。
 * ch0(Roll)：转向角速度，右转为负，单位rad/s。
 */
#include "car_mode.h"
#include "car_loop.h"

#define MODE7_MAX_LINEAR_MPS       (2.0f)
#define MODE7_MAX_YAW_RATE_RAD_S   (3.0f)
#define MODE7_STICK_DEADBAND       (50.0f)
#define MODE7_STICK_MAX            (1000.0f)
#define MODE7_STICK_ACTIVE_RANGE   (MODE7_STICK_MAX - MODE7_STICK_DEADBAND)

car_mode7_state_t g_car_mode7_state = {0};

static float car_mode7_stick_to_target(float stick, float max_target)
{
    stick = car_math_limit_absf(stick, MODE7_STICK_MAX);
    stick = car_math_soft_deadband(stick, MODE7_STICK_DEADBAND);
    return stick * (max_target / MODE7_STICK_ACTIVE_RANGE);
}

void car_mode7_init(void)
{
    car_mode7_reset();
}

void car_mode7_reset(void)
{
    g_car_mode7_state = (car_mode7_state_t){0};
}

void car_mode7_update_100HZ(uint32 now_ms)
{
    (void)now_ms;

    g_car_mode7_state.raw_forward_mps =
        car_mode7_stick_to_target(g_air_crsf_std_ch1,
                                  MODE7_MAX_LINEAR_MPS);
    /*
     * 状态结构中的strafe字段为兼容现有菜单诊断而保留；在两轮模式下，
     * 这些字段存放角速度目标，单位由原m/s改为rad/s。
     */
    g_car_mode7_state.raw_strafe_mps =
        -car_mode7_stick_to_target(g_air_crsf_std_ch0,
                                   MODE7_MAX_YAW_RATE_RAD_S);

    g_car_mode7_state.velocity_forward_target_mps =
        car_filter_lpf1_apply(g_car_mode7_state.velocity_forward_target_mps,
                              g_car_mode7_state.raw_forward_mps,
                              ODOMETER_UPDATE_DT_S,
                              mode7_velocity_smooth_tau_s);
    g_car_mode7_state.velocity_strafe_target_mps =
        car_filter_lpf1_apply(g_car_mode7_state.velocity_strafe_target_mps,
                              g_car_mode7_state.raw_strafe_mps,
                              ODOMETER_UPDATE_DT_S,
                              mode7_velocity_smooth_tau_s);

    g_car_mode7_state.velocity_forward_feedback_mps =
        g_odometer.body_vel[y];
    g_car_mode7_state.velocity_strafe_feedback_mps =
        -g_imufilter_1000hz.gyroz * 0.017453292519943295f;

    g_car_mode7_state.forward_target =
        g_car_mode7_state.velocity_forward_target_mps;
    g_car_mode7_state.strafe_target =
        g_car_mode7_state.velocity_strafe_target_mps;
    g_car_mode7_state.output_valid = 1U;

    car_forward_target = g_car_mode7_state.forward_target;
    car_yaw_rate_target = g_car_mode7_state.strafe_target;
}
