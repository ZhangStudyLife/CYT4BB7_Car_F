#include "zf_common_headfile.h"
#include "car_safety.h"

static void car_platform_init(void)
{
    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    SCB_DisableDCache();
    debug_init();                       // 调试串口信息初始化
}

int main(void)
{
    car_mode2_diag_t mode2_diag;

    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
        car_mode2_get_diag(&mode2_diag);

        wifi_justfloat(car_mode_get(),                                      /* I1  car mode */
                       car_mode_is_control_enabled(),                       /* I2  control enabled */
                       car_safety_is_output_allowed(),                      /* I3  output allowed */
                       car_safety_get_fault(),                              /* I4  safety fault */
                       air_comm_car_is_run_data_fresh(),                    /* I5  air data fresh */
                       g_air_state,                                         /* I6  aircraft state */
                       g_air_std_ch0, g_air_std_ch1, g_air_std_ch2,          /* I7-I9   CRSF ch0-ch2 */
                       g_air_std_ch3, g_air_std_ch4, g_air_std_ch5,          /* I10-I12 CRSF ch3-ch5 */
                       g_air_std_ch6, g_air_std_ch7, g_air_std_ch8,          /* I13-I15 CRSF ch6-ch8 */
                       g_air_yaw_angle_target_deg,                           /* I16 aircraft yaw target, deg */
                       g_air_sync_time_ms,                                  /* I17 aircraft time, ms */
                       g_air_car_plan_valid,                                /* I18 car plan valid */
                       g_air_car_plan_strafe_mps,                           /* I19 plan strafe, m/s */
                       g_air_car_plan_forward_mps,                          /* I20 plan forward, m/s */
                       g_air_beacon_lost_flag,                              /* I21 beacon lost */
                       tick_1000us_cnt,                                     /* I22 car air-data time, ms */
                       g_euler.roll, g_euler.pitch, g_euler.yaw,             /* I23-I25 car Euler, deg */
                       g_imufilter_1000hz.accx,                              /* I26 car IMU acc X, g */
                       g_imufilter_1000hz.accy,                              /* I27 car IMU acc Y, g */
                       g_imufilter_1000hz.accz,                              /* I28 car IMU acc Z, g */
                       g_imufilter_1000hz.gyroz,                             /* I29 car horizontal yaw rate, deg/s */
                       mode2_diag.yaw_target_deg,                            /* I30 car yaw target, deg */
                       g_car_yaw_feedback_deg,                              /* I31 car yaw feedback, deg */
                       car_speed_encoder_cnt_to_mps(g_car_base_speed_command), /* I32 raw speed command, m/s */
                       car_speed_encoder_cnt_to_mps(g_car_base_speed_target),  /* I33 speed-loop target, m/s */
                       car_speed_encoder_cnt_to_mps(0.5f *
                           (g_car_speed_left_filtered + g_car_speed_right_filtered)), /* I34 speed feedback, m/s */
                       mode2_diag.gyroz_target_dps,                          /* I35 yaw-rate target, deg/s */
                       g_car_gyroz_feedback_dps);                            /* I36 yaw-rate feedback, deg/s */
    }
}
