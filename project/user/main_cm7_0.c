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
    car_drive_diag_t car_diag;
    uint32 last_log_control_tick = 0U;

    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
        if (g_car_realtime_diag.control_tick_count != last_log_control_tick)
        {
            last_log_control_tick = g_car_realtime_diag.control_tick_count;
            car_mode_get_diag(&car_diag);
            wifi_justfloat(car_mode_get(), car_mode_is_control_enabled(),    /* I1-I2 mode/control */
                           car_safety_is_output_allowed(), car_safety_get_fault(), /* I3-I4 safety */
                           air_comm_car_is_run_data_fresh(), g_air_sync_time_ms, /* I5-I6 air link/time */
                           g_air_car_plan_valid,                             /* I7  plan valid */
                           g_air_car_plan_strafe_mps,                        /* I8  plan strafe, m/s */
                           g_air_car_plan_forward_mps,                       /* I9  plan forward, m/s */
                           g_air_std_ch0, g_air_std_ch1,                     /* I10-I11 manual command */
                           g_car_yaw_feedback_deg, car_diag.yaw_target_deg, /* I12-I13 yaw, deg */
                           car_diag.large_turn_state,                      /* I14 large-turn state */
                           car_diag.large_turn_rearm_required,             /* I15 large-turn rearm */
                           car_diag.speed_brake_active,                    /* I16 brake state */
                           car_diag.gyroz_target_dps,                       /* I17 yaw-rate target */
                           g_car_gyroz_feedback_dps,                         /* I18 yaw-rate feedback */
                           g_imufilter_1000hz.gyroz, car_diag.gyroz_output, /* I19-I20 gyro/output */
                           car_speed_encoder_cnt_to_mps(g_car_base_speed_command), /* I21 speed command */
                           car_speed_encoder_cnt_to_mps(g_car_base_speed_target),  /* I22 speed target */
                           car_speed_encoder_cnt_to_mps(Left_Target_Speed),  /* I23 left target */
                           car_speed_encoder_cnt_to_mps(Right_Target_Speed), /* I24 right target */
                           car_speed_encoder_cnt_to_mps(g_car_speed_left_filtered),  /* I25 left speed */
                           car_speed_encoder_cnt_to_mps(g_car_speed_right_filtered), /* I26 right speed */
                           encoder_get_left_count(), encoder_get_right_count(), /* I27-I28 encoder */
                           g_car_speed_left_motor_output,                    /* I29 left motor output */
                           g_car_speed_right_motor_output,                   /* I30 right motor output */
                           g_euler.roll, g_euler.pitch);                     /* I31-I32 Euler, deg */
        }
    }
}
