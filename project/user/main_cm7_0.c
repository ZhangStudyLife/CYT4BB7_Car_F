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

        wifi_justfloat(g_air_std_ch0,
                        g_air_std_ch1,
                        g_air_std_ch4,
                        g_air_std_ch5,
                        g_air_std_ch6,
                        car_mode_get(),
                        car_mode_is_control_enabled(),
                        car_safety_is_output_allowed(),
                        car_safety_get_fault(),
                        g_euler.roll,
                        g_euler.pitch,
                        g_euler.yaw,
                        g_car_gyroz_feedback_dps,
                        mode2_diag.yaw_target_deg,
                        mode2_diag.yaw_error_deg,
                        mode2_diag.gyroz_target_dps,
                        mode2_diag.gyroz_output,
                        mode2_diag.large_turn_state,
                        mode2_diag.large_turn_rearm_required,
                        mode2_diag.speed_brake_active,
                        car_speed_encoder_cnt_to_mps(g_car_base_speed_command),
                        car_speed_encoder_cnt_to_mps(g_car_base_speed_target),
                        car_speed_encoder_cnt_to_mps(Left_Target_Speed),
                        car_speed_encoder_cnt_to_mps(Right_Target_Speed),
                        car_speed_encoder_cnt_to_mps(g_car_speed_left_filtered),
                        car_speed_encoder_cnt_to_mps(g_car_speed_right_filtered),
                        encoder_get_left_count(),
                        encoder_get_right_count(),
                        g_car_speed_left_motor_output,
                        g_car_speed_right_motor_output);
    }
}
