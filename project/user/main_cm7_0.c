#include "zf_common_headfile.h"
#include "../code/Protocols/crsf/crsf.h"

static void car_platform_init(void)
{
    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    SCB_DisableDCache();
    debug_init();                       // 调试串口信息初始化
}

int main(void)
{
    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
        wifi_justfloat(g_euler.yaw,
                       g_car_base_speed_command,
                       Left_Target_Speed, Right_Target_Speed,
                       g_car_speed_left_filtered, g_car_speed_right_filtered,
                       g_car_speed_left_brake_ff,
                       g_car_speed_right_brake_ff,
                       g_car_speed_left_motor_output,
                       g_car_speed_right_motor_output,
                       g_car_speed_brake_active,
                       -g_car_gyroz_target_dps, g_car_gyroz_feedback_dps,
                       g_car_gyroz_output,
                       g_car_yaw_target_deg,
                       g_car_world_velocity_x_command,
                       g_car_world_velocity_y_command,
                       g_car_world_speed_magnitude,
                       g_car_world_speed_limit,
                       g_car_world_heading_target_deg,
                       g_car_world_heading_error_deg,
                       g_car_world_alignment_scale,
                       g_car_world_body_speed_feedback,
                       g_car_world_reverse_active);
    }
}
