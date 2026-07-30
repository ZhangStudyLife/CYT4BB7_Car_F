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
        wifi_justfloat(g_euler.roll, g_euler.pitch, g_euler.yaw,
                       g_car_base_speed_command,
                       g_car_base_speed_target,
                       g_car_base_speed_delta,
                       g_car_speed_accel_ff,
                       Left_Target_Speed, Right_Target_Speed,
                       g_car_speed_left_filtered, g_car_speed_right_filtered,
                       Left_Speed_PID.incremental_output, Right_Speed_PID.incremental_output,
                       Left_Speed_PID.output, Right_Speed_PID.output,
                       g_car_speed_left_brake_ff,
                       g_car_speed_right_brake_ff,
                       g_car_speed_left_motor_output,
                       g_car_speed_right_motor_output,
                       g_car_speed_brake_active,
                       -g_car_gyroz_target_dps, g_car_gyroz_feedback_dps,
                       g_car_gyroz_p_term, g_car_gyroz_i_term,
                       g_car_gyroz_ff_term,
                       g_car_gyroz_output,
                       g_car_yaw_target_deg,
                       g_car_yaw_p_term, g_car_yaw_d_term,
                       car_yaw_control_mode);
    }
}
