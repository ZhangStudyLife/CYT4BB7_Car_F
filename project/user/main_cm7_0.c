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
    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();

        wifi_justfloat(g_euler.yaw,
                        g_air_std_ch0,
                        g_air_std_ch1,
                        g_air_std_ch2,
                        g_air_std_ch3,
                        g_air_std_ch4,
                        g_air_std_ch5,
                        g_air_std_ch6,
                        g_air_std_ch7,
                        g_air_std_ch8,
                        car_mode_get(),
                        car_mode_is_control_enabled(),
                        car_safety_is_output_allowed(),
                        car_safety_get_fault(),
                        g_car_base_speed_command,
                        g_car_base_speed_target,
                        Left_Target_Speed,
                        Right_Target_Speed);
    }
}
