#include "zf_common_headfile.h"

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
                       encoder_get_left_count(), encoder_get_right_count());
    }
}
