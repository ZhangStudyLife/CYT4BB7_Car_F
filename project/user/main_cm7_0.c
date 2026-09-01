#include "zf_common_headfile.h"
#include "car_safety.h"

#define CAR_MODE45_DEBUG_FLOAT_COUNT    (38U)

#if (CAR_MODE45_DEBUG_FLOAT_COUNT > (WIFI_JUSTFLOAT_MAX_FLOAT_NUM - 1U))
#error "Mode4/5 debug channels exceed JustFloat protocol capacity"
#endif

static void car_platform_init(void)
{
    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    SCB_DisableDCache();
    debug_init();                       // 调试串口信息初始化
}

int main(void)
{
    car_drive_diag_t car_diag;
    car_mode_e mode;
    uint32 last_log_control_tick = 0U;
    float log_data[CAR_MODE45_DEBUG_FLOAT_COUNT];

    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
        if (g_car_realtime_diag.control_tick_count != last_log_control_tick)
        {
            last_log_control_tick = g_car_realtime_diag.control_tick_count;
            mode = car_mode_get();
            memset(&car_diag, 0, sizeof(car_diag));
            car_mode_get_diag(&car_diag);

        }
    }
}
