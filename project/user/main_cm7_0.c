/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
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
