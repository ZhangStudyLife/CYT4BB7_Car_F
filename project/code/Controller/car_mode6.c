/* Mode6：飞机遥控器直控模式
 * 数据流：飞机串口 → g_air_crsf_std_ch → car_forward/strafe_target
 * ch0(Roll) → 左右平移, ch1(Pitch) → 前后，yaw由控制层锁0
 * ch4为总开关：1=运行, 0=不运行
 */
#include "car_mode.h"
#include "car_loop.h"
#include "Common/car_math.h"

#define MODE6_MAX_CONTROL_SPEED (200.0f)
#define MODE6_STICK_DEADBAND    (50.0f)

void car_mode6_init(void)
{
}

void car_mode6_reset(void)
{
}

void car_mode6_update_25HZ(uint32 now_ms)
{
    (void)now_ms;

    if (g_air_crsf_std_ch4 < 0.5f)
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    car_forward_target = car_math_soft_deadband(g_air_crsf_std_ch1, MODE6_STICK_DEADBAND) *
                         (MODE6_MAX_CONTROL_SPEED / 1000.0f);
    car_strafe_target  = car_math_soft_deadband(g_air_crsf_std_ch0, MODE6_STICK_DEADBAND) *
                         (MODE6_MAX_CONTROL_SPEED / 1000.0f);
}
