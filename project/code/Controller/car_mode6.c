/* Mode6：两轮差速遥控直控模式。
 * 数据流：飞机串口 → g_air_crsf_std_ch → 线速度/角速度目标。
 * ch0(Roll)控制转向角速度，ch1(Pitch)控制前后线速度。
 * ch4为总开关：1=运行，0=停止。
 */
#include "car_mode.h"
#include "car_loop.h"
#include "Common/car_math.h"

#define MODE6_MAX_LINEAR_MPS       (2.0f)
#define MODE6_MAX_YAW_RATE_RAD_S   (3.0f)
#define MODE6_STICK_MAX            (1000.0f)
#define MODE6_STICK_DEADBAND       (50.0f)
#define MODE6_STICK_ACTIVE_RANGE   (MODE6_STICK_MAX - MODE6_STICK_DEADBAND)

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
        car_yaw_rate_target = 0.0f;
        return;
    }

    car_forward_target =
        car_math_soft_deadband(
            car_math_limit_absf(g_air_crsf_std_ch1, MODE6_STICK_MAX),
            MODE6_STICK_DEADBAND) *
        (MODE6_MAX_LINEAR_MPS / MODE6_STICK_ACTIVE_RANGE);

    /*
     * 遥控Roll正值表示向右打方向，而控制层约定俯视逆时针角速度为正，
     * 因此向右打方向对应顺时针（负角速度）。
     */
    car_yaw_rate_target =
        -car_math_soft_deadband(
            car_math_limit_absf(g_air_crsf_std_ch0, MODE6_STICK_MAX),
            MODE6_STICK_DEADBAND) *
        (MODE6_MAX_YAW_RATE_RAD_S / MODE6_STICK_ACTIVE_RANGE);
}
