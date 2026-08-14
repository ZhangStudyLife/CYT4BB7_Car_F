#include "car_mode.h"
#include <math.h>

#define MODE1_TARGET_SPEED_LIMIT_MPS (3.0f) /* Mode1车体速度目标模长上限，单位m/s。 */

void car_mode1_init(void)
{
    car_mode1_reset();
}

void car_mode1_reset(void)
{
}

void car_mode1_update_100HZ(uint32 now_ms)
{
    float strafe_mps = g_air_car_plan_strafe_mps;
    float forward_mps = g_air_car_plan_forward_mps;
    float speed_mps = sqrtf(strafe_mps * strafe_mps +
                            forward_mps * forward_mps);
    uint8 command_valid = ((g_air_car_plan_valid >= 0.5f) &&
                           (speed_mps > 0.0f)) ? 1U : 0U;

    if (speed_mps > MODE1_TARGET_SPEED_LIMIT_MPS)
    {
        strafe_mps *= MODE1_TARGET_SPEED_LIMIT_MPS / speed_mps;
        forward_mps *= MODE1_TARGET_SPEED_LIMIT_MPS / speed_mps;
    }

    car_mode2_update_body_100HZ(now_ms, strafe_mps, forward_mps,
                                command_valid);
}
