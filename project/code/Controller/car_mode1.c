#include "car_mode.h"
#include <math.h>

#define MODE1_TARGET_SPEED_LIMIT_MPS (3.0f) /* Mode1车体速度目标模长上限，单位m/s。 */
#define MODE1_YAW_LPF_ALPHA          (0.20f)
#define MODE1_YAW_DEADBAND_DEG       (1.0f)

static uint8 s_mode1_yaw_filter_initialized;
static float s_mode1_yaw_target_filtered_deg;

static float mode1_wrap_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void car_mode1_init(void)
{
    car_mode1_reset();
}

void car_mode1_reset(void)
{
    s_mode1_yaw_filter_initialized = 0U;
    s_mode1_yaw_target_filtered_deg = 0.0f;
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
        speed_mps = MODE1_TARGET_SPEED_LIMIT_MPS;
    }

    if (command_valid != 0U)
    {
        float raw_yaw_deg = mode1_wrap_deg(
            g_car_yaw_feedback_deg + atan2f(strafe_mps, forward_mps) *
                                         57.29577951308232f);
        float yaw_delta_deg;
        float relative_yaw_rad;

        if (s_mode1_yaw_filter_initialized == 0U)
        {
            s_mode1_yaw_target_filtered_deg = raw_yaw_deg;
            s_mode1_yaw_filter_initialized = 1U;
        }
        else
        {
            yaw_delta_deg = mode1_wrap_deg(
                raw_yaw_deg - s_mode1_yaw_target_filtered_deg);
            if (fabsf(yaw_delta_deg) > MODE1_YAW_DEADBAND_DEG)
            {
                s_mode1_yaw_target_filtered_deg = mode1_wrap_deg(
                    s_mode1_yaw_target_filtered_deg +
                    MODE1_YAW_LPF_ALPHA * yaw_delta_deg);
            }
        }

        relative_yaw_rad = mode1_wrap_deg(
            s_mode1_yaw_target_filtered_deg - g_car_yaw_feedback_deg) *
                           0.017453292519943295f;
        strafe_mps = speed_mps * sinf(relative_yaw_rad);
        forward_mps = speed_mps * cosf(relative_yaw_rad);
    }
    else
    {
        s_mode1_yaw_filter_initialized = 0U;
    }

    car_mode2_update_body_100HZ(now_ms, strafe_mps, forward_mps,
                                command_valid);
}
