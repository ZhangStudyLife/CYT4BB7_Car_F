#include "car_mode.h"

#define CAR_MODE_CH4_ENABLE_THRESHOLD (0.5f)
#define CAR_MODE_SWITCH_LOW_THRESHOLD  (0.5f)
#define CAR_MODE_SWITCH_HIGH_THRESHOLD (1.5f)

static volatile car_mode_e s_car_mode = CAR_MODE_0;
static volatile uint8 s_car_control_enabled = 0U;
static car_mode_e s_last_car_mode = CAR_MODE_0;
static uint8 s_last_control_enabled = 0U;

static uint8 car_mode_switch_position(float value)
{
    if (value > CAR_MODE_SWITCH_HIGH_THRESHOLD)
    {
        return 2U;
    }
    if (value < CAR_MODE_SWITCH_LOW_THRESHOLD)
    {
        return 0U;
    }
    return 1U;
}

static car_mode_e car_mode_from_ch5_ch6(float ch5, float ch6)
{
    return (car_mode_e)(car_mode_switch_position(ch5) * 3U +
                        car_mode_switch_position(ch6));
}

static void car_mode_reset_all(void)
{
    car_mode0_reset();
    car_mode1_reset();
    car_mode2_reset();
    car_mode3_reset();
    car_mode4_reset();
    car_mode5_reset();
    car_mode6_reset();
    car_mode7_reset();
    car_mode8_reset();
}

void car_mode_init(void)
{
    car_mode0_init();
    car_mode1_init();
    car_mode2_init();
    car_mode3_init();
    car_mode4_init();
    car_mode5_init();
    car_mode6_init();
    car_mode7_init();
    car_mode8_init();
    car_mode_reset();
}

void car_mode_reset(void)
{
    s_car_mode = CAR_MODE_0;
    s_last_car_mode = CAR_MODE_0;
    s_car_control_enabled = 0U;
    s_last_control_enabled = 0U;
    car_mode_reset_all();
}

void car_mode_reset_control(void)
{
    car_mode_reset_all();
}

car_mode_e car_mode_get(void)
{
    return s_car_mode;
}

uint8 car_mode_is_control_enabled(void)
{
    return s_car_control_enabled;
}

uint8 car_mode_update_100HZ(uint32 now_ms)
{
    car_mode_e mode;
    uint8 control_enabled;

    if (air_comm_car_is_run_data_fresh() == 0U)
    {
        mode = CAR_MODE_0;
        control_enabled = 0U;
    }
    else
    {
        mode = car_mode_from_ch5_ch6(g_air_std_ch5, g_air_std_ch6);
        control_enabled =
            (g_air_std_ch4 >= CAR_MODE_CH4_ENABLE_THRESHOLD) ? 1U : 0U;
    }

    s_car_mode = mode;
    s_car_control_enabled = control_enabled;
    if ((mode != s_last_car_mode) ||
        (control_enabled != s_last_control_enabled))
    {
        car_mode_reset_all();
    }
    s_last_car_mode = mode;
    s_last_control_enabled = control_enabled;

    if (control_enabled == 0U)
    {
        return 0U;
    }

    switch (mode)
    {
        case CAR_MODE_0:
            /* 8月9日临时使用 Mode0 挡位进入 Mode2 业务。 */
            car_mode2_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_1:
            /* 8月9日临时使用 Mode1 挡位进入 Mode2 业务。 */
            car_mode2_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_2:
            car_mode2_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_3:
            car_mode3_update_100HZ(now_ms);
            return 0U;
        case CAR_MODE_4:
            /* 8月9日临时使用 Mode4 挡位进入 Mode2 业务。 */
            car_mode2_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_5:
            car_mode5_update_100HZ(now_ms);
            return 0U;
        case CAR_MODE_6:
            car_mode6_update_100HZ(now_ms);
            return 0U;
        case CAR_MODE_7:
            car_mode7_update_100HZ(now_ms);
            return 0U;
        case CAR_MODE_8:
            car_mode8_update_100HZ(now_ms);
            return 0U;
        default:
            return 0U;
    }
}
