#include "car_mode.h"
#include "car_loop.h"

#define CAR_MODE_CH4_ENABLE_THRESHOLD (0.5f)
#define CAR_MODE_SW_LOW   (0.5f)
#define CAR_MODE_SW_HIGH  (1.5f)

static car_mode_e s_car_mode = CAR_MODE_0;
static car_mode_e s_last_car_mode = CAR_MODE_0;
static uint8 s_last_control_enabled = 0U;

static car_mode_e car_mode_from_ch5_ch6(float ch5, float ch6)
{
    uint8 p5 = (ch5 > CAR_MODE_SW_HIGH) ? 2U : ((ch5 < CAR_MODE_SW_LOW) ? 0U : 1U);
    uint8 p6 = (ch6 > CAR_MODE_SW_HIGH) ? 2U : ((ch6 < CAR_MODE_SW_LOW) ? 0U : 1U);
    return (car_mode_e)(p5 * 3U + p6);
}

static uint8 car_mode_allows_output(car_mode_e mode)
{
    return ((CAR_MODE_3 == mode) ||
            (CAR_MODE_4 == mode) ||
            (CAR_MODE_2 == mode) ||
            (CAR_MODE_5 == mode) ||
            (CAR_MODE_6 == mode) ||
            (CAR_MODE_7 == mode) ||
            (CAR_MODE_8 == mode)) ? 1U : 0U;
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
    Control_Reset();
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
}

static void car_mode_handle_transition_25HZ(car_mode_e mode, uint8 control_enabled)
{
    uint8 need_reset = 0U;

    if(s_last_car_mode != mode)
    {
        need_reset = 1U;
    }

    if(s_last_control_enabled != control_enabled)
    {
        need_reset = 1U;
    }

    if(0U != need_reset)
    {
        car_mode_reset_all();
    }

    s_last_car_mode = mode;
    s_last_control_enabled = control_enabled;
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
    s_last_control_enabled = 0U;
    car_control_enabled = 0U;
    car_emergency_stop_active = 1U;
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
    car_mode_reset_all();
}

car_mode_e car_mode_get(void)
{
    return s_car_mode;
}

void car_mode_update_25HZ(uint32 now_ms)
{
    if(air_comm_car_is_run_data_fresh() == 0U)
    {
        car_control_enabled = 0U;
        s_car_mode = CAR_MODE_0;
        car_emergency_stop_active = 1U;
        car_mode_handle_transition_25HZ(s_car_mode, car_control_enabled);
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    /* 信标采集期间固定复用 Mode8 遥控，只由 Air 转发的 CH4 停车开关放行。 */
    if(beacon_position_recorder_is_active() != 0U)
    {
        car_control_enabled =
            (g_air_crsf_std_ch4 >= CAR_MODE_CH4_ENABLE_THRESHOLD) ? 1U : 0U;
        s_car_mode = (car_control_enabled != 0U) ? CAR_MODE_8 : CAR_MODE_0;
        car_emergency_stop_active = (car_control_enabled != 0U) ? 0U : 1U;
        car_mode_handle_transition_25HZ(s_car_mode, car_control_enabled);

        if(car_control_enabled == 0U)
        {
            car_forward_target = 0.0f;
            car_strafe_target = 0.0f;
            return;
        }

        car_mode8_update_25HZ(now_ms);
        return;
    }

    car_control_enabled = (g_air_crsf_std_ch4 >= CAR_MODE_CH4_ENABLE_THRESHOLD) ? 1U : 0U;
    if(0U == car_control_enabled)
    {
        s_car_mode = CAR_MODE_0;
        car_emergency_stop_active = 1U;
        car_mode_handle_transition_25HZ(s_car_mode, car_control_enabled);
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    s_car_mode = car_mode_from_ch5_ch6(g_air_crsf_std_ch5, g_air_crsf_std_ch6);
    car_emergency_stop_active = (0U != car_mode_allows_output(s_car_mode)) ? 0U : 1U;
    car_mode_handle_transition_25HZ(s_car_mode, car_control_enabled);

    if(0U != car_emergency_stop_active)
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    switch(s_car_mode)
    {
    case CAR_MODE_2:
        car_mode2_update_25HZ(now_ms);
        break;

    case CAR_MODE_3:
        car_mode3_update_25HZ(now_ms);
        break;

    case CAR_MODE_4:
        car_mode4_update_25HZ(now_ms);
        break;

    case CAR_MODE_5:
        car_mode5_update_25HZ(now_ms);
        break;

    case CAR_MODE_6:
        car_mode6_update_25HZ(now_ms);
        break;

    case CAR_MODE_8:
        car_mode8_update_25HZ(now_ms);
        break;

    default:
        break;
    }
}

void car_mode_update_100HZ(uint32 now_ms)
{
    if((0U == car_control_enabled) || (0U != car_emergency_stop_active))
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    switch(s_car_mode)
    {
    case CAR_MODE_2:
        car_mode2_update_100HZ(now_ms);
        break;

    case CAR_MODE_3:
        car_mode3_update_100HZ(now_ms);
        break;

    case CAR_MODE_4:
        car_mode4_update_100HZ(now_ms);
        break;

    case CAR_MODE_5:
        car_mode5_update_100HZ(now_ms);
        break;

    case CAR_MODE_7:
        car_mode7_update_100HZ(now_ms);
        break;

    case CAR_MODE_8:
        car_mode8_update_100HZ(now_ms);
        break;

    default:
        break;
    }
}
