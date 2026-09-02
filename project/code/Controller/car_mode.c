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

void car_mode_get_diag(car_drive_diag_t *diag)
{
    if (s_car_control_enabled != 0U)
    {
        switch (s_car_mode)
        {
            case CAR_MODE_1:
                car_mode1_get_diag(diag);
                return;
            case CAR_MODE_2:
                car_mode2_get_diag(diag);
                return;
            case CAR_MODE_4:
                car_mode4_get_diag(diag);
                return;
            case CAR_MODE_5:
                car_mode5_get_diag(diag);
                return;
            default:
                break;
        }
    }

    diag->yaw_target_deg = 0.0f;
    diag->yaw_error_deg = 0.0f;
    diag->gyroz_target_dps = 0.0f;
    diag->gyroz_output = 0.0f;
    diag->large_turn_state = 0U;
    diag->large_turn_rearm_required = 0U;
    diag->speed_brake_active = 0U;
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
            car_mode0_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_1:
            car_mode1_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_2:
            car_mode2_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_3:
            car_mode3_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_4:
            car_mode4_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_5:
            car_mode5_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_6:
            car_mode6_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_7:
            car_mode7_update_100HZ(now_ms);
            return 1U;
        case CAR_MODE_8:
            car_mode8_update_100HZ(now_ms);
            return 1U;
        default:
            return 0U;
    }
}
