#include "zf_common_headfile.h"
#include "car_safety.h"

#define CAR_MODE1245_DEBUG_FLOAT_COUNT    (42U)

#if (CAR_MODE1245_DEBUG_FLOAT_COUNT > (WIFI_JUSTFLOAT_MAX_FLOAT_NUM - 1U))
#error "Mode1/2/4/5 debug channels exceed JustFloat protocol capacity"
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
    float log_data[CAR_MODE1245_DEBUG_FLOAT_COUNT];

    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
        if (g_car_realtime_diag.control_tick_count != last_log_control_tick)
        {
            last_log_control_tick = g_car_realtime_diag.control_tick_count;
            mode = car_mode_get();
            if ((mode != CAR_MODE_1) && (mode != CAR_MODE_2) &&
                (mode != CAR_MODE_4) && (mode != CAR_MODE_5))
            {
                continue;
            }

            memset(&car_diag, 0, sizeof(car_diag));
            car_mode_get_diag(&car_diag);

            log_data[0] = (float)mode;
            log_data[1] = (float)car_mode_is_control_enabled();
            log_data[2] = (float)g_car_realtime_diag.control_tick_count;
            log_data[3] = g_air_std_ch0;
            log_data[4] = g_air_std_ch1;
            log_data[5] = g_air_car_plan_valid;
            log_data[6] = g_air_car_plan_strafe_mps;
            log_data[7] = g_air_car_plan_forward_mps;
            log_data[8] = g_air_sync_time_ms;
            log_data[9] = g_car_yaw_feedback_deg;
            log_data[10] = car_diag.yaw_target_deg;
            log_data[11] = (float)car_diag.large_turn_state;
            log_data[12] = (float)car_diag.large_turn_rearm_required;
            log_data[13] = (float)car_diag.speed_brake_active;
            log_data[14] = g_car_base_speed_command;
            log_data[15] = g_car_base_speed_target;
            log_data[16] = (float)encoder_get_left_count();
            log_data[17] = (float)encoder_get_right_count();
            log_data[18] = g_car_speed_left_filtered;
            log_data[19] = g_car_speed_right_filtered;
            log_data[20] = Left_Target_Speed;
            log_data[21] = Right_Target_Speed;
            log_data[22] = g_car_speed_left_motor_output;
            log_data[23] = g_car_speed_right_motor_output;
            log_data[24] = car_diag.yaw_p_term;
            log_data[25] = car_diag.yaw_d_term;
            log_data[26] = car_diag.gyroz_target_dps;
            log_data[27] = g_imufilter_1000hz.gyroz;
            log_data[28] = g_car_gyroz_feedback_dps;
            log_data[29] = car_diag.gyroz_p_term;
            log_data[30] = car_diag.gyroz_i_term;
            log_data[31] = car_diag.gyroz_ff_term;
            log_data[32] = car_diag.left_speed_p_term;
            log_data[33] = car_diag.left_speed_i_term;
            log_data[34] = car_diag.left_speed_d_term;
            log_data[35] = car_diag.left_speed_ff_term;
            log_data[36] = car_diag.left_brake_ff;
            log_data[37] = car_diag.right_speed_p_term;
            log_data[38] = car_diag.right_speed_i_term;
            log_data[39] = car_diag.right_speed_d_term;
            log_data[40] = car_diag.right_speed_ff_term;
            log_data[41] = car_diag.right_brake_ff;
            (void)wifi_justfloat_Array(log_data, CAR_MODE1245_DEBUG_FLOAT_COUNT);
        }
    }
}
