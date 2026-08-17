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

            /*
             * JustFloat channels:
             *  0 mode
             *  1 target_speed_mps, 2 target_yaw_deg
             *  3 left_target_speed, 4 right_target_speed
             *  5 left_speed_feedback, 6 right_speed_feedback
             *  7..11 left_speed P/I/D/FF/brake_FF
             * 12..16 right_speed P/I/D/FF/brake_FF
             * 17..19 Euler roll/pitch/yaw (deg)
             * 20..22 gyro x/y/z (dps), 23..25 accel x/y/z (g)
             * 26 target_yawrate, 27 feedback_yawrate (dps)
             * 28..31 yawrate P/I/FF/output
             * 32..34 yaw P/D/output
             * 35 negative_pressure_throttle, 36 left_motor, 37 right_motor
             */
            log_data[0] = (float)mode;
            log_data[1] = car_speed_encoder_cnt_to_mps(g_car_base_speed_command);
            log_data[2] = car_diag.yaw_target_deg;
            log_data[3] = Left_Target_Speed;
            log_data[4] = Right_Target_Speed;
            log_data[5] = g_car_speed_left_filtered;
            log_data[6] = g_car_speed_right_filtered;
            log_data[7] = car_diag.left_speed_p_term;
            log_data[8] = car_diag.left_speed_i_term;
            log_data[9] = car_diag.left_speed_d_term;
            log_data[10] = car_diag.left_speed_ff_term;
            log_data[11] = car_diag.left_brake_ff;
            log_data[12] = car_diag.right_speed_p_term;
            log_data[13] = car_diag.right_speed_i_term;
            log_data[14] = car_diag.right_speed_d_term;
            log_data[15] = car_diag.right_speed_ff_term;
            log_data[16] = car_diag.right_brake_ff;
            log_data[17] = g_euler.roll;
            log_data[18] = g_euler.pitch;
            log_data[19] = g_euler.yaw;
            log_data[20] = g_imufilter_1000hz.gyrox;
            log_data[21] = g_imufilter_1000hz.gyroy;
            log_data[22] = g_imufilter_1000hz.gyroz;
            log_data[23] = g_imufilter_1000hz.accx;
            log_data[24] = g_imufilter_1000hz.accy;
            log_data[25] = g_imufilter_1000hz.accz;
            log_data[26] = car_diag.gyroz_target_dps;
            log_data[27] = g_car_gyroz_feedback_dps;
            log_data[28] = car_diag.gyroz_p_term;
            log_data[29] = car_diag.gyroz_i_term;
            log_data[30] = car_diag.gyroz_ff_term;
            log_data[31] = car_diag.gyroz_output;
            log_data[32] = car_diag.yaw_p_term;
            log_data[33] = car_diag.yaw_d_term;
            log_data[34] = car_diag.yaw_output;
            log_data[35] = g_car_negative_pressure_throttle;
            log_data[36] = g_car_speed_left_motor_output;
            log_data[37] = g_car_speed_right_motor_output;
            (void)wifi_justfloat_Array(log_data, CAR_MODE45_DEBUG_FLOAT_COUNT);
        }
    }
}
