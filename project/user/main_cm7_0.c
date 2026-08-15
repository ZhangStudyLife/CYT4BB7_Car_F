#include "zf_common_headfile.h"
#include "car_safety.h"

#define CAR_MODE12_DEBUG_FLOAT_COUNT      (48U)

#if (CAR_MODE12_DEBUG_FLOAT_COUNT > (WIFI_JUSTFLOAT_MAX_FLOAT_NUM - 1U))
#error "Mode1/2 debug channels exceed JustFloat protocol capacity"
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
    uint32 last_log_control_tick = 0U;
    float log_data[CAR_MODE12_DEBUG_FLOAT_COUNT];

    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
        if (g_car_realtime_diag.control_tick_count != last_log_control_tick)
        {
            last_log_control_tick = g_car_realtime_diag.control_tick_count;
            memset(&car_diag, 0, sizeof(car_diag));
            car_mode_get_diag(&car_diag);

            log_data[0] = (float)car_mode_get();                 /* I1 当前模式。 */
            log_data[1] = (float)car_mode_is_control_enabled();  /* I2 控制使能。 */
            log_data[2] = (car_mode_get() == CAR_MODE_1) ?
                              g_air_car_plan_strafe_mps : g_air_std_ch0; /* I3 Mode1规划横移m/s；Mode2遥控ch0。 */
            log_data[3] = (car_mode_get() == CAR_MODE_1) ?
                              g_air_car_plan_forward_mps : g_air_std_ch1; /* I4 Mode1规划前进m/s；Mode2遥控ch1。 */
            log_data[4] = g_car_yaw_feedback_deg;                /* I5 车Yaw反馈，deg。 */
            log_data[5] = car_diag.yaw_target_deg;               /* I6 车Yaw目标，deg。 */
            log_data[6] = car_diag.yaw_error_deg;                /* I7 Yaw误差，deg。 */
            log_data[7] = (float)car_diag.large_turn_state;      /* I8 大转状态：0正常1刹车2自旋3退出。 */
            log_data[8] = (float)car_diag.large_turn_direction;  /* I9 大转方向：-1/0/1。 */
            log_data[9] = (float)car_diag.large_turn_rearm_required; /* I10 大转重新使能标志。 */
            log_data[10] = (float)car_diag.large_turn_trigger_cycles; /* I11 大转触发计数。 */
            log_data[11] = (float)car_diag.large_turn_finish_cycles;  /* I12 大转完成计数。 */
            log_data[12] = (float)car_diag.large_turn_elapsed_cycles; /* I13 大转持续计数。 */
            log_data[13] = car_diag.large_turn_target_yaw_deg;    /* I14 大转锁存Yaw目标，deg。 */
            log_data[14] = car_diag.large_turn_target_speed_mps;  /* I15 大转锁存平移速度，m/s。 */
            log_data[15] = g_car_base_speed_command;              /* I16 平移速度指令滤波前，编码器count。 */
            log_data[16] = g_car_base_speed_target;               /* I17 平移速度规划后，编码器count。 */
            log_data[17] = (float)encoder_get_left_count();       /* I18 左轮速度滤波前，count/10ms。 */
            log_data[18] = g_car_speed_left_filtered;             /* I19 左轮速度滤波后，count/10ms。 */
            log_data[19] = (float)encoder_get_right_count();      /* I20 右轮速度滤波前，count/10ms。 */
            log_data[20] = g_car_speed_right_filtered;            /* I21 右轮速度滤波后，count/10ms。 */
            log_data[21] = Left_Target_Speed;                     /* I22 左轮目标速度，count/10ms。 */
            log_data[22] = Right_Target_Speed;                    /* I23 右轮目标速度，count/10ms。 */
            log_data[23] = g_car_speed_left_motor_output;         /* I24 左电机最终输出。 */
            log_data[24] = g_car_speed_right_motor_output;        /* I25 右电机最终输出。 */
            log_data[25] = car_diag.yaw_p_term;                   /* I26 Yaw角度环P项，deg/s。 */
            log_data[26] = car_diag.yaw_d_term;                   /* I27 Yaw角度环D项，deg/s。 */
            log_data[27] = car_diag.yaw_output;                   /* I28 Yaw角度环PID输出，deg/s。 */
            log_data[28] = car_diag.gyroz_target_dps;             /* I29 角速度环目标，deg/s。 */
            log_data[29] = g_imufilter_1000hz.gyroz;              /* I30 角速度二次滤波前，deg/s。 */
            log_data[30] = g_car_gyroz_feedback_dps;              /* I31 角速度二次滤波后，deg/s。 */
            log_data[31] = car_diag.gyroz_p_term;                 /* I32 角速度环P项。 */
            log_data[32] = car_diag.gyroz_i_term;                 /* I33 角速度环I项。 */
            log_data[33] = car_diag.gyroz_ff_term;                /* I34 角速度环前馈项。 */
            log_data[34] = car_diag.gyroz_output;                 /* I35 角速度环总输出。 */
            log_data[35] = car_diag.left_speed_p_term;            /* I36 左速度环P增量。 */
            log_data[36] = car_diag.left_speed_i_term;            /* I37 左速度环I增量。 */
            log_data[37] = car_diag.left_speed_d_term;            /* I38 左速度环D增量。 */
            log_data[38] = car_diag.left_speed_ff_term;           /* I39 左速度环静态前馈。 */
            log_data[39] = car_diag.left_brake_ff;                /* I40 左轮实际制动前馈贡献。 */
            log_data[40] = car_diag.right_speed_p_term;           /* I41 右速度环P增量。 */
            log_data[41] = car_diag.right_speed_i_term;           /* I42 右速度环I增量。 */
            log_data[42] = car_diag.right_speed_d_term;           /* I43 右速度环D增量。 */
            log_data[43] = car_diag.right_speed_ff_term;          /* I44 右速度环静态前馈。 */
            log_data[44] = car_diag.right_brake_ff;               /* I45 右轮实际制动前馈贡献。 */
            log_data[45] = (float)car_diag.speed_brake_active;    /* I46 常规停车制动状态。 */
            log_data[46] = car_speed_encoder_cnt_to_mps(
                0.5f * (g_car_speed_left_filtered +
                        g_car_speed_right_filtered));             /* I47 实际下发飞机的前进速度，m/s。 */
            log_data[47] = g_air_sync_time_ms;                    /* I48 飞机同步时间，ms。 */
            (void)wifi_justfloat_Array(log_data, CAR_MODE12_DEBUG_FLOAT_COUNT);
        }
    }
}
