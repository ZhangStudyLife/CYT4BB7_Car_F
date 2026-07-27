#include "zf_common_headfile.h"
#include "../code/Protocols/crsf/crsf.h"

static void car_platform_init(void)
{
    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    SCB_DisableDCache();
    debug_init();                       // 调试串口信息初始化
}

int main(void)
{
    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
        wifi_justfloat(g_euler.roll, g_euler.pitch, g_euler.yaw,
                    encoder_get_left_count(), encoder_get_right_count(),
                    g_car_speed_left_filtered, g_car_speed_right_filtered,
                    Left_Target_Speed, Right_Target_Speed,
                    Left_Speed_PID.p_term, Left_Speed_PID.i_term, Left_Speed_PID.d_term,
                    Right_Speed_PID.p_term, Right_Speed_PID.i_term, Right_Speed_PID.d_term,
                    (int16_t)Left_Speed_PID.output, (int16_t)Right_Speed_PID.output);
        // wifi_justfloat(g_euler.roll, g_euler.pitch, g_euler.yaw,
        //                encoder_get_left_count(), encoder_get_right_count(),
        //                g_car_speed_left_filtered, g_car_speed_right_filtered,
        //                Left_Target_Speed, Right_Target_Speed,
        //                Left_Speed_PID.p_term, Left_Speed_PID.i_term, Left_Speed_PID.d_term,
        //                Right_Speed_PID.p_term, Right_Speed_PID.i_term, Right_Speed_PID.d_term,
        //                (int16_t)Left_Speed_PID.output, (int16_t)Right_Speed_PID.output,
        //                g_car_gyroz_feedback_equivalent,
        //                g_car_gyroz_p_term, g_car_gyroz_i_term,
        //                g_car_gyroz_error, g_car_gyroz_output);
    }
}
