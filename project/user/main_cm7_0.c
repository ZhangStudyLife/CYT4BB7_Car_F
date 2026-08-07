#include "zf_common_headfile.h"
#include "../code/Protocols/crsf/crsf.h"
#include <math.h>

#define CAR_REMOTE_RAD_TO_DEG (57.29577951308232f)

static uint8 s_car_remote_direction_active = 0U;
static float s_car_remote_direction_deg = 0.0f;

/*
 * 按小车世界坐标控制的同一约定计算摇杆方向：CH2为X轴，CH1为Y轴。
 * 返回范围为[-180°, 180°]；迟滞区间和回中状态保持上一次有效方向。
 */
static float car_remote_direction_deg_get(
    const crsf_control_snapshot_t *snapshot)
{
    float raw_x = (float)snapshot->channel[1];
    float raw_y = (float)snapshot->channel[0];
    float magnitude_sq = raw_x * raw_x + raw_y * raw_y;
    float enter_deadzone_sq = CAR_WORLD_INPUT_ENTER_DEADZONE *
                              CAR_WORLD_INPUT_ENTER_DEADZONE;
    float exit_deadzone_sq = CAR_WORLD_INPUT_EXIT_DEADZONE *
                             CAR_WORLD_INPUT_EXIT_DEADZONE;

    if (s_car_remote_direction_active != 0U)
    {
        if (magnitude_sq <= exit_deadzone_sq)
        {
            s_car_remote_direction_active = 0U;
        }
        else if (magnitude_sq >= enter_deadzone_sq)
        {
            s_car_remote_direction_deg =
                atan2f(raw_y, raw_x) * CAR_REMOTE_RAD_TO_DEG;
        }
    }
    else if (magnitude_sq >= enter_deadzone_sq)
    {
        s_car_remote_direction_active = 1U;
        s_car_remote_direction_deg =
            atan2f(raw_y, raw_x) * CAR_REMOTE_RAD_TO_DEG;
    }

    return s_car_remote_direction_deg;
}

static void car_platform_init(void)
{
    clock_init(SYSTEM_CLOCK_250M);      // 时钟配置及系统初始化<务必保留>
    SCB_DisableDCache();
    debug_init();                       // 调试串口信息初始化
}

int main(void)
{
    crsf_control_snapshot_t remote_snapshot;
    float remote_direction_deg;

    car_platform_init();
    car_loop_init();

    while(true)
    {
        car_loop_poll();
        CRSF_GetControlSnapshot(&remote_snapshot);
        remote_direction_deg =
            car_remote_direction_deg_get(&remote_snapshot);

        wifi_justfloat(g_euler.yaw,
                       g_car_base_speed_command,
                       Left_Target_Speed, Right_Target_Speed,
                       g_car_speed_left_filtered, g_car_speed_right_filtered,
                       g_car_speed_left_brake_ff,
                       g_car_speed_right_brake_ff,
                       g_car_speed_left_motor_output,
                       g_car_speed_right_motor_output,
                       g_car_speed_brake_active,
                       -g_car_gyroz_target_dps, g_car_gyroz_feedback_dps,
                       g_car_gyroz_output,
                       g_car_yaw_target_deg,
                       g_car_world_velocity_x_command,
                       g_car_world_velocity_y_command,
                       g_car_world_speed_magnitude,
                       g_car_world_speed_limit,
                       g_car_world_heading_target_deg,
                       g_car_world_heading_error_deg,
                       g_car_world_alignment_scale,
                       g_car_world_body_speed_feedback,
                       g_car_world_reverse_active,
                       g_car_negative_pressure_throttle,
                       g_car_negative_pressure_target,
                       g_car_negative_pressure_boost,
                       g_car_negative_pressure_state,
                       g_car_large_turn_state,
                       (float)remote_snapshot.channel[0],
                       (float)remote_snapshot.channel[1],
                       remote_direction_deg);
    }
}
