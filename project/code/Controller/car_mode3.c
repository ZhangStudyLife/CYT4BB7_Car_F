#include "car_mode.h"

void car_mode3_init(void)
{
    car_mode3_reset();
}

void car_mode3_reset(void)
{
    g_car_base_speed_command = 0.0f;
    g_car_base_speed_target = 0.0f;
    Left_Target_Speed = 0.0f;
    Right_Target_Speed = 0.0f;
    g_car_speed_left_motor_output = 0.0f;
    g_car_speed_right_motor_output = 0.0f;
    motor_left_set_speed(0);
    motor_right_set_speed(0);
}

void car_mode3_update_100HZ(uint32 now_ms)
{
    (void)now_ms;
    car_mode3_reset();
}
