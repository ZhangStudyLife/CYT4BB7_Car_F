#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"
#include "pid_core.h"

extern volatile uint8_t timer_100HZ_flag;
extern volatile uint16 g_tick_1000HZ;
extern volatile uint32 tick_1000us_cnt;
extern volatile float g_car_speed_left_filtered;
extern volatile float g_car_speed_right_filtered;
extern volatile float g_car_base_speed_command;
extern volatile float g_car_base_speed_target;
extern volatile float g_car_base_speed_delta;
extern volatile float g_car_speed_accel_ff;
extern volatile float g_car_speed_left_brake_ff;
extern volatile float g_car_speed_right_brake_ff;
extern volatile float g_car_speed_left_motor_output;
extern volatile float g_car_speed_right_motor_output;
extern volatile float g_car_speed_brake_active;
extern volatile float Left_Target_Speed;
extern volatile float Right_Target_Speed;
extern pid_t Left_Speed_PID;
extern pid_t Right_Speed_PID;
extern float car_speed_left_kp;
extern float car_speed_left_ki;
extern float car_speed_left_kd;
extern float car_speed_right_kp;
extern float car_speed_right_ki;
extern float car_speed_right_kd;
extern float car_speed_filter_alpha;
extern float car_speed_ff_slope;
extern float car_speed_ff_static;
extern float car_speed_ff_deadband;
extern float car_speed_ff_transition;
extern float car_speed_brake_static;
extern float car_speed_delta_output_limit;
extern float car_speed_accel_kff;
extern float car_speed_accel_step_limit;
extern float car_speed_decel_step_limit;
extern float car_speed_accel_ff_limit;
extern float car_gyroz_kff;
extern float car_gyroz_kp;
extern float car_gyroz_ki;
extern float car_gyroz_k_turn;
extern float car_yaw_kp;
extern float car_yaw_kd;
extern float car_yaw_rate_limit_dps;
extern float car_yaw_control_mode;
extern volatile float g_car_gyroz_target_dps;
extern volatile float g_car_gyroz_feedback_dps;
extern volatile float g_car_gyroz_feedback_equivalent;
extern volatile float g_car_gyroz_error;
extern volatile float g_car_gyroz_ff_term;
extern volatile float g_car_gyroz_p_term;
extern volatile float g_car_gyroz_i_term;
extern volatile float g_car_gyroz_output;
extern volatile float g_car_yaw_target_deg;
extern volatile float g_car_yaw_p_term;
extern volatile float g_car_yaw_d_term;
extern volatile float g_car_world_velocity_x_command;
extern volatile float g_car_world_velocity_y_command;
extern volatile float g_car_world_speed_magnitude;
extern volatile float g_car_world_speed_limit;
extern volatile float g_car_world_heading_target_deg;
extern volatile float g_car_world_heading_error_deg;
extern volatile float g_car_world_alignment_scale;
extern volatile float g_car_world_body_speed_feedback;
extern volatile float g_car_world_reverse_active;

extern volatile float g_air_tof_fused_height_mm;
extern volatile float g_air_euler_roll;
extern volatile float g_air_euler_pitch;
extern volatile float g_air_euler_yaw;
extern volatile float g_air_pos_est_vel_x;
extern volatile float g_air_pos_est_vel_y;
extern volatile float g_air_state;

typedef struct
{
    float tof_raw_height_mm[4];
    float flow_raw_x;
    float flow_raw_y;
    float flow_filtered_x;
    float flow_filtered_y;
    float imu_raw_gyro[3];
    float imu_raw_acc[3];
    float imu_filtered_gyro[3];
    float imu_filtered_acc[3];
    float camera_spi_online[2];
    float camera_spi_ready[2];
    float camera_spi_error_code;
    float camera_spi_rx_head[2][2];
} air_diag_telemetry_t;

extern volatile air_diag_telemetry_t g_air_diag_telemetry;

extern volatile float g_air_crsf_std_ch0;
extern volatile float g_air_crsf_std_ch1;
extern volatile float g_air_crsf_std_ch2;
extern volatile float g_air_crsf_std_ch3;
extern volatile float g_air_crsf_std_ch4;
extern volatile float g_air_crsf_std_ch5;
extern volatile float g_air_crsf_std_ch6;
extern volatile float g_air_crsf_std_ch7;
extern volatile float g_air_crsf_std_ch8;
extern volatile float g_air_yaw_angle_target_deg;
extern volatile float g_air_sync_time_ms;
extern volatile float g_air_car_plan_valid;
extern volatile float g_air_car_plan_strafe_mps;
extern volatile float g_air_car_plan_forward_mps;
extern volatile float g_air_car_plan_camera;
extern volatile float g_air_car_plan_beacon_index;
extern volatile float g_air_car_plan_dist_px;
extern volatile float g_air_beacon_lost_flag;

uint8 car_menu_is_runtime_locked(void);
void car_yaw_control_100HZ(void);
void car_gyroz_control_100HZ(void);
void car_loop_init(void);
void car_loop_poll(void);

#endif
