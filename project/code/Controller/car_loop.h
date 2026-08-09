#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"
#include "pid_core.h"

/* 遥控轴已标准化到[-1000, 1000]，使用径向迟滞死区抑制中心方向跳变。 */
#define CAR_WORLD_INPUT_ENTER_DEADZONE (250.0f)
#define CAR_WORLD_INPUT_EXIT_DEADZONE  (150.0f)
#define CAR_WORLD_INPUT_CONFIRM_CYCLES (3U)

extern volatile uint32 tick_1000us_cnt;

typedef struct
{
    uint32 imu_tick_count;
    uint32 control_tick_count;
    uint32 control_period_fault_count;
    uint32 background_coalesced_count;
    uint32 command_stale_count;
    uint32 imu_snapshot_fault_count;
    uint32 imu_stale_fault_count;
} car_realtime_diag_t;

extern volatile uint32 g_car_background_100hz_generation;
extern volatile car_realtime_diag_t g_car_realtime_diag;
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
extern volatile float car_speed_left_kp;
extern volatile float car_speed_left_ki;
extern volatile float car_speed_left_kd;
extern volatile float car_speed_right_kp;
extern volatile float car_speed_right_ki;
extern volatile float car_speed_right_kd;
extern volatile float car_speed_filter_alpha;
extern volatile float car_speed_ff_slope;
extern volatile float car_speed_ff_static;
extern volatile float car_speed_ff_deadband;
extern volatile float car_speed_ff_transition;
extern volatile float car_speed_brake_static;
extern volatile float car_speed_delta_output_limit;
extern volatile float car_speed_accel_kff;
extern volatile float car_speed_accel_step_limit;
extern volatile float car_speed_decel_step_limit;
extern volatile float car_speed_accel_ff_limit;
extern volatile float car_gyroz_kff;
extern volatile float car_gyroz_kp;
extern volatile float car_gyroz_ki;
extern volatile float car_gyroz_k_turn;
extern volatile float car_yaw_kp;
extern volatile float car_yaw_kd;
extern volatile float car_yaw_rate_limit_dps;
extern volatile float car_gyroz_rear_kff;
extern volatile float car_gyroz_rear_kp;
extern volatile float car_gyroz_rear_ki;
extern volatile float car_gyroz_rear_k_turn;
extern volatile float car_yaw_rear_kp;
extern volatile float car_yaw_rear_kd;
extern volatile float car_yaw_rear_rate_limit_dps;
extern volatile float car_yaw_control_mode;
extern volatile float car_world_drive_mode;
extern volatile float car_world_auto_hysteresis_deg;
extern volatile float car_world_auto_front_bias_deg;
extern volatile float car_negative_pressure_hold_throttle;
extern volatile float car_negative_pressure_turn_throttle;
extern volatile float car_negative_pressure_boost_throttle;
extern volatile float car_negative_pressure_speed_error_abs;
extern volatile float car_negative_pressure_speed_error_ratio;
extern volatile float car_negative_pressure_stable_cycles;
extern volatile float car_negative_pressure_speed_delta_threshold;
extern volatile float car_negative_pressure_accel_timeout_cycles;
extern volatile float car_negative_pressure_turn_enter_angle_deg;
extern volatile float car_negative_pressure_turn_exit_angle_deg;
extern volatile float car_negative_pressure_turn_enter_target_rate_dps;
extern volatile float car_negative_pressure_turn_exit_target_rate_dps;
extern volatile float car_negative_pressure_turn_enter_feedback_rate_dps;
extern volatile float car_negative_pressure_turn_exit_feedback_rate_dps;
extern volatile float car_large_turn_brake_speed;
extern volatile float car_large_turn_brake_target_speed;
extern volatile float car_large_turn_brake_ff;
extern volatile float car_large_turn_brake_rate_limit_dps;
extern volatile float car_large_turn_enter_angle_deg;
extern volatile float car_large_turn_pivot_exit_angle_deg;
extern volatile float car_large_turn_exit_speed_start_angle_deg;
extern volatile float car_large_turn_finish_angle_deg;
extern volatile float car_large_turn_brake_stable_cycles;
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
extern volatile float g_car_world_motion_heading_deg;
extern volatile float g_car_world_heading_error_deg;
extern volatile float g_car_world_alignment_scale;
extern volatile float g_car_world_body_speed_feedback;
extern volatile float g_car_world_reverse_active;
extern volatile float g_car_world_drive_mode_active;
extern volatile float g_car_world_drive_sign;
extern volatile float g_car_world_transition_reason;
extern volatile float g_car_yaw_profile_active;
extern volatile float g_car_negative_pressure_throttle;
extern volatile float g_car_negative_pressure_target;
extern volatile float g_car_negative_pressure_boost;
extern volatile float g_car_negative_pressure_state;
extern volatile float g_car_large_turn_state;

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
void car_loop_imu_1000HZ_isr(void);
void car_loop_motion_100HZ_isr(void);
void car_loop_release_background_100HZ_isr(void);

#endif
