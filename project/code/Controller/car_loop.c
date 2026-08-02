#include "car_loop.h"
#include "../Protocols/crsf/crsf.h"
#include "../car_safety.h"
#include "../negative_pressure_motor.h"
#include <math.h>

volatile uint32 tick_1000us_cnt = 0U;
volatile uint32 g_car_background_100hz_generation = 0U;
volatile car_realtime_diag_t g_car_realtime_diag = {0};

static volatile uint32 s_system_time_ms = 0U;
static uint32 s_background_100hz_processed_generation = 0U;
static uint32 s_control_last_imu_tick = 0U;
static uint32 s_control_last_imu_sample_count = 0U;
static uint8 s_control_imu_stale_cycles = 0U;
static crsf_control_snapshot_t s_car_control_input = {0};
static imu_realtime_snapshot_t s_car_imu_snapshot = {0};
/* static uint16 s_air_comm_beep_tick = 200U; */
static uint8 s_air_menu_runtime_locked = 0U;
static uint8 s_menu_runtime_was_locked = 0U;
volatile float g_car_gyroz_feedback_dps = 0.0f;
static uint8 s_car_yaw_stopped = 0U;
static uint8 s_car_speed_filter_initialized = 0U;
static uint8 s_car_yaw_mode_initialized = 0U;
static uint8 s_car_yaw_angle_mode_active = 1U;
static uint8 s_car_speed_brake_active = 0U;
static uint8 s_car_world_command_active = 0U;
static float s_car_speed_left_brake_direction = 0.0f;
static float s_car_speed_right_brake_direction = 0.0f;

#define CAR_SPEED_I_LIMIT (2000.0f)
#define CAR_SPEED_ACCEL_TURN_FULL_RATIO (0.20f)
#define CAR_SPEED_ACCEL_TURN_DISABLE_RATIO (0.50f)
#define CAR_SPEED_PLAN_MIN_STEP (1.0f)
#define CAR_SPEED_SLEW_AND_ACCEL_FF_ENABLE (0U)
#define CAR_SPEED_LOOP_TUNE_ENABLE (0U)
#define CAR_SPEED_LOOP_TUNE_THRESHOLD (300)
#define CAR_IMU_STALE_CONTROL_LIMIT (3U)
#define CAR_PIT_TICKS_PER_US        (8U)

#define CAR_WORLD_INPUT_DEADZONE (50.0f)
#define CAR_WORLD_SPEED_LIMIT_LOW (200.0f)
#define CAR_WORLD_SPEED_LIMIT_MID (400.0f)
#define CAR_WORLD_SPEED_LIMIT_HIGH (500.0f)
#define CAR_WORLD_ALIGNMENT_STOP_DEG (90.0f)
#define CAR_WHEEL_TARGET_ABS_LIMIT (1000.0f)

#define CAR_GYROZ_DEG_TO_RAD (0.017453292519943295f)
#define CAR_GYROZ_RAD_TO_DEG (57.29577951308232f)
#define CAR_GYROZ_EQUIVALENT_SCALE (28.1448005f)
#define CAR_GYROZ_DEBUG_INPUT_LOW_THRESHOLD (300)
#define CAR_GYROZ_DEBUG_INPUT_HIGH_THRESHOLD (800)
#define CAR_GYROZ_DEBUG_TARGET_LOW_DPS (400.0f)
#define CAR_GYROZ_DEBUG_TARGET_HIGH_DPS (800.0f)
#define CAR_GYROZ_OUTPUT_LIMIT (500.0f)
#define CAR_YAW_STOP_ANGLE_DEG (2.0f)
#define CAR_YAW_WAKE_ANGLE_DEG (4.0f)
#define CAR_GYROZ_STOP_TARGET_RATE_DPS (2.0f)
#define CAR_GYROZ_WAKE_TARGET_RATE_DPS (4.0f)
#define CAR_YAW_STOP_RATE_DPS (5.0f)
#define CAR_WHEEL_STOP_SPEED (5.0f)
#define CAR_NEGATIVE_PRESSURE_FIXED_THROTTLE (4000U)

volatile float car_speed_left_kp = 12.00f;
volatile float car_speed_left_ki = 0.90f;
volatile float car_speed_left_kd = 3.00f;
volatile float car_speed_right_kp = 12.00f;
volatile float car_speed_right_ki = 0.900f;
volatile float car_speed_right_kd = 3.00f;
volatile float car_speed_filter_alpha = 0.557f;
volatile float car_speed_ff_slope = 0.00f;
volatile float car_speed_ff_static = 600.0f;
volatile float car_speed_ff_deadband = 10.0f;
volatile float car_speed_ff_transition = 100.0f;
volatile float car_speed_brake_static = 0.0f;
volatile float car_speed_delta_output_limit = 6000.0f;
volatile float car_speed_accel_kff = 10.0f;
volatile float car_speed_accel_step_limit = 40.0f;
volatile float car_speed_decel_step_limit = 40.0f;
volatile float car_speed_accel_ff_limit = 800.0f;
volatile float car_gyroz_kff = 0.12f;
/* 角速度PI使用100 Hz每采样周期离散系数，数值已由原连续时间系数等效换算。 */
volatile float car_gyroz_kp = 1.35f;
volatile float car_gyroz_ki = 0.025f;
volatile float car_gyroz_k_turn = 1.0f;
volatile float car_yaw_kp = 8.50f;
volatile float car_yaw_kd = 4.00f;
volatile float car_yaw_rate_limit_dps = 800.0f;
volatile float car_yaw_control_mode = 1.0f;

volatile float g_car_speed_left_filtered = 0.0f;
volatile float g_car_speed_right_filtered = 0.0f;
volatile float g_car_base_speed_command = 0.0f;
volatile float g_car_base_speed_target = 0.0f;
volatile float g_car_base_speed_delta = 0.0f;
volatile float g_car_speed_accel_ff = 0.0f;
volatile float g_car_speed_left_brake_ff = 0.0f;
volatile float g_car_speed_right_brake_ff = 0.0f;
volatile float g_car_speed_left_motor_output = 0.0f;
volatile float g_car_speed_right_motor_output = 0.0f;
volatile float g_car_speed_brake_active = 0.0f;
volatile float Left_Target_Speed = 0.0f;
volatile float Right_Target_Speed = 0.0f;
pid_t Left_Speed_PID;
pid_t Right_Speed_PID;
static pid_t s_car_yaw_pid;
static pid_t s_car_gyroz_pid;

volatile float g_car_gyroz_target_dps = 0.0f;
volatile float g_car_gyroz_feedback_equivalent = 0.0f;
volatile float g_car_gyroz_error = 0.0f;
volatile float g_car_gyroz_ff_term = 0.0f;
volatile float g_car_gyroz_p_term = 0.0f;
volatile float g_car_gyroz_i_term = 0.0f;
volatile float g_car_gyroz_output = 0.0f;
volatile float g_car_yaw_target_deg = 0.0f;
volatile float g_car_yaw_p_term = 0.0f;
volatile float g_car_yaw_d_term = 0.0f;
volatile float g_car_world_velocity_x_command = 0.0f;
volatile float g_car_world_velocity_y_command = 0.0f;
volatile float g_car_world_speed_magnitude = 0.0f;
volatile float g_car_world_speed_limit = 0.0f;
volatile float g_car_world_heading_target_deg = 0.0f;
volatile float g_car_world_heading_error_deg = 0.0f;
volatile float g_car_world_alignment_scale = 0.0f;
volatile float g_car_world_body_speed_feedback = 0.0f;
volatile float g_car_world_reverse_active = 0.0f;

volatile float g_air_tof_fused_height_mm = 0.0f;
volatile float g_air_euler_roll = 0.0f;
volatile float g_air_euler_pitch = 0.0f;
volatile float g_air_euler_yaw = 0.0f;
volatile float g_air_pos_est_vel_x = 0.0f;
volatile float g_air_pos_est_vel_y = 0.0f;
volatile float g_air_state = 0.0f;
volatile air_diag_telemetry_t g_air_diag_telemetry;
volatile float g_air_reserved0 = 0.0f;
volatile float g_air_reserved1 = 0.0f;
volatile float g_air_reserved2 = 0.0f;
volatile float g_air_crsf_std_ch0 = 0.0f;
volatile float g_air_crsf_std_ch1 = 0.0f;
volatile float g_air_crsf_std_ch2 = 0.0f;
volatile float g_air_crsf_std_ch3 = 0.0f;
volatile float g_air_crsf_std_ch4 = 0.0f;
volatile float g_air_crsf_std_ch5 = 0.0f;
volatile float g_air_crsf_std_ch6 = 0.0f;
volatile float g_air_crsf_std_ch7 = 0.0f;
volatile float g_air_crsf_std_ch8 = 0.0f;
volatile float g_air_yaw_angle_target_deg = 0.0f;
volatile float g_air_sync_time_ms = 0.0f;
volatile float g_air_car_plan_valid = 0.0f;
volatile float g_air_car_plan_strafe_mps = 0.0f;
volatile float g_air_car_plan_forward_mps = 0.0f;
volatile float g_air_car_plan_camera = 0.0f;
volatile float g_air_car_plan_beacon_index = 0.0f;
volatile float g_air_car_plan_dist_px = 0.0f;
volatile float g_air_beacon_lost_flag = 0.0f;

#define AIR_RUN_DATA_CRITICAL_COUNT (15U)
#define AIR_RUN_DATA_DIAGNOSTIC_LEGACY_COUNT (45U)
#define AIR_RUN_DATA_DIAGNOSTIC_V1_COUNT (48U)
#define AIR_RUN_DATA_DIAGNOSTIC_COUNT (52U)

#define AIR_MENU_STATE_INIT (0.0f)
#define AIR_MENU_STATE_STANDBY (1.0f)

static void car_loop_update_air_runtime_state(float air_state)
{
    g_air_state = air_state;
    s_air_menu_runtime_locked = ((air_state == AIR_MENU_STATE_INIT) ||
                                 (air_state == AIR_MENU_STATE_STANDBY))
                                    ? 0U
                                    : 1U;
}

static void on_air_data(const float *data, uint8 count)
{
    if (count == AIR_RUN_DATA_CRITICAL_COUNT)
    {
        car_loop_update_air_runtime_state(data[0]);
        g_air_crsf_std_ch0 = data[1];
        g_air_crsf_std_ch1 = data[2];
        g_air_crsf_std_ch2 = data[3];
        g_air_crsf_std_ch3 = data[4];
        g_air_crsf_std_ch4 = data[5];
        g_air_crsf_std_ch5 = data[6];
        g_air_crsf_std_ch6 = data[7];
        g_air_crsf_std_ch7 = data[8];
        g_air_crsf_std_ch8 = data[9];
        g_air_yaw_angle_target_deg = data[10];
        g_air_car_plan_valid = data[11];
        g_air_car_plan_strafe_mps = data[12];
        g_air_car_plan_forward_mps = data[13];
        g_air_beacon_lost_flag = data[14];
        return;
    }

    if ((count != AIR_RUN_DATA_DIAGNOSTIC_LEGACY_COUNT) &&
        (count != AIR_RUN_DATA_DIAGNOSTIC_V1_COUNT) &&
        (count != AIR_RUN_DATA_DIAGNOSTIC_COUNT))
    {
        return;
    }

    g_air_tof_fused_height_mm = data[0];
    g_air_euler_roll = data[1];
    g_air_euler_pitch = data[2];
    g_air_euler_yaw = data[3];
    g_air_pos_est_vel_x = data[4];
    g_air_pos_est_vel_y = data[5];
    car_loop_update_air_runtime_state(data[6]);
    g_air_crsf_std_ch0 = data[7];
    g_air_crsf_std_ch1 = data[8];
    g_air_crsf_std_ch2 = data[9];
    g_air_crsf_std_ch3 = data[10];
    g_air_crsf_std_ch4 = data[11];
    g_air_crsf_std_ch5 = data[12];
    g_air_crsf_std_ch6 = data[13];
    g_air_crsf_std_ch7 = data[14];
    g_air_yaw_angle_target_deg = data[15];
    g_air_sync_time_ms = data[16];
    g_air_car_plan_valid = data[17];
    g_air_car_plan_strafe_mps = data[18];
    g_air_car_plan_forward_mps = data[19];
    g_air_car_plan_camera = data[20];
    g_air_car_plan_beacon_index = data[21];
    g_air_car_plan_dist_px = data[22];
    g_air_beacon_lost_flag = data[23];
    g_air_crsf_std_ch8 = data[24];
    g_air_diag_telemetry.tof_raw_height_mm[0] = data[25];
    g_air_diag_telemetry.tof_raw_height_mm[1] = data[26];
    g_air_diag_telemetry.tof_raw_height_mm[2] = data[27];
    g_air_diag_telemetry.tof_raw_height_mm[3] = data[28];
    g_air_diag_telemetry.flow_raw_x = data[29];
    g_air_diag_telemetry.flow_raw_y = data[30];
    g_air_diag_telemetry.flow_filtered_x = data[31];
    g_air_diag_telemetry.flow_filtered_y = data[32];
    g_air_diag_telemetry.imu_raw_gyro[0] = data[33];
    g_air_diag_telemetry.imu_raw_gyro[1] = data[34];
    g_air_diag_telemetry.imu_raw_gyro[2] = data[35];
    g_air_diag_telemetry.imu_raw_acc[0] = data[36];
    g_air_diag_telemetry.imu_raw_acc[1] = data[37];
    g_air_diag_telemetry.imu_raw_acc[2] = data[38];
    g_air_diag_telemetry.imu_filtered_gyro[0] = data[39];
    g_air_diag_telemetry.imu_filtered_gyro[1] = data[40];
    g_air_diag_telemetry.imu_filtered_gyro[2] = data[41];
    g_air_diag_telemetry.imu_filtered_acc[0] = data[42];
    g_air_diag_telemetry.imu_filtered_acc[1] = data[43];
    g_air_diag_telemetry.imu_filtered_acc[2] = data[44];

    if (count >= AIR_RUN_DATA_DIAGNOSTIC_V1_COUNT)
    {
        uint8 camera_status0 = (uint8)data[45];
        uint8 camera_status1 = (uint8)data[46];

        g_air_diag_telemetry.camera_spi_online[0] = (float)(camera_status0 & 0x01U);
        g_air_diag_telemetry.camera_spi_online[1] = (float)(camera_status1 & 0x01U);
        g_air_diag_telemetry.camera_spi_ready[0] = (float)((camera_status0 >> 1) & 0x01U);
        g_air_diag_telemetry.camera_spi_ready[1] = (float)((camera_status1 >> 1) & 0x01U);
        g_air_diag_telemetry.camera_spi_error_code = data[47];
    }
    if (count >= AIR_RUN_DATA_DIAGNOSTIC_COUNT)
    {
        g_air_diag_telemetry.camera_spi_rx_head[0][0] = data[48];
        g_air_diag_telemetry.camera_spi_rx_head[0][1] = data[49];
        g_air_diag_telemetry.camera_spi_rx_head[1][0] = data[50];
        g_air_diag_telemetry.camera_spi_rx_head[1][1] = data[51];
    }
}

uint8 car_menu_is_runtime_locked(void)
{
    return s_air_menu_runtime_locked;
}

static float car_speed_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float car_angle_wrap_deg(float angle_deg)
{
    if (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    else if (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void car_yaw_outer_reset(void)
{
    PID_Reset(&s_car_yaw_pid);
    g_car_yaw_p_term = 0.0f;
    g_car_yaw_d_term = 0.0f;
}

static float car_world_speed_limit_get(void)
{
    if (s_car_control_input.channel[6] == 0)
    {
        return CAR_WORLD_SPEED_LIMIT_LOW;
    }
    if (s_car_control_input.channel[6] == 1)
    {
        return CAR_WORLD_SPEED_LIMIT_MID;
    }
    return CAR_WORLD_SPEED_LIMIT_HIGH;
}

static float car_speed_tune_target_get(void)
{
    float target_abs = car_world_speed_limit_get();

    if (s_car_control_input.channel[1] > CAR_SPEED_LOOP_TUNE_THRESHOLD)
    {
        return target_abs;
    }
    if (s_car_control_input.channel[1] < -CAR_SPEED_LOOP_TUNE_THRESHOLD)
    {
        return -target_abs;
    }

    return 0.0f;
}

static void car_world_control_state_reset(float yaw_target_deg)
{
    s_car_world_command_active = 0U;

    g_car_base_speed_command = 0.0f;
    g_car_yaw_target_deg = car_angle_wrap_deg(yaw_target_deg);
    g_car_world_velocity_x_command = 0.0f;
    g_car_world_velocity_y_command = 0.0f;
    g_car_world_speed_magnitude = 0.0f;
    g_car_world_speed_limit = 0.0f;
    g_car_world_heading_target_deg = g_car_yaw_target_deg;
    g_car_world_heading_error_deg = 0.0f;
    g_car_world_alignment_scale = 0.0f;
    g_car_world_body_speed_feedback = 0.0f;
    g_car_world_reverse_active = 0.0f;
}

static void car_world_command_update_100HZ(void)
{
    float raw_x;
    float raw_y;
    float raw_magnitude;
    float direction_scale;
    float world_heading_deg;
    float heading_error_deg;
    float alignment_scale;
    float desired_speed;

    g_car_world_speed_limit = car_world_speed_limit_get();
    g_car_world_body_speed_feedback =
        0.5f * (g_car_speed_left_filtered + g_car_speed_right_filtered);

    /* Yaw-rate debug mode is intentionally rotation-only. */
    if (s_car_yaw_angle_mode_active == 0U)
    {
        s_car_world_command_active = 0U;
        g_car_base_speed_command = 0.0f;
        g_car_world_velocity_x_command = 0.0f;
        g_car_world_velocity_y_command = 0.0f;
        g_car_world_speed_magnitude = 0.0f;
        g_car_world_heading_target_deg = s_car_imu_snapshot.euler.yaw;
        g_car_world_heading_error_deg = 0.0f;
        g_car_world_alignment_scale = 0.0f;
        g_car_world_reverse_active = 0.0f;
        return;
    }

    raw_x = (float)s_car_control_input.channel[1];
    raw_y = (float)s_car_control_input.channel[0];
    raw_magnitude = sqrtf(raw_x * raw_x + raw_y * raw_y);

    if (raw_magnitude <= CAR_WORLD_INPUT_DEADZONE)
    {
        if (s_car_world_command_active != 0U)
        {
            s_car_world_command_active = 0U;
            g_car_yaw_target_deg = s_car_imu_snapshot.euler.yaw;
            car_yaw_outer_reset();
        }

        g_car_base_speed_command = 0.0f;
        g_car_world_velocity_x_command = 0.0f;
        g_car_world_velocity_y_command = 0.0f;
        g_car_world_speed_magnitude = 0.0f;
        g_car_world_heading_target_deg = g_car_yaw_target_deg;
        g_car_world_heading_error_deg =
            car_angle_wrap_deg(g_car_yaw_target_deg - s_car_imu_snapshot.euler.yaw);
        g_car_world_alignment_scale = 0.0f;
        g_car_world_reverse_active = 0.0f;
        return;
    }

    if (s_car_world_command_active == 0U)
    {
        s_car_world_command_active = 1U;
        car_yaw_outer_reset();
    }

    /* CH1/CH2 only define direction; CH7 selects the speed magnitude. */
    direction_scale = g_car_world_speed_limit / raw_magnitude;
    g_car_world_velocity_x_command = raw_x * direction_scale;
    g_car_world_velocity_y_command = raw_y * direction_scale;
    g_car_world_speed_magnitude = g_car_world_speed_limit;

    world_heading_deg = atan2f(g_car_world_velocity_y_command,
                               g_car_world_velocity_x_command) *
                        CAR_GYROZ_RAD_TO_DEG;
    heading_error_deg =
        car_angle_wrap_deg(world_heading_deg - s_car_imu_snapshot.euler.yaw);

    /* 始终让车头对准世界坐标指令方向，不再通过倒车缩短转向路径。 */
    if (fabsf(heading_error_deg) >=
        CAR_WORLD_ALIGNMENT_STOP_DEG)
    {
        alignment_scale = 0.0f;
    }
    else
    {
        alignment_scale = cosf(heading_error_deg *
                               CAR_GYROZ_DEG_TO_RAD);
        alignment_scale = car_speed_clampf(alignment_scale, 0.0f, 1.0f);
    }

    desired_speed = g_car_world_speed_magnitude * alignment_scale;

    g_car_yaw_target_deg = world_heading_deg;
    g_car_base_speed_command = desired_speed;
    g_car_world_heading_target_deg = world_heading_deg;
    g_car_world_heading_error_deg = heading_error_deg;
    g_car_world_alignment_scale = alignment_scale;
    g_car_world_reverse_active = 0.0f;
}

static uint8 car_world_brake_requested(void)
{
    return (s_car_world_command_active == 0U) ? 1U : 0U;
}

static void car_speed_plan_reset(void)
{
    g_car_base_speed_command = 0.0f;
    g_car_base_speed_target = 0.0f;
    g_car_base_speed_delta = 0.0f;
    g_car_speed_accel_ff = 0.0f;
    g_car_speed_left_brake_ff = 0.0f;
    g_car_speed_right_brake_ff = 0.0f;
    g_car_speed_left_motor_output = 0.0f;
    g_car_speed_right_motor_output = 0.0f;
    g_car_speed_brake_active = 0.0f;
    s_car_speed_brake_active = 0U;
    s_car_speed_left_brake_direction = 0.0f;
    s_car_speed_right_brake_direction = 0.0f;
}

static float car_speed_plan_step(float step)
{
    step = fabsf(step);
    return (step >= CAR_SPEED_PLAN_MIN_STEP) ?
           step : CAR_SPEED_PLAN_MIN_STEP;
}

static void car_speed_plan_update(void)
{
    float previous_target = g_car_base_speed_target;
    float intermediate_command = g_car_base_speed_command;
    float step;
    float command_delta;
    float accel_limit = fabsf(car_speed_accel_ff_limit);

    /* 调试速度环时直接传递阶跃目标，并停用加速度前馈。 */
    if (CAR_SPEED_SLEW_AND_ACCEL_FF_ENABLE == 0U)
    {
        g_car_base_speed_target = g_car_base_speed_command;
        g_car_base_speed_delta = g_car_base_speed_target - previous_target;
        g_car_speed_accel_ff = 0.0f;
        return;
    }

    /* 换向时先减速到零，下一周期再向相反方向加速。 */
    if (previous_target * g_car_base_speed_command < 0.0f)
    {
        intermediate_command = 0.0f;
        step = car_speed_plan_step(car_speed_decel_step_limit);
    }
    else if (fabsf(g_car_base_speed_command) > fabsf(previous_target))
    {
        step = car_speed_plan_step(car_speed_accel_step_limit);
    }
    else
    {
        step = car_speed_plan_step(car_speed_decel_step_limit);
    }

    command_delta = intermediate_command - previous_target;
    if (command_delta > step)
    {
        command_delta = step;
    }
    else if (command_delta < -step)
    {
        command_delta = -step;
    }

    g_car_base_speed_target = previous_target + command_delta;
    g_car_base_speed_delta = command_delta;

    if ((car_speed_accel_kff <= 0.0f) || (accel_limit <= 0.0f))
    {
        g_car_speed_accel_ff = 0.0f;
    }
    else
    {
        g_car_speed_accel_ff = car_speed_clampf(
            car_speed_accel_kff * g_car_base_speed_delta,
            -accel_limit, accel_limit);
    }
}

static float car_speed_accel_ff_apply_turn_guard(float base_speed_target,
                                                  float base_speed_delta)
{
    float previous_target = base_speed_target - base_speed_delta;
    float current_abs = fabsf(base_speed_target);
    float previous_abs = fabsf(previous_target);
    float reference_abs = (current_abs > previous_abs) ?
                          current_abs : previous_abs;
    float turn_target;
    float turn_ratio;
    float accel_scale;

    if (reference_abs < CAR_SPEED_PLAN_MIN_STEP)
    {
        g_car_speed_accel_ff = 0.0f;
        return 0.0f;
    }

    turn_target = 0.5f * (Left_Target_Speed - Right_Target_Speed);
    turn_ratio = fabsf(turn_target) / reference_abs;
    if (turn_ratio <= CAR_SPEED_ACCEL_TURN_FULL_RATIO)
    {
        accel_scale = 1.0f;
    }
    else if (turn_ratio >= CAR_SPEED_ACCEL_TURN_DISABLE_RATIO)
    {
        accel_scale = 0.0f;
    }
    else
    {
        accel_scale = (CAR_SPEED_ACCEL_TURN_DISABLE_RATIO - turn_ratio) /
                      (CAR_SPEED_ACCEL_TURN_DISABLE_RATIO -
                       CAR_SPEED_ACCEL_TURN_FULL_RATIO);
    }

    g_car_speed_accel_ff *= accel_scale;
    return g_car_speed_accel_ff;
}

static float car_speed_direction(float feedback, float fallback)
{
    if (fabsf(feedback) > CAR_WHEEL_STOP_SPEED)
    {
        return (feedback > 0.0f) ? 1.0f : -1.0f;
    }

    if (fallback > 0.0f)
    {
        return 1.0f;
    }
    if (fallback < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

static void car_speed_brake_state_clear(void)
{
    s_car_speed_brake_active = 0U;
    s_car_speed_left_brake_direction = 0.0f;
    s_car_speed_right_brake_direction = 0.0f;
    g_car_speed_brake_active = 0.0f;
}

static void car_speed_brake_state_update(uint8 brake_requested)
{
    float previous_target;
    float turn_target = 0.5f * (Left_Target_Speed - Right_Target_Speed);

    if ((brake_requested == 0U) ||
        (fabsf(g_car_gyroz_target_dps) >=
         CAR_GYROZ_STOP_TARGET_RATE_DPS) ||
        (fabsf(turn_target) > CAR_WHEEL_STOP_SPEED))
    {
        car_speed_brake_state_clear();
        return;
    }

    if (s_car_speed_brake_active != 0U)
    {
        return;
    }

    if ((fabsf(g_car_speed_left_filtered) <= CAR_WHEEL_STOP_SPEED) &&
        (fabsf(g_car_speed_right_filtered) <= CAR_WHEEL_STOP_SPEED))
    {
        return;
    }

    previous_target = g_car_base_speed_target - g_car_base_speed_delta;
    s_car_speed_left_brake_direction =
        car_speed_direction(g_car_speed_left_filtered, previous_target);
    s_car_speed_right_brake_direction =
        car_speed_direction(g_car_speed_right_filtered, previous_target);
    s_car_speed_brake_active = 1U;
    g_car_speed_brake_active = 1.0f;
}

static float car_speed_brake_feedforward(float feedback, float direction)
{
    float speed_abs = fabsf(feedback);
    float full_speed = fabsf(car_speed_ff_deadband);
    float brake_scale;

    if ((s_car_speed_brake_active == 0U) ||
        (direction == 0.0f) ||
        (feedback * direction <= 0.0f) ||
        (car_speed_brake_static <= 0.0f) ||
        (speed_abs <= CAR_WHEEL_STOP_SPEED))
    {
        return 0.0f;
    }

    if (full_speed <= CAR_WHEEL_STOP_SPEED)
    {
        full_speed = CAR_WHEEL_STOP_SPEED + 1.0f;
    }

    if (speed_abs >= full_speed)
    {
        brake_scale = 1.0f;
    }
    else
    {
        brake_scale = (speed_abs - CAR_WHEEL_STOP_SPEED) /
                      (full_speed - CAR_WHEEL_STOP_SPEED);
    }

    return -direction * fabsf(car_speed_brake_static) * brake_scale;
}

static int16 car_speed_pid_update(pid_t *pid, float target, float feedback,
                                  float kp, float ki, float kd,
                                  float external_ff, float brake_ff)
{
    float output;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = car_speed_ff_slope;
    pid->i_limit = CAR_SPEED_I_LIMIT;
    PID_SetStaticFeedforward(pid, car_speed_ff_static);
    PID_SetFeedforwardTransition(pid, car_speed_ff_deadband,
                                 car_speed_ff_transition);
    PID_SetExternalFeedforward(pid, external_ff);
    PID_SetIncrementLimit(pid, car_speed_delta_output_limit);
    PID_SetOutputLimits(pid, -(float)MOTOR_PWM_MAX, (float)MOTOR_PWM_MAX);

    output = PID_UpdateIncremental(pid, target, feedback) + brake_ff;
    output = car_speed_clampf(output,
                              -(float)MOTOR_PWM_MAX,
                              (float)MOTOR_PWM_MAX);
    return (int16)output;
}

static float car_yaw_get_error_deg(void)
{
    return car_angle_wrap_deg(g_car_yaw_target_deg - s_car_imu_snapshot.euler.yaw);
}

static uint8 car_yaw_command_is_within_stop_limit(float angle_limit_deg,
                                                   float rate_limit_dps)
{
    if (s_car_yaw_angle_mode_active != 0U)
    {
        return (fabsf(car_yaw_get_error_deg()) < angle_limit_deg) ? 1U : 0U;
    }

    return (fabsf(g_car_gyroz_target_dps) < rate_limit_dps) ? 1U : 0U;
}

void car_yaw_control_100HZ(void)
{
    float desired_rate_dps;
    float yaw_error_deg;

    yaw_error_deg = car_yaw_get_error_deg();

    s_car_yaw_pid.kp = car_yaw_kp;
    s_car_yaw_pid.ki = 0.0f;
    s_car_yaw_pid.kd = car_yaw_kd;
    s_car_yaw_pid.kff = 0.0f;
    PID_SetOutputLimits(&s_car_yaw_pid,
                        -car_yaw_rate_limit_dps,
                        car_yaw_rate_limit_dps);
    /* 以已归一化角度误差作为位置式PID输入，避免跨越正负180度。 */
    desired_rate_dps = PID_Update(&s_car_yaw_pid, yaw_error_deg, 0.0f);
    g_car_yaw_p_term = s_car_yaw_pid.p_term;
    g_car_yaw_d_term = s_car_yaw_pid.d_term;
    g_car_gyroz_target_dps = -desired_rate_dps;
}

static void car_gyroz_debug_target_100HZ(void)
{
    int16 yaw_rate_command = s_car_control_input.channel[3];
    float desired_rate_dps = 0.0f;

    if (yaw_rate_command > CAR_GYROZ_DEBUG_INPUT_HIGH_THRESHOLD)
    {
        desired_rate_dps = CAR_GYROZ_DEBUG_TARGET_HIGH_DPS;
    }
    else if (yaw_rate_command > CAR_GYROZ_DEBUG_INPUT_LOW_THRESHOLD)
    {
        desired_rate_dps = CAR_GYROZ_DEBUG_TARGET_LOW_DPS;
    }
    else if (yaw_rate_command < -CAR_GYROZ_DEBUG_INPUT_HIGH_THRESHOLD)
    {
        desired_rate_dps = -CAR_GYROZ_DEBUG_TARGET_HIGH_DPS;
    }
    else if (yaw_rate_command < -CAR_GYROZ_DEBUG_INPUT_LOW_THRESHOLD)
    {
        desired_rate_dps = -CAR_GYROZ_DEBUG_TARGET_LOW_DPS;
    }

    g_car_gyroz_target_dps = -desired_rate_dps;
    g_car_yaw_target_deg = s_car_imu_snapshot.euler.yaw;
    g_car_yaw_p_term = 0.0f;
    g_car_yaw_d_term = 0.0f;
}

static void car_yaw_mode_prepare_100HZ(void)
{
    uint8 angle_mode = (car_yaw_control_mode >= 0.5f) ? 1U : 0U;

    if ((s_car_yaw_mode_initialized == 0U) ||
        (angle_mode != s_car_yaw_angle_mode_active))
    {
        s_car_yaw_mode_initialized = 1U;
        s_car_yaw_angle_mode_active = angle_mode;
        car_world_control_state_reset(s_car_imu_snapshot.euler.yaw);
        car_yaw_outer_reset();
        g_car_gyroz_target_dps = 0.0f;
        g_car_gyroz_feedback_equivalent = 0.0f;
        g_car_gyroz_error = 0.0f;
        g_car_gyroz_ff_term = 0.0f;
        g_car_gyroz_p_term = 0.0f;
        g_car_gyroz_i_term = 0.0f;
        g_car_gyroz_output = 0.0f;
        PID_Reset(&s_car_gyroz_pid);
    }
}

static void car_yaw_target_update_100HZ(void)
{
    if (s_car_yaw_angle_mode_active != 0U)
    {
        car_yaw_control_100HZ();
    }
    else
    {
        car_gyroz_debug_target_100HZ();
    }
}

void car_gyroz_control_100HZ(void)
{
    float base_speed = g_car_base_speed_target;
    float desired_rate_dps;
    float left_abs;
    float right_abs;
    float wheel_peak;
    float wheel_scale;

    /* Keep the Loongson scale, with the sign adapted to this car's gyro polarity. */
    desired_rate_dps = -g_car_gyroz_target_dps;
    g_car_gyroz_feedback_equivalent = g_car_gyroz_feedback_dps *
                                      CAR_GYROZ_DEG_TO_RAD *
                                      CAR_GYROZ_EQUIVALENT_SCALE;

    s_car_gyroz_pid.kp = car_gyroz_kp;
    s_car_gyroz_pid.ki = car_gyroz_ki;
    s_car_gyroz_pid.kd = 0.0f;
    s_car_gyroz_pid.kff = car_gyroz_kff;
    s_car_gyroz_pid.i_limit = CAR_GYROZ_OUTPUT_LIMIT;
    PID_SetOutputLimits(&s_car_gyroz_pid,
                        -CAR_GYROZ_OUTPUT_LIMIT,
                        CAR_GYROZ_OUTPUT_LIMIT);
    g_car_gyroz_output = PID_Update(&s_car_gyroz_pid,
                                    desired_rate_dps,
                                    g_car_gyroz_feedback_dps);
    g_car_gyroz_error = s_car_gyroz_pid.error;
    g_car_gyroz_ff_term = s_car_gyroz_pid.ff_term;
    g_car_gyroz_p_term = s_car_gyroz_pid.p_term;
    g_car_gyroz_i_term = s_car_gyroz_pid.i_term;

    Left_Target_Speed = base_speed + car_gyroz_k_turn * g_car_gyroz_output;
    Right_Target_Speed = base_speed - car_gyroz_k_turn * g_car_gyroz_output;

    left_abs = fabsf(Left_Target_Speed);
    right_abs = fabsf(Right_Target_Speed);
    wheel_peak = (left_abs > right_abs) ? left_abs : right_abs;
    if (wheel_peak > CAR_WHEEL_TARGET_ABS_LIMIT)
    {
        wheel_scale = CAR_WHEEL_TARGET_ABS_LIMIT / wheel_peak;
        Left_Target_Speed *= wheel_scale;
        Right_Target_Speed *= wheel_scale;
    }
}

static void car_pit_init_exact(pit_index_enum pit_index, uint32 period_us)
{
    volatile stc_TCPWM_GRP_CNT_t *counter =
        (volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[2].CNT[pit_index];

    pit_init(pit_index, period_us);
    pit_disable(pit_index);
    /* TCPWM 从 0 数到 PERIOD（含端点），精确 N tick 周期应写 N-1。 */
    Cy_Tcpwm_Counter_SetPeriod(
        counter, period_us * CAR_PIT_TICKS_PER_US - 1U);
    pit_enable(pit_index);
}

void car_loop_init(void)
{
    tick_1000us_cnt = 0U;
    g_car_background_100hz_generation = 0U;
    memset((void *)&g_car_realtime_diag, 0, sizeof(g_car_realtime_diag));
    s_system_time_ms = 0U;
    s_background_100hz_processed_generation = 0U;
    s_control_last_imu_tick = 0U;
    s_control_last_imu_sample_count = 0U;
    s_control_imu_stale_cycles = 0U;
    memset(&s_car_control_input, 0, sizeof(s_car_control_input));
    memset(&s_car_imu_snapshot, 0, sizeof(s_car_imu_snapshot));
    /* s_air_comm_beep_tick = 200U; */
    s_air_menu_runtime_locked = 0U;
    s_menu_runtime_was_locked = 0U;
    g_car_gyroz_feedback_dps = 0.0f;
    s_car_yaw_stopped = 0U;
    s_car_yaw_mode_initialized = 0U;
    s_car_yaw_angle_mode_active = 1U;
    car_speed_plan_reset();
    car_world_control_state_reset(0.0f);
    g_car_yaw_p_term = 0.0f;
    g_car_yaw_d_term = 0.0f;
    g_car_speed_left_filtered = 0.0f;
    g_car_speed_right_filtered = 0.0f;
    s_car_speed_filter_initialized = 0U;
    g_car_gyroz_target_dps = 0.0f;
    g_car_gyroz_feedback_equivalent = 0.0f;
    g_car_gyroz_error = 0.0f;
    g_car_gyroz_ff_term = 0.0f;
    g_car_gyroz_p_term = 0.0f;
    g_car_gyroz_i_term = 0.0f;
    g_car_gyroz_output = 0.0f;

    PID_Init(&Left_Speed_PID, car_speed_left_kp, car_speed_left_ki,
             car_speed_left_kd, car_speed_ff_slope, CAR_SPEED_I_LIMIT);
    PID_Init(&Right_Speed_PID, car_speed_right_kp, car_speed_right_ki,
             car_speed_right_kd, car_speed_ff_slope, CAR_SPEED_I_LIMIT);
    PID_Init(&s_car_yaw_pid, car_yaw_kp, 0.0f, car_yaw_kd, 0.0f, 0.0f);
    PID_Init(&s_car_gyroz_pid, car_gyroz_kp, car_gyroz_ki, 0.0f,
             car_gyroz_kff, CAR_GYROZ_OUTPUT_LIMIT);
    PID_SetOutputLimits(&s_car_gyroz_pid,
                        -CAR_GYROZ_OUTPUT_LIMIT,
                        CAR_GYROZ_OUTPUT_LIMIT);

    menu_init();
    menu_config_init();
    motor_init();
    motor_stop();
    negative_pressure_init();
    car_safety_init();
    encoder_control_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    IMU_ResetYaw();
    (void)IMU_GetRealtimeSnapshot(&s_car_imu_snapshot);
    s_control_last_imu_sample_count = s_car_imu_snapshot.sample_count;
    car_world_control_state_reset(0.0f);
    wifi_core_Init();
    air_comm_car_init();
    air_comm_set_run_data_callback(on_air_data);
    crsf_init();
    CRSF_GetControlSnapshot(&s_car_control_input);
    Beep_Init();
    //Beep_Play(100U, 0.2f, 1U);
    car_pit_init_exact(PIT_CH0, 1000U);
    system_delay_us(500U);
    car_pit_init_exact(PIT_CH1, 10000U);
    car_safety_set_motion_scheduler_started();
}

void car_loop_imu_1000HZ_isr(void)
{
    g_car_realtime_diag.imu_tick_count++;
    if (IMU_Update_1000HZ() != 0U)
    {
        g_car_gyroz_feedback_dps +=
            0.06089863f *
            (g_imufilter_1000hz.gyroz - g_car_gyroz_feedback_dps);
    }
}

static uint8 car_run_command_is_neutral(void)
{
    return (((s_car_control_input.channel[0] >= -(int16)CAR_WORLD_INPUT_DEADZONE) &&
             (s_car_control_input.channel[0] <=  (int16)CAR_WORLD_INPUT_DEADZONE)) &&
            ((s_car_control_input.channel[1] >= -(int16)CAR_WORLD_INPUT_DEADZONE) &&
             (s_car_control_input.channel[1] <=  (int16)CAR_WORLD_INPUT_DEADZONE)) &&
            ((s_car_control_input.channel[3] >= -(int16)CAR_WORLD_INPUT_DEADZONE) &&
             (s_car_control_input.channel[3] <=  (int16)CAR_WORLD_INPUT_DEADZONE))) ? 1U : 0U;
}

static void car_total_emergency_stop(void)
{
    PID_Reset(&Left_Speed_PID);
    PID_Reset(&Right_Speed_PID);
    car_yaw_outer_reset();
    PID_Reset(&s_car_gyroz_pid);
    car_speed_plan_reset();
    car_speed_brake_state_clear();
    car_world_control_state_reset(s_car_imu_snapshot.euler.yaw);

    Left_Target_Speed = 0.0f;
    Right_Target_Speed = 0.0f;
    g_car_speed_left_motor_output = 0.0f;
    g_car_speed_right_motor_output = 0.0f;
    g_car_gyroz_target_dps = 0.0f;
    g_car_gyroz_error = 0.0f;
    g_car_gyroz_ff_term = 0.0f;
    g_car_gyroz_p_term = 0.0f;
    g_car_gyroz_i_term = 0.0f;
    g_car_gyroz_output = 0.0f;
    s_car_yaw_stopped = 0U;

    motor_stop();
    negative_pressure_disable();
}

static void car_speed_control_100HZ(void)
{
    car_safety_input_t safety_input;
    int16 left_raw;
    int16 right_raw;
    float effective_accel_ff;
    float left_accel_ff;
    float right_accel_ff;
    float left_brake_ff;
    float right_brake_ff;
    int16 left_motor_output;
    int16 right_motor_output;

    encoder_update_100HZ();
    left_raw = encoder_get_left_count();
    right_raw = encoder_get_right_count();

    if (s_car_speed_filter_initialized == 0U)
    {
        g_car_speed_left_filtered = (float)left_raw;
        g_car_speed_right_filtered = (float)right_raw;
        s_car_speed_filter_initialized = 1U;
        return;
    }

    g_car_speed_left_filtered += car_speed_filter_alpha *
                                 ((float)left_raw - g_car_speed_left_filtered);
    g_car_speed_right_filtered += car_speed_filter_alpha *
                                  ((float)right_raw - g_car_speed_right_filtered);

    safety_input.link_up = s_car_control_input.link_up;
    safety_input.imu_healthy = s_car_imu_snapshot.healthy;
    safety_input.maintenance_active =
        ((IMUCalib_IsBusy() != 0U) ||
         (car_safety_is_maintenance_requested() != 0U)) ? 1U : 0U;
    safety_input.run_switch_on =
        (s_car_control_input.channel[7] != 0) ? 1U : 0U;
    safety_input.command_neutral = car_run_command_is_neutral();
    safety_input.left_target_speed = Left_Target_Speed;
    safety_input.right_target_speed = Right_Target_Speed;
    safety_input.left_feedback_speed = g_car_speed_left_filtered;
    safety_input.right_feedback_speed = g_car_speed_right_filtered;
    safety_input.left_motor_output = g_car_speed_left_motor_output;
    safety_input.right_motor_output = g_car_speed_right_motor_output;
    safety_input.gyroz_dps = g_car_gyroz_feedback_dps;
    car_safety_update_100HZ(&safety_input);

    if (car_safety_is_output_allowed() == 0U)
    {
        car_total_emergency_stop();
        return;
    }

    /* 遥控器CH5对应CRSF_STD[4]：CH8总使能有效时控制固定40%负压。 */
    if (s_car_control_input.channel[4] != 0)
    {
        if (negative_pressure_is_enabled() == 0U)
        {
            negative_pressure_enable();
        }
        negative_pressure_set_throttle(CAR_NEGATIVE_PRESSURE_FIXED_THROTTLE,
                                       CAR_NEGATIVE_PRESSURE_FIXED_THROTTLE);
    }
    else
    {
        negative_pressure_disable();
    }

    if (CAR_SPEED_LOOP_TUNE_ENABLE != 0U)
    {
        /* 单独调速度环：CH2触发正负阶跃，CH7选择200/400/700目标速度。 */
        g_car_base_speed_command = car_speed_tune_target_get();
        car_speed_plan_update();
        Left_Target_Speed = g_car_base_speed_target;
        Right_Target_Speed = g_car_base_speed_target;

        g_car_yaw_target_deg = s_car_imu_snapshot.euler.yaw;
        g_car_yaw_p_term = 0.0f;
        g_car_yaw_d_term = 0.0f;
        g_car_gyroz_target_dps = 0.0f;
        g_car_gyroz_error = 0.0f;
        g_car_gyroz_ff_term = 0.0f;
        g_car_gyroz_p_term = 0.0f;
        g_car_gyroz_i_term = 0.0f;
        g_car_gyroz_output = 0.0f;
    }
    else
    {
        car_yaw_mode_prepare_100HZ();
        car_world_command_update_100HZ();
        car_speed_plan_update();
        Left_Target_Speed = g_car_base_speed_target;
        Right_Target_Speed = g_car_base_speed_target;
        car_yaw_target_update_100HZ();
    }

    if (s_car_yaw_stopped != 0U)
    {
        if ((Left_Target_Speed == 0.0f) &&
            (car_yaw_command_is_within_stop_limit(
                 CAR_YAW_WAKE_ANGLE_DEG,
                 CAR_GYROZ_WAKE_TARGET_RATE_DPS) != 0U) &&
            (fabsf(g_car_gyroz_feedback_dps) <
             (2.0f * CAR_YAW_STOP_RATE_DPS)) &&
            (fabsf(g_car_speed_left_filtered) <
             (2.0f * CAR_WHEEL_STOP_SPEED)) &&
            (fabsf(g_car_speed_right_filtered) <
             (2.0f * CAR_WHEEL_STOP_SPEED)))
        {
            car_speed_plan_reset();
            Left_Target_Speed = 0.0f;
            Right_Target_Speed = 0.0f;
            g_car_gyroz_target_dps = 0.0f;
            motor_stop();
            return;
        }
        s_car_yaw_stopped = 0U;
    }

    if ((Left_Target_Speed == 0.0f) &&
        (car_yaw_command_is_within_stop_limit(
             CAR_YAW_STOP_ANGLE_DEG,
             CAR_GYROZ_STOP_TARGET_RATE_DPS) != 0U) &&
        (fabsf(g_car_gyroz_feedback_dps) < CAR_YAW_STOP_RATE_DPS) &&
        (fabsf(g_car_speed_left_filtered) < CAR_WHEEL_STOP_SPEED) &&
        (fabsf(g_car_speed_right_filtered) < CAR_WHEEL_STOP_SPEED))
    {
        PID_Reset(&Left_Speed_PID);
        PID_Reset(&Right_Speed_PID);
        PID_Reset(&s_car_yaw_pid);
        PID_Reset(&s_car_gyroz_pid);
        g_car_gyroz_target_dps = 0.0f;
        g_car_gyroz_error = 0.0f;
        g_car_gyroz_ff_term = 0.0f;
        g_car_gyroz_p_term = 0.0f;
        g_car_gyroz_i_term = 0.0f;
        g_car_gyroz_output = 0.0f;
        car_speed_plan_reset();
        Left_Target_Speed = 0.0f;
        Right_Target_Speed = 0.0f;
        s_car_yaw_stopped = 1U;
        motor_stop();
        return;
    }

    if (CAR_SPEED_LOOP_TUNE_ENABLE != 0U)
    {
        /* 防止角速度混合和刹车前馈影响速度环阶跃响应。 */
        car_speed_brake_state_clear();
    }
    else
    {
        car_gyroz_control_100HZ();
        car_speed_brake_state_update(car_world_brake_requested());
    }

    effective_accel_ff =
        car_speed_accel_ff_apply_turn_guard(g_car_base_speed_target,
                                            g_car_base_speed_delta);
    left_accel_ff = effective_accel_ff;
    right_accel_ff = effective_accel_ff;
    left_brake_ff = car_speed_brake_feedforward(
        g_car_speed_left_filtered, s_car_speed_left_brake_direction);
    right_brake_ff = car_speed_brake_feedforward(
        g_car_speed_right_filtered, s_car_speed_right_brake_direction);
    g_car_speed_left_brake_ff = left_brake_ff;
    g_car_speed_right_brake_ff = right_brake_ff;

    left_motor_output = car_speed_pid_update(&Left_Speed_PID,
                                             Left_Target_Speed,
                                             g_car_speed_left_filtered,
                                             car_speed_left_kp,
                                             car_speed_left_ki,
                                             car_speed_left_kd,
                                             left_accel_ff,
                                             left_brake_ff);
    right_motor_output = car_speed_pid_update(&Right_Speed_PID,
                                              Right_Target_Speed,
                                              g_car_speed_right_filtered,
                                              car_speed_right_kp,
                                              car_speed_right_ki,
                                              car_speed_right_kd,
                                              right_accel_ff,
                                              right_brake_ff);
    g_car_speed_left_motor_output = (float)left_motor_output;
    g_car_speed_right_motor_output = (float)right_motor_output;
    motor_left_set_speed(left_motor_output);
    motor_right_set_speed(right_motor_output);
}

void car_loop_motion_100HZ_isr(void)
{
    uint32 imu_tick_now = tick_1000us_cnt;

    if ((g_car_realtime_diag.control_tick_count != 0U) &&
        ((uint32)(imu_tick_now - s_control_last_imu_tick) != 10U))
    {
        g_car_realtime_diag.control_period_fault_count++;
    }
    s_control_last_imu_tick = imu_tick_now;
    g_car_realtime_diag.control_tick_count++;
    s_system_time_ms += 10U;

    CRSF_GetControlSnapshot(&s_car_control_input);
    if (s_car_control_input.link_up == 0U)
    {
        g_car_realtime_diag.command_stale_count++;
    }

    if (IMU_GetRealtimeSnapshot(&s_car_imu_snapshot) == 0U)
    {
        s_car_imu_snapshot.healthy = 0U;
        g_car_realtime_diag.imu_snapshot_fault_count++;
    }
    else
    {
        if (s_car_imu_snapshot.sample_count ==
            s_control_last_imu_sample_count)
        {
            if (s_control_imu_stale_cycles < CAR_IMU_STALE_CONTROL_LIMIT)
            {
                s_control_imu_stale_cycles++;
                if (s_control_imu_stale_cycles ==
                    CAR_IMU_STALE_CONTROL_LIMIT)
                {
                    g_car_realtime_diag.imu_stale_fault_count++;
                }
            }
        }
        else
        {
            s_control_last_imu_sample_count =
                s_car_imu_snapshot.sample_count;
            s_control_imu_stale_cycles = 0U;
        }

        if (s_control_imu_stale_cycles >= CAR_IMU_STALE_CONTROL_LIMIT)
        {
            s_car_imu_snapshot.healthy = 0U;
        }
        if (s_car_imu_snapshot.healthy == 0U)
        {
            g_car_realtime_diag.imu_snapshot_fault_count++;
        }
    }

    car_speed_control_100HZ();
}

void car_loop_release_background_100HZ_isr(void)
{
    g_car_background_100hz_generation++;
}

static void car_loop_background_100HZ(void)
{
    float car_data[11];
    imu_realtime_snapshot_t telemetry_snapshot;
    uint8 menu_runtime_locked;

    air_comm_car_update_100HZ();

    menu_runtime_locked = car_menu_is_runtime_locked();
    if (menu_runtime_locked != 0U)
    {
        if (s_menu_runtime_was_locked == 0U)
        {
            menu_air_abort_param_sync_runtime();
            menu_air_command_abort_runtime();
            menu_runtime_suspend();
        }
        else if (menu_external_view_runtime_active() == 0U)
        {
            menu_discard_key_events();
        }
    }

    if (menu_runtime_locked == 0U)
    {
        if (s_menu_runtime_was_locked != 0U)
        {
            menu_runtime_resume();
        }
        menu_air_command_update_100HZ();
        menu_air_update_100HZ();
        menu_update_100HZ();
    }
    else if (menu_external_view_runtime_active() != 0U)
    {
        menu_update_100HZ();
    }
    s_menu_runtime_was_locked = menu_runtime_locked;

    /* if(air_comm_car_is_online() == 0U)
    {
        if(s_air_comm_beep_tick >= 200U)
        {
            s_air_comm_beep_tick = 0U;
            Beep_Enable();
        }
        else if(s_air_comm_beep_tick == 100U)
        {
            Beep_Disable();
        }
        s_air_comm_beep_tick++;
    }
    else if(s_air_comm_beep_tick != 200U)
    {
        s_air_comm_beep_tick = 200U;
        Beep_Disable();
    } */
    Beep_Update_100HZ();

    car_data[0] = 0.0f;
    car_data[1] = 0.0f;
    car_data[2] = 0.0f;
    (void)IMU_GetRealtimeSnapshot(&telemetry_snapshot);
    car_data[3] = telemetry_snapshot.euler.yaw;
    car_data[4] = g_car_gyroz_feedback_dps;
    car_data[5] = 0.0f;
    car_data[6] = 0.0f;
    car_data[7] = 0.0f;
    car_data[8] = 0.0f;
    car_data[9] = 0.0f;
    car_data[10] = (float)s_system_time_ms;
    air_comm_send_run_data(car_data, 11U);
}

void car_loop_poll(void)
{
    uint32 background_generation;
    uint32 generation_delta;

    CRSF_Poll();
    air_comm_car_poll();
    IMU_ServicePoll();

    background_generation = g_car_background_100hz_generation;
    generation_delta =
        background_generation - s_background_100hz_processed_generation;
    if (generation_delta != 0U)
    {
        s_background_100hz_processed_generation = background_generation;
        if (generation_delta > 1U)
        {
            g_car_realtime_diag.background_coalesced_count +=
                generation_delta - 1U;
        }
        car_loop_background_100HZ();
    }

    wifi_core_Poll();
}
