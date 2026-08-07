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
static uint8 s_car_yaw_control_mode_active = 1U;
static uint8 s_car_speed_brake_active = 0U;
static uint8 s_car_world_command_active = 0U;
static uint8 s_car_world_input_confirm_cycles = 0U;
static uint8 s_car_fixed_drive_active = 0U;
static uint8 s_car_fixed_drive_confirm_cycles = 0U;
static int8 s_car_fixed_drive_direction = 0;
static uint8 s_car_fixed_turn_trigger_previous = 0U;
static uint8 s_car_fixed_turn_active = 0U;
static float s_car_speed_left_brake_direction = 0.0f;
static float s_car_speed_right_brake_direction = 0.0f;
static uint16 s_car_negative_pressure_throttle = 0U;
static uint16 s_car_negative_pressure_longitudinal_stable_cycles = 0U;
static uint16 s_car_negative_pressure_turn_exit_cycles = 0U;
static uint16 s_car_negative_pressure_accel_cycles = 0U;
static uint8 s_car_negative_pressure_longitudinal_state = 0U;
static uint8 s_car_negative_pressure_turn_active = 0U;
static float s_car_negative_pressure_previous_speed_target = 0.0f;
/* 大角度掉头锁存的内部运行状态，均在100 Hz控制周期中更新。 */
/* BRAKE阶段轮速连续达标计数，用于避免阈值附近抖动。 */
static uint16 s_car_large_turn_brake_stable_cycles = 0U;
/* NORMAL阶段大角度指令连续有效计数，达标后才正式锁存掉头。 */
static uint16 s_car_large_turn_trigger_stable_cycles = 0U;
/* EXIT阶段航向误差连续达标计数，达标后才确认掉头完成。 */
static uint16 s_car_large_turn_finish_stable_cycles = 0U;
/* 一次锁存掉头已经执行的控制周期数，用于动作总超时保护。 */
static uint16 s_car_large_turn_elapsed_cycles = 0U;
/* 当前掉头阶段：0正常、1制动、2原地转向、3退出收敛。 */
static uint8 s_car_large_turn_state = 0U;
/* 置1后忽略普通摇杆回中和方向抖动，直到完成或安全退出。 */
static uint8 s_car_large_turn_latched = 0U;
/* 超时退出后置1，要求摇杆先回中一次才能重新触发掉头。 */
static uint8 s_car_large_turn_rearm_required = 0U;
/* 已锁存掉头方向：1为正方向，-1为负方向，0为未锁存。 */
static int8 s_car_large_turn_direction = 0;
/* 触发确认期间的候选方向，用于拒绝正负方向来回跳变。 */
static int8 s_car_large_turn_trigger_direction = 0;
/* 掉头开始时锁存的绝对目标航向，回中后也不会被当前航向覆盖。 */
static float s_car_large_turn_target_yaw_deg = 0.0f;
/* 掉头开始时锁存的速度档，用于EXIT阶段恢复前进分量。 */
static float s_car_large_turn_speed_limit = 0.0f;

#define CAR_SPEED_I_LIMIT (2000.0f)
#define CAR_SPEED_ACCEL_TURN_FULL_RATIO (0.20f)
#define CAR_SPEED_ACCEL_TURN_DISABLE_RATIO (0.50f)
#define CAR_SPEED_PLAN_MIN_STEP (1.0f)
#define CAR_SPEED_SLEW_AND_ACCEL_FF_ENABLE (0U)
#define CAR_SPEED_LOOP_TUNE_ENABLE (0U)
#define CAR_SPEED_LOOP_TUNE_THRESHOLD (300)
#define CAR_IMU_STALE_CONTROL_LIMIT (3U)
#define CAR_PIT_TICKS_PER_US        (8U)

#define CAR_WORLD_SPEED_LIMIT_LOW (200.0f)
#define CAR_WORLD_SPEED_LIMIT_MID (400.0f)
#define CAR_WORLD_SPEED_LIMIT_HIGH (500.0f)
#define CAR_WORLD_ALIGNMENT_STOP_DEG (90.0f)
#define CAR_WHEEL_TARGET_ABS_LIMIT (1000.0f)

#define CAR_GYROZ_DEG_TO_RAD (0.017453292519943295f)
#define CAR_GYROZ_RAD_TO_DEG (57.29577951308232f)
#define CAR_GYROZ_EQUIVALENT_SCALE (28.1448005f)
#define CAR_GYROZ_DEBUG_INPUT_LOW_THRESHOLD (300)
#define CAR_GYROZ_DEBUG_TARGET_LOW_DPS (400.0f)
#define CAR_GYROZ_DEBUG_TARGET_MID_DPS (600.0f)
#define CAR_GYROZ_DEBUG_TARGET_HIGH_DPS (800.0f)
#define CAR_GYROZ_OUTPUT_LIMIT (600.0f)
#define CAR_YAW_STOP_ANGLE_DEG (2.0f)
#define CAR_YAW_WAKE_ANGLE_DEG (1.0f)
#define CAR_GYROZ_STOP_TARGET_RATE_DPS (2.0f)
#define CAR_GYROZ_WAKE_TARGET_RATE_DPS (4.0f)
#define CAR_YAW_STOP_RATE_DPS (9.0f)
#define CAR_WHEEL_STOP_SPEED (8.0f)

#define CAR_NEGATIVE_PRESSURE_LONGITUDINAL_NONE (0U)
#define CAR_NEGATIVE_PRESSURE_LONGITUDINAL_ACCEL (1U)
#define CAR_NEGATIVE_PRESSURE_LONGITUDINAL_BRAKE (2U)

/* 负压输出只保留关闭、平稳、动态加减速和转向四种状态（三个有效油门档）。 */
#define CAR_NEGATIVE_PRESSURE_STATE_DISABLED (0U)
#define CAR_NEGATIVE_PRESSURE_STATE_HOLD (1U)
#define CAR_NEGATIVE_PRESSURE_STATE_DYNAMIC (2U)
#define CAR_NEGATIVE_PRESSURE_STATE_TURN (3U)

#define CAR_YAW_CONTROL_MODE_RATE_DEBUG (0U)
#define CAR_YAW_CONTROL_MODE_WORLD      (1U)
#define CAR_YAW_CONTROL_MODE_FIXED      (2U)

#define CAR_FIXED_TURN_ANGLE_LOW_DEG  (45.0f)
#define CAR_FIXED_TURN_ANGLE_MID_DEG  (90.0f)
#define CAR_FIXED_TURN_ANGLE_HIGH_DEG (180.0f)
#define CAR_FIXED_TURN_RIGHT_DIRECTION (1.0f)

#define CAR_LARGE_TURN_STATE_NORMAL (0U)
#define CAR_LARGE_TURN_STATE_BRAKE (1U)
#define CAR_LARGE_TURN_STATE_PIVOT (2U)
#define CAR_LARGE_TURN_STATE_EXIT (3U)
/* 大角度指令在当前控制周期确认后立即锁存掉头。 */
#define CAR_LARGE_TURN_TRIGGER_STABLE_CYCLES (1U)
/* 航向误差连续2个周期（20 ms）小于Finish，才释放掉头锁存。 */
#define CAR_LARGE_TURN_FINISH_STABLE_CYCLES (2U)
/* 一次掉头最多执行500个周期（5 s），超时后安全停止并等待回中。 */
#define CAR_LARGE_TURN_TIMEOUT_CYCLES (500U)
/* EXIT仅接受与锁存航向相差不超过80°的实时摇杆指令恢复前进。 */
#define CAR_LARGE_TURN_EXIT_COMMAND_MATCH_DEG (80.0f)
/* BRAKE保留目标至少比公共平移速度阈值低5，避免参数相等时迟迟无法进入PIVOT。 */
#define CAR_LARGE_TURN_BRAKE_TARGET_MARGIN (5.0f)
/* BRAKE公共制动前馈从满幅衰减到零所对应的平移速度跨度。 */
#define CAR_LARGE_TURN_BRAKE_FF_FADE_SPAN (40.0f)
/* EXIT恢复起点与完成角之间至少保留1°，避免退化配置产生除零。 */
#define CAR_LARGE_TURN_EXIT_RECOVERY_MIN_SPAN_DEG (1.0f)

volatile float car_speed_left_kp = 10.80f;
volatile float car_speed_left_ki = 0.52f;
volatile float car_speed_left_kd = 1.00f;
volatile float car_speed_right_kp = 10.80f;
volatile float car_speed_right_ki = 0.52f;
volatile float car_speed_right_kd = 1.00f;
volatile float car_speed_filter_alpha = 0.557f;
volatile float car_speed_ff_slope = 0.00f;
volatile float car_speed_ff_static = 800.0f;
volatile float car_speed_ff_deadband = 10.0f;
volatile float car_speed_ff_transition = 100.0f;
volatile float car_speed_brake_static = 800.0f;
volatile float car_speed_delta_output_limit = 6000.0f;
volatile float car_speed_accel_kff = 10.0f;
volatile float car_speed_accel_step_limit = 40.0f;
volatile float car_speed_decel_step_limit = 600.0f;
volatile float car_speed_accel_ff_limit = 800.0f;
volatile float car_gyroz_kff = 0.12f;
/* 角速度PI使用100 Hz每采样周期离散系数，数值已由原连续时间系数等效换算。 */
volatile float car_gyroz_kp = 1.27f;
volatile float car_gyroz_ki = 0.022f;
volatile float car_gyroz_k_turn = 1.0f;
volatile float car_yaw_kp = 6.00f;
volatile float car_yaw_kd = 0.50f;
volatile float car_yaw_rate_limit_dps = 1000.0f;
volatile float car_yaw_control_mode = 1.0f;
volatile float car_negative_pressure_hold_throttle = 4000.0f;
volatile float car_negative_pressure_turn_throttle = 4000.0f;
volatile float car_negative_pressure_boost_throttle = 4000.0f;
volatile float car_negative_pressure_speed_error_abs = 40.0f;
volatile float car_negative_pressure_speed_error_ratio = 0.20f;
volatile float car_negative_pressure_stable_cycles = 5.0f;
volatile float car_negative_pressure_speed_delta_threshold = 50.0f;
volatile float car_negative_pressure_accel_timeout_cycles = 50.0f;
volatile float car_negative_pressure_turn_enter_angle_deg = 20.0f;
volatile float car_negative_pressure_turn_exit_angle_deg = 10.0f;
volatile float car_negative_pressure_turn_enter_target_rate_dps = 100.0f;
volatile float car_negative_pressure_turn_exit_target_rate_dps = 40.0f;
volatile float car_negative_pressure_turn_enter_feedback_rate_dps = 50.0f;
volatile float car_negative_pressure_turn_exit_feedback_rate_dps = 40.0f;
/* 车体公共平移速度低于该值后，BRAKE阶段才允许进入PIVOT。 */
volatile float car_large_turn_brake_speed = 80.0f;
/* BRAKE阶段固定保留的基础目标速度。 */
volatile float car_large_turn_brake_target_speed = 20.0f;
/* BRAKE阶段叠加到左右轮的公共平移制动前馈，不改变主动转向差速。 */
volatile float car_large_turn_brake_ff = 1200.0f;
/* BRAKE阶段限制目标角速度，兼顾快速减速与持续转向。 */
volatile float car_large_turn_brake_rate_limit_dps = 300.0f;
/* 航向误差达到该角度时开始大角度掉头流程。 */
volatile float car_large_turn_enter_angle_deg = 90.0f;
/* 剩余航向误差低于该角度时，从原地转向进入EXIT收敛阶段。 */
volatile float car_large_turn_pivot_exit_angle_deg = 35.0f;
/* EXIT阶段从该误差开始线性恢复前进速度，默认与PIVOT退出角一致。 */
volatile float car_large_turn_exit_speed_start_angle_deg = 35.0f;
/* EXIT阶段剩余航向误差低于该值并稳定后，确认掉头完成。 */
volatile float car_large_turn_finish_angle_deg = 3.0f;
/* BRAKE阶段轮速需要连续满足阈值的周期数，默认1表示立即切换。 */
volatile float car_large_turn_brake_stable_cycles = 1.0f;

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
volatile float g_car_negative_pressure_throttle = 0.0f;
volatile float g_car_negative_pressure_target = 0.0f;
volatile float g_car_negative_pressure_boost = 0.0f;
volatile float g_car_negative_pressure_state = 0.0f;
volatile float g_car_large_turn_state = 0.0f;

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

static void car_gyroz_control_reset(void)
{
    PID_Reset(&s_car_gyroz_pid);
    g_car_gyroz_target_dps = 0.0f;
    g_car_gyroz_error = 0.0f;
    g_car_gyroz_ff_term = 0.0f;
    g_car_gyroz_p_term = 0.0f;
    g_car_gyroz_i_term = 0.0f;
    g_car_gyroz_output = 0.0f;
}

/*
 * 清除一次掉头的全部内部状态，恢复为普通运动模式。
 * 初始化、模式切换和安全急停都会调用；普通摇杆回中不再直接调用。
 */
static void car_large_turn_state_reset(void)
{
    s_car_large_turn_brake_stable_cycles = 0U;
    s_car_large_turn_trigger_stable_cycles = 0U;
    s_car_large_turn_finish_stable_cycles = 0U;
    s_car_large_turn_elapsed_cycles = 0U;
    s_car_large_turn_state = CAR_LARGE_TURN_STATE_NORMAL;
    s_car_large_turn_latched = 0U;
    s_car_large_turn_rearm_required = 0U;
    s_car_large_turn_direction = 0;
    s_car_large_turn_trigger_direction = 0;
    s_car_large_turn_target_yaw_deg = 0.0f;
    s_car_large_turn_speed_limit = 0.0f;
    g_car_large_turn_state = (float)CAR_LARGE_TURN_STATE_NORMAL;
}

/* 正常掉头阶段共享锁存航向，阶段切换仅更新状态并保留控制器历史。 */
static void car_large_turn_state_set(uint8 state)
{
    uint8 previous_state = s_car_large_turn_state;

    if (state == previous_state)
    {
        return;
    }

    s_car_large_turn_state = state;
    s_car_large_turn_brake_stable_cycles = 0U;
    g_car_large_turn_state = (float)state;

    if (state == CAR_LARGE_TURN_STATE_NORMAL)
    {
        s_car_large_turn_latched = 0U;
        s_car_large_turn_elapsed_cycles = 0U;
        s_car_large_turn_finish_stable_cycles = 0U;
        s_car_large_turn_direction = 0;
        s_car_large_turn_target_yaw_deg = 0.0f;
        s_car_large_turn_speed_limit = 0.0f;
    }
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

/* 获取EXIT阶段有效摇杆方向与锁存方向的归一化夹角。 */
static uint8 car_large_turn_exit_command_error_get(float *command_error_deg)
{
    float raw_x = (float)s_car_control_input.channel[1];
    float raw_y = (float)s_car_control_input.channel[0];
    float raw_magnitude = sqrtf(raw_x * raw_x + raw_y * raw_y);
    float command_heading_deg;

    if ((command_error_deg == NULL) ||
        (raw_magnitude < CAR_WORLD_INPUT_ENTER_DEADZONE))
    {
        return 0U;
    }

    command_heading_deg = atan2f(raw_y, raw_x) * CAR_GYROZ_RAD_TO_DEG;
    *command_error_deg = car_angle_wrap_deg(
        command_heading_deg - s_car_large_turn_target_yaw_deg);
    return 1U;
}

/* 摇杆仍指向锁存方向时，EXIT才允许恢复前进速度。 */
static uint8 car_large_turn_exit_forward_requested(void)
{
    float command_error_deg;

    if (car_large_turn_exit_command_error_get(&command_error_deg) == 0U)
    {
        return 0U;
    }

    return (fabsf(command_error_deg) <=
            CAR_LARGE_TURN_EXIT_COMMAND_MATCH_DEG) ? 1U : 0U;
}

/*
 * 将锁存的目标航向和速度档重新写回世界坐标控制量。
 * 这样即使遥控摇杆回中，BRAKE/PIVOT/EXIT仍使用掉头开始时的目标；
 * BRAKE和PIVOT保持基础速度为0；EXIT仅在实时摇杆仍指向锁存目标时
 * 才按剩余误差逐渐恢复前进速度，回中或改向不会产生自动前冲。
 */
static void car_large_turn_apply_latched_command(void)
{
    float yaw_error_deg;
    float yaw_error_abs;
    float alignment_scale = 0.0f;
    float recovery_start_angle_abs;
    float pivot_exit_angle_abs;
    float finish_angle_abs;
    float recovery_progress;
    float recovery_span;
    float target_rad;

    yaw_error_deg = car_angle_wrap_deg(
        s_car_large_turn_target_yaw_deg - s_car_imu_snapshot.euler.yaw);
    yaw_error_abs = fabsf(yaw_error_deg);
    target_rad = s_car_large_turn_target_yaw_deg * CAR_GYROZ_DEG_TO_RAD;

    recovery_start_angle_abs =
        fabsf(car_large_turn_exit_speed_start_angle_deg);
    pivot_exit_angle_abs = fabsf(car_large_turn_pivot_exit_angle_deg);
    finish_angle_abs = fabsf(car_large_turn_finish_angle_deg);
    if (finish_angle_abs > pivot_exit_angle_abs)
    {
        finish_angle_abs = pivot_exit_angle_abs;
    }
    if (recovery_start_angle_abs > pivot_exit_angle_abs)
    {
        recovery_start_angle_abs = pivot_exit_angle_abs;
    }
    if (recovery_start_angle_abs <
        (finish_angle_abs + CAR_LARGE_TURN_EXIT_RECOVERY_MIN_SPAN_DEG))
    {
        recovery_start_angle_abs =
            finish_angle_abs + CAR_LARGE_TURN_EXIT_RECOVERY_MIN_SPAN_DEG;
    }

    /*
     * EXIT达到恢复阈值后按剩余角度线性恢复前进速度；默认恢复阈值
     * 与PIVOT退出角一致，因此进入EXIT后的下一个控制周期开始恢复。
     */
    if ((s_car_large_turn_state == CAR_LARGE_TURN_STATE_EXIT) &&
        (car_large_turn_exit_forward_requested() != 0U) &&
        (yaw_error_abs < recovery_start_angle_abs))
    {
        recovery_span = recovery_start_angle_abs - finish_angle_abs;
        recovery_progress =
            (recovery_start_angle_abs - yaw_error_abs) / recovery_span;
        recovery_progress = car_speed_clampf(recovery_progress, 0.0f, 1.0f);
        alignment_scale = recovery_progress;
    }

    s_car_world_command_active = 1U;
    g_car_yaw_target_deg = s_car_large_turn_target_yaw_deg;
    g_car_base_speed_command = s_car_large_turn_speed_limit * alignment_scale;
    g_car_world_velocity_x_command =
        cosf(target_rad) * s_car_large_turn_speed_limit;
    g_car_world_velocity_y_command =
        sinf(target_rad) * s_car_large_turn_speed_limit;
    g_car_world_speed_magnitude = s_car_large_turn_speed_limit;
    g_car_world_speed_limit = s_car_large_turn_speed_limit;
    g_car_world_heading_target_deg = s_car_large_turn_target_yaw_deg;
    g_car_world_heading_error_deg = yaw_error_deg;
    g_car_world_alignment_scale = alignment_scale;
    g_car_world_reverse_active = 0.0f;
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
    s_car_world_input_confirm_cycles = 0U;
    s_car_fixed_drive_active = 0U;
    s_car_fixed_drive_confirm_cycles = 0U;
    s_car_fixed_drive_direction = 0;
    s_car_fixed_turn_trigger_previous =
        (s_car_control_input.channel[8] != 0) ? 1U : 0U;
    s_car_fixed_turn_active = 0U;
    car_large_turn_state_reset();

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

static float car_fixed_turn_angle_get(void)
{
    if (s_car_control_input.channel[5] == 0)
    {
        return CAR_FIXED_TURN_ANGLE_LOW_DEG;
    }
    if (s_car_control_input.channel[5] == 1)
    {
        return CAR_FIXED_TURN_ANGLE_MID_DEG;
    }
    return CAR_FIXED_TURN_ANGLE_HIGH_DEG;
}

static float car_fixed_drive_command_get(void)
{
    float input = (float)s_car_control_input.channel[1];
    float input_abs = fabsf(input);
    int8 direction = (input >= 0.0f) ? 1 : -1;

    if (s_car_fixed_drive_active != 0U)
    {
        if (input_abs <= CAR_WORLD_INPUT_EXIT_DEADZONE)
        {
            s_car_fixed_drive_active = 0U;
            s_car_fixed_drive_confirm_cycles = 0U;
            s_car_fixed_drive_direction = 0;
        }
        else if (input_abs >= CAR_WORLD_INPUT_ENTER_DEADZONE)
        {
            s_car_fixed_drive_direction = direction;
        }
    }
    else if (input_abs >= CAR_WORLD_INPUT_ENTER_DEADZONE)
    {
        if (direction != s_car_fixed_drive_direction)
        {
            s_car_fixed_drive_direction = direction;
            s_car_fixed_drive_confirm_cycles = 1U;
        }
        else if (s_car_fixed_drive_confirm_cycles <
                 CAR_WORLD_INPUT_CONFIRM_CYCLES)
        {
            s_car_fixed_drive_confirm_cycles++;
        }

        if (s_car_fixed_drive_confirm_cycles >=
            CAR_WORLD_INPUT_CONFIRM_CYCLES)
        {
            s_car_fixed_drive_active = 1U;
            s_car_fixed_drive_confirm_cycles = 0U;
        }
    }
    else
    {
        s_car_fixed_drive_confirm_cycles = 0U;
        s_car_fixed_drive_direction = 0;
    }

    if (s_car_fixed_drive_active == 0U)
    {
        return 0.0f;
    }
    return (float)s_car_fixed_drive_direction * car_world_speed_limit_get();
}

static void car_fixed_angle_command_update_100HZ(void)
{
    uint8 trigger_active =
        (s_car_control_input.channel[8] != 0) ? 1U : 0U;
    float speed_command;
    float yaw_error_deg;
    float alignment_scale;
    float target_rad;

    g_car_world_speed_limit = car_world_speed_limit_get();
    g_car_world_body_speed_feedback =
        0.5f * (g_car_speed_left_filtered + g_car_speed_right_filtered);

    if ((trigger_active != 0U) &&
        (s_car_fixed_turn_trigger_previous == 0U))
    {
        g_car_yaw_target_deg = car_angle_wrap_deg(
            s_car_imu_snapshot.euler.yaw +
            CAR_FIXED_TURN_RIGHT_DIRECTION * car_fixed_turn_angle_get());
        s_car_fixed_turn_active = 1U;
    }
    s_car_fixed_turn_trigger_previous = trigger_active;

    yaw_error_deg = car_angle_wrap_deg(
        g_car_yaw_target_deg - s_car_imu_snapshot.euler.yaw);
    if (s_car_fixed_turn_active != 0U)
    {
        yaw_error_deg = CAR_FIXED_TURN_RIGHT_DIRECTION *
                        fabsf(yaw_error_deg);
        if (fabsf(yaw_error_deg) <= CAR_YAW_WAKE_ANGLE_DEG)
        {
            s_car_fixed_turn_active = 0U;
        }
    }

    if (fabsf(yaw_error_deg) >= CAR_WORLD_ALIGNMENT_STOP_DEG)
    {
        alignment_scale = 0.0f;
    }
    else
    {
        alignment_scale = cosf(yaw_error_deg * CAR_GYROZ_DEG_TO_RAD);
        alignment_scale = car_speed_clampf(alignment_scale, 0.0f, 1.0f);
    }

    speed_command = car_fixed_drive_command_get();
    g_car_base_speed_command = speed_command * alignment_scale;
    s_car_world_command_active =
        ((s_car_fixed_drive_active != 0U) ||
         (fabsf(yaw_error_deg) > CAR_YAW_STOP_ANGLE_DEG)) ? 1U : 0U;

    target_rad = g_car_yaw_target_deg * CAR_GYROZ_DEG_TO_RAD;
    g_car_world_velocity_x_command = cosf(target_rad) * speed_command;
    g_car_world_velocity_y_command = sinf(target_rad) * speed_command;
    g_car_world_speed_magnitude = fabsf(speed_command);
    g_car_world_heading_target_deg = g_car_yaw_target_deg;
    g_car_world_heading_error_deg = yaw_error_deg;
    g_car_world_alignment_scale = alignment_scale;
    g_car_world_reverse_active = (speed_command < 0.0f) ? 1.0f : 0.0f;
}

static void car_world_command_update_100HZ(void)
{
    float raw_x;
    float raw_y;
    float raw_magnitude;
    float world_heading_deg;
    float heading_error_deg;
    float alignment_scale;
    float desired_speed;
    float target_rad;
    float exit_command_error_deg;
    uint8 input_ready = 0U;
    uint8 heading_update_requested = 0U;

    g_car_world_speed_limit = car_world_speed_limit_get();
    g_car_world_body_speed_feedback =
        0.5f * (g_car_speed_left_filtered + g_car_speed_right_filtered);

    /* 世界坐标模式以外的目标由各自的上层生成器负责。 */
    if (s_car_yaw_control_mode_active != CAR_YAW_CONTROL_MODE_WORLD)
    {
        s_car_world_command_active = 0U;
        s_car_world_input_confirm_cycles = 0U;
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

    /* EXIT收到明显的新方向时释放旧锁存，并在当前周期解析新目标。 */
    if (s_car_large_turn_latched != 0U)
    {
        if ((s_car_large_turn_state == CAR_LARGE_TURN_STATE_EXIT) &&
            (car_large_turn_exit_command_error_get(
                 &exit_command_error_deg) != 0U) &&
            (fabsf(exit_command_error_deg) >
             CAR_LARGE_TURN_EXIT_COMMAND_MATCH_DEG))
        {
            car_large_turn_state_set(CAR_LARGE_TURN_STATE_NORMAL);
        }
        else
        {
            car_large_turn_apply_latched_command();
            return;
        }
    }

    raw_x = (float)s_car_control_input.channel[1];
    raw_y = (float)s_car_control_input.channel[0];
    raw_magnitude = sqrtf(raw_x * raw_x + raw_y * raw_y);

    /* 掉头超时后必须先回中，防止持续故障时反复重新触发。 */
    if (s_car_large_turn_rearm_required != 0U)
    {
        if (raw_magnitude <= CAR_WORLD_INPUT_EXIT_DEADZONE)
        {
            s_car_large_turn_rearm_required = 0U;
        }
        else
        {
            s_car_world_command_active = 0U;
            s_car_world_input_confirm_cycles = 0U;
            g_car_yaw_target_deg = s_car_imu_snapshot.euler.yaw;
            g_car_base_speed_command = 0.0f;
            g_car_world_velocity_x_command = 0.0f;
            g_car_world_velocity_y_command = 0.0f;
            g_car_world_speed_magnitude = 0.0f;
            g_car_world_heading_target_deg = g_car_yaw_target_deg;
            g_car_world_heading_error_deg = 0.0f;
            g_car_world_alignment_scale = 0.0f;
            g_car_world_reverse_active = 0.0f;
            return;
        }
    }

    if (s_car_world_command_active != 0U)
    {
        if (raw_magnitude > CAR_WORLD_INPUT_EXIT_DEADZONE)
        {
            input_ready = 1U;
            heading_update_requested =
                (raw_magnitude >= CAR_WORLD_INPUT_ENTER_DEADZONE) ? 1U : 0U;
        }
    }
    else if (raw_magnitude >= CAR_WORLD_INPUT_ENTER_DEADZONE)
    {
        if (s_car_world_input_confirm_cycles <
            CAR_WORLD_INPUT_CONFIRM_CYCLES)
        {
            s_car_world_input_confirm_cycles++;
        }
        if (s_car_world_input_confirm_cycles >=
            CAR_WORLD_INPUT_CONFIRM_CYCLES)
        {
            input_ready = 1U;
            heading_update_requested = 1U;
        }
    }
    else
    {
        s_car_world_input_confirm_cycles = 0U;
    }

    if (input_ready == 0U)
    {
        if (s_car_world_command_active != 0U)
        {
            s_car_world_command_active = 0U;
            s_car_world_input_confirm_cycles = 0U;
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
    s_car_world_input_confirm_cycles = 0U;

    /* 迟滞区间保持上一次有效方向，只在明确离开中心后更新atan2。 */
    if (heading_update_requested != 0U)
    {
        world_heading_deg = atan2f(raw_y, raw_x) * CAR_GYROZ_RAD_TO_DEG;
    }
    else
    {
        world_heading_deg = g_car_yaw_target_deg;
    }

    /* CH1/CH2 only define direction; CH7 selects the speed magnitude. */
    target_rad = world_heading_deg * CAR_GYROZ_DEG_TO_RAD;
    g_car_world_velocity_x_command = cosf(target_rad) * g_car_world_speed_limit;
    g_car_world_velocity_y_command = sinf(target_rad) * g_car_world_speed_limit;
    g_car_world_speed_magnitude = g_car_world_speed_limit;

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
    float brake_target;
    float brake_target_max;
    float step;
    float command_delta;
    float accel_limit = fabsf(car_speed_accel_ff_limit);

    /*
     * 大角度转向制动阶段只允许左右轮使用同一个固定保留速度。
     * 保留目标自动限制在公共平移速度判定阈值以下，防止错误菜单参数
     * 使速度环稳定在判定阈值之上，导致BRAKE无法进入PIVOT。
     */
    if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_BRAKE)
    {
        brake_target = fabsf(car_large_turn_brake_target_speed);
        brake_target_max = fabsf(car_large_turn_brake_speed) -
                           CAR_LARGE_TURN_BRAKE_TARGET_MARGIN;
        if (brake_target_max < 0.0f)
        {
            brake_target_max = 0.0f;
        }
        if (brake_target > brake_target_max)
        {
            brake_target = brake_target_max;
        }

        step = car_speed_plan_step(car_speed_decel_step_limit);
        command_delta = brake_target - previous_target;
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
        g_car_speed_accel_ff = 0.0f;
        return;
    }

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

/*
 * 大角度BRAKE只制动车体公共平移分量：左右轮叠加相同前馈，
 * 保留角速度内环生成的差速；接近PIVOT切换阈值时线性退出，避免反冲。
 */
static float car_large_turn_brake_feedforward(void)
{
    float body_speed;
    float body_speed_abs;
    float brake_ff_abs = fabsf(car_large_turn_brake_ff);
    float release_speed;
    float brake_scale;

    if ((s_car_large_turn_state != CAR_LARGE_TURN_STATE_BRAKE) ||
        (brake_ff_abs <= 0.0f))
    {
        return 0.0f;
    }

    body_speed = 0.5f * (g_car_speed_left_filtered +
                         g_car_speed_right_filtered);
    body_speed_abs = fabsf(body_speed);
    release_speed = fabsf(car_large_turn_brake_speed);
    if (body_speed_abs <= release_speed)
    {
        return 0.0f;
    }

    if (release_speed < CAR_SPEED_PLAN_MIN_STEP)
    {
        release_speed = CAR_SPEED_PLAN_MIN_STEP;
    }
    brake_scale =
        (body_speed_abs - release_speed) /
        CAR_LARGE_TURN_BRAKE_FF_FADE_SPAN;
    brake_scale = car_speed_clampf(brake_scale, 0.0f, 1.0f);

    return -car_speed_direction(body_speed, 0.0f) *
           brake_ff_abs * brake_scale;
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
    if (s_car_yaw_control_mode_active != CAR_YAW_CONTROL_MODE_RATE_DEBUG)
    {
        return (fabsf(car_yaw_get_error_deg()) < angle_limit_deg) ? 1U : 0U;
    }

    return (fabsf(g_car_gyroz_target_dps) < rate_limit_dps) ? 1U : 0U;
}

static uint16 car_negative_pressure_float_to_u16(float value,
                                                 uint16 min_value,
                                                 uint16 max_value)
{
    value = car_speed_clampf(value, (float)min_value, (float)max_value);
    return (uint16)(value + 0.5f);
}

static uint16 car_large_turn_brake_stable_cycles_get(void)
{
    return car_negative_pressure_float_to_u16(
        car_large_turn_brake_stable_cycles, 1U, 100U);
}

/*
 * 掉头执行超过2秒时进行安全退出：取消锁存、停止运动并清除控制器历史。
 * 同时设置重新使能标志，必须检测到摇杆回中后才允许再次触发掉头。
 */
static void car_large_turn_timeout_abort(void)
{
    car_large_turn_state_reset();
    s_car_large_turn_rearm_required = 1U;
    s_car_world_command_active = 0U;
    s_car_world_input_confirm_cycles = 0U;

    g_car_base_speed_command = 0.0f;
    g_car_yaw_target_deg = s_car_imu_snapshot.euler.yaw;
    g_car_world_velocity_x_command = 0.0f;
    g_car_world_velocity_y_command = 0.0f;
    g_car_world_speed_magnitude = 0.0f;
    g_car_world_heading_target_deg = g_car_yaw_target_deg;
    g_car_world_heading_error_deg = 0.0f;
    g_car_world_alignment_scale = 0.0f;
    g_car_world_reverse_active = 0.0f;
    car_yaw_outer_reset();
    car_gyroz_control_reset();
}

/*
 * 100 Hz大角度掉头状态机：
 * NORMAL确认指令 -> BRAKE减速 -> PIVOT原地旋转 -> EXIT收敛 -> NORMAL。
 * 一旦进入BRAKE或PIVOT，普通回中不再打断流程；只有完成、超时或安全急停退出。
 */
static void car_large_turn_state_update_100HZ(void)
{
    uint16 stable_cycles = car_large_turn_brake_stable_cycles_get();
    float yaw_error_deg = car_yaw_get_error_deg();
    float yaw_error_abs = fabsf(yaw_error_deg);
    float enter_angle_abs = fabsf(car_large_turn_enter_angle_deg);
    float pivot_exit_angle_abs =
        fabsf(car_large_turn_pivot_exit_angle_deg);
    float finish_angle_abs = fabsf(car_large_turn_finish_angle_deg);
    float brake_speed_abs = fabsf(car_large_turn_brake_speed);
    float body_speed_feedback;
    uint8 translation_slow;
    int8 trigger_direction;

    if (s_car_yaw_control_mode_active != CAR_YAW_CONTROL_MODE_WORLD)
    {
        car_large_turn_state_reset();
        return;
    }

    if ((s_car_large_turn_state == CAR_LARGE_TURN_STATE_NORMAL) &&
        (s_car_world_command_active == 0U))
    {
        s_car_large_turn_trigger_stable_cycles = 0U;
        s_car_large_turn_trigger_direction = 0;
        return;
    }

    if (s_car_large_turn_latched != 0U)
    {
        if (s_car_large_turn_elapsed_cycles < CAR_LARGE_TURN_TIMEOUT_CYCLES)
        {
            s_car_large_turn_elapsed_cycles++;
        }
        if (s_car_large_turn_elapsed_cycles >= CAR_LARGE_TURN_TIMEOUT_CYCLES)
        {
            car_large_turn_timeout_abort();
            return;
        }
    }

    if (enter_angle_abs < pivot_exit_angle_abs)
    {
        enter_angle_abs = pivot_exit_angle_abs;
    }
    if (finish_angle_abs > pivot_exit_angle_abs)
    {
        finish_angle_abs = pivot_exit_angle_abs;
    }

    body_speed_feedback = 0.5f * (g_car_speed_left_filtered +
                                  g_car_speed_right_filtered);
    translation_slow = (fabsf(body_speed_feedback) <= brake_speed_abs)
                           ? 1U : 0U;

    if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_NORMAL)
    {
        /* 连续确认大角度指令及其方向，避免单帧抖动误触发。 */
        if (yaw_error_abs >= enter_angle_abs)
        {
            trigger_direction = (yaw_error_deg >= 0.0f) ? 1 : -1;
            if (trigger_direction != s_car_large_turn_trigger_direction)
            {
                s_car_large_turn_trigger_direction = trigger_direction;
                s_car_large_turn_trigger_stable_cycles = 1U;
            }
            else if (s_car_large_turn_trigger_stable_cycles <
                     CAR_LARGE_TURN_TRIGGER_STABLE_CYCLES)
            {
                s_car_large_turn_trigger_stable_cycles++;
            }

            if (s_car_large_turn_trigger_stable_cycles >=
                CAR_LARGE_TURN_TRIGGER_STABLE_CYCLES)
            {
                s_car_large_turn_latched = 1U;
                s_car_large_turn_direction = trigger_direction;
                s_car_large_turn_target_yaw_deg = g_car_yaw_target_deg;
                s_car_large_turn_speed_limit = fabsf(g_car_world_speed_limit);
                if (s_car_large_turn_speed_limit < CAR_SPEED_PLAN_MIN_STEP)
                {
                    s_car_large_turn_speed_limit = car_world_speed_limit_get();
                }
                s_car_large_turn_elapsed_cycles = 0U;
                s_car_large_turn_trigger_stable_cycles = 0U;
                s_car_large_turn_trigger_direction = 0;
                car_large_turn_state_set(
                    (translation_slow != 0U) ? CAR_LARGE_TURN_STATE_PIVOT
                                             : CAR_LARGE_TURN_STATE_BRAKE);
            }
        }
        else
        {
            s_car_large_turn_trigger_stable_cycles = 0U;
            s_car_large_turn_trigger_direction = 0;
        }
    }
    else if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_BRAKE)
    {
        /* 先降低车体公共平移速度，满足阈值后立即进入原地转向。 */
        if (yaw_error_abs <= pivot_exit_angle_abs)
        {
            car_large_turn_state_set(CAR_LARGE_TURN_STATE_EXIT);
        }
        else if (translation_slow != 0U)
        {
            if (s_car_large_turn_brake_stable_cycles < stable_cycles)
            {
                s_car_large_turn_brake_stable_cycles++;
            }
            if (s_car_large_turn_brake_stable_cycles >= stable_cycles)
            {
                car_large_turn_state_set(CAR_LARGE_TURN_STATE_PIVOT);
            }
        }
        else
        {
            s_car_large_turn_brake_stable_cycles = 0U;
        }
    }
    else if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_PIVOT)
    {
        /* 使用锁存方向原地旋转，剩余误差足够小时进入EXIT。 */
        if (yaw_error_abs <= pivot_exit_angle_abs)
        {
            car_large_turn_state_set(CAR_LARGE_TURN_STATE_EXIT);
        }
    }
    else
    {
        /* EXIT允许恢复前进分量，连续达到Finish条件后释放锁存。 */
        if (yaw_error_abs <= finish_angle_abs)
        {
            if (s_car_large_turn_finish_stable_cycles <
                CAR_LARGE_TURN_FINISH_STABLE_CYCLES)
            {
                s_car_large_turn_finish_stable_cycles++;
            }
            if (s_car_large_turn_finish_stable_cycles >=
                CAR_LARGE_TURN_FINISH_STABLE_CYCLES)
            {
                car_large_turn_state_set(CAR_LARGE_TURN_STATE_NORMAL);
            }
        }
        else
        {
            s_car_large_turn_finish_stable_cycles = 0U;
        }
    }

    if ((s_car_large_turn_state == CAR_LARGE_TURN_STATE_BRAKE) ||
        (s_car_large_turn_state == CAR_LARGE_TURN_STATE_PIVOT))
    {
        g_car_base_speed_command = 0.0f;
    }
}

static uint16 car_negative_pressure_stable_cycles_get(void)
{
    return car_negative_pressure_float_to_u16(
        car_negative_pressure_stable_cycles, 1U, 1000U);
}

static uint16 car_negative_pressure_accel_timeout_cycles_get(void)
{
    return car_negative_pressure_float_to_u16(
        car_negative_pressure_accel_timeout_cycles, 1U, 1000U);
}

static float car_negative_pressure_speed_error_limit_get(float target_abs)
{
    float error_limit = fabsf(car_negative_pressure_speed_error_abs);
    float ratio_limit = fabsf(car_negative_pressure_speed_error_ratio) *
                        target_abs;

    return (ratio_limit > error_limit) ? ratio_limit : error_limit;
}

static void car_negative_pressure_state_reset(void)
{
    s_car_negative_pressure_throttle = 0U;
    s_car_negative_pressure_longitudinal_stable_cycles = 0U;
    s_car_negative_pressure_turn_exit_cycles = 0U;
    s_car_negative_pressure_accel_cycles = 0U;
    s_car_negative_pressure_longitudinal_state =
        CAR_NEGATIVE_PRESSURE_LONGITUDINAL_NONE;
    s_car_negative_pressure_turn_active = 0U;
    s_car_negative_pressure_previous_speed_target = 0.0f;
    g_car_negative_pressure_throttle = 0.0f;
    g_car_negative_pressure_target = 0.0f;
    g_car_negative_pressure_boost = 0.0f;
    g_car_negative_pressure_state =
        (float)CAR_NEGATIVE_PRESSURE_STATE_DISABLED;
}

static void car_negative_pressure_longitudinal_state_set(uint8 state)
{
    s_car_negative_pressure_longitudinal_state = state;
    s_car_negative_pressure_longitudinal_stable_cycles = 0U;
    s_car_negative_pressure_accel_cycles = 0U;
}

static void car_negative_pressure_turn_state_update(uint16 stable_cycles,
                                                    float yaw_error_abs,
                                                    float target_rate_abs,
                                                    float feedback_rate_abs)
{
    float enter_angle_abs =
        fabsf(car_negative_pressure_turn_enter_angle_deg);
    float exit_angle_abs =
        fabsf(car_negative_pressure_turn_exit_angle_deg);
    float enter_target_rate_abs =
        fabsf(car_negative_pressure_turn_enter_target_rate_dps);
    float exit_target_rate_abs =
        fabsf(car_negative_pressure_turn_exit_target_rate_dps);
    float enter_feedback_rate_abs =
        fabsf(car_negative_pressure_turn_enter_feedback_rate_dps);
    float exit_feedback_rate_abs =
        fabsf(car_negative_pressure_turn_exit_feedback_rate_dps);

    if (enter_angle_abs < exit_angle_abs)
    {
        enter_angle_abs = exit_angle_abs;
    }
    if (enter_target_rate_abs < exit_target_rate_abs)
    {
        enter_target_rate_abs = exit_target_rate_abs;
    }
    if (enter_feedback_rate_abs < exit_feedback_rate_abs)
    {
        enter_feedback_rate_abs = exit_feedback_rate_abs;
    }

    if (s_car_negative_pressure_turn_active == 0U)
    {
        if ((((s_car_yaw_control_mode_active != CAR_YAW_CONTROL_MODE_RATE_DEBUG) &&
              (yaw_error_abs >= enter_angle_abs))) ||
            (target_rate_abs >= enter_target_rate_abs) ||
            (feedback_rate_abs >= enter_feedback_rate_abs))
        {
            s_car_negative_pressure_turn_active = 1U;
            s_car_negative_pressure_turn_exit_cycles = 0U;
        }
        return;
    }

    if ((((s_car_yaw_control_mode_active != CAR_YAW_CONTROL_MODE_RATE_DEBUG) &&
          (yaw_error_abs >= exit_angle_abs))) ||
        (target_rate_abs >= exit_target_rate_abs) ||
        (feedback_rate_abs >= exit_feedback_rate_abs))
    {
        s_car_negative_pressure_turn_exit_cycles = 0U;
        return;
    }

    if (s_car_negative_pressure_turn_exit_cycles < stable_cycles)
    {
        s_car_negative_pressure_turn_exit_cycles++;
    }
    if (s_car_negative_pressure_turn_exit_cycles >= stable_cycles)
    {
        s_car_negative_pressure_turn_active = 0U;
        s_car_negative_pressure_turn_exit_cycles = 0U;
    }
}

static uint8 car_negative_pressure_demand_update(void)
{
    uint16 stable_cycles = car_negative_pressure_stable_cycles_get();
    uint16 accel_timeout_cycles =
        car_negative_pressure_accel_timeout_cycles_get();
    float target = g_car_base_speed_target;
    float target_abs = fabsf(target);
    float previous_target_abs =
        fabsf(s_car_negative_pressure_previous_speed_target);
    float target_delta = target_abs - previous_target_abs;
    float feedback = 0.5f * (g_car_speed_left_filtered +
                             g_car_speed_right_filtered);
    float feedback_abs = fabsf(feedback);
    float error_limit =
        car_negative_pressure_speed_error_limit_get(target_abs);
    float speed_delta_threshold =
        fabsf(car_negative_pressure_speed_delta_threshold);
    float yaw_error_abs =
        (s_car_yaw_control_mode_active != CAR_YAW_CONTROL_MODE_RATE_DEBUG)
                              ? fabsf(car_yaw_get_error_deg()) : 0.0f;
    float target_rate_abs = fabsf(g_car_gyroz_target_dps);
    float feedback_rate_abs = fabsf(g_car_gyroz_feedback_dps);
    float accel_exit_speed;
    uint8 demand_state = CAR_NEGATIVE_PRESSURE_STATE_HOLD;

    if (speed_delta_threshold < CAR_SPEED_PLAN_MIN_STEP)
    {
        speed_delta_threshold = CAR_SPEED_PLAN_MIN_STEP;
    }
    car_negative_pressure_turn_state_update(stable_cycles,
                                            yaw_error_abs,
                                            target_rate_abs,
                                            feedback_rate_abs);

    /* 大角度转向由运动状态机唯一决定负压档位，避免重复阈值互相冲突。 */
    if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_BRAKE)
    {
        car_negative_pressure_longitudinal_state_set(
            CAR_NEGATIVE_PRESSURE_LONGITUDINAL_NONE);
        s_car_negative_pressure_previous_speed_target = target;
        return CAR_NEGATIVE_PRESSURE_STATE_DYNAMIC;
    }
    if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_PIVOT)
    {
        car_negative_pressure_longitudinal_state_set(
            CAR_NEGATIVE_PRESSURE_LONGITUDINAL_NONE);
        s_car_negative_pressure_previous_speed_target = target;
        return CAR_NEGATIVE_PRESSURE_STATE_TURN;
    }
    if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_EXIT)
    {
        car_negative_pressure_longitudinal_state_set(
            CAR_NEGATIVE_PRESSURE_LONGITUDINAL_NONE);
        s_car_negative_pressure_previous_speed_target = target;

        /* EXIT仍在修正航向，统一使用固定转向档，不再按速度切换增压档。 */
        return CAR_NEGATIVE_PRESSURE_STATE_TURN;
    }

    /* 只由新的速度目标变化进入纵向增压，普通稳态速度误差不再触发。 */
    if ((target_delta <= -speed_delta_threshold) &&
        (feedback_abs > (target_abs + CAR_WHEEL_STOP_SPEED)))
    {
        car_negative_pressure_longitudinal_state_set(
            CAR_NEGATIVE_PRESSURE_LONGITUDINAL_BRAKE);
    }
    else if ((target_delta >= speed_delta_threshold) &&
             (target_abs > (feedback_abs + CAR_WHEEL_STOP_SPEED)))
    {
        car_negative_pressure_longitudinal_state_set(
            CAR_NEGATIVE_PRESSURE_LONGITUDINAL_ACCEL);
    }

    if (s_car_negative_pressure_longitudinal_state ==
        CAR_NEGATIVE_PRESSURE_LONGITUDINAL_ACCEL)
    {
        if (s_car_negative_pressure_accel_cycles < accel_timeout_cycles)
        {
            s_car_negative_pressure_accel_cycles++;
        }

        accel_exit_speed = target_abs - error_limit;
        if (accel_exit_speed < 0.0f)
        {
            accel_exit_speed = 0.0f;
        }

        if (feedback_abs >= accel_exit_speed)
        {
            if (s_car_negative_pressure_longitudinal_stable_cycles <
                stable_cycles)
            {
                s_car_negative_pressure_longitudinal_stable_cycles++;
            }
        }
        else
        {
            s_car_negative_pressure_longitudinal_stable_cycles = 0U;
        }

        if ((s_car_negative_pressure_longitudinal_stable_cycles >=
             stable_cycles) ||
            (s_car_negative_pressure_accel_cycles >=
             accel_timeout_cycles))
        {
            car_negative_pressure_longitudinal_state_set(
                CAR_NEGATIVE_PRESSURE_LONGITUDINAL_NONE);
        }
    }
    else if (s_car_negative_pressure_longitudinal_state ==
             CAR_NEGATIVE_PRESSURE_LONGITUDINAL_BRAKE)
    {
        if (feedback_abs <= (target_abs + error_limit))
        {
            if (s_car_negative_pressure_longitudinal_stable_cycles <
                stable_cycles)
            {
                s_car_negative_pressure_longitudinal_stable_cycles++;
            }
        }
        else
        {
            s_car_negative_pressure_longitudinal_stable_cycles = 0U;
        }

        if (s_car_negative_pressure_longitudinal_stable_cycles >=
            stable_cycles)
        {
            car_negative_pressure_longitudinal_state_set(
                CAR_NEGATIVE_PRESSURE_LONGITUDINAL_NONE);
        }
    }

    /* 普通模式优先保障明显加减速，速度稳定后再切换到固定转向档。 */
    if (s_car_negative_pressure_longitudinal_state ==
        CAR_NEGATIVE_PRESSURE_LONGITUDINAL_ACCEL)
    {
        demand_state = CAR_NEGATIVE_PRESSURE_STATE_DYNAMIC;
    }
    else if (s_car_negative_pressure_longitudinal_state ==
             CAR_NEGATIVE_PRESSURE_LONGITUDINAL_BRAKE)
    {
        demand_state = CAR_NEGATIVE_PRESSURE_STATE_DYNAMIC;
    }
    else if (s_car_negative_pressure_turn_active != 0U)
    {
        demand_state = CAR_NEGATIVE_PRESSURE_STATE_TURN;
    }

    s_car_negative_pressure_previous_speed_target = target;
    return demand_state;
}

static void car_negative_pressure_control_100HZ(void)
{
    uint16 hold_throttle;
    uint16 turn_throttle;
    uint16 boost_throttle;
    uint16 target_throttle;
    uint8 demand_state;

    if (s_car_control_input.channel[4] == 0)
    {
        negative_pressure_disable();
        car_negative_pressure_state_reset();
        return;
    }

    hold_throttle = car_negative_pressure_float_to_u16(
        car_negative_pressure_hold_throttle,
        0U,
        NEGATIVE_PRESSURE_THROTTLE_LIMIT_MAX);
    boost_throttle = car_negative_pressure_float_to_u16(
        car_negative_pressure_boost_throttle,
        hold_throttle,
        NEGATIVE_PRESSURE_THROTTLE_LIMIT_MAX);
    turn_throttle = car_negative_pressure_float_to_u16(
        car_negative_pressure_turn_throttle,
        hold_throttle,
        boost_throttle);

    if (negative_pressure_is_enabled() == 0U)
    {
        negative_pressure_enable();
    }

    demand_state = car_negative_pressure_demand_update();

    if (demand_state == CAR_NEGATIVE_PRESSURE_STATE_DYNAMIC)
    {
        target_throttle = boost_throttle;
    }
    else if (demand_state == CAR_NEGATIVE_PRESSURE_STATE_TURN)
    {
        target_throttle = turn_throttle;
    }
    else
    {
        target_throttle = hold_throttle;
    }

    /* 档位差较小，直接到达目标，避免软件斜坡延迟负压建立。 */
    s_car_negative_pressure_throttle = target_throttle;

    negative_pressure_set_throttle(s_car_negative_pressure_throttle,
                                   s_car_negative_pressure_throttle);
    g_car_negative_pressure_throttle =
        (float)s_car_negative_pressure_throttle;
    g_car_negative_pressure_target = (float)target_throttle;
    g_car_negative_pressure_boost =
        (demand_state == CAR_NEGATIVE_PRESSURE_STATE_DYNAMIC) ? 1.0f : 0.0f;
    g_car_negative_pressure_state = (float)demand_state;
}

void car_yaw_control_100HZ(void)
{
    float desired_rate_dps;
    float yaw_error_deg;
    float brake_rate_limit_abs;

    yaw_error_deg = car_yaw_get_error_deg();

    /* 固定180度右转时锁存方向，接近目标后恢复普通误差符号。 */
    if ((s_car_yaw_control_mode_active == CAR_YAW_CONTROL_MODE_FIXED) &&
        (s_car_fixed_turn_active != 0U))
    {
        yaw_error_deg = CAR_FIXED_TURN_RIGHT_DIRECTION *
                        fabsf(yaw_error_deg);
    }
    /* 大角度制动和原地转向阶段锁存方向，避免正负180度边界翻转。 */
    else if (((s_car_large_turn_state == CAR_LARGE_TURN_STATE_BRAKE) ||
         (s_car_large_turn_state == CAR_LARGE_TURN_STATE_PIVOT)) &&
        (s_car_large_turn_direction != 0))
    {
        yaw_error_deg = (float)s_car_large_turn_direction *
                        fabsf(yaw_error_deg);
    }

    s_car_yaw_pid.kp = car_yaw_kp;
    s_car_yaw_pid.ki = 0.0f;
    s_car_yaw_pid.kd = car_yaw_kd;
    s_car_yaw_pid.kff = 0.0f;
    PID_SetOutputLimits(&s_car_yaw_pid,
                        -car_yaw_rate_limit_dps,
                        car_yaw_rate_limit_dps);
    /* 以已归一化角度误差作为位置式PID输入，避免跨越正负180度。 */
    desired_rate_dps = PID_Update(&s_car_yaw_pid, yaw_error_deg, 0.0f);
    if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_BRAKE)
    {
        brake_rate_limit_abs = fabsf(car_large_turn_brake_rate_limit_dps);
        desired_rate_dps = car_speed_clampf(desired_rate_dps,
                                            -brake_rate_limit_abs,
                                             brake_rate_limit_abs);
    }
    g_car_yaw_p_term = s_car_yaw_pid.p_term;
    g_car_yaw_d_term = s_car_yaw_pid.d_term;
    g_car_gyroz_target_dps = -desired_rate_dps;
}

static void car_gyroz_debug_target_100HZ(void)
{
    int16 yaw_rate_command = s_car_control_input.channel[3];
    float selected_rate_dps;
    float desired_rate_dps = 0.0f;

    if (s_car_control_input.channel[5] == 0)
    {
        selected_rate_dps = CAR_GYROZ_DEBUG_TARGET_LOW_DPS;
    }
    else if (s_car_control_input.channel[5] == 1)
    {
        selected_rate_dps = CAR_GYROZ_DEBUG_TARGET_MID_DPS;
    }
    else
    {
        selected_rate_dps = CAR_GYROZ_DEBUG_TARGET_HIGH_DPS;
    }

    if (yaw_rate_command > CAR_GYROZ_DEBUG_INPUT_LOW_THRESHOLD)
    {
        desired_rate_dps = selected_rate_dps;
    }
    else if (yaw_rate_command < -CAR_GYROZ_DEBUG_INPUT_LOW_THRESHOLD)
    {
        desired_rate_dps = -selected_rate_dps;
    }

    g_car_gyroz_target_dps = -desired_rate_dps;
    g_car_yaw_target_deg = s_car_imu_snapshot.euler.yaw;
    g_car_yaw_p_term = 0.0f;
    g_car_yaw_d_term = 0.0f;
}

static void car_yaw_mode_prepare_100HZ(void)
{
    uint8 control_mode;

    if (car_yaw_control_mode < 0.5f)
    {
        control_mode = CAR_YAW_CONTROL_MODE_RATE_DEBUG;
    }
    else if (car_yaw_control_mode < 1.5f)
    {
        control_mode = CAR_YAW_CONTROL_MODE_WORLD;
    }
    else
    {
        control_mode = CAR_YAW_CONTROL_MODE_FIXED;
    }

    if ((s_car_yaw_mode_initialized == 0U) ||
        (control_mode != s_car_yaw_control_mode_active))
    {
        s_car_yaw_mode_initialized = 1U;
        s_car_yaw_control_mode_active = control_mode;
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
    if (s_car_yaw_control_mode_active != CAR_YAW_CONTROL_MODE_RATE_DEBUG)
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
    s_car_yaw_control_mode_active = CAR_YAW_CONTROL_MODE_WORLD;
    car_speed_plan_reset();
    car_negative_pressure_state_reset();
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
    float raw_x = (float)s_car_control_input.channel[1];
    float raw_y = (float)s_car_control_input.channel[0];
    float deadzone_sq = CAR_WORLD_INPUT_EXIT_DEADZONE *
                        CAR_WORLD_INPUT_EXIT_DEADZONE;

    return (((raw_x * raw_x + raw_y * raw_y) <= deadzone_sq) &&
            (s_car_control_input.channel[3] >=
             -(int16)CAR_WORLD_INPUT_EXIT_DEADZONE) &&
            (s_car_control_input.channel[3] <=
              (int16)CAR_WORLD_INPUT_EXIT_DEADZONE)) ? 1U : 0U;
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
    car_negative_pressure_state_reset();
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
    float large_turn_brake_ff;
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

    if (CAR_SPEED_LOOP_TUNE_ENABLE != 0U)
    {
        /* 单独调速度环：CH2触发正负阶跃，CH7选择200/400/700目标速度。 */
        car_large_turn_state_reset();
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
        if (s_car_yaw_control_mode_active == CAR_YAW_CONTROL_MODE_FIXED)
        {
            car_fixed_angle_command_update_100HZ();
        }
        else
        {
            car_world_command_update_100HZ();
        }
        car_large_turn_state_update_100HZ();
        car_speed_plan_update();
        Left_Target_Speed = g_car_base_speed_target;
        Right_Target_Speed = g_car_base_speed_target;
        car_yaw_target_update_100HZ();
    }

    /* CH5控制负压总使能；按加减速、行进转弯和原地转向选择压力档位。 */
    car_negative_pressure_control_100HZ();

    if (s_car_yaw_stopped != 0U)
    {
        if ((Left_Target_Speed == 0.0f) &&
            (Right_Target_Speed == 0.0f) &&
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
        (Right_Target_Speed == 0.0f) &&
        (car_yaw_command_is_within_stop_limit(
             CAR_YAW_STOP_ANGLE_DEG,
             CAR_GYROZ_STOP_TARGET_RATE_DPS) != 0U) &&
        (fabsf(g_car_gyroz_feedback_dps) < CAR_YAW_STOP_RATE_DPS) &&
        (fabsf(g_car_speed_left_filtered) < CAR_WHEEL_STOP_SPEED) &&
        (fabsf(g_car_speed_right_filtered) < CAR_WHEEL_STOP_SPEED))
    {
        PID_Reset(&Left_Speed_PID);
        PID_Reset(&Right_Speed_PID);
        car_yaw_outer_reset();
        car_gyroz_control_reset();
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
    else if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_BRAKE)
    {
        /* BRAKE禁用普通分轮停车前馈，后续仅叠加公共平移制动前馈。 */
        car_gyroz_control_100HZ();
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
    large_turn_brake_ff = car_large_turn_brake_feedforward();
    if (s_car_large_turn_state == CAR_LARGE_TURN_STATE_BRAKE)
    {
        left_brake_ff = large_turn_brake_ff;
        right_brake_ff = large_turn_brake_ff;
    }
    else
    {
        left_brake_ff = car_speed_brake_feedforward(
            g_car_speed_left_filtered, s_car_speed_left_brake_direction);
        right_brake_ff = car_speed_brake_feedforward(
            g_car_speed_right_filtered, s_car_speed_right_brake_direction);
    }
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
