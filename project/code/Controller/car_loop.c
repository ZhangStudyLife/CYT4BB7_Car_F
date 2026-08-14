#include "car_loop.h"
#include "car_mode.h"
#include "../car_safety.h"
#include "../Common/car_math.h"
#include "../negative_pressure_motor.h"

#define CAR_IMU_STALE_CONTROL_LIMIT (3U) /* IMU连续未更新的控制周期上限 */
#define CAR_PIT_TICKS_PER_US        (8U) /* PIT每微秒计数值 */
#define AIR_RUN_DATA_CRITICAL_COUNT          (15U) /* 飞行关键数据数量 */
#define AIR_RUN_DATA_DIAGNOSTIC_LEGACY_COUNT (45U) /* 旧诊断数据数量 */
#define AIR_RUN_DATA_DIAGNOSTIC_V1_COUNT     (48U) /* V1诊断数据数量 */
#define AIR_RUN_DATA_DIAGNOSTIC_COUNT        (52U) /* 当前诊断数据数量 */
#define AIR_MENU_STATE_INIT                  (0.0f) /* 飞机初始化状态 */
#define AIR_MENU_STATE_STANDBY               (1.0f) /* 飞机待机状态 */

volatile uint32 tick_1000us_cnt = 0U;
volatile uint32 g_car_background_100hz_generation = 0U;
volatile car_realtime_diag_t g_car_realtime_diag = {0};
volatile float g_car_speed_left_filtered = 0.0f;
volatile float g_car_speed_right_filtered = 0.0f;
volatile float g_car_yaw_feedback_deg = 0.0f;
volatile float g_car_gyroz_feedback_dps = 0.0f;
volatile float g_car_base_speed_command = 0.0f;
volatile float g_car_base_speed_target = 0.0f;
volatile float g_car_speed_left_motor_output = 0.0f;
volatile float g_car_speed_right_motor_output = 0.0f;
volatile float Left_Target_Speed = 0.0f;
volatile float Right_Target_Speed = 0.0f;

volatile float g_air_tof_fused_height_mm = 0.0f;
volatile float g_air_euler_roll = 0.0f;
volatile float g_air_euler_pitch = 0.0f;
volatile float g_air_euler_yaw = 0.0f;
volatile float g_air_pos_est_vel_x = 0.0f;
volatile float g_air_pos_est_vel_y = 0.0f;
volatile float g_air_state = 0.0f;
volatile air_diag_telemetry_t g_air_diag_telemetry;
volatile float g_air_std_ch0 = 0.0f;
volatile float g_air_std_ch1 = 0.0f;
volatile float g_air_std_ch2 = 0.0f;
volatile float g_air_std_ch3 = 0.0f;
volatile float g_air_std_ch4 = 0.0f;
volatile float g_air_std_ch5 = 0.0f;
volatile float g_air_std_ch6 = 0.0f;
volatile float g_air_std_ch7 = 0.0f;
volatile float g_air_std_ch8 = 0.0f;
volatile float g_air_yaw_angle_target_deg = 0.0f;
volatile float g_air_sync_time_ms = 0.0f;
volatile float g_air_car_plan_valid = 0.0f;
volatile float g_air_car_plan_strafe_mps = 0.0f;
volatile float g_air_car_plan_forward_mps = 0.0f;
volatile float g_air_car_plan_camera = 0.0f;
volatile float g_air_car_plan_beacon_index = 0.0f;
volatile float g_air_car_plan_dist_px = 0.0f;
volatile float g_air_beacon_lost_flag = 0.0f;

static volatile uint32 s_system_time_ms = 0U;
static uint32 s_background_100hz_processed_generation = 0U;
static uint32 s_control_last_imu_tick = 0U;
static uint32 s_control_last_imu_sample_count = 0U;
static uint8 s_control_imu_stale_cycles = 0U;
static uint8 s_car_speed_filter_initialized = 0U;
static imu_realtime_snapshot_t s_car_imu_snapshot = {0};
static uint16 s_air_comm_beep_tick = 200U;
static uint8 s_air_menu_runtime_locked = 0U;
static uint8 s_menu_runtime_was_locked = 0U;

static void car_loop_update_air_runtime_state(float air_state)
{
    g_air_state = air_state;
    s_air_menu_runtime_locked = ((air_state == AIR_MENU_STATE_INIT) ||
                                 (air_state == AIR_MENU_STATE_STANDBY))
                                    ? 0U : 1U;
}

static void on_air_data(const float *data, uint8 count)
{
    if (count == AIR_RUN_DATA_CRITICAL_COUNT)
    {
        car_loop_update_air_runtime_state(data[0]);
        g_air_std_ch0 = data[1];
        g_air_std_ch1 = data[2];
        g_air_std_ch2 = data[3];
        g_air_std_ch3 = data[4];
        g_air_std_ch4 = data[5];
        g_air_std_ch5 = data[6];
        g_air_std_ch6 = data[7];
        g_air_std_ch7 = data[8];
        g_air_std_ch8 = data[9];
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
    g_air_std_ch0 = data[7];
    g_air_std_ch1 = data[8];
    g_air_std_ch2 = data[9];
    g_air_std_ch3 = data[10];
    g_air_std_ch4 = data[11];
    g_air_std_ch5 = data[12];
    g_air_std_ch6 = data[13];
    g_air_std_ch7 = data[14];
    g_air_yaw_angle_target_deg = data[15];
    g_air_sync_time_ms = data[16];
    g_air_car_plan_valid = data[17];
    g_air_car_plan_strafe_mps = data[18];
    g_air_car_plan_forward_mps = data[19];
    g_air_car_plan_camera = data[20];
    g_air_car_plan_beacon_index = data[21];
    g_air_car_plan_dist_px = data[22];
    g_air_beacon_lost_flag = data[23];
    g_air_std_ch8 = data[24];
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

        g_air_diag_telemetry.camera_spi_online[0] =
            (float)(camera_status0 & 0x01U);
        g_air_diag_telemetry.camera_spi_online[1] =
            (float)(camera_status1 & 0x01U);
        g_air_diag_telemetry.camera_spi_ready[0] =
            (float)((camera_status0 >> 1) & 0x01U);
        g_air_diag_telemetry.camera_spi_ready[1] =
            (float)((camera_status1 >> 1) & 0x01U);
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

static void car_pit_init_exact(pit_index_enum pit_index, uint32 period_us)
{
    volatile stc_TCPWM_GRP_CNT_t *counter =
        (volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[2].CNT[pit_index];

    pit_init(pit_index, period_us);
    pit_disable(pit_index);
    Cy_Tcpwm_Counter_SetPeriod(
        counter, period_us * CAR_PIT_TICKS_PER_US - 1U);
    pit_enable(pit_index);
}

static void car_loop_update_encoder_feedback_100HZ(void)
{
    int16 left_raw;
    int16 right_raw;
    float filter_alpha;

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
    switch (car_mode_get())
    {
        case CAR_MODE_1:
            filter_alpha = mode1_speed_filter_alpha;
            break;
        case CAR_MODE_2:
            filter_alpha = mode2_speed_filter_alpha;
            break;
        case CAR_MODE_4:
            filter_alpha = mode4_speed_filter_alpha;
            break;
        case CAR_MODE_5:
            filter_alpha = mode5_speed_filter_alpha;
            break;
        case CAR_MODE_8:
            filter_alpha = mode8_speed_filter_alpha;
            break;
        default:
            filter_alpha = 0.557f;
            break;
    }
    filter_alpha = car_math_clampf(filter_alpha, 0.0f, 1.0f);
    g_car_speed_left_filtered += filter_alpha *
        ((float)left_raw - g_car_speed_left_filtered);
    g_car_speed_right_filtered += filter_alpha *
        ((float)right_raw - g_car_speed_right_filtered);
}

static void car_total_emergency_stop(void)
{
    car_mode_reset_control();
    motor_stop();
    negative_pressure_disable();
}

static uint8 car_loop_safety_allows_output_100HZ(void)
{
    car_safety_input_t input;

    input.link_up = air_comm_car_is_run_data_fresh();
    input.imu_healthy = s_car_imu_snapshot.healthy;
    input.maintenance_active =
        ((IMUCalib_IsBusy() != 0U) ||
         (car_safety_is_maintenance_requested() != 0U)) ? 1U : 0U;
    input.run_switch_on =
        ((input.link_up != 0U) && (g_air_std_ch4 >= 0.5f)) ? 1U : 0U;
    input.left_target_speed = Left_Target_Speed;
    input.right_target_speed = Right_Target_Speed;
    input.left_feedback_speed = g_car_speed_left_filtered;
    input.right_feedback_speed = g_car_speed_right_filtered;
    input.left_motor_output = g_car_speed_left_motor_output;
    input.right_motor_output = g_car_speed_right_motor_output;
    input.gyroz_dps = g_car_gyroz_feedback_dps;
    car_safety_update_100HZ(&input);
    return car_safety_is_output_allowed();
}

void car_loop_init(void)
{
    tick_1000us_cnt = 0U;
    g_car_background_100hz_generation = 0U;
    memset((void *)&g_car_realtime_diag, 0, sizeof(g_car_realtime_diag));
    memset(&s_car_imu_snapshot, 0, sizeof(s_car_imu_snapshot));
    s_system_time_ms = 0U;
    s_background_100hz_processed_generation = 0U;
    s_control_last_imu_tick = 0U;
    s_control_last_imu_sample_count = 0U;
    s_control_imu_stale_cycles = 0U;
    s_car_speed_filter_initialized = 0U;
    s_air_comm_beep_tick = 200U;
    s_air_menu_runtime_locked = 0U;
    s_menu_runtime_was_locked = 0U;
    g_car_speed_left_filtered = 0.0f;
    g_car_speed_right_filtered = 0.0f;
    g_car_yaw_feedback_deg = 0.0f;
    g_car_gyroz_feedback_dps = 0.0f;

    menu_init();
    menu_config_init();
    motor_init();
    motor_stop();
    negative_pressure_init();
    negative_pressure_disable();
    car_safety_init();
    encoder_control_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    IMU_ResetYaw();
    (void)IMU_GetRealtimeSnapshot(&s_car_imu_snapshot);
    g_car_yaw_feedback_deg = s_car_imu_snapshot.euler.yaw;
    s_control_last_imu_sample_count = s_car_imu_snapshot.sample_count;
    car_mode_init();
    wifi_core_Init();
    air_comm_car_init();
    air_comm_set_run_data_callback(on_air_data);
    Beep_Init();
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

    if (air_comm_car_is_run_data_fresh() == 0U)
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
        g_car_yaw_feedback_deg = s_car_imu_snapshot.euler.yaw;
        if (s_car_imu_snapshot.sample_count == s_control_last_imu_sample_count)
        {
            if (s_control_imu_stale_cycles < CAR_IMU_STALE_CONTROL_LIMIT)
            {
                s_control_imu_stale_cycles++;
            }
        }
        else
        {
            s_control_last_imu_sample_count = s_car_imu_snapshot.sample_count;
            s_control_imu_stale_cycles = 0U;
        }
        if (s_control_imu_stale_cycles >= CAR_IMU_STALE_CONTROL_LIMIT)
        {
            s_car_imu_snapshot.healthy = 0U;
            g_car_realtime_diag.imu_stale_fault_count++;
        }
    }

    car_loop_update_encoder_feedback_100HZ();
    negative_pressure_disable();
    if (car_loop_safety_allows_output_100HZ() == 0U)
    {
        car_total_emergency_stop();
        return;
    }
    if (car_mode_update_100HZ(s_system_time_ms) == 0U)
    {
        car_total_emergency_stop();
    }
}

void car_loop_release_background_100HZ_isr(void)
{
    g_car_background_100hz_generation++;
}

static void car_loop_background_100HZ(void)
{
    car_drive_diag_t car_diag;
    float car_data[11];
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

    if (air_comm_car_is_online() == 0U)
    {
        if (s_air_comm_beep_tick >= 200U)
        {
            s_air_comm_beep_tick = 0U;
            Beep_Enable();
        }
        else if (s_air_comm_beep_tick == 100U)
        {
            Beep_Disable();
        }
        s_air_comm_beep_tick++;
    }
    else if (s_air_comm_beep_tick != 200U)
    {
        s_air_comm_beep_tick = 200U;
        Beep_Disable();
    }
    Beep_Update_100HZ();

    car_mode_get_diag(&car_diag);
    car_data[0] = 0.0f;
    car_data[1] = car_speed_encoder_cnt_to_mps(
        0.5f * (g_car_speed_left_filtered + g_car_speed_right_filtered));
    car_data[2] = car_diag.yaw_target_deg; /* 当前模式yaw控制目标 */
    car_data[3] = g_car_yaw_feedback_deg;
    car_data[4] = g_car_gyroz_feedback_dps;
    car_data[5] = 0.0f;
    car_data[6] = car_speed_encoder_cnt_to_mps(g_car_base_speed_target);
    car_data[7] = (float)car_diag.large_turn_state; /* 0正常 1刹车 2原地转 3退出 */
    car_data[8] = 0.0f;
    car_data[9] = 0.0f;
    car_data[10] = (float)s_system_time_ms;
    air_comm_send_run_data(car_data, 11U);
}

void car_loop_poll(void)
{
    uint32 background_generation;
    uint32 generation_delta;

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
