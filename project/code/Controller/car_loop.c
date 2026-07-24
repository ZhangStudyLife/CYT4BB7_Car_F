/*
 * Main loop scheduler.
 *
 * The control tick is split by frequency flags. Heavy work stays in the main
 * loop; PIT interrupt only updates flags.
 */
#include "car_loop.h"

volatile uint8_t timer_100HZ_flag = 0U;
volatile uint8_t timer_25HZ_flag = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint32 tick_1000us_cnt = 0U;

float car_forward_target = 0.0f;
float car_strafe_target = 0.0f;
uint8 car_control_enabled = 0U;
uint8 car_emergency_stop_active = 1U;
float g_car_plan_strafe_mps = 0.0f;
float g_car_plan_forward_mps = 0.0f;

static uint32 s_telemetry_timestamp_count = 0U;
static uint32 s_system_time_ms = 0U;
static uint16 s_air_comm_beep_tick = 200U;
static uint32 s_beacon_beep_enter_count = 0U;
static uint8 s_air_menu_runtime_locked = 0U;
static uint8 s_air_run_data_seen = 0U;
static uint8 s_air_takeoff_reset_pending = 0U;
static uint8 s_menu_runtime_was_locked = 0U;
/* Car yaw rate for Air upload, 10Hz low-pass output, unit: deg/s. */
static float s_car_yaw_rate_lpf_dps = 0.0f;

volatile float g_air_tof_fused_height_mm;
volatile float g_air_euler_roll;
volatile float g_air_euler_pitch;
volatile float g_air_euler_yaw;
volatile float g_air_pos_est_vel_x;
volatile float g_air_pos_est_vel_y;
volatile float g_air_state;
volatile air_diag_telemetry_t g_air_diag_telemetry;
volatile float g_air_reserved0;
volatile float g_air_reserved1;
volatile float g_air_reserved2;
volatile float g_air_crsf_std_ch0;
volatile float g_air_crsf_std_ch1;
volatile float g_air_crsf_std_ch2;
volatile float g_air_crsf_std_ch3;
volatile float g_air_crsf_std_ch4;
volatile float g_air_crsf_std_ch5;
volatile float g_air_crsf_std_ch6;
volatile float g_air_crsf_std_ch7;
volatile float g_air_crsf_std_ch8;
volatile float g_air_yaw_angle_target_deg = 0.0f;
volatile float g_air_sync_time_ms = 0.0f;
volatile float g_air_car_plan_valid = 0.0f;
volatile float g_air_car_plan_strafe_mps = 0.0f;
volatile float g_air_car_plan_forward_mps = 0.0f;
volatile float g_air_car_plan_camera = 0.0f;
volatile float g_air_car_plan_beacon_index = 0.0f;
volatile float g_air_car_plan_dist_px = 0.0f;
volatile float g_air_beacon_lost_flag = 0.0f;

#define AIR_RUN_DATA_CRITICAL_COUNT (15U) /* 飞行期间关键数据包的float数量 */
#define AIR_RUN_DATA_DIAGNOSTIC_LEGACY_COUNT (45U) /* 兼容旧版常态诊断包的float数量 */
#define AIR_RUN_DATA_DIAGNOSTIC_V1_COUNT (48U) /* 兼容首版SPI诊断包的float数量 */
#define AIR_RUN_DATA_DIAGNOSTIC_COUNT (52U) /* 常态完整诊断包的float数量 */
#define AIR_RUN_CRITICAL_STATE (0U) /* 飞机运行状态 */
#define AIR_RUN_CRITICAL_CRSF_CH0 (1U) /* CRSF标准化通道0 */
#define AIR_RUN_CRITICAL_CRSF_CH1 (2U) /* CRSF标准化通道1 */
#define AIR_RUN_CRITICAL_CRSF_CH2 (3U) /* CRSF标准化通道2 */
#define AIR_RUN_CRITICAL_CRSF_CH3 (4U) /* CRSF标准化通道3 */
#define AIR_RUN_CRITICAL_CRSF_CH4 (5U) /* CRSF标准化通道4 */
#define AIR_RUN_CRITICAL_CRSF_CH5 (6U) /* CRSF标准化通道5 */
#define AIR_RUN_CRITICAL_CRSF_CH6 (7U) /* CRSF标准化通道6 */
#define AIR_RUN_CRITICAL_CRSF_CH7 (8U) /* CRSF标准化通道7 */
#define AIR_RUN_CRITICAL_CRSF_CH8 (9U) /* CRSF标准化通道8 */
#define AIR_RUN_CRITICAL_YAW_TARGET (10U) /* 飞机yaw目标角，单位deg */
#define AIR_RUN_CRITICAL_PLAN_VALID (11U) /* 车模规划结果有效标志 */
#define AIR_RUN_CRITICAL_PLAN_STRAFE (12U) /* 车模规划横移速度，单位m/s */
#define AIR_RUN_CRITICAL_PLAN_FORWARD (13U) /* 车模规划前进速度，单位m/s */
#define AIR_RUN_CRITICAL_BEACON_LOST (14U) /* 信标丢失标志 */
#define AIR_RUN_DATA_TOF_FUSED_HEIGHT_MM (0U)
#define AIR_RUN_DATA_EULER_ROLL (1U)
#define AIR_RUN_DATA_EULER_PITCH (2U)
#define AIR_RUN_DATA_EULER_YAW (3U)
#define AIR_RUN_DATA_POS_EST_VEL_X (4U)
#define AIR_RUN_DATA_POS_EST_VEL_Y (5U)
#define AIR_RUN_DATA_STATE (6U)
#define AIR_RUN_DATA_CRSF_STD_CH0 (7U)
#define AIR_RUN_DATA_CRSF_STD_CH1 (8U)
#define AIR_RUN_DATA_CRSF_STD_CH2 (9U)
#define AIR_RUN_DATA_CRSF_STD_CH3 (10U)
#define AIR_RUN_DATA_CRSF_STD_CH4 (11U)
#define AIR_RUN_DATA_CRSF_STD_CH5 (12U)
#define AIR_RUN_DATA_CRSF_STD_CH6 (13U)
#define AIR_RUN_DATA_CRSF_STD_CH7 (14U)
#define AIR_RUN_DATA_YAW_ANGLE_TARGET_DEG (15U)
#define AIR_RUN_DATA_SYNC_TIME_MS (16U)
#define AIR_RUN_DATA_CAR_PLAN_VALID (17U)
#define AIR_RUN_DATA_CAR_PLAN_STRAFE_MPS (18U)
#define AIR_RUN_DATA_CAR_PLAN_FORWARD_MPS (19U)
#define AIR_RUN_DATA_CAR_PLAN_CAMERA (20U)
#define AIR_RUN_DATA_CAR_PLAN_BEACON_INDEX (21U)
#define AIR_RUN_DATA_CAR_PLAN_DIST_PX (22U)
#define AIR_RUN_DATA_BEACON_LOST_FLAG (23U)
#define AIR_RUN_DATA_CRSF_STD_CH8 (24U)

#define AIR_YAW_TARGET_DEG_TO_RAD (-0.017453292519943295f)
#define AIR_MENU_STATE_INIT (0.0f)
#define AIR_MENU_STATE_STANDBY (1.0f)
#define AIR_MENU_STATE_TAKEOFF (2.0f)
#define AIR_MENU_STATE_FLYING (3.0f)
#define AIR_MENU_STATE_RUNTIME_MIN (2.0f)

/**
 * @brief 更新飞机状态及由状态驱动的车端菜单运行锁。
 * @param air_state 飞机运行状态枚举对应的浮点值。
 * @return 无。
 */
static void car_loop_update_air_runtime_state(float air_state)
{
    /* 通信回调只挂起任务级复位，由100Hz主循环统一修改估计器状态。 */
    if(((air_state == AIR_MENU_STATE_TAKEOFF) ||
        (air_state == AIR_MENU_STATE_FLYING)) &&
       ((s_air_run_data_seen == 0U) ||
        ((g_air_state != AIR_MENU_STATE_TAKEOFF) &&
         (g_air_state != AIR_MENU_STATE_FLYING))))
    {
        s_air_takeoff_reset_pending = 1U;
    }

    g_air_state = air_state;
    s_air_run_data_seen = 1U;
    if(g_air_state >= AIR_MENU_STATE_RUNTIME_MIN)
    {
        s_air_menu_runtime_locked = 1U;
    }
    else if(g_air_state == AIR_MENU_STATE_STANDBY)
    {
        s_air_menu_runtime_locked = 0U;
    }
    else if(g_air_state != AIR_MENU_STATE_INIT)
    {
        s_air_menu_runtime_locked = 1U;
    }
}

/**
 * @brief 按数据数量解析飞机下发的关键运行包或完整诊断包。
 * @param data 飞机下发的float数据数组。
 * @param count 数组中的float数量，合法值为15、45、48或52。
 * @return 无。
 */
static void on_air_data(const float *data, uint8 count)
{
    if (count == AIR_RUN_DATA_CRITICAL_COUNT)
    {
        car_loop_update_air_runtime_state(data[AIR_RUN_CRITICAL_STATE]);
        g_air_crsf_std_ch0 = data[AIR_RUN_CRITICAL_CRSF_CH0];
        g_air_crsf_std_ch1 = data[AIR_RUN_CRITICAL_CRSF_CH1];
        g_air_crsf_std_ch2 = data[AIR_RUN_CRITICAL_CRSF_CH2];
        g_air_crsf_std_ch3 = data[AIR_RUN_CRITICAL_CRSF_CH3];
        g_air_crsf_std_ch4 = data[AIR_RUN_CRITICAL_CRSF_CH4];
        g_air_crsf_std_ch5 = data[AIR_RUN_CRITICAL_CRSF_CH5];
        g_air_crsf_std_ch6 = data[AIR_RUN_CRITICAL_CRSF_CH6];
        g_air_crsf_std_ch7 = data[AIR_RUN_CRITICAL_CRSF_CH7];
        g_air_crsf_std_ch8 = data[AIR_RUN_CRITICAL_CRSF_CH8];
        g_air_yaw_angle_target_deg = data[AIR_RUN_CRITICAL_YAW_TARGET];
        g_air_car_plan_valid = data[AIR_RUN_CRITICAL_PLAN_VALID];
        g_air_car_plan_strafe_mps = data[AIR_RUN_CRITICAL_PLAN_STRAFE];
        g_air_car_plan_forward_mps = data[AIR_RUN_CRITICAL_PLAN_FORWARD];
        g_air_beacon_lost_flag = data[AIR_RUN_CRITICAL_BEACON_LOST];
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
    car_loop_update_air_runtime_state(data[AIR_RUN_DATA_STATE]);
    g_air_crsf_std_ch0 = data[7];
    g_air_crsf_std_ch1 = data[8];
    g_air_crsf_std_ch2 = data[9];
    g_air_crsf_std_ch3 = data[10];
    g_air_crsf_std_ch4 = data[11];
    g_air_crsf_std_ch5 = data[12];
    g_air_crsf_std_ch6 = data[13];
    g_air_crsf_std_ch7 = data[14];
    g_air_yaw_angle_target_deg = data[AIR_RUN_DATA_YAW_ANGLE_TARGET_DEG];
    g_air_crsf_std_ch8 = data[AIR_RUN_DATA_CRSF_STD_CH8];
    g_air_sync_time_ms = data[AIR_RUN_DATA_SYNC_TIME_MS];
    g_air_car_plan_valid = data[AIR_RUN_DATA_CAR_PLAN_VALID];
    g_air_car_plan_strafe_mps = data[AIR_RUN_DATA_CAR_PLAN_STRAFE_MPS];
    g_air_car_plan_forward_mps = data[AIR_RUN_DATA_CAR_PLAN_FORWARD_MPS];
    g_air_car_plan_camera = data[AIR_RUN_DATA_CAR_PLAN_CAMERA];
    g_air_car_plan_beacon_index = data[AIR_RUN_DATA_CAR_PLAN_BEACON_INDEX];
    g_air_car_plan_dist_px = data[AIR_RUN_DATA_CAR_PLAN_DIST_PX];
    g_air_beacon_lost_flag = data[AIR_RUN_DATA_BEACON_LOST_FLAG];
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
    if(count >= AIR_RUN_DATA_DIAGNOSTIC_V1_COUNT)
    {
        uint8 camera_status0 = (uint8)data[45];
        uint8 camera_status1 = (uint8)data[46];

        g_air_diag_telemetry.camera_spi_online[0] = (float)(camera_status0 & 0x01U);
        g_air_diag_telemetry.camera_spi_online[1] = (float)(camera_status1 & 0x01U);
        g_air_diag_telemetry.camera_spi_ready[0] = (float)((camera_status0 >> 1) & 0x01U);
        g_air_diag_telemetry.camera_spi_ready[1] = (float)((camera_status1 >> 1) & 0x01U);
        g_air_diag_telemetry.camera_spi_error_code = data[47];
    }
    if(count >= AIR_RUN_DATA_DIAGNOSTIC_COUNT)
    {
        g_air_diag_telemetry.camera_spi_rx_head[0][0] = data[48];
        g_air_diag_telemetry.camera_spi_rx_head[0][1] = data[49];
        g_air_diag_telemetry.camera_spi_rx_head[1][0] = data[50];
        g_air_diag_telemetry.camera_spi_rx_head[1][1] = data[51];
    }
}

static void car_loop_runtime_reset(void)
{
    timer_100HZ_flag = 0U;
    timer_25HZ_flag = 0U;
    g_tick_1000HZ = 0U;
    tick_1000us_cnt = 0U;

    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
    car_control_enabled = 0U;
    car_emergency_stop_active = 1U;
    g_air_state = 0.0f;
    g_air_car_plan_valid = 0.0f;
    g_air_car_plan_strafe_mps = 0.0f;
    g_air_car_plan_forward_mps = 0.0f;
    g_car_plan_strafe_mps = 0.0f;
    g_car_plan_forward_mps = 0.0f;
    g_air_car_plan_camera = 0.0f;
    g_air_car_plan_beacon_index = 0.0f;
    g_air_car_plan_dist_px = 0.0f;
    g_air_beacon_lost_flag = 0.0f;
    g_air_diag_telemetry = (air_diag_telemetry_t){0};
    s_car_yaw_rate_lpf_dps = 0.0f;
    s_telemetry_timestamp_count = 0U;
    s_system_time_ms = 0U;
    s_beacon_beep_enter_count = 0U;
    s_air_menu_runtime_locked = 0U;
    s_air_run_data_seen = 0U;
    s_air_takeoff_reset_pending = 0U;
    s_menu_runtime_was_locked = 0U;
}

uint8 car_menu_is_runtime_locked(void)
{
    if(beacon_position_recorder_is_active() != 0U)
    {
        if((air_comm_car_is_run_data_fresh() == 0U) ||
           (s_air_menu_runtime_locked != 0U) ||
           (g_air_crsf_std_ch7 > 0.5f))
        {
            return 1U;
        }

        return 0U;
    }

    if((g_air_crsf_std_ch4 > 0.5f) ||
       (g_air_crsf_std_ch7 > 0.5f))
    {
        return 1U;
    }

    if((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
    {
        return 1U;
    }

    if(s_air_menu_runtime_locked != 0U)
    {
        return 1U;
    }

    if(((s_air_run_data_seen != 0U) &&
        (air_comm_car_is_run_data_fresh() == 0U)) ||
       ((s_air_run_data_seen == 0U) &&
        (air_comm_car_is_online() != 0U)))
    {
        return 1U;
    }

    return 0U;
}

void car_loop_init(void)
{
    car_loop_runtime_reset();

    menu_init();
    beacon_config_init();
    menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    beacon_position_recorder_init();
    odometer_init();
    beacon_detection_reset();
    fixator_init();
    LightSequence_Reset();
    carplanfix_reset();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    Control_Init();
    car_mode_init();
    wifi_core_Init();
    ALX_AOA_Init();
    air_comm_car_init();
    air_comm_set_run_data_callback(on_air_data);
    Beep_Init();
    Beep_Play(100, 1.0f, 1);
    pit_init(PIT_CH0, 1000);
}

static void car_loop_1000HZ(void)
{
    IMU_Update_1000HZ();
    s_car_yaw_rate_lpf_dps += 0.06089863f *
                              (g_imufilter_1000hz.gyroz - s_car_yaw_rate_lpf_dps);
    odometer_update_1000HZ();
    beacon_detection_update_1000HZ();
}

static void car_loop_100HZ(void)
{
    float car_data[11];
    float odometer_raw_position[2];
    float odometer_fixed_position[2];
    float beacon_detected_flag;
    light_sequence_result_t light_sequence_result;
    uint8 menu_runtime_locked;

    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count * 10U;

    encoder_update_100HZ();
    odometer_update_100HZ();
    /* 新一轮起飞统一清除上一轮定位、检测和灯序识别状态。 */
    if(s_air_takeoff_reset_pending != 0U)
    {
        odometer_reset();
        beacon_detection_reset();
        fixator_reset();
        LightSequence_Reset();
        carplanfix_reset();
        s_air_takeoff_reset_pending = 0U;
    }
    beacon_position_recorder_update_100HZ();
    beacon_detection_update_100HZ();
    fixator_update_100HZ();

    odometer_raw_position[x] = g_odometer.position[x];
    odometer_raw_position[y] = g_odometer.position[y];
    odometer_fixed_position[x] = g_odometer.position[x];
    odometer_fixed_position[y] = g_odometer.position[y];
    beacon_detected_flag = (g_beacon_detection.enter_event != 0U) ? 1.0f : 0.0f;

    if((g_fixator.pending_fix != 0U) && (g_fixator.last_match_valid != 0U))
    {
        odometer_raw_position[x] = g_fixator.before_position[x];
        odometer_raw_position[y] = g_fixator.before_position[y];
        odometer_fixed_position[x] = g_fixator.fixed_position[x];
        odometer_fixed_position[y] = g_fixator.fixed_position[y];
    }

    /* 同一Air灭灯事件先应用fixator修正，再用修正后的里程计坐标分析灯序。 */
    odometer_apply_pending_fix();
    LightSequence_Update((g_air_beacon_lost_flag > 0.5f) ? 1U : 0U,
                         g_odometer.position[x],
                         g_odometer.position[y],
                         tick_1000us_cnt);
    LightSequence_GetResult(&light_sequence_result);

    if ((g_beacon_detection.enter_event != 0U) &&
        (g_beacon_detection.enter_count != s_beacon_beep_enter_count))
    {
        s_beacon_beep_enter_count = g_beacon_detection.enter_count;
        Beep_Play(100U, 0.10f, 1U);
    }

    menu_runtime_locked = car_menu_is_runtime_locked();

    if(menu_runtime_locked != 0U)
    {
        if(s_menu_runtime_was_locked == 0U)
        {
            menu_air_abort_param_sync_runtime();
            menu_air_command_abort_runtime();
            menu_runtime_suspend();
        }
        else if(menu_external_view_runtime_active() == 0U)
        {
            menu_discard_key_events();
        }
    }

    air_comm_car_update_100HZ();

    if(menu_runtime_locked == 0U)
    {
        if(s_menu_runtime_was_locked != 0U)
        {
            menu_runtime_resume();
        }
        menu_air_command_update_100HZ();
        menu_air_update_100HZ();
        menu_update_100HZ();
    }
    else if(menu_external_view_runtime_active() != 0U)
    {
        /* 运行锁定期间只处理明确允许的外部页面输入。 */
        menu_update_100HZ();
    }
    s_menu_runtime_was_locked = menu_runtime_locked;

    (void)carplanfix_resolve(&light_sequence_result,
                             s_system_time_ms,
                             (car_mode_get() == CAR_MODE_3) ? 1U : 0U,
                             (carplanfix_mode3_beacon1_enable > 0.5f) ? 1U : 0U,
                             ((car_mode_get() == CAR_MODE_3) &&
                              (carplanfix_enable > 0.5f)) ? 1U : 0U,
                             (g_air_car_plan_valid > 0.5f) ? 1U : 0U,
                             g_air_car_plan_forward_mps,
                             g_air_car_plan_strafe_mps,
                             &g_car_plan_forward_mps,
                             &g_car_plan_strafe_mps);
    car_mode_update_100HZ(s_system_time_ms);


    // 如果车机串口通信离线,车端的蜂鸣器报警,为1s的鸣叫,1s的停止
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

    if ((car_control_enabled != 0U) &&
        (car_emergency_stop_active == 0U) &&
        (air_comm_car_is_run_data_fresh() != 0U))
    {
        float yaw_target_rad = 0.0f;

        if((CAR_MODE_2 == car_mode_get()) ||
           (CAR_MODE_3 == car_mode_get()) ||
           (CAR_MODE_4 == car_mode_get()) ||
           (CAR_MODE_5 == car_mode_get()) ||
           (CAR_MODE_8 == car_mode_get()))
        {
            yaw_target_rad = g_air_yaw_angle_target_deg * AIR_YAW_TARGET_DEG_TO_RAD;
        }

        Control_100Hz(car_forward_target, car_strafe_target, yaw_target_rad);
    }
    else
    {
        Control_Stop();
    }

    car_data[0] = g_odometer.body_vel[x]; /* 实际横向速度，右正，m/s */
    car_data[1] = g_odometer.body_vel[y]; /* 实际前向速度，前正，m/s */
    car_data[2] = (g_beacon_detection.on_beacon != 0U) ? 1.0f : 0.0f;
    car_data[3] = g_euler.yaw;
    car_data[4] = s_car_yaw_rate_lpf_dps;
    car_data[5] = 0.0f;
    car_data[6] = 0.0f;
    car_data[7] = 0.0f;
    car_data[8] = 0.0f;
    car_data[9] = 0.0f;
    car_data[10] = (float)s_system_time_ms;
    air_comm_send_run_data(car_data, 11);

    // wifi_justfloat((float)car_mode_get(),
    //                g_air_euler_yaw,
    //                g_air_yaw_angle_target_deg,
    //                control_yaw_angle_current,
    //                g_car_mode8_state.velocity_forward_target_mps,
    //                g_car_mode8_state.velocity_strafe_target_mps,
    //                g_car_mode8_state.velocity_forward_feedback_mps,
    //                g_car_mode8_state.velocity_strafe_feedback_mps,
    //                g_car_mode8_state.forward_pid_output,
    //                g_car_mode8_state.strafe_pid_output,
    //                g_car_mode8_state.forward_target,
    //                g_car_mode8_state.strafe_target,
    //                (float)g_car_mode8_state.output_valid);

/*     wifi_justfloat(g_air_tof_fused_height_mm,
                 g_air_euler_roll,
                 g_air_euler_pitch,
                 g_air_euler_yaw,
                 g_air_pos_est_vel_x,
                 g_air_pos_est_vel_y,
                 g_air_state,
                 g_air_crsf_std_ch0,
                 g_air_crsf_std_ch1,
                 g_air_crsf_std_ch2,
                 g_air_crsf_std_ch3,
                 g_air_crsf_std_ch4,
                 g_air_crsf_std_ch5,
                 g_air_crsf_std_ch6,
                 g_air_crsf_std_ch7,
                 g_air_crsf_std_ch8); */
    // wifi_justfloat(g_imufilter_1000hz.gyrox,                       /* I1 */
    //                    g_imufilter_1000hz.gyroy,                       /* I2 */
    //                    g_imufilter_1000hz.gyroz,                       /* I3 */
    //                    g_imufilter_1000hz.accx,                        /* I4 */
    //                    g_imufilter_1000hz.accy,                        /* I5 */
    //                    g_imufilter_1000hz.accz,                        /* I6 */
    //                    g_euler.pitch,                                  /* I7 */
    //                    g_euler.roll,                                   /* I8 */
    //                    g_euler.yaw,                                    /* I9 */
    //                    encoder_get_left_front_filtered_count(),         /* I10 */
    //                    encoder_get_right_front_filtered_count(),        /* I11 */
    //                    encoder_get_left_rear_filtered_count(),          /* I12 */
    //                    encoder_get_right_rear_filtered_count(),         /* I13 */
    //                    g_odometer.body_vel[x],                         /* I14 */
    //                    g_odometer.body_vel[y],                         /* I15 */
    //                    g_odometer.vel[x],                              /* I16 */
    //                    g_odometer.vel[y],                              /* I17 */
    //                    odometer_raw_position[x],                       /* I18 */
    //                    odometer_raw_position[y],                       /* I19 */                           /* I20 */
    //                    odometer_fixed_position[x],                     /* I21 */
    //                    odometer_fixed_position[y],                     /* I22 */
    //                    g_air_car_plan_strafe_mps,                      /* I23: Air target velocity X */
    //                    g_air_car_plan_forward_mps,                     /* I24: Air target velocity Y */
    //                    g_air_beacon_lost_flag,
    //                    beacon_detected_flag);
    wifi_justfloat(g_air_beacon_lost_flag,                         /* I1: Air熄灯标志 */
                   g_odometer.position[x],                         /* I2: 修正后X坐标，m */
                   g_odometer.position[y],                         /* I3: 修正后Y坐标，m */
                   (float)light_sequence_result.status,            /* I4: 识别状态 */
                   (float)light_sequence_result.last_beacon_id,    /* I5: 最近匹配灯号 */
                   (float)light_sequence_result.candidate_count,   /* I6: 剩余候选数量 */
                   (float)light_sequence_result.candidate_mask,    /* I7: 候选位掩码 */
                   (light_sequence_result.candidate_mask & 0x01U) != 0U ? 1.0f : 0.0f, /* I8: 序列1候选 */
                   (light_sequence_result.candidate_mask & 0x02U) != 0U ? 1.0f : 0.0f, /* I9: 序列2候选 */
                   (light_sequence_result.candidate_mask & 0x04U) != 0U ? 1.0f : 0.0f, /* I10: 序列3候选 */
                   (light_sequence_result.candidate_mask & 0x08U) != 0U ? 1.0f : 0.0f, /* I11: 序列4候选 */
                   (float)light_sequence_result.sequence_id,       /* I12: 最终灯序，0为未确定 */
                   (float)light_sequence_result.accepted_event_count, /* I13: 正式灭灯事件数 */
                   (float)g_carplanfix_state.status,               /* I14: 路径状态 */
                   (float)g_carplanfix_state.disable_reason,       /* I15: 失效原因 */
                   (g_carplanfix_state.mode3_beacon1_pending != 0U)
                       ? 1.0f
                       : (float)g_carplanfix_state.target_beacon_id, /* I16: 当前实际目标灯 */
                   (float)g_carplanfix_state.route_index,          /* I17: 当前路径下标 */
                   (float)g_carplanfix_state.near_beacon,          /* I18: 位于目标灯0.5m内 */
                   (float)g_carplanfix_state.correction_valid,     /* I19: 本周期修正有效 */
                   g_car_plan_forward_mps,                         /* I20: 车端前向规划速度 */
                   g_car_plan_strafe_mps,                          /* I21: 车端横向规划速度 */
                   (float)g_carplanfix_state.target_zone_entered,  /* I22: 已进入当前目标区域 */
                   (g_carplanfix_state.status == CARPLANFIX_STATUS_DISABLED)
                       ? 1.0f : 0.0f);                             /* I23: carplanfix已永久禁用 */

}

static void car_loop_25HZ(void)
{
    ALX_AOA_Update_25HZ(s_system_time_ms);
    car_mode_update_25HZ(s_system_time_ms);
}

void car_loop_poll(void)
{
    uint16 imu_tick_guard = 0U;

    while ((g_tick_1000HZ > 0U) && (imu_tick_guard < 100U))
    {
        g_tick_1000HZ--;
        car_loop_1000HZ();
        imu_tick_guard++;
    }

    air_comm_car_poll();

    if (timer_25HZ_flag)
    {
        timer_25HZ_flag = 0U;
        car_loop_25HZ();
    }

    if (timer_100HZ_flag)
    {
        timer_100HZ_flag = 0U;
        car_loop_100HZ();
    }

    wifi_core_Poll();
}
