#include "car_loop.h"
#include "../Protocols/crsf/crsf.h"
#include <math.h>

volatile uint8_t timer_100HZ_flag = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint32 tick_1000us_cnt = 0U;

static uint32 s_system_time_ms = 0U;
/* static uint16 s_air_comm_beep_tick = 200U; */
static uint8 s_air_menu_runtime_locked = 0U;
static uint8 s_menu_runtime_was_locked = 0U;
static float s_car_yaw_rate_lpf_dps = 0.0f;
static uint8 s_car_speed_filter_initialized = 0U;

#define CAR_SPEED_CONTROL_DT_S (0.01f)

#define CAR_GYROZ_DEG_TO_RAD (0.017453292519943295f)
#define CAR_GYROZ_EQUIVALENT_SCALE (28.1448005f)

float car_speed_left_kp = 18.0f;
float car_speed_left_ki = 0.0f;
float car_speed_left_kd = 0.027f;
float car_speed_right_kp = 18.0f;
float car_speed_right_ki = 0.0f;
float car_speed_right_kd = 0.027f;
float car_speed_filter_alpha = 0.557f;
float car_speed_ff_slope = 3.5f;
float car_speed_static_ff = 540.0f;
float car_gyroz_kp = 1.50f;
float car_gyroz_ki = 0.40f;
float car_gyroz_k_turn = 1.0f;

volatile float g_car_speed_left_filtered = 0.0f;
volatile float g_car_speed_right_filtered = 0.0f;
volatile float Left_Target_Speed = 0.0f;
volatile float Right_Target_Speed = 0.0f;
pid_t Left_Speed_PID;
pid_t Right_Speed_PID;

volatile float g_car_gyroz_target_dps = 0.0f;
volatile float g_car_gyroz_feedback_equivalent = 0.0f;
volatile float g_car_gyroz_error = 0.0f;
volatile float g_car_gyroz_p_term = 0.0f;
volatile float g_car_gyroz_i_term = 0.0f;
volatile float g_car_gyroz_output = 0.0f;
static float s_car_gyroz_last_error = 0.0f;

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

static int16 car_speed_pid_update(pid_t *pid, float target, float feedback,
                                  float kp, float ki, float kd)
{
    float static_feedforward = car_speed_ff_slope * target;
    float output;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    if (fabsf(target) < 50.0f)
    {
        PID_Reset(pid);
        pid->ff_term = 0.0f;
        pid->output = 0.0f;
        return 0;
    }

    if (target > 0.0f)
    {
        static_feedforward += car_speed_static_ff;
    }
    else if (target < 0.0f)
    {
        static_feedforward -= car_speed_static_ff;
    }

    /* The Loongson loop differentiates error, so feed feedback-target as measurement. */
    output = static_feedforward +
             PID_Update(pid, 0.0f, feedback - target, CAR_SPEED_CONTROL_DT_S);

    if (output > (float)MOTOR_PWM_MAX)
    {
        output = (float)MOTOR_PWM_MAX;
    }
    else if (output < -(float)MOTOR_PWM_MAX)
    {
        output = -(float)MOTOR_PWM_MAX;
    }

    pid->ff_term = static_feedforward;
    pid->output = output;
    return (int16)output;
}

void car_gyroz_control_100HZ(void)
{
    float base_speed = Left_Target_Speed;
    float target_equivalent;
    float delta_output;

    /* Keep the Loongson scale, with the sign adapted to this car's gyro polarity. */
    target_equivalent = -g_car_gyroz_target_dps *
                        CAR_GYROZ_DEG_TO_RAD *
                        CAR_GYROZ_EQUIVALENT_SCALE;
    g_car_gyroz_feedback_equivalent = s_car_yaw_rate_lpf_dps *
                                      CAR_GYROZ_DEG_TO_RAD *
                                      CAR_GYROZ_EQUIVALENT_SCALE;
    g_car_gyroz_error = target_equivalent - g_car_gyroz_feedback_equivalent;

    /* Loongson incremental PI, with Ki doubled for the 200 Hz to 100 Hz move. */
    g_car_gyroz_p_term = car_gyroz_kp *
                         (g_car_gyroz_error - s_car_gyroz_last_error);
    g_car_gyroz_i_term = car_gyroz_ki * g_car_gyroz_error;
    delta_output = g_car_gyroz_p_term + g_car_gyroz_i_term;
    g_car_gyroz_output += delta_output;
    s_car_gyroz_last_error = g_car_gyroz_error;

    Left_Target_Speed = base_speed + car_gyroz_k_turn * g_car_gyroz_output;
    Right_Target_Speed = base_speed - car_gyroz_k_turn * g_car_gyroz_output;
}

void car_loop_init(void)
{
    timer_100HZ_flag = 0U;
    g_tick_1000HZ = 0U;
    tick_1000us_cnt = 0U;
    s_system_time_ms = 0U;
    /* s_air_comm_beep_tick = 200U; */
    s_air_menu_runtime_locked = 0U;
    s_menu_runtime_was_locked = 0U;
    s_car_yaw_rate_lpf_dps = 0.0f;
    g_car_speed_left_filtered = 0.0f;
    g_car_speed_right_filtered = 0.0f;
    s_car_speed_filter_initialized = 0U;
    g_car_gyroz_target_dps = 0.0f;
    g_car_gyroz_feedback_equivalent = 0.0f;
    g_car_gyroz_error = 0.0f;
    g_car_gyroz_p_term = 0.0f;
    g_car_gyroz_i_term = 0.0f;
    g_car_gyroz_output = 0.0f;
    s_car_gyroz_last_error = 0.0f;

    PID_Init(&Left_Speed_PID, car_speed_left_kp, car_speed_left_ki,
             car_speed_left_kd, 0.0f,
             CAR_SPEED_CONTROL_DT_S, 0.0f, 0.0f);
    PID_Init(&Right_Speed_PID, car_speed_right_kp, car_speed_right_ki,
             car_speed_right_kd, 0.0f,
             CAR_SPEED_CONTROL_DT_S, 0.0f, 0.0f);

    menu_init();
    menu_config_init();
    motor_init();
    motor_stop();
    encoder_control_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    wifi_core_Init();
    air_comm_car_init();
    air_comm_set_run_data_callback(on_air_data);
    crsf_init();
    Beep_Init();
    Beep_Play(100U, 1.0f, 1U);
    pit_init(PIT_CH0, 1000U);
}

static void car_loop_1000HZ(void)
{
    IMU_Update_1000HZ();
    s_car_yaw_rate_lpf_dps += 0.06089863f *
                              (g_imufilter_1000hz.gyroz - s_car_yaw_rate_lpf_dps);
}

static void car_speed_control_100HZ(void)
{
    int16 left_raw;
    int16 right_raw;
    int16 speed_command;

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

    speed_command = CRSF_STD[1];
    if (speed_command <= -800)
    {
        Left_Target_Speed = -700.0f;
    }
    else if (speed_command <= -400)
    {
        Left_Target_Speed = -400.0f;
    }
    else if (speed_command <= -100)
    {
        Left_Target_Speed = -200.0f;
    }
    else if (speed_command < 100)
    {
        Left_Target_Speed = 0.0f;
    }
    else if (speed_command < 400)
    {
        Left_Target_Speed = 200.0f;
    }
    else if (speed_command < 800)
    {
        Left_Target_Speed = 400.0f;
    }
    else
    {
        Left_Target_Speed = 700.0f;
    }
    Right_Target_Speed = Left_Target_Speed;

    if (CRSF_STD[7] == 0)
    {
        motor_stop();
        return;
    }

    car_gyroz_control_100HZ();

    motor_left_set_speed(car_speed_pid_update(&Left_Speed_PID,
                                              Left_Target_Speed,
                                              g_car_speed_left_filtered,
                                              car_speed_left_kp,
                                              car_speed_left_ki,
                                              car_speed_left_kd));
    motor_right_set_speed(car_speed_pid_update(&Right_Speed_PID,
                                               Right_Target_Speed,
                                               g_car_speed_right_filtered,
                                               car_speed_right_kp,
                                               car_speed_right_ki,
                                               car_speed_right_kd));
}

static void car_loop_100HZ(void)
{
    float car_data[11];
    uint8 menu_runtime_locked;

    s_system_time_ms += 10U;
    CRSF_Update_100HZ();
    car_speed_control_100HZ();
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
    car_data[3] = g_euler.yaw;
    car_data[4] = s_car_yaw_rate_lpf_dps;
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
    uint16 imu_tick_guard = 0U;

    while ((g_tick_1000HZ > 0U) && (imu_tick_guard < 100U))
    {
        g_tick_1000HZ--;
        car_loop_1000HZ();
        imu_tick_guard++;
    }

    air_comm_car_poll();

    if (timer_100HZ_flag != 0U)
    {
        timer_100HZ_flag = 0U;
        car_loop_100HZ();
    }

    wifi_core_Poll();
}
