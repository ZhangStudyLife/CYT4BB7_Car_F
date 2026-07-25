/*
 * 模块职责：主循环调度器。
 * PIT 中断只置标志位，car_loop_poll() 在主循环中清标志并执行周期任务。
 *
 * 调用节奏：
 *   car_loop_init() 在系统启动时调用一次。
 *   car_loop_poll() 在 while(1) 主循环中轮询，内部按 1000/100/25 Hz 分频执行。
 *
 * 数据来源：
 *   空地串口接收无人机运行数据和遥控通道。
 *   遥控通道来自无人机 CRSF 转发的 ch0~ch8。
 */

#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"

/*
 * 控制输入源编译开关。
 *
 * REMOTE：使用Air转发的CH4/遥控与任务目标。
 * DEBUG_MENU：忽略遥控发车门控，使用Car -> C_Debug菜单生成同一种控制命令。
 *
 * 调试完成后只需把CAR_CONTROL_SOURCE改回REMOTE；PID、底盘参数和后续控制链路
 * 完全共用，不需要复制调试参数。
 */
#define CAR_CONTROL_SOURCE_REMOTE      (0U)
#define CAR_CONTROL_SOURCE_DEBUG_MENU  (1U)
#ifndef CAR_CONTROL_SOURCE
#define CAR_CONTROL_SOURCE             CAR_CONTROL_SOURCE_DEBUG_MENU
#endif

#if ((CAR_CONTROL_SOURCE != CAR_CONTROL_SOURCE_REMOTE) && \
     (CAR_CONTROL_SOURCE != CAR_CONTROL_SOURCE_DEBUG_MENU))
#error "Invalid CAR_CONTROL_SOURCE"
#endif

/* 100Hz 周期任务标志，PIT 中断置 1，poll 中清 0 */
extern volatile uint8_t timer_100HZ_flag;
/* 25Hz 周期任务标志 */
extern volatile uint8_t timer_25HZ_flag;
/* 1000Hz 节拍计数，用于超时判断和在线状态老化 */
extern volatile uint16 g_tick_1000HZ;
/* 1ms tick 累计计数 */
extern volatile uint32 tick_1000us_cnt;

/* 前进速度目标，单位 m/s，正方向为车头朝向 */
extern float car_forward_target;
/* 两轮目标角速度，单位 rad/s，俯视逆时针为正 */
extern float car_yaw_rate_target;
/* 控制使能标志：1=允许电机输出，0=停止输出 */
extern uint8 car_control_enabled;
/* 急停激活标志：1=急停生效，电机强制停止 */
extern uint8 car_emergency_stop_active;

/* 遥控通道标准化值（来自无人机CRSF转发，摇杆通道通常为-1000~+1000）。 */
/* 两轮手动模式：ch0=Roll转向，ch1=Pitch前后，ch4=发车开关，ch5/ch6=模式。 */
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
extern float g_car_plan_strafe_mps;
extern float g_car_plan_forward_mps;
extern volatile float g_air_car_plan_camera;
extern volatile float g_air_car_plan_beacon_index;
extern volatile float g_air_car_plan_dist_px;
extern volatile float g_air_beacon_lost_flag;

/* Car实际输出或Air处于起飞、飞行、降落时返回1。 */
uint8 car_menu_is_runtime_locked(void);
/* 初始化：清零所有状态和标志位 */
void car_loop_init(void);
/* 主循环轮询：内部按 1000/100/25Hz 分频执行各周期任务 */
void car_loop_poll(void);

#endif
