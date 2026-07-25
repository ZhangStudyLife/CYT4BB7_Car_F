/*********************************************************************************************************************
* 菜单用户配置头文件 - 参数集中管理
*
* 功能：集中管理两轮差速底盘控制参数，参数支持菜单调节和Flash存档
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef _MENU_CONFIG_H_
#define _MENU_CONFIG_H_

/* 1=Car上电时从指定存档加载本车参数，0=使用Car代码参数。 */
#define MENU_CAR_BOOT_FLASH_LOAD_ENABLE  (0U)
#define MENU_CAR_BOOT_FLASH_LOAD_SLOT    (0U)

/* 1=Car首次连接Air时覆盖Air参数，0=保留Air代码参数并从Air读取。 */
#define MENU_AIR_BOOT_OVERRIDE_ENABLE        (0U)
/* 覆盖开启时：1=使用Car Flash中的Air参数，0=使用Car代码中的Air参数。 */
#define MENU_AIR_BOOT_FLASH_LOAD_ENABLE      (0U)
/* 覆盖关闭时，该槽仍单独作为c1_screen_mode的启动恢复来源。 */
#define MENU_AIR_BOOT_FLASH_LOAD_SLOT        (0U)



//====================================================速度环参数声明====================================================
extern float wheel_kp;                 // 轮速环比例系数
extern float wheel_ki;                 // 轮速环积分系数
extern float wheel_kd;                 // 轮速环微分系数
extern float wheel_output_limit;       // 轮速环输出限幅 (PWM)
extern float wheel_i_limit;            // 轮速环积分限幅

//====================================================用户配置接口====================================================
void menu_config_init(void);           // 初始化菜单配置（注册参数和设置根菜单）

extern float yaw_angle_kp;
extern float yaw_angle_ki;
extern float yaw_angle_kd;
extern float yaw_angle_i_limit;
extern float yaw_angle_output_limit;

extern float yaw_rate_kp;
extern float yaw_rate_ki;
extern float yaw_rate_kd;
extern float yaw_rate_i_limit;
extern float yaw_rate_output_limit;

extern float mode7_velocity_smooth_tau_s;
extern float mode7_velocity_output_limit;
extern float mode7_velocity_pid_output_limit;
extern float mode7_velocity_i_limit;
extern float mode7_velocity_strafe_kp;
extern float mode7_velocity_strafe_ki;
extern float mode7_velocity_strafe_kd;
extern float mode7_velocity_forward_kp;
extern float mode7_velocity_forward_ki;
extern float mode7_velocity_forward_kd;

extern float mode5_velocity_smooth_tau_s;
extern float mode5_velocity_output_limit;
extern float mode5_velocity_pid_output_limit;
extern float mode5_velocity_i_limit;
extern float mode5_velocity_strafe_kp;
extern float mode5_velocity_strafe_ki;
extern float mode5_velocity_strafe_kd;
extern float mode5_velocity_forward_kp;
extern float mode5_velocity_forward_ki;
extern float mode5_velocity_forward_kd;

extern float mode2_velocity_smooth_tau_s;
extern float mode2_velocity_output_limit;
extern float mode2_velocity_pid_output_limit;
extern float mode2_velocity_i_limit;
extern float mode2_velocity_strafe_kp;
extern float mode2_velocity_strafe_ki;
extern float mode2_velocity_strafe_kd;
extern float mode2_velocity_forward_kp;
extern float mode2_velocity_forward_ki;
extern float mode2_velocity_forward_kd;

extern float mode8_velocity_smooth_tau_s;
extern float mode8_velocity_output_limit;
extern float mode8_velocity_pid_output_limit;
extern float mode8_velocity_i_limit;
extern float mode8_velocity_strafe_kp;
extern float mode8_velocity_strafe_ki;
extern float mode8_velocity_strafe_kd;
extern float mode8_velocity_forward_kp;
extern float mode8_velocity_forward_ki;
extern float mode8_velocity_forward_kd;

extern float mode4_velocity_smooth_tau_s;
extern float mode4_velocity_output_limit;
extern float mode4_velocity_pid_output_limit;
extern float mode4_velocity_i_limit;
extern float mode4_velocity_strafe_kp;
extern float mode4_velocity_strafe_ki;
extern float mode4_velocity_strafe_kd;
extern float mode4_velocity_forward_kp;
extern float mode4_velocity_forward_ki;
extern float mode4_velocity_forward_kd;

extern float mode3_velocity_smooth_tau_s;
extern float mode3_velocity_output_limit;
extern float mode3_velocity_pid_output_limit;
extern float mode3_velocity_i_limit;
extern float mode3_velocity_strafe_kp;
extern float mode3_velocity_strafe_ki;
extern float mode3_velocity_strafe_kd;
extern float mode3_velocity_forward_kp;
extern float mode3_velocity_forward_ki;
extern float mode3_velocity_forward_kd;

extern float s_curve_max_iter;
extern float s_curve_conv_tol;
extern float s_curve_min_dist;
extern float carplanfix_enable;
extern float carplanfix_mode3_beacon1_enable;
extern float motor_test_pwm;            /* C_Debug左轮PWM */
extern float wheel_left_count_per_meter;
extern float wheel_right_count_per_meter;
extern float wheel_track_m;

/* 两轮菜单调试命令。发车状态不写入Flash，上电固定为停止。 */
#define CAR_DEBUG_MODE_PWM          (0U)
#define CAR_DEBUG_MODE_WHEEL_SPEED  (1U)
#define CAR_DEBUG_MODE_YAW_RATE     (2U)
#define CAR_DEBUG_MODE_HEADING      (3U)

extern uint8 car_debug_run_enabled;
extern float car_debug_mode;
extern float car_debug_right_pwm;
extern float car_debug_left_target_count;
extern float car_debug_right_target_count;
extern float car_debug_linear_mps;
extern float car_debug_yaw_rate_rad_s;
extern float car_debug_heading_deg;

#endif
