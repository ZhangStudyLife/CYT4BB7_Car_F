/*********************************************************************************************************************
* 菜单用户配置实现文件 - 参数集中管理
*
* 功能：注册四电机速度环参数，并保留Flash存档读取/保存入口
********************************************************************************************************************/

#include "menu_config.h"

#define MENU_CAR_S_CURVE_MAX_ITER_INDEX  (55U)
#define MENU_CAR_S_CURVE_CONV_TOL_INDEX  (56U)
#define MENU_CAR_S_CURVE_MIN_DIST_INDEX  (57U)
#define MENU_CAR_CARPLANFIX_ENABLE_INDEX  (78U)
#define MENU_CAR_CARPLANFIX_START_B1_ENABLE_INDEX (79U)
#define MENU_CAR_EXPECTED_PARAM_COUNT     (80U)

#if (MENU_CAR_EXPECTED_PARAM_COUNT > MENU_MAX_PARAMS)
#error "Car menu parameter count exceeds MENU_MAX_PARAMS"
#endif

#if ((MENU_CAR_BOOT_FLASH_LOAD_ENABLE != 0U) && (MENU_CAR_BOOT_FLASH_LOAD_ENABLE != 1U))
#error "Car boot Flash load enable must be 0 or 1"
#endif

#if (MENU_CAR_BOOT_FLASH_LOAD_SLOT >= MENU_SLOT_COUNT)
#error "Car boot Flash load slot exceeds menu slot count"
#endif

//====================================================参数变量区====================================================
// 轮速PID参数（四个电机共用）
float wheel_kp = 2.3f;                  // 比例系数
float wheel_ki = 0.08f;                  // 积分系数
float wheel_kd = 0.0f;                  // 微分系数
float wheel_output_limit = 5000.0f;     // 输出限幅 (PWM)
float wheel_i_limit = 2500.0f;          // 积分限幅

//====================================================用户函数声明====================================================
float yaw_angle_kp = 3.8f;
float yaw_angle_ki = 0.0f;
float yaw_angle_kd = 1.0f;
float yaw_angle_i_limit = 4.0f;
float yaw_angle_output_limit = 100.0f;

float yaw_rate_kp = 125.0f;
float yaw_rate_ki = 0.03f;
float yaw_rate_kd = 0.0f;
float yaw_rate_i_limit = 50.0f;
float yaw_rate_output_limit = 3000.0f;

float mode7_velocity_smooth_tau_s = 0.12f;
float mode7_velocity_output_limit = 650.0f;
float mode7_velocity_pid_output_limit = 250.0f;
float mode7_velocity_i_limit = 0.0f;
float mode7_velocity_strafe_kp = 80.0f;
float mode7_velocity_strafe_ki = 0.0f;
float mode7_velocity_strafe_kd = 20.0f;
float mode7_velocity_forward_kp = 80.0f;
float mode7_velocity_forward_ki = 0.0f;
float mode7_velocity_forward_kd = 20.0f;

float mode5_velocity_smooth_tau_s = 0.162f;
float mode5_velocity_output_limit = 650.0f;
float mode5_velocity_pid_output_limit = 250.0f;
float mode5_velocity_i_limit = 0.0f;
float mode5_velocity_strafe_kp = 80.0f;
float mode5_velocity_strafe_ki = 0.0f;
float mode5_velocity_strafe_kd = 20.0f;
float mode5_velocity_forward_kp = 80.0f;
float mode5_velocity_forward_ki = 0.0f;
float mode5_velocity_forward_kd = 20.0f;

float mode2_velocity_smooth_tau_s = 0.162f;
float mode2_velocity_output_limit = 650.0f;
float mode2_velocity_pid_output_limit = 250.0f;
float mode2_velocity_i_limit = 0.0f;
float mode2_velocity_strafe_kp = 80.0f;
float mode2_velocity_strafe_ki = 0.0f;
float mode2_velocity_strafe_kd = 20.0f;
float mode2_velocity_forward_kp = 80.0f;
float mode2_velocity_forward_ki = 0.0f;
float mode2_velocity_forward_kd = 20.0f;

float mode8_velocity_smooth_tau_s = 0.162f;
float mode8_velocity_output_limit = 650.0f;
float mode8_velocity_pid_output_limit = 250.0f;
float mode8_velocity_i_limit = 0.0f;
float mode8_velocity_strafe_kp = 80.0f;
float mode8_velocity_strafe_ki = 0.0f;
float mode8_velocity_strafe_kd = 20.0f;
float mode8_velocity_forward_kp = 80.0f;
float mode8_velocity_forward_ki = 0.0f;
float mode8_velocity_forward_kd = 20.0f;

float mode4_velocity_smooth_tau_s = 0.162f;
float mode4_velocity_output_limit = 650.0f;
float mode4_velocity_pid_output_limit = 250.0f;
float mode4_velocity_i_limit = 0.0f;
float mode4_velocity_strafe_kp = 80.0f;
float mode4_velocity_strafe_ki = 0.0f;
float mode4_velocity_strafe_kd = 20.0f;
float mode4_velocity_forward_kp = 80.0f;
float mode4_velocity_forward_ki = 0.0f;
float mode4_velocity_forward_kd = 20.0f;

float mode3_velocity_smooth_tau_s = 0.162f;
float mode3_velocity_output_limit = 650.0f;
float mode3_velocity_pid_output_limit = 250.0f;
float mode3_velocity_i_limit = 0.0f;
float mode3_velocity_strafe_kp = 80.0f;
float mode3_velocity_strafe_ki = 0.0f;
float mode3_velocity_strafe_kd = 20.0f;
float mode3_velocity_forward_kp = 80.0f;
float mode3_velocity_forward_ki = 0.0f;
float mode3_velocity_forward_kd = 20.0f;

float s_curve_max_iter = 50.0f;
float s_curve_conv_tol = 0.001f;
float s_curve_min_dist = 5.0f;
float carplanfix_enable = 0.0f;
float carplanfix_mode3_beacon1_enable = 1.0f;

static void load_slot_0_function(void);
static void load_slot_1_function(void);
static void save_slot_0_function(void);
static void save_slot_1_function(void);
static void load_air_slot_0_function(void);
static void load_air_slot_1_function(void);
static void load_air_slot_2_function(void);
static void load_air_slot_3_function(void);
static void save_air_slot_0_function(void);
static void save_air_slot_1_function(void);
static void save_air_slot_2_function(void);
static void save_air_slot_3_function(void);
static void sync_air_function(void);
static void diag_imu_function(void);
static void diag_encoder_function(void);
static void diag_position_function(void);
static void diag_air_state_function(void);
static void diag_air_tof_function(void);
static void diag_air_flow_function(void);
static void diag_air_imu_function(void);
static void diag_air_attitude_function(void);
static void diag_air_rc_function(void);
static void diag_2bl3_status_function(void);

//====================================================菜单树定义====================================================
// 轮速PID子菜单（增量式）
static menu_item_t wheel_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 0},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 1},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 2},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 3},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 4},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t yaw_rate_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 5},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 6},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 7},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 8},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 9},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 加载存档子菜单
static menu_item_t load_slot_menu[] = {
    {"Load Car0", MENU_TYPE_FUNCTION, .function = load_slot_0_function},
    {"Load Car1", MENU_TYPE_FUNCTION, .function = load_slot_1_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 保存存档子菜单
static menu_item_t save_slot_menu[] = {
    {"Save Car0", MENU_TYPE_FUNCTION, .function = save_slot_0_function},
    {"Save Car1", MENU_TYPE_FUNCTION, .function = save_slot_1_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 主菜单
static menu_item_t yaw_angle_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 10},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 11},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 12},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 13},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 14},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode7_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 15},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 16},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 17},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 18},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 19},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 20},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 21},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 22},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 23},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 24},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode5_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 35},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 36},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 37},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 38},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 39},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 40},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 41},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 42},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 43},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 44},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode2_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 45},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 46},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 47},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 48},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 49},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 50},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 51},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 52},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 53},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 54},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode8_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 25},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 26},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 27},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 28},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 29},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 30},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 31},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 32},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 33},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 34},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode4_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 58},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 59},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 60},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 61},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 62},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 63},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 64},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 65},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 66},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 67},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode3_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 68},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 69},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 70},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 71},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 72},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 73},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 74},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 75},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 76},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 77},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t s_curve_menu[] = {
    {"MaxIter", MENU_TYPE_PARAMETER, .param_index = MENU_CAR_S_CURVE_MAX_ITER_INDEX},
    {"ConvTol", MENU_TYPE_PARAMETER, .param_index = MENU_CAR_S_CURVE_CONV_TOL_INDEX},
    {"MinDist", MENU_TYPE_PARAMETER, .param_index = MENU_CAR_S_CURVE_MIN_DIST_INDEX},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t core1_image_param_menu[] = {
    {"Camera", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Beacon", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Car Lamp", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Near Lamp", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Tracking", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static const char * const s_core1_param_group_names[] = {
    "Core1 Camera",
    "Core1 Beacon",
    "Core1 Car Lamp",
    "Core1 Near Lamp",
    "Core1 Tracking"
};

typedef char core1_param_group_count_must_match[
    ((sizeof(s_core1_param_group_names) /
      sizeof(s_core1_param_group_names[0])) == 5U) ? 1 : -1];

/* 2BL3图像参数二级分类菜单。 */
static menu_item_t bl3_image_param_menu[] = {
    {"Stream", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Threshold", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Beacon Area", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Car Lamp", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Reflection", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Weak Center", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Shape Filter", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Vertical Top", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Saturated Top", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Background", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Near Lamp", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Tracking", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Calibration", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

/* 二级菜单显示名与Air参数定义中的分组名映射。 */
static const char * const s_bl3_param_group_names[] = {
    "2BL3 Stream",
    "2BL3 Threshold",
    "2BL3 Beacon Area",
    "2BL3 Car Lamp",
    "2BL3 Reflection",
    "2BL3 Weak Center",
    "2BL3 Shape Filter",
    "2BL3 Vertical Top",
    "2BL3 Saturated Top",
    "2BL3 Background",
    "2BL3 Near Lamp",
    "2BL3 Tracking",
    "2BL3 Calibration"
};

typedef char bl3_param_group_count_must_match[
    ((sizeof(s_bl3_param_group_names) / sizeof(s_bl3_param_group_names[0])) == 13U) ? 1 : -1];

static menu_item_t air_param_menu[] = {
    {"Basic", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Gyro PID", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Angle PID", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Estimation", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode2 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode2 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode3 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode3 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode4 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode4 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode5 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode5 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode7 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode8 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode8 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Yaw Change", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Core1 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"2BL3 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Car Plan", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

enum
{
    AIR_PARAM_MENU_COUNT = (sizeof(air_param_menu) / sizeof(air_param_menu[0])) - 1U,
    CORE1_PARAM_MENU_COUNT = (sizeof(core1_image_param_menu) /
                              sizeof(core1_image_param_menu[0])) - 1U,
    BL3_PARAM_MENU_COUNT = (sizeof(bl3_image_param_menu) / sizeof(bl3_image_param_menu[0])) - 1U,
    AIR_PARAM_MENU_STORAGE_COUNT = MENU_AIR_MAX_PARAMS + AIR_PARAM_MENU_COUNT +
                                   CORE1_PARAM_MENU_COUNT +
                                   BL3_PARAM_MENU_COUNT + 1U
};

static menu_item_t s_air_param_menu_storage[AIR_PARAM_MENU_STORAGE_COUNT];
static menu_item_t *s_air_group_menus[AIR_PARAM_MENU_COUNT];
static menu_item_t *s_core1_group_menus[CORE1_PARAM_MENU_COUNT];
/* 2BL3二级分组菜单在统一存储区中的起始指针。 */
static menu_item_t *s_bl3_group_menus[BL3_PARAM_MENU_COUNT];

static menu_item_t load_air_slot_menu[] = {
    {"Load Air0", MENU_TYPE_FUNCTION, .function = load_air_slot_0_function},
    {"Load Air1", MENU_TYPE_FUNCTION, .function = load_air_slot_1_function},
    {"Load Air2", MENU_TYPE_FUNCTION, .function = load_air_slot_2_function},
    {"Load Air3", MENU_TYPE_FUNCTION, .function = load_air_slot_3_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t save_air_slot_menu[] = {
    {"Save Air0", MENU_TYPE_FUNCTION, .function = save_air_slot_0_function},
    {"Save Air1", MENU_TYPE_FUNCTION, .function = save_air_slot_1_function},
    {"Save Air2", MENU_TYPE_FUNCTION, .function = save_air_slot_2_function},
    {"Save Air3", MENU_TYPE_FUNCTION, .function = save_air_slot_3_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_param_menu[] = {
    {"Wheel PID", MENU_TYPE_SUBMENU, .submenu = wheel_pid_menu},
    {"YawRate PID", MENU_TYPE_SUBMENU, .submenu = yaw_rate_pid_menu},
    {"YawAng PID", MENU_TYPE_SUBMENU, .submenu = yaw_angle_pid_menu},
    {"Mode2 Vel", MENU_TYPE_SUBMENU, .submenu = mode2_velocity_pid_menu},
    {"Mode5 Vel", MENU_TYPE_SUBMENU, .submenu = mode5_velocity_pid_menu},
    {"Mode7 Vel", MENU_TYPE_SUBMENU, .submenu = mode7_velocity_pid_menu},
    {"Mode3 Vel", MENU_TYPE_SUBMENU, .submenu = mode3_velocity_pid_menu},
    {"Mode4 Vel", MENU_TYPE_SUBMENU, .submenu = mode4_velocity_pid_menu},
    {"Mode8 Vel", MENU_TYPE_SUBMENU, .submenu = mode8_velocity_pid_menu},
    {"S Curve", MENU_TYPE_SUBMENU, .submenu = s_curve_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_diag_menu[] = {
    {"IMU", MENU_TYPE_DIAG_VIEW, .function = diag_imu_function},
    {"Encoder", MENU_TYPE_DIAG_VIEW, .function = diag_encoder_function},
    {"Position", MENU_TYPE_DIAG_VIEW, .function = diag_position_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_diag_menu[] = {
    {"A_State", MENU_TYPE_DIAG_VIEW, .function = diag_air_state_function},
    {"2BL3 Status", MENU_TYPE_DIAG_VIEW, .function = diag_2bl3_status_function},
    {"A_ToF", MENU_TYPE_DIAG_VIEW, .function = diag_air_tof_function},
    {"A_Flow", MENU_TYPE_DIAG_VIEW, .function = diag_air_flow_function},
    {"A_IMU", MENU_TYPE_DIAG_VIEW, .function = diag_air_imu_function},
    {"A_Attitude", MENU_TYPE_DIAG_VIEW, .function = diag_air_attitude_function},
    {"A_RC", MENU_TYPE_DIAG_VIEW, .function = diag_air_rc_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t carplanfix_menu[] = {
    {"Enable", MENU_TYPE_PARAMETER, .param_index = MENU_CAR_CARPLANFIX_ENABLE_INDEX},
    {"Start B1", MENU_TYPE_PARAMETER, .param_index = MENU_CAR_CARPLANFIX_START_B1_ENABLE_INDEX},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_menu[] = {
    {"C_params", MENU_TYPE_SUBMENU, .submenu = car_param_menu},
    {"C_Diag", MENU_TYPE_SUBMENU, .submenu = car_diag_menu},
    {"C_Load", MENU_TYPE_SUBMENU, .submenu = load_slot_menu},
    {"C_Save", MENU_TYPE_SUBMENU, .submenu = save_slot_menu},
    {"C_Beacon", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"PlanFix", MENU_TYPE_SUBMENU, .submenu = carplanfix_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_menu[] = {
    {"A_params", MENU_TYPE_SUBMENU, .submenu = air_param_menu},
    {"A_Diag", MENU_TYPE_SUBMENU, .submenu = air_diag_menu},
    {"Sync Air", MENU_TYPE_FUNCTION, .function = sync_air_function},
    {"A_Load", MENU_TYPE_SUBMENU, .submenu = load_air_slot_menu},
    {"A_Save", MENU_TYPE_SUBMENU, .submenu = save_air_slot_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t main_menu[] = {
    {"Car", MENU_TYPE_SUBMENU, .submenu = car_menu},
    {"Air", MENU_TYPE_SUBMENU, .submenu = air_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static uint8 menu_build_air_param_menus(void)
{
    uint16 cursor = 0U;
    uint8 group;
    uint8 other_group;
    uint8 core1_top_found = 0U;
    uint8 bl3_top_found = 0U;
    uint16 index;
    uint16 other_index;
    uint8 group_count;
    const menu_air_param_config_t *config;
    const menu_air_param_config_t *other_config;
    const char *item_name;
    menu_item_t *item;

    memset(s_air_param_menu_storage, 0, sizeof(s_air_param_menu_storage));
    memset(s_air_group_menus, 0, sizeof(s_air_group_menus));
    memset(s_core1_group_menus, 0, sizeof(s_core1_group_menus));
    memset(s_bl3_group_menus, 0, sizeof(s_bl3_group_menus));

    for(index = 0U; index < menu_get_air_param_count(); index++)
    {
        config = menu_get_air_param_config(index);
        if(config == NULL)
        {
            return 1U;
        }

        for(other_index = index + 1U;
            other_index < menu_get_air_param_count();
            other_index++)
        {
            other_config = menu_get_air_param_config(other_index);
            if((other_config == NULL) || (strcmp(config->name, other_config->name) == 0))
            {
                return 1U;
            }
        }
    }

    for(group = 0U; group < AIR_PARAM_MENU_COUNT; group++)
    {
        for(other_group = (uint8)(group + 1U);
            other_group < AIR_PARAM_MENU_COUNT;
            other_group++)
        {
            if(strcmp(air_param_menu[group].name, air_param_menu[other_group].name) == 0)
            {
                return 1U;
            }
        }
    }

    for(group = 0U; group < AIR_PARAM_MENU_COUNT; group++)
    {
        if(strcmp(air_param_menu[group].name, "Core1 Img") == 0)
        {
            s_air_group_menus[group] = core1_image_param_menu;
            core1_top_found = 1U;
            continue;
        }
        if(strcmp(air_param_menu[group].name, "2BL3 Img") == 0)
        {
            s_air_group_menus[group] = bl3_image_param_menu;
            bl3_top_found = 1U;
            continue;
        }

        s_air_group_menus[group] = &s_air_param_menu_storage[cursor];
        group_count = 0U;

        for(index = 0U; index < menu_get_air_param_count(); index++)
        {
            config = menu_get_air_param_config(index);
            if((config == NULL) || (config->menu_name == NULL) ||
               (config->visible == 0U) ||
               (strcmp(config->menu_name, air_param_menu[group].name) != 0))
            {
                continue;
            }

            if((group_count >= MENU_MAX_ITEMS) ||
               (cursor >= AIR_PARAM_MENU_STORAGE_COUNT))
            {
                return 1U;
            }

            item = &s_air_param_menu_storage[cursor++];
            strncpy(item->name, config->name, sizeof(item->name) - 1U);
            item->name[sizeof(item->name) - 1U] = '\0';
            item->type = MENU_TYPE_AIR_PARAMETER;
            item->param_index = index;
            group_count++;
        }

        if(cursor >= AIR_PARAM_MENU_STORAGE_COUNT)
        {
            return 1U;
        }
        item = &s_air_param_menu_storage[cursor++];
        item->name[0] = '\0';
        item->type = MENU_TYPE_SUBMENU;
        item->submenu = NULL;
    }

    if((core1_top_found == 0U) || (bl3_top_found == 0U))
    {
        return 1U;
    }

    for(group = 0U; group < CORE1_PARAM_MENU_COUNT; group++)
    {
        s_core1_group_menus[group] = &s_air_param_menu_storage[cursor];
        group_count = 0U;

        for(index = 0U; index < menu_get_air_param_count(); index++)
        {
            config = menu_get_air_param_config(index);
            if((config == NULL) || (config->menu_name == NULL) ||
               (config->visible == 0U) ||
               (strcmp(config->menu_name,
                       s_core1_param_group_names[group]) != 0))
            {
                continue;
            }

            if((group_count >= MENU_MAX_ITEMS) ||
               (cursor >= AIR_PARAM_MENU_STORAGE_COUNT))
            {
                return 1U;
            }

            item = &s_air_param_menu_storage[cursor++];
            strncpy(item->name, config->name, sizeof(item->name) - 1U);
            item->name[sizeof(item->name) - 1U] = '\0';
            item->type = MENU_TYPE_AIR_PARAMETER;
            item->param_index = index;
            group_count++;
        }

        if(cursor >= AIR_PARAM_MENU_STORAGE_COUNT)
        {
            return 1U;
        }
        item = &s_air_param_menu_storage[cursor++];
        item->name[0] = '\0';
        item->type = MENU_TYPE_SUBMENU;
        item->submenu = NULL;
        core1_image_param_menu[group].submenu = s_core1_group_menus[group];
    }

    for(group = 0U; group < BL3_PARAM_MENU_COUNT; group++)
    {
        s_bl3_group_menus[group] = &s_air_param_menu_storage[cursor];
        group_count = 0U;

        for(index = 0U; index < menu_get_air_param_count(); index++)
        {
            config = menu_get_air_param_config(index);
            if((config == NULL) || (config->menu_name == NULL) ||
               (config->visible == 0U) ||
               (strcmp(config->menu_name, s_bl3_param_group_names[group]) != 0))
            {
                continue;
            }

            if((group_count >= MENU_MAX_ITEMS) ||
               (cursor >= AIR_PARAM_MENU_STORAGE_COUNT))
            {
                return 1U;
            }

            item = &s_air_param_menu_storage[cursor++];
            item_name = config->name;
            if(strcmp(config->name, "bl3_stream_mode") == 0)
            {
                item_name = "ImageMode";
            }
            strncpy(item->name, item_name, sizeof(item->name) - 1U);
            item->name[sizeof(item->name) - 1U] = '\0';
            item->type = MENU_TYPE_AIR_PARAMETER;
            item->param_index = index;
            group_count++;
        }

        if(cursor >= AIR_PARAM_MENU_STORAGE_COUNT)
        {
            return 1U;
        }
        item = &s_air_param_menu_storage[cursor++];
        item->name[0] = '\0';
        item->type = MENU_TYPE_SUBMENU;
        item->submenu = NULL;
        bl3_image_param_menu[group].submenu = s_bl3_group_menus[group];
    }

    for(group = 0U; group < AIR_PARAM_MENU_COUNT; group++)
    {
        air_param_menu[group].submenu = s_air_group_menus[group];
    }

    return 0U;
}

//====================================================用户配置初始化====================================================
void menu_config_init(void)
{
    // 注册轮速PID参数（四个电机共用）
    menu_register_param(&wheel_kp, 0.1f, 0.0f, 100.0f);                    // 参数0
    menu_register_param(&wheel_ki, 0.1f, 0.0f, 100.0f);                    // 参数1
    menu_register_param(&wheel_kd, 0.1f, 0.0f, 100.0f);                    // 参数2
    menu_register_param(&wheel_output_limit, 100.0f, 1000.0f, 10000.0f);   // 参数3
    menu_register_param(&wheel_i_limit, 100.0f, 0.0f, 10000.0f);           // 参数4 积分限幅

    menu_register_param(&yaw_rate_kp, 0.1f, 0.0f, 500.0f);
    menu_register_param(&yaw_rate_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&yaw_rate_kd, 0.1f, 0.0f, 500.0f);
    menu_register_param(&yaw_rate_i_limit, 0.1f, 0.0f, 1000.0f);
    menu_register_param(&yaw_rate_output_limit, 1.0f, 0.0f, 5000.0f);

    menu_register_param(&yaw_angle_kp, 0.1f, 0.0f, 50.0f);
    menu_register_param(&yaw_angle_ki, 0.01f, 0.0f, 50.0f);
    menu_register_param(&yaw_angle_kd, 0.01f, 0.0f, 50.0f);
    menu_register_param(&yaw_angle_i_limit, 0.1f, 0.0f, 100.0f);
    menu_register_param(&yaw_angle_output_limit, 0.1f, 0.0f, 100.0f);

    menu_register_param(&mode7_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode7_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode7_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode7_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode7_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&mode8_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode8_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode8_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode8_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode8_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&mode5_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode5_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode5_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode5_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode5_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&mode2_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode2_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode2_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode2_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode2_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&s_curve_max_iter, 1.0f, 5.0f, 200.0f);
    menu_register_param(&s_curve_conv_tol, 0.001f, 0.001f, 1000.0f);
    menu_register_param(&s_curve_min_dist, 0.1f, 1.0f, 1000.0f);

    menu_register_param(&mode4_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode4_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode4_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode4_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode4_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode4_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode4_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode4_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode4_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode4_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&mode3_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode3_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode3_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode3_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode3_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode3_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode3_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode3_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode3_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode3_velocity_forward_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&carplanfix_enable, 1.0f, 0.0f, 1.0f);
    menu_register_param(&carplanfix_mode3_beacon1_enable, 1.0f, 0.0f, 1.0f);

    if(menu_get_param_count() != MENU_CAR_EXPECTED_PARAM_COUNT)
    {
        menu_show_error("Car Menu Error");
        return;
    }

    if((MENU_CAR_BOOT_FLASH_LOAD_ENABLE != 0U) &&
       (menu_flash_check_slot(MENU_CAR_BOOT_FLASH_LOAD_SLOT) != 0U))
    {
        menu_flash_load_params(MENU_CAR_BOOT_FLASH_LOAD_SLOT);
    }

    menu_air_support_init();
    if(menu_build_air_param_menus() != 0U)
    {
        menu_show_error("Air Menu Error");
        return;
    }
    car_menu[4].submenu = beacon_position_recorder_get_menu();
    if(car_menu[4].submenu == NULL)
    {
        menu_show_error("Beacon Menu Error");
        return;
    }
    menu_set_root(main_menu);
}

//====================================================用户函数实现====================================================
static void load_slot_0_function(void)
{
    menu_load_slot(0);
}

static void load_slot_1_function(void)
{
    menu_load_slot(1);
}

static void save_slot_0_function(void)
{
    menu_save_slot(0);
}

static void save_slot_1_function(void)
{
    menu_save_slot(1);
}

static void load_air_slot_0_function(void)
{
    if(menu_load_air_slot(0U) == 0U)
    {
        menu_show_progress("Air Loading");
    }
}

static void load_air_slot_1_function(void)
{
    if(menu_load_air_slot(1U) == 0U)
    {
        menu_show_progress("Air Loading");
    }
}

static void load_air_slot_2_function(void)
{
    if(menu_load_air_slot(2U) == 0U)
    {
        menu_show_progress("Air Loading");
    }
}

static void load_air_slot_3_function(void)
{
    if(menu_load_air_slot(3U) == 0U)
    {
        menu_show_progress("Air Loading");
    }
}

static void save_air_slot_0_function(void)
{
    if(menu_save_air_slot(0U) == 0U)
    {
        menu_show_success("Air Save OK");
    }
}

static void save_air_slot_1_function(void)
{
    if(menu_save_air_slot(1U) == 0U)
    {
        menu_show_success("Air Save OK");
    }
}

static void save_air_slot_2_function(void)
{
    if(menu_save_air_slot(2U) == 0U)
    {
        menu_show_success("Air Save OK");
    }
}

static void save_air_slot_3_function(void)
{
    if(menu_save_air_slot(3U) == 0U)
    {
        menu_show_success("Air Save OK");
    }
}

static void sync_air_function(void)
{
    if(menu_sync_all_air_params() == 0U)
    {
        menu_show_progress("Air Syncing");
    }
}

/* 显示一行诊断文本（line 0-7，每行16像素高） */
static void diag_show_line(uint8 line, const char *text)
{
    if(line >= MENU_MAX_VISIBLE_LINES)
    {
        return;
    }

    menu_show_text_line(line, text, UI_COLOR_NORMAL);
}

static void diag_clear_lines(uint8 first, uint8 last)
{
    uint8 line;

    for(line = first; (line <= last) && (line < MENU_MAX_VISIBLE_LINES); line++)
    {
        menu_clear_line(line);
    }
}

/* 诊断页初始化：保留屏幕内容，由行缓存仅更新变化部分。 */
static void diag_begin(void)
{
    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_set_font(UI_FONT_NORMAL);
}

/* 诊断页：IMU数据（欧拉角 + 陀螺仪 + 加速度计） */
static void diag_imu_function(void)
{
    char text[32];

    diag_begin();
    sprintf(text, "IMU RPY");
    diag_show_line(0U, text);
    sprintf(text, "R:%7.2f", (double)g_euler.roll);
    diag_show_line(1U, text);
    sprintf(text, "P:%7.2f Y:%7.2f", (double)g_euler.pitch, (double)g_euler.yaw);
    diag_show_line(2U, text);
    sprintf(text, "Gx:%7.2f", (double)g_imufilter_1000hz.gyrox);
    diag_show_line(3U, text);
    sprintf(text, "Gy:%7.2f Gz:%7.2f", (double)g_imufilter_1000hz.gyroy, (double)g_imufilter_1000hz.gyroz);
    diag_show_line(4U, text);
    sprintf(text, "Ax:%6.3f Ay:%6.3f", (double)g_imufilter_1000hz.accx, (double)g_imufilter_1000hz.accy);
    diag_show_line(5U, text);
    sprintf(text, "Az:%6.3f Ready:%u", (double)g_imufilter_1000hz.accz, (unsigned int)g_imu_ready);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* 诊断页：四轮编码器（滤波值 + 原始值） */
static void diag_encoder_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Encoder filt");
    sprintf(text, "LF:%7.1f", (double)encoder_get_left_front_filtered_count());
    diag_show_line(1U, text);
    sprintf(text, "RF:%7.1f", (double)encoder_get_right_front_filtered_count());
    diag_show_line(2U, text);
    sprintf(text, "LR:%7.1f", (double)encoder_get_left_rear_filtered_count());
    diag_show_line(3U, text);
    sprintf(text, "RR:%7.1f", (double)encoder_get_right_rear_filtered_count());
    diag_show_line(4U, text);
    sprintf(text, "Raw %d %d", (int)encoder_get_left_front_count(), (int)encoder_get_right_front_count());
    diag_show_line(5U, text);
    sprintf(text, "Raw %d %d", (int)encoder_get_left_rear_count(), (int)encoder_get_right_rear_count());
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* 诊断页：里程计位置与全局速度。 */
static void diag_position_function(void)
{
    char text[32];

    diag_begin();
    sprintf(text, "Odo Xr:%6.3f", (double)g_odometer.position[x]);
    diag_show_line(0U, text);
    sprintf(text, "Odo Yf:%6.3f", (double)g_odometer.position[y]);
    diag_show_line(1U, text);
    sprintf(text, "Vel R/F:%4.1f%4.1f",
            (double)g_odometer.vel[x],
            (double)g_odometer.vel[y]);
    diag_show_line(2U, text);
    diag_clear_lines(3U, 6U);
    diag_show_line(7U, "Back/Enter Exit");
}

/* Air诊断页：通信在线状态、飞行状态和Air时间戳。 */
static void diag_air_state_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air State");
    sprintf(text, "On:%u Fresh:%u",
            (unsigned int)air_comm_car_is_online(),
            (unsigned int)air_comm_car_is_run_data_fresh());
    diag_show_line(1U, text);
    sprintf(text, "State:%u", (unsigned int)(uint8)g_air_state);
    diag_show_line(2U, text);
    sprintf(text, "Sync:%7.0fms", (double)g_air_sync_time_ms);
    diag_show_line(3U, text);
    diag_clear_lines(4U, 6U);
    diag_show_line(7U, "Back/Enter Exit");
}

/**
 * @brief 显示2BL3链路和菜单同步状态。
 * @return 无。
 */
static void diag_2bl3_status_function(void)
{
    uint16 index;
    uint16 available_count = 0U;
    uint16 spi_error_code;
    uint8 spi_error0;
    uint8 spi_error1;
    char text[32];
    const char *failed_name = "--";
    const menu_air_param_config_t *config;
    menu_air_sync_status_t sync_status;

    menu_get_air_sync_status(&sync_status);
    spi_error_code = (uint16)g_air_diag_telemetry.camera_spi_error_code;
    spi_error0 = (uint8)(spi_error_code >> 8);
    spi_error1 = (uint8)(spi_error_code & 0xFFU);
    for(index = 0U; index < menu_get_air_param_count(); index++)
    {
        if(menu_air_param_is_available(index) != 0U)
        {
            available_count++;
        }
    }
    if(sync_status.last_failed_index < menu_get_air_param_count())
    {
        config = menu_get_air_param_config(sync_status.last_failed_index);
        if(config != NULL)
        {
            failed_name = config->name;
        }
    }

    diag_begin();
    diag_show_line(0U, "2BL3 Status");
    snprintf(text, sizeof(text), "On:%u Fr:%u Cam:%u",
             (unsigned int)air_comm_car_is_online(),
             (unsigned int)air_comm_car_is_run_data_fresh(),
             (unsigned int)(uint8)g_air_car_plan_camera);
    diag_show_line(1U, text);
    snprintf(text, sizeof(text), "S:%u%u R:%u%u E:%u/%u",
             (unsigned int)(uint8)g_air_diag_telemetry.camera_spi_online[0],
             (unsigned int)(uint8)g_air_diag_telemetry.camera_spi_online[1],
             (unsigned int)(uint8)g_air_diag_telemetry.camera_spi_ready[0],
             (unsigned int)(uint8)g_air_diag_telemetry.camera_spi_ready[1],
             (unsigned int)spi_error0,
             (unsigned int)spi_error1);
    diag_show_line(2U, text);
    snprintf(text, sizeof(text), "H:%02X%02X/%02X%02X",
             (unsigned int)(uint8)g_air_diag_telemetry.camera_spi_rx_head[0][0],
             (unsigned int)(uint8)g_air_diag_telemetry.camera_spi_rx_head[0][1],
             (unsigned int)(uint8)g_air_diag_telemetry.camera_spi_rx_head[1][0],
             (unsigned int)(uint8)g_air_diag_telemetry.camera_spi_rx_head[1][1]);
    diag_show_line(3U, text);
    snprintf(text, sizeof(text), "Sync:%u/%u D:%u",
             (unsigned int)available_count,
             (unsigned int)menu_get_air_param_count(),
             (unsigned int)sync_status.dirty_count);
    diag_show_line(4U, text);
    snprintf(text, sizeof(text), "Ack:%u/%u F:%.11s",
             (unsigned int)sync_status.last_failed_result,
             (unsigned int)sync_status.last_failed_status,
             failed_name);
    diag_show_line(5U, text);
    diag_clear_lines(6U, 6U);
    diag_show_line(7U, "Back/Enter Exit");
}

/* Air诊断页：四路TOF高度与融合高度。 */
static void diag_air_tof_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air ToF mm");
    sprintf(text, "T1:%6.0f T2:%6.0f",
            (double)g_air_diag_telemetry.tof_raw_height_mm[0],
            (double)g_air_diag_telemetry.tof_raw_height_mm[1]);
    diag_show_line(1U, text);
    sprintf(text, "T3:%6.0f T4:%6.0f",
            (double)g_air_diag_telemetry.tof_raw_height_mm[2],
            (double)g_air_diag_telemetry.tof_raw_height_mm[3]);
    diag_show_line(2U, text);
    sprintf(text, "Fused:%8.1f", (double)g_air_tof_fused_height_mm);
    diag_show_line(3U, text);
    diag_clear_lines(4U, 6U);
    diag_show_line(7U, "Back/Enter Exit");
}

static void diag_air_flow_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air Flow X/Y");
    sprintf(text, "Raw:%7.1f %7.1f",
            (double)g_air_diag_telemetry.flow_raw_x,
            (double)g_air_diag_telemetry.flow_raw_y);
    diag_show_line(1U, text);
    sprintf(text, "Filt:%6.2f %6.2f",
            (double)g_air_diag_telemetry.flow_filtered_x,
            (double)g_air_diag_telemetry.flow_filtered_y);
    diag_show_line(2U, text);
    diag_clear_lines(3U, 6U);
    diag_show_line(7U, "Back/Enter Exit");
}

static void diag_air_imu_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air IMU X/Y/Z");
    sprintf(text, "RG:%6.1f %6.1f %6.1f",
            (double)g_air_diag_telemetry.imu_raw_gyro[0],
            (double)g_air_diag_telemetry.imu_raw_gyro[1],
            (double)g_air_diag_telemetry.imu_raw_gyro[2]);
    diag_show_line(1U, text);
    sprintf(text, "RA:%5.2f %5.2f %5.2f",
            (double)g_air_diag_telemetry.imu_raw_acc[0],
            (double)g_air_diag_telemetry.imu_raw_acc[1],
            (double)g_air_diag_telemetry.imu_raw_acc[2]);
    diag_show_line(2U, text);
    sprintf(text, "FG:%6.1f %6.1f %6.1f",
            (double)g_air_diag_telemetry.imu_filtered_gyro[0],
            (double)g_air_diag_telemetry.imu_filtered_gyro[1],
            (double)g_air_diag_telemetry.imu_filtered_gyro[2]);
    diag_show_line(3U, text);
    sprintf(text, "FA:%5.2f %5.2f %5.2f",
            (double)g_air_diag_telemetry.imu_filtered_acc[0],
            (double)g_air_diag_telemetry.imu_filtered_acc[1],
            (double)g_air_diag_telemetry.imu_filtered_acc[2]);
    diag_show_line(4U, text);
    sprintf(text, "RP:%7.2f %7.2f", (double)g_air_euler_roll, (double)g_air_euler_pitch);
    diag_show_line(5U, text);
    sprintf(text, "Y:%9.2f", (double)g_air_euler_yaw);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* Air诊断页：姿态角、TOF高度和位置估计速度。 */
static void diag_air_attitude_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air Attitude");
    sprintf(text, "Roll:%8.2f", (double)g_air_euler_roll);
    diag_show_line(1U, text);
    sprintf(text, "Pitch:%7.2f", (double)g_air_euler_pitch);
    diag_show_line(2U, text);
    sprintf(text, "Yaw:%9.2f", (double)g_air_euler_yaw);
    diag_show_line(3U, text);
    sprintf(text, "TOF:%7.1fmm", (double)g_air_tof_fused_height_mm);
    diag_show_line(4U, text);
    sprintf(text, "Vx:%8.3f", (double)g_air_pos_est_vel_x);
    diag_show_line(5U, text);
    sprintf(text, "Vy:%8.3f", (double)g_air_pos_est_vel_y);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* Air诊断页：CRSF标准化通道0-8 */
static void diag_air_rc_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air RC");
    sprintf(text, "0:%5.0f 1:%5.0f", (double)g_air_crsf_std_ch0, (double)g_air_crsf_std_ch1);
    diag_show_line(1U, text);
    sprintf(text, "2:%5.0f 3:%5.0f", (double)g_air_crsf_std_ch2, (double)g_air_crsf_std_ch3);
    diag_show_line(2U, text);
    sprintf(text, "4:%5.0f 5:%5.0f", (double)g_air_crsf_std_ch4, (double)g_air_crsf_std_ch5);
    diag_show_line(3U, text);
    sprintf(text, "6:%5.0f 7:%5.0f", (double)g_air_crsf_std_ch6, (double)g_air_crsf_std_ch7);
    diag_show_line(4U, text);
    sprintf(text, "8:%5.0f", (double)g_air_crsf_std_ch8);
    diag_show_line(5U, text);
    sprintf(text, "State:%u", (unsigned int)(uint8)g_air_state);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}
