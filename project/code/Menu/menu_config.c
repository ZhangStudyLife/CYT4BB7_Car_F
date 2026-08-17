#include "menu_config.h"

#define MENU_CAR_EXPECTED_PARAM_COUNT (182U)

static void diag_imu_function(void);
static void diag_encoder_function(void);
static void diag_air_state_function(void);
static void diag_air_tof_function(void);
static void diag_air_flow_function(void);
static void diag_air_imu_function(void);
static void diag_air_attitude_function(void);
static void diag_air_rc_function(void);
static void diag_2bl3_status_function(void);
static void pwm_test_function(void);

static int16 s_pwm_test_left = 0;
static int16 s_pwm_test_right = 0;
static uint8 s_pwm_test_state = 0U;

static menu_item_t car_diag_menu[] = {
    {"IMU", MENU_TYPE_DIAG_VIEW, .function = diag_imu_function},
    {"Encoder", MENU_TYPE_DIAG_VIEW, .function = diag_encoder_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_input_menu[] = {
    {"Plan Limit", MENU_TYPE_PARAMETER, .param_index = 14U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_left_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 0U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 1U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 2U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_right_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 3U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 4U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 5U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_speed_menu[] = {
    {"Filter", MENU_TYPE_PARAMETER, .param_index = 6U},
    {"FF Static", MENU_TYPE_PARAMETER, .param_index = 7U},
    {"Wheel Limit", MENU_TYPE_PARAMETER, .param_index = 16U},
    {"I Limit", MENU_TYPE_PARAMETER, .param_index = 17U},
    {"FF Dead", MENU_TYPE_PARAMETER, .param_index = 18U},
    {"FF Trans", MENU_TYPE_PARAMETER, .param_index = 19U},
    {"Inc Limit", MENU_TYPE_PARAMETER, .param_index = 20U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_yaw_rate_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 8U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 9U},
    {"Kff", MENU_TYPE_PARAMETER, .param_index = 10U},
    {"K Turn", MENU_TYPE_PARAMETER, .param_index = 11U},
    {"Out Limit", MENU_TYPE_PARAMETER, .param_index = 23U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_yaw_angle_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 12U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 13U},
    {"Stop Angle", MENU_TYPE_PARAMETER, .param_index = 15U},
    {"Rate Limit", MENU_TYPE_PARAMETER, .param_index = 24U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_brake_menu[] = {
    {"Static FF", MENU_TYPE_PARAMETER, .param_index = 21U},
    {"Decel Step", MENU_TYPE_PARAMETER, .param_index = 22U},
    {"Gz Stop", MENU_TYPE_PARAMETER, .param_index = 25U},
    {"Wheel Stop", MENU_TYPE_PARAMETER, .param_index = 26U},
    {"Target Margin", MENU_TYPE_PARAMETER, .param_index = 39U},
    {"FF Fade", MENU_TYPE_PARAMETER, .param_index = 40U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_large_turn_menu[] = {
    {"Brake Speed", MENU_TYPE_PARAMETER, .param_index = 27U},
    {"Brake Target", MENU_TYPE_PARAMETER, .param_index = 28U},
    {"Brake FF", MENU_TYPE_PARAMETER, .param_index = 29U},
    {"Brake Rate", MENU_TYPE_PARAMETER, .param_index = 30U},
    {"Enter Deg", MENU_TYPE_PARAMETER, .param_index = 31U},
    {"Pivot Exit", MENU_TYPE_PARAMETER, .param_index = 32U},
    {"Exit Start", MENU_TYPE_PARAMETER, .param_index = 33U},
    {"Finish Deg", MENU_TYPE_PARAMETER, .param_index = 34U},
    {"Trigger Cyc", MENU_TYPE_PARAMETER, .param_index = 35U},
    {"Finish Cyc", MENU_TYPE_PARAMETER, .param_index = 36U},
    {"Timeout Cyc", MENU_TYPE_PARAMETER, .param_index = 37U},
    {"Cmd Match", MENU_TYPE_PARAMETER, .param_index = 38U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode1_menu[] = {
    {"Input", MENU_TYPE_SUBMENU, .submenu = car_mode1_input_menu},
    {"L Speed", MENU_TYPE_SUBMENU, .submenu = car_mode1_left_speed_menu},
    {"R Speed", MENU_TYPE_SUBMENU, .submenu = car_mode1_right_speed_menu},
    {"Speed", MENU_TYPE_SUBMENU, .submenu = car_mode1_speed_menu},
    {"Yaw Rate", MENU_TYPE_SUBMENU, .submenu = car_mode1_yaw_rate_menu},
    {"Yaw Angle", MENU_TYPE_SUBMENU, .submenu = car_mode1_yaw_angle_menu},
    {"Brake", MENU_TYPE_SUBMENU, .submenu = car_mode1_brake_menu},
    {"Large Turn", MENU_TYPE_SUBMENU, .submenu = car_mode1_large_turn_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_input_menu[] = {
    {"Target m/s", MENU_TYPE_PARAMETER, .param_index = 55U},
    {"Deadzone", MENU_TYPE_PARAMETER, .param_index = 56U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_left_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 41U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 42U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 43U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_right_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 44U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 45U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 46U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_speed_menu[] = {
    {"Filter", MENU_TYPE_PARAMETER, .param_index = 47U},
    {"FF Static", MENU_TYPE_PARAMETER, .param_index = 48U},
    {"Wheel Limit", MENU_TYPE_PARAMETER, .param_index = 58U},
    {"I Limit", MENU_TYPE_PARAMETER, .param_index = 59U},
    {"FF Dead", MENU_TYPE_PARAMETER, .param_index = 60U},
    {"FF Trans", MENU_TYPE_PARAMETER, .param_index = 61U},
    {"Inc Limit", MENU_TYPE_PARAMETER, .param_index = 62U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_yaw_rate_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 49U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 50U},
    {"Kff", MENU_TYPE_PARAMETER, .param_index = 51U},
    {"K Turn", MENU_TYPE_PARAMETER, .param_index = 52U},
    {"Out Limit", MENU_TYPE_PARAMETER, .param_index = 65U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_yaw_angle_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 53U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 54U},
    {"Stop Angle", MENU_TYPE_PARAMETER, .param_index = 57U},
    {"Rate Limit", MENU_TYPE_PARAMETER, .param_index = 66U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_brake_menu[] = {
    {"Static FF", MENU_TYPE_PARAMETER, .param_index = 63U},
    {"Decel Step", MENU_TYPE_PARAMETER, .param_index = 64U},
    {"Gz Stop", MENU_TYPE_PARAMETER, .param_index = 67U},
    {"Wheel Stop", MENU_TYPE_PARAMETER, .param_index = 68U},
    {"Target Margin", MENU_TYPE_PARAMETER, .param_index = 81U},
    {"FF Fade", MENU_TYPE_PARAMETER, .param_index = 82U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_large_turn_menu[] = {
    {"Brake Speed", MENU_TYPE_PARAMETER, .param_index = 69U},
    {"Brake Target", MENU_TYPE_PARAMETER, .param_index = 70U},
    {"Brake FF", MENU_TYPE_PARAMETER, .param_index = 71U},
    {"Brake Rate", MENU_TYPE_PARAMETER, .param_index = 72U},
    {"Enter Deg", MENU_TYPE_PARAMETER, .param_index = 73U},
    {"Pivot Exit", MENU_TYPE_PARAMETER, .param_index = 74U},
    {"Exit Start", MENU_TYPE_PARAMETER, .param_index = 75U},
    {"Finish Deg", MENU_TYPE_PARAMETER, .param_index = 76U},
    {"Trigger Cyc", MENU_TYPE_PARAMETER, .param_index = 77U},
    {"Finish Cyc", MENU_TYPE_PARAMETER, .param_index = 78U},
    {"Timeout Cyc", MENU_TYPE_PARAMETER, .param_index = 79U},
    {"Cmd Match", MENU_TYPE_PARAMETER, .param_index = 80U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode2_menu[] = {
    {"Input", MENU_TYPE_SUBMENU, .submenu = car_mode2_input_menu},
    {"L Speed", MENU_TYPE_SUBMENU, .submenu = car_mode2_left_speed_menu},
    {"R Speed", MENU_TYPE_SUBMENU, .submenu = car_mode2_right_speed_menu},
    {"Speed", MENU_TYPE_SUBMENU, .submenu = car_mode2_speed_menu},
    {"Yaw Rate", MENU_TYPE_SUBMENU, .submenu = car_mode2_yaw_rate_menu},
    {"Yaw Angle", MENU_TYPE_SUBMENU, .submenu = car_mode2_yaw_angle_menu},
    {"Brake", MENU_TYPE_SUBMENU, .submenu = car_mode2_brake_menu},
    {"Large Turn", MENU_TYPE_SUBMENU, .submenu = car_mode2_large_turn_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_input_menu[] = {
    {"Plan Limit", MENU_TYPE_PARAMETER, .param_index = 97U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_left_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 83U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 84U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 85U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_right_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 86U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 87U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 88U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_speed_menu[] = {
    {"Filter", MENU_TYPE_PARAMETER, .param_index = 89U},
    {"FF Static", MENU_TYPE_PARAMETER, .param_index = 90U},
    {"Wheel Limit", MENU_TYPE_PARAMETER, .param_index = 99U},
    {"I Limit", MENU_TYPE_PARAMETER, .param_index = 100U},
    {"FF Dead", MENU_TYPE_PARAMETER, .param_index = 101U},
    {"FF Trans", MENU_TYPE_PARAMETER, .param_index = 102U},
    {"Inc Limit", MENU_TYPE_PARAMETER, .param_index = 103U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_yaw_rate_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 91U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 92U},
    {"Kff", MENU_TYPE_PARAMETER, .param_index = 93U},
    {"K Turn", MENU_TYPE_PARAMETER, .param_index = 94U},
    {"Out Limit", MENU_TYPE_PARAMETER, .param_index = 106U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_yaw_angle_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 95U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 96U},
    {"Stop Angle", MENU_TYPE_PARAMETER, .param_index = 98U},
    {"Rate Limit", MENU_TYPE_PARAMETER, .param_index = 107U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_brake_menu[] = {
    {"Static FF", MENU_TYPE_PARAMETER, .param_index = 104U},
    {"Decel Step", MENU_TYPE_PARAMETER, .param_index = 105U},
    {"Gz Stop", MENU_TYPE_PARAMETER, .param_index = 108U},
    {"Wheel Stop", MENU_TYPE_PARAMETER, .param_index = 109U},
    {"Target Margin", MENU_TYPE_PARAMETER, .param_index = 122U},
    {"FF Fade", MENU_TYPE_PARAMETER, .param_index = 123U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_large_turn_menu[] = {
    {"Brake Speed", MENU_TYPE_PARAMETER, .param_index = 110U},
    {"Brake Target", MENU_TYPE_PARAMETER, .param_index = 111U},
    {"Brake FF", MENU_TYPE_PARAMETER, .param_index = 112U},
    {"Brake Rate", MENU_TYPE_PARAMETER, .param_index = 113U},
    {"Enter Deg", MENU_TYPE_PARAMETER, .param_index = 114U},
    {"Pivot Exit", MENU_TYPE_PARAMETER, .param_index = 115U},
    {"Exit Start", MENU_TYPE_PARAMETER, .param_index = 116U},
    {"Finish Deg", MENU_TYPE_PARAMETER, .param_index = 117U},
    {"Trigger Cyc", MENU_TYPE_PARAMETER, .param_index = 118U},
    {"Finish Cyc", MENU_TYPE_PARAMETER, .param_index = 119U},
    {"Timeout Cyc", MENU_TYPE_PARAMETER, .param_index = 120U},
    {"Cmd Match", MENU_TYPE_PARAMETER, .param_index = 121U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_fuya_menu[] = {
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 178U},
    {"Target", MENU_TYPE_PARAMETER, .param_index = 179U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode4_menu[] = {
    {"Input", MENU_TYPE_SUBMENU, .submenu = car_mode4_input_menu},
    {"L Speed", MENU_TYPE_SUBMENU, .submenu = car_mode4_left_speed_menu},
    {"R Speed", MENU_TYPE_SUBMENU, .submenu = car_mode4_right_speed_menu},
    {"Speed", MENU_TYPE_SUBMENU, .submenu = car_mode4_speed_menu},
    {"Yaw Rate", MENU_TYPE_SUBMENU, .submenu = car_mode4_yaw_rate_menu},
    {"Yaw Angle", MENU_TYPE_SUBMENU, .submenu = car_mode4_yaw_angle_menu},
    {"Brake", MENU_TYPE_SUBMENU, .submenu = car_mode4_brake_menu},
    {"Large Turn", MENU_TYPE_SUBMENU, .submenu = car_mode4_large_turn_menu},
    {"Fuya", MENU_TYPE_SUBMENU, .submenu = car_mode4_fuya_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_input_menu[] = {
    {"Target m/s", MENU_TYPE_PARAMETER, .param_index = 138U},
    {"Deadzone", MENU_TYPE_PARAMETER, .param_index = 139U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_left_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 124U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 125U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 126U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_right_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 127U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 128U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 129U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_speed_menu[] = {
    {"Filter", MENU_TYPE_PARAMETER, .param_index = 130U},
    {"FF Static", MENU_TYPE_PARAMETER, .param_index = 131U},
    {"Wheel Limit", MENU_TYPE_PARAMETER, .param_index = 141U},
    {"I Limit", MENU_TYPE_PARAMETER, .param_index = 142U},
    {"FF Dead", MENU_TYPE_PARAMETER, .param_index = 143U},
    {"FF Trans", MENU_TYPE_PARAMETER, .param_index = 144U},
    {"Inc Limit", MENU_TYPE_PARAMETER, .param_index = 145U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_yaw_rate_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 132U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 133U},
    {"Kff", MENU_TYPE_PARAMETER, .param_index = 134U},
    {"K Turn", MENU_TYPE_PARAMETER, .param_index = 135U},
    {"Out Limit", MENU_TYPE_PARAMETER, .param_index = 148U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_yaw_angle_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 136U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 137U},
    {"Stop Angle", MENU_TYPE_PARAMETER, .param_index = 140U},
    {"Rate Limit", MENU_TYPE_PARAMETER, .param_index = 149U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_brake_menu[] = {
    {"Static FF", MENU_TYPE_PARAMETER, .param_index = 146U},
    {"Decel Step", MENU_TYPE_PARAMETER, .param_index = 147U},
    {"Gz Stop", MENU_TYPE_PARAMETER, .param_index = 150U},
    {"Wheel Stop", MENU_TYPE_PARAMETER, .param_index = 151U},
    {"Target Margin", MENU_TYPE_PARAMETER, .param_index = 164U},
    {"FF Fade", MENU_TYPE_PARAMETER, .param_index = 165U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_large_turn_menu[] = {
    {"Brake Speed", MENU_TYPE_PARAMETER, .param_index = 152U},
    {"Brake Target", MENU_TYPE_PARAMETER, .param_index = 153U},
    {"Brake FF", MENU_TYPE_PARAMETER, .param_index = 154U},
    {"Brake Rate", MENU_TYPE_PARAMETER, .param_index = 155U},
    {"Enter Deg", MENU_TYPE_PARAMETER, .param_index = 156U},
    {"Pivot Exit", MENU_TYPE_PARAMETER, .param_index = 157U},
    {"Exit Start", MENU_TYPE_PARAMETER, .param_index = 158U},
    {"Finish Deg", MENU_TYPE_PARAMETER, .param_index = 159U},
    {"Trigger Cyc", MENU_TYPE_PARAMETER, .param_index = 160U},
    {"Finish Cyc", MENU_TYPE_PARAMETER, .param_index = 161U},
    {"Timeout Cyc", MENU_TYPE_PARAMETER, .param_index = 162U},
    {"Cmd Match", MENU_TYPE_PARAMETER, .param_index = 163U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_fuya_menu[] = {
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 180U},
    {"Target", MENU_TYPE_PARAMETER, .param_index = 181U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode5_menu[] = {
    {"Input", MENU_TYPE_SUBMENU, .submenu = car_mode5_input_menu},
    {"L Speed", MENU_TYPE_SUBMENU, .submenu = car_mode5_left_speed_menu},
    {"R Speed", MENU_TYPE_SUBMENU, .submenu = car_mode5_right_speed_menu},
    {"Speed", MENU_TYPE_SUBMENU, .submenu = car_mode5_speed_menu},
    {"Yaw Rate", MENU_TYPE_SUBMENU, .submenu = car_mode5_yaw_rate_menu},
    {"Yaw Angle", MENU_TYPE_SUBMENU, .submenu = car_mode5_yaw_angle_menu},
    {"Brake", MENU_TYPE_SUBMENU, .submenu = car_mode5_brake_menu},
    {"Large Turn", MENU_TYPE_SUBMENU, .submenu = car_mode5_large_turn_menu},
    {"Fuya", MENU_TYPE_SUBMENU, .submenu = car_mode5_fuya_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode8_left_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 166U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 167U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 168U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode8_right_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 169U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 170U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 171U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode8_speed_menu[] = {
    {"Filter", MENU_TYPE_PARAMETER, .param_index = 172U},
    {"FF Static", MENU_TYPE_PARAMETER, .param_index = 173U},
    {"I Limit", MENU_TYPE_PARAMETER, .param_index = 174U},
    {"FF Dead", MENU_TYPE_PARAMETER, .param_index = 175U},
    {"FF Trans", MENU_TYPE_PARAMETER, .param_index = 176U},
    {"Inc Limit", MENU_TYPE_PARAMETER, .param_index = 177U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_mode8_menu[] = {
    {"L Speed", MENU_TYPE_SUBMENU, .submenu = car_mode8_left_speed_menu},
    {"R Speed", MENU_TYPE_SUBMENU, .submenu = car_mode8_right_speed_menu},
    {"Speed", MENU_TYPE_SUBMENU, .submenu = car_mode8_speed_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_menu[] = {
    {"Mode1", MENU_TYPE_SUBMENU, .submenu = car_mode1_menu},
    {"Mode2", MENU_TYPE_SUBMENU, .submenu = car_mode2_menu},
    {"Mode4", MENU_TYPE_SUBMENU, .submenu = car_mode4_menu},
    {"Mode5", MENU_TYPE_SUBMENU, .submenu = car_mode5_menu},
    {"Mode8", MENU_TYPE_SUBMENU, .submenu = car_mode8_menu},
    {"PWM Test", MENU_TYPE_FUNCTION, .function = pwm_test_function},
    {"C_Diag", MENU_TYPE_SUBMENU, .submenu = car_diag_menu},
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
    {"Mode1 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode2 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode4 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode5 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode7 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode8 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode8 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Core1 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"2BL3 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
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
                                   BL3_PARAM_MENU_COUNT + 3U,
    AIR_MENU_YAW_INDEX = 2U,
    AIR_MENU_CAR_PLAN_INDEX = 3U
};

static menu_item_t s_air_param_menu_storage[AIR_PARAM_MENU_STORAGE_COUNT];
static menu_item_t *s_air_group_menus[AIR_PARAM_MENU_COUNT];
static menu_item_t *s_core1_group_menus[CORE1_PARAM_MENU_COUNT];
static menu_item_t *s_bl3_group_menus[BL3_PARAM_MENU_COUNT];

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

static menu_item_t air_menu[] = {
    {"A_Diag", MENU_TYPE_SUBMENU, .submenu = air_diag_menu},
    {"A_Params", MENU_TYPE_SUBMENU, .submenu = air_param_menu},
    {"Yaw", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Car Plan", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t main_menu[] = {
    {"Car", MENU_TYPE_SUBMENU, .submenu = car_menu},
    {"Air", MENU_TYPE_SUBMENU, .submenu = air_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static uint8 menu_build_air_param_group(const char *group_name,
                                        uint16 *cursor,
                                        menu_item_t **menu)
{
    uint16 index;
    uint8 group_count = 0U;
    const menu_air_param_config_t *config;
    const char *item_name;
    menu_item_t *item;

    *menu = &s_air_param_menu_storage[*cursor];
    for(index = 0U; index < menu_get_air_param_count(); index++)
    {
        config = menu_get_air_param_config(index);
        if((config == NULL) || (config->menu_name == NULL) ||
           (config->visible == 0U) ||
           (strcmp(config->menu_name, group_name) != 0))
        {
            continue;
        }

        if((group_count >= MENU_MAX_ITEMS) ||
           (*cursor >= AIR_PARAM_MENU_STORAGE_COUNT))
        {
            return 1U;
        }

        item = &s_air_param_menu_storage[(*cursor)++];
        item_name = config->name;
        if((strncmp(item_name, "mode1_", 6U) == 0) ||
           (strncmp(item_name, "mode2_", 6U) == 0) ||
           (strncmp(item_name, "mode4_", 6U) == 0) ||
           (strncmp(item_name, "mode5_", 6U) == 0))
        {
            item_name += 6U;
            if(strcmp(item_name, "car_vel_error_lpf_hz") == 0)
            {
                item_name = "car_vel_lpf_hz";
            }
            else if(strcmp(item_name, "car_turn_accel_lpf_hz") == 0)
            {
                item_name = "turn_accel_lpf_hz";
            }
        }
        strncpy(item->name, item_name, sizeof(item->name) - 1U);
        item->name[sizeof(item->name) - 1U] = '\0';
        item->type = MENU_TYPE_AIR_PARAMETER;
        item->param_index = index;
        group_count++;
    }

    if(*cursor >= AIR_PARAM_MENU_STORAGE_COUNT)
    {
        return 1U;
    }
    item = &s_air_param_menu_storage[(*cursor)++];
    item->name[0] = '\0';
    item->type = MENU_TYPE_SUBMENU;
    item->submenu = NULL;
    return 0U;
}

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

        if(menu_build_air_param_group(air_param_menu[group].name,
                                      &cursor,
                                      &s_air_group_menus[group]) != 0U)
        {
            return 1U;
        }
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
               (strcmp(config->menu_name, s_core1_param_group_names[group]) != 0))
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

    if((menu_build_air_param_group("Yaw", &cursor,
                                   &air_menu[AIR_MENU_YAW_INDEX].submenu) != 0U) ||
       (menu_build_air_param_group("Car Plan", &cursor,
                                   &air_menu[AIR_MENU_CAR_PLAN_INDEX].submenu) != 0U))
    {
        return 1U;
    }

    return 0U;
}

void menu_config_init(void)
{
    menu_register_param(&mode1_speed_left_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode1_speed_left_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode1_speed_left_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode1_speed_right_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode1_speed_right_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode1_speed_right_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode1_speed_filter_alpha, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode1_speed_ff_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode1_gyroz_kp, 0.05f, 0.0f, 10.0f);
    menu_register_param(&mode1_gyroz_ki, 0.002f, 0.0f, 1.0f);
    menu_register_param(&mode1_gyroz_kff, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode1_gyroz_k_turn, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode1_yaw_kp, 0.25f, 0.0f, 20.0f);
    menu_register_param(&mode1_yaw_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode1_plan_speed_limit_mps, 0.1f, 0.0f, 5.0f);
    menu_register_param(&mode1_alignment_stop_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode1_wheel_target_limit, 50.0f, 100.0f, 3000.0f);
    menu_register_param(&mode1_speed_i_limit, 100.0f, 0.0f, 10000.0f);
    menu_register_param(&mode1_speed_ff_deadband, 1.0f, 0.0f, 200.0f);
    menu_register_param(&mode1_speed_ff_transition, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode1_speed_increment_limit, 250.0f, 0.0f, 20000.0f);
    menu_register_param(&mode1_speed_brake_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode1_speed_decel_step, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode1_gyroz_output_limit, 25.0f, 0.0f, 2000.0f);
    menu_register_param(&mode1_yaw_rate_limit_dps, 50.0f, 0.0f, 2000.0f);
    menu_register_param(&mode1_gyroz_stop_target_dps, 0.5f, 0.0f, 20.0f);
    menu_register_param(&mode1_wheel_stop_speed, 1.0f, 0.0f, 100.0f);
    menu_register_param(&mode1_large_turn_brake_speed, 5.0f, 0.0f, 500.0f);
    menu_register_param(&mode1_large_turn_brake_target, 5.0f, 0.0f, 500.0f);
    menu_register_param(&mode1_large_turn_brake_ff, 50.0f, 0.0f, 5000.0f);
    menu_register_param(&mode1_large_turn_brake_rate_dps, 25.0f, 0.0f, 1000.0f);
    menu_register_param(&mode1_large_turn_enter_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode1_large_turn_pivot_exit_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode1_large_turn_exit_start_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode1_large_turn_finish_deg, 0.5f, 0.0f, 30.0f);
    menu_register_param(&mode1_large_turn_trigger_cycles, 1.0f, 1.0f, 100.0f);
    menu_register_param(&mode1_large_turn_finish_cycles, 1.0f, 1.0f, 100.0f);
    menu_register_param(&mode1_large_turn_timeout_cycles, 50.0f, 10.0f, 5000.0f);
    menu_register_param(&mode1_exit_command_match_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode1_brake_target_margin, 1.0f, 0.0f, 100.0f);
    menu_register_param(&mode1_brake_ff_fade_span, 5.0f, 1.0f, 500.0f);
    menu_register_param(&mode2_speed_left_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode2_speed_left_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode2_speed_left_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode2_speed_right_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode2_speed_right_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode2_speed_right_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode2_speed_filter_alpha, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode2_speed_ff_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode2_gyroz_kp, 0.05f, 0.0f, 10.0f);
    menu_register_param(&mode2_gyroz_ki, 0.002f, 0.0f, 1.0f);
    menu_register_param(&mode2_gyroz_kff, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode2_gyroz_k_turn, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode2_yaw_kp, 0.25f, 0.0f, 20.0f);
    menu_register_param(&mode2_yaw_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode2_target_speed_mps, 0.1f, 0.0f, 5.0f);
    menu_register_param(&mode2_input_deadzone, 10.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_alignment_stop_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode2_wheel_target_limit, 50.0f, 100.0f, 3000.0f);
    menu_register_param(&mode2_speed_i_limit, 100.0f, 0.0f, 10000.0f);
    menu_register_param(&mode2_speed_ff_deadband, 1.0f, 0.0f, 200.0f);
    menu_register_param(&mode2_speed_ff_transition, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode2_speed_increment_limit, 250.0f, 0.0f, 20000.0f);
    menu_register_param(&mode2_speed_brake_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode2_speed_decel_step, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode2_gyroz_output_limit, 25.0f, 0.0f, 2000.0f);
    menu_register_param(&mode2_yaw_rate_limit_dps, 50.0f, 0.0f, 2000.0f);
    menu_register_param(&mode2_gyroz_stop_target_dps, 0.5f, 0.0f, 20.0f);
    menu_register_param(&mode2_wheel_stop_speed, 1.0f, 0.0f, 100.0f);
    menu_register_param(&mode2_large_turn_brake_speed, 5.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_large_turn_brake_target, 5.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_large_turn_brake_ff, 50.0f, 0.0f, 5000.0f);
    menu_register_param(&mode2_large_turn_brake_rate_dps, 25.0f, 0.0f, 1000.0f);
    menu_register_param(&mode2_large_turn_enter_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode2_large_turn_pivot_exit_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode2_large_turn_exit_start_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode2_large_turn_finish_deg, 0.5f, 0.0f, 30.0f);
    menu_register_param(&mode2_large_turn_trigger_cycles, 1.0f, 1.0f, 100.0f);
    menu_register_param(&mode2_large_turn_finish_cycles, 1.0f, 1.0f, 100.0f);
    menu_register_param(&mode2_large_turn_timeout_cycles, 50.0f, 10.0f, 5000.0f);
    menu_register_param(&mode2_exit_command_match_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode2_brake_target_margin, 1.0f, 0.0f, 100.0f);
    menu_register_param(&mode2_brake_ff_fade_span, 5.0f, 1.0f, 500.0f);
    menu_register_param(&mode4_speed_left_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode4_speed_left_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode4_speed_left_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode4_speed_right_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode4_speed_right_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode4_speed_right_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode4_speed_filter_alpha, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode4_speed_ff_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode4_gyroz_kp, 0.05f, 0.0f, 10.0f);
    menu_register_param(&mode4_gyroz_ki, 0.002f, 0.0f, 1.0f);
    menu_register_param(&mode4_gyroz_kff, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode4_gyroz_k_turn, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode4_yaw_kp, 0.25f, 0.0f, 20.0f);
    menu_register_param(&mode4_yaw_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode4_plan_speed_limit_mps, 0.1f, 0.0f, 5.0f);
    menu_register_param(&mode4_alignment_stop_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode4_wheel_target_limit, 50.0f, 100.0f, 3000.0f);
    menu_register_param(&mode4_speed_i_limit, 100.0f, 0.0f, 10000.0f);
    menu_register_param(&mode4_speed_ff_deadband, 1.0f, 0.0f, 200.0f);
    menu_register_param(&mode4_speed_ff_transition, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode4_speed_increment_limit, 250.0f, 0.0f, 20000.0f);
    menu_register_param(&mode4_speed_brake_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode4_speed_decel_step, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode4_gyroz_output_limit, 25.0f, 0.0f, 2000.0f);
    menu_register_param(&mode4_yaw_rate_limit_dps, 50.0f, 0.0f, 2000.0f);
    menu_register_param(&mode4_gyroz_stop_target_dps, 0.5f, 0.0f, 20.0f);
    menu_register_param(&mode4_wheel_stop_speed, 1.0f, 0.0f, 100.0f);
    menu_register_param(&mode4_large_turn_brake_speed, 5.0f, 0.0f, 500.0f);
    menu_register_param(&mode4_large_turn_brake_target, 5.0f, 0.0f, 500.0f);
    menu_register_param(&mode4_large_turn_brake_ff, 50.0f, 0.0f, 5000.0f);
    menu_register_param(&mode4_large_turn_brake_rate_dps, 25.0f, 0.0f, 1000.0f);
    menu_register_param(&mode4_large_turn_enter_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode4_large_turn_pivot_exit_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode4_large_turn_exit_start_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode4_large_turn_finish_deg, 0.5f, 0.0f, 30.0f);
    menu_register_param(&mode4_large_turn_trigger_cycles, 1.0f, 1.0f, 100.0f);
    menu_register_param(&mode4_large_turn_finish_cycles, 1.0f, 1.0f, 100.0f);
    menu_register_param(&mode4_large_turn_timeout_cycles, 50.0f, 10.0f, 5000.0f);
    menu_register_param(&mode4_exit_command_match_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode4_brake_target_margin, 1.0f, 0.0f, 100.0f);
    menu_register_param(&mode4_brake_ff_fade_span, 5.0f, 1.0f, 500.0f);
    menu_register_param(&mode5_speed_left_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode5_speed_left_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode5_speed_left_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode5_speed_right_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode5_speed_right_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode5_speed_right_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode5_speed_filter_alpha, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode5_speed_ff_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode5_gyroz_kp, 0.05f, 0.0f, 10.0f);
    menu_register_param(&mode5_gyroz_ki, 0.002f, 0.0f, 1.0f);
    menu_register_param(&mode5_gyroz_kff, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode5_gyroz_k_turn, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode5_yaw_kp, 0.25f, 0.0f, 20.0f);
    menu_register_param(&mode5_yaw_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode5_target_speed_mps, 0.1f, 0.0f, 5.0f);
    menu_register_param(&mode5_input_deadzone, 10.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_alignment_stop_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode5_wheel_target_limit, 50.0f, 100.0f, 3000.0f);
    menu_register_param(&mode5_speed_i_limit, 100.0f, 0.0f, 10000.0f);
    menu_register_param(&mode5_speed_ff_deadband, 1.0f, 0.0f, 200.0f);
    menu_register_param(&mode5_speed_ff_transition, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode5_speed_increment_limit, 250.0f, 0.0f, 20000.0f);
    menu_register_param(&mode5_speed_brake_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode5_speed_decel_step, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode5_gyroz_output_limit, 25.0f, 0.0f, 2000.0f);
    menu_register_param(&mode5_yaw_rate_limit_dps, 50.0f, 0.0f, 2000.0f);
    menu_register_param(&mode5_gyroz_stop_target_dps, 0.5f, 0.0f, 20.0f);
    menu_register_param(&mode5_wheel_stop_speed, 1.0f, 0.0f, 100.0f);
    menu_register_param(&mode5_large_turn_brake_speed, 5.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_large_turn_brake_target, 5.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_large_turn_brake_ff, 50.0f, 0.0f, 5000.0f);
    menu_register_param(&mode5_large_turn_brake_rate_dps, 25.0f, 0.0f, 1000.0f);
    menu_register_param(&mode5_large_turn_enter_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode5_large_turn_pivot_exit_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode5_large_turn_exit_start_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode5_large_turn_finish_deg, 0.5f, 0.0f, 30.0f);
    menu_register_param(&mode5_large_turn_trigger_cycles, 1.0f, 1.0f, 100.0f);
    menu_register_param(&mode5_large_turn_finish_cycles, 1.0f, 1.0f, 100.0f);
    menu_register_param(&mode5_large_turn_timeout_cycles, 50.0f, 10.0f, 5000.0f);
    menu_register_param(&mode5_exit_command_match_deg, 5.0f, 0.0f, 180.0f);
    menu_register_param(&mode5_brake_target_margin, 1.0f, 0.0f, 100.0f);
    menu_register_param(&mode5_brake_ff_fade_span, 5.0f, 1.0f, 500.0f);
    menu_register_param(&mode8_speed_left_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode8_speed_left_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode8_speed_left_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode8_speed_right_kp, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode8_speed_right_ki, 0.05f, 0.0f, 5.0f);
    menu_register_param(&mode8_speed_right_kd, 0.05f, 0.0f, 20.0f);
    menu_register_param(&mode8_speed_filter_alpha, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode8_speed_ff_static, 50.0f, 0.0f, 3000.0f);
    menu_register_param(&mode8_speed_i_limit, 100.0f, 0.0f, 10000.0f);
    menu_register_param(&mode8_speed_ff_deadband, 1.0f, 0.0f, 200.0f);
    menu_register_param(&mode8_speed_ff_transition, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode8_speed_increment_limit, 250.0f, 0.0f, 20000.0f);
    menu_register_param(&mode4_fuya_enable, 1.0f, 0.0f, 1.0f);
    menu_register_param(&mode4_fuya_target, 500.0f, 2000.0f, 6000.0f);
    menu_register_param(&mode5_fuya_enable, 1.0f, 0.0f, 1.0f);
    menu_register_param(&mode5_fuya_target, 500.0f, 2000.0f, 6000.0f);

    if(menu_get_param_count() != MENU_CAR_EXPECTED_PARAM_COUNT)
    {
        menu_show_error("Car Menu Error");
        return;
    }

    menu_air_support_init();
    if(menu_build_air_param_menus() != 0U)
    {
        menu_show_error("Air Menu Error");
        return;
    }

    menu_set_root(main_menu);
}

static void diag_show_line(uint8 line, const char *text)
{
    if(line < MENU_MAX_VISIBLE_LINES)
    {
        menu_show_text_line(line, text, UI_COLOR_NORMAL);
    }
}

static void diag_clear_lines(uint8 first, uint8 last)
{
    uint8 line;

    for(line = first; (line <= last) && (line < MENU_MAX_VISIBLE_LINES); line++)
    {
        menu_clear_line(line);
    }
}

static void diag_begin(void)
{
    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_set_font(UI_FONT_NORMAL);
}

static void pwm_test_render(void)
{
    char text[32];
    uint8 selected_right = s_pwm_test_state & 1U;
    uint8 editing = (s_pwm_test_state >= 2U) ? 1U : 0U;

    diag_begin();
    diag_show_line(0U, "PWM Test");
    snprintf(text, sizeof(text), "Left :%6d", (int)s_pwm_test_left);
    menu_show_text_line(1U, text,
                        (selected_right == 0U) ?
                        (editing != 0U ? UI_COLOR_EDITING : UI_COLOR_SELECTED) :
                        UI_COLOR_NORMAL);
    snprintf(text, sizeof(text), "Right:%6d", (int)s_pwm_test_right);
    menu_show_text_line(2U, text,
                        (selected_right != 0U) ?
                        (editing != 0U ? UI_COLOR_EDITING : UI_COLOR_SELECTED) :
                        UI_COLOR_NORMAL);
    diag_show_line(3U, (selected_right != 0U) ?
                   "Selected: Right" : "Selected: Left");
    diag_show_line(4U, (editing != 0U) ?
                   "State: Edit" : "State: Select");
    diag_clear_lines(5U, 7U);
}

static uint8_t pwm_test_on_key(uint8_t key)
{
    int16 *pwm;
    int32 value;

    switch ((menu_key_t)key)
    {
        case KEY_ENTER:
            if (s_pwm_test_state < 2U)
            {
                s_pwm_test_state += 2U;
                pwm_test_render();
            }
            return 1U;

        case KEY_BACK:
            if (s_pwm_test_state >= 2U)
            {
                s_pwm_test_state -= 2U;
                pwm_test_render();
                return 1U;
            }
            return 0U;

        case KEY_UP:
        case KEY_DOWN:
            if (s_pwm_test_state < 2U)
            {
                s_pwm_test_state = (key == (uint8_t)KEY_DOWN) ? 1U : 0U;
            }
            else
            {
                pwm = ((s_pwm_test_state & 1U) != 0U) ?
                      &s_pwm_test_right : &s_pwm_test_left;
                value = (int32)(*pwm) +
                        ((key == (uint8_t)KEY_UP) ? 500 : -500);
                if (value > 5000)
                {
                    value = 5000;
                }
                else if (value < -5000)
                {
                    value = -5000;
                }
                *pwm = (int16)value;
                car_pwm_test_control(1U, s_pwm_test_left, s_pwm_test_right);
            }
            pwm_test_render();
            return 1U;

        default:
            return 0U;
    }
}

static void pwm_test_exit(void)
{
    car_pwm_test_control(0U, 0, 0);
}

static void pwm_test_function(void)
{
    static const menu_external_view_config_t config = {
        .render = pwm_test_render,
        .on_exit = pwm_test_exit,
        .on_key = pwm_test_on_key,
        .refresh_periodic = 1U,
        .long_back_only = 0U,
        .allow_runtime_locked = 0U
    };

    s_pwm_test_left = 0;
    s_pwm_test_right = 0;
    s_pwm_test_state = 0U;
    car_pwm_test_control(1U, 0, 0);
    if (menu_enter_external_view(&config) != 0U)
    {
        car_pwm_test_control(0U, 0, 0);
    }
}

static void diag_imu_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "IMU RPY");
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

static void diag_encoder_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Encoder");
    sprintf(text, "Left:%d", (int)encoder_get_left_count());
    diag_show_line(1U, text);
    sprintf(text, "Right:%d", (int)encoder_get_right_count());
    diag_show_line(2U, text);
    diag_clear_lines(3U, 6U);
    diag_show_line(7U, "Back/Enter Exit");
}

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

static void diag_air_rc_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air RC");
    sprintf(text, "0:%5.0f 1:%5.0f", (double)g_air_std_ch0, (double)g_air_std_ch1);
    diag_show_line(1U, text);
    sprintf(text, "2:%5.0f 3:%5.0f", (double)g_air_std_ch2, (double)g_air_std_ch3);
    diag_show_line(2U, text);
    sprintf(text, "4:%5.0f 5:%5.0f", (double)g_air_std_ch4, (double)g_air_std_ch5);
    diag_show_line(3U, text);
    sprintf(text, "6:%5.0f 7:%5.0f", (double)g_air_std_ch6, (double)g_air_std_ch7);
    diag_show_line(4U, text);
    sprintf(text, "8:%5.0f", (double)g_air_std_ch8);
    diag_show_line(5U, text);
    sprintf(text, "State:%u", (unsigned int)(uint8)g_air_state);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}
