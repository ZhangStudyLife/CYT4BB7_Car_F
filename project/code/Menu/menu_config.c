#include "menu_config.h"

#define MENU_CAR_EXPECTED_PARAM_COUNT (14U)

static void diag_imu_function(void);
static void diag_encoder_function(void);
static void diag_air_state_function(void);
static void diag_air_tof_function(void);
static void diag_air_flow_function(void);
static void diag_air_imu_function(void);
static void diag_air_attitude_function(void);
static void diag_air_rc_function(void);
static void diag_2bl3_status_function(void);

static menu_item_t car_diag_menu[] = {
    {"IMU", MENU_TYPE_DIAG_VIEW, .function = diag_imu_function},
    {"Encoder", MENU_TYPE_DIAG_VIEW, .function = diag_encoder_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_left_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 0U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 1U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 2U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_right_speed_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 3U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 4U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 5U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_speed_common_menu[] = {
    {"Filter", MENU_TYPE_PARAMETER, .param_index = 6U},
    {"FF Static", MENU_TYPE_PARAMETER, .param_index = 7U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_gyroz_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 8U},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 9U},
    {"Kff", MENU_TYPE_PARAMETER, .param_index = 10U},
    {"K Turn", MENU_TYPE_PARAMETER, .param_index = 11U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_yaw_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 12U},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 13U},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_menu[] = {
    {"L_Speed", MENU_TYPE_SUBMENU, .submenu = car_left_speed_menu},
    {"R_Speed", MENU_TYPE_SUBMENU, .submenu = car_right_speed_menu},
    {"Speed Com", MENU_TYPE_SUBMENU, .submenu = car_speed_common_menu},
    {"Yaw Angle", MENU_TYPE_SUBMENU, .submenu = car_yaw_menu},
    {"Yaw Rate", MENU_TYPE_SUBMENU, .submenu = car_gyroz_menu},
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

    return 0U;
}

void menu_config_init(void)
{
    menu_register_param(&car_speed_left_kp, 0.1f, 0.0f, 100.0f);
    menu_register_param(&car_speed_left_ki, 0.1f, 0.0f, 10.0f);
    menu_register_param(&car_speed_left_kd, 0.1f, 0.0f, 100.0f);
    menu_register_param(&car_speed_right_kp, 0.1f, 0.0f, 100.0f);
    menu_register_param(&car_speed_right_ki, 0.1f, 0.0f, 10.0f);
    menu_register_param(&car_speed_right_kd, 0.1f, 0.0f, 100.0f);
    menu_register_param(&car_speed_filter_alpha, 0.01f, 0.0f, 1.0f);
    menu_register_param(&car_speed_ff_static, 10.0f, 0.0f, 3000.0f);
    menu_register_param(&car_gyroz_kp, 0.01f, 0.0f, 10.0f);
    menu_register_param(&car_gyroz_ki, 0.001f, 0.0f, 10.0f);
    menu_register_param(&car_gyroz_kff, 0.01f, 0.0f, 1.0f);
    menu_register_param(&car_gyroz_k_turn, 0.1f, 0.0f, 10.0f);
    menu_register_param(&car_yaw_kp, 0.1f, 0.0f, 20.0f);
    menu_register_param(&car_yaw_kd, 0.1f, 0.0f, 500.0f);

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
