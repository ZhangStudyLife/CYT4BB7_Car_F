#include "beacon_position_recorder.h"
#include "Controller/car_loop.h"

#define BEACON_POSITION_RECORDER_FLASH_MAGIC   (0x42505243UL)
#define BEACON_POSITION_RECORDER_FLASH_VERSION (2U)

#define BEACON_MAP_CELL_PIXELS                 (14)
#define BEACON_MAP_GRID_CELLS                  (8)
#define BEACON_MAP_START_X                     (8)
#define BEACON_MAP_BOTTOM_Y                    (120)
#define BEACON_MAP_TOP_Y                       \
    (BEACON_MAP_BOTTOM_Y - (BEACON_MAP_GRID_CELLS * BEACON_MAP_CELL_PIXELS))
#define BEACON_MAP_END_X                       \
    (BEACON_MAP_START_X + (BEACON_MAP_GRID_CELLS * BEACON_MAP_CELL_PIXELS))

#if (BEACON_POSITION_RECORDER_FLASH_PAGE >= FLASH_PAGE_NUM)
#error "Beacon position recorder Flash page exceeds Work Flash range"
#endif
#if (BEACON_POSITION_RECORDER_FLASH_PAGE == BEACON_CONFIG_FLASH_PAGE)
#error "Beacon recorder and config cannot share the same Flash page"
#endif

typedef struct
{
    uint32 magic;
    uint16 version;
    uint16 point_count;
    uint8 full;
    uint8 reserved[3];
    float position[BEACON_POSITION_RECORDER_AXIS_NUM];
    float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM];
    uint32 checksum;
} beacon_position_recorder_flash_data_t;

typedef char beacon_position_recorder_flash_data_must_be_word_aligned[
    ((sizeof(beacon_position_recorder_flash_data_t) % sizeof(uint32)) == 0U) ? 1 : -1];
typedef char beacon_position_recorder_flash_data_must_fit_page[
    (sizeof(beacon_position_recorder_flash_data_t) <= FLASH_PAGE_SIZE) ? 1 : -1];

beacon_position_recorder_t g_beacon_position_recorder = {0};

static uint8 s_last_ch8_high;
static uint8 s_ch8_synced;
static uint8 s_save_pending;
static uint8 s_menu_built;

static beacon_position_recorder_flash_data_t s_map_data;
static uint8 s_map_data_valid;
static int32 s_map_origin_x;
static int32 s_map_origin_y;
static float s_map_min_x;
static float s_map_max_x;
static float s_map_min_y;
static float s_map_max_y;
static float s_edit_position[BEACON_POSITION_RECORDER_AXIS_NUM];
static float s_edit_points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM];
static menu_external_param_config_t s_edit_param_config[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM];
static beacon_config_data_t s_predata_edit;
static menu_external_param_config_t
    s_predata_param_config[BEACON_CONFIG_BEACON_COUNT + 1U][BEACON_POSITION_RECORDER_AXIS_NUM];
static menu_item_t s_beacon_menu[5U];
static menu_item_t s_edit_menu[BEACON_POSITION_RECORDER_MAX_POINTS + 2U];
static menu_item_t s_point_menu[BEACON_POSITION_RECORDER_MAX_POINTS][4U];
static menu_item_t s_predata_edit_menu[BEACON_CONFIG_BEACON_COUNT + 3U];
static menu_item_t s_predata_point_menu[BEACON_CONFIG_BEACON_COUNT + 1U][3U];

static void beacon_position_recorder_menu_record(void);
static void beacon_position_recorder_menu_map(void);
static void beacon_position_recorder_menu_edit(void);
static void beacon_position_recorder_menu_save(void);
static void beacon_position_recorder_menu_edit_predata(void);
static void beacon_position_recorder_menu_save_predata(void);
static void beacon_position_recorder_render_record(void);
static void beacon_position_recorder_render_map(void);

static void beacon_position_recorder_reset_coordinate_consumers(void)
{
    odometer_reset();
    beacon_detection_reset();
    fixator_reset();
    LightSequence_Reset();
}

static void beacon_position_recorder_setup_xy_config(
    menu_external_param_config_t config[BEACON_POSITION_RECORDER_AXIS_NUM],
    float *value_x,
    float *value_y,
    float min_x,
    float max_x,
    float min_y,
    float max_y)
{
    config[x].variable = value_x;
    config[x].step = 0.1f;
    config[x].min_val = min_x;
    config[x].max_val = max_x;
    config[y].variable = value_y;
    config[y].step = 0.1f;
    config[y].min_val = min_y;
    config[y].max_val = max_y;
}

static uint8 beacon_position_recorder_point_valid(
    const float point[BEACON_POSITION_RECORDER_AXIS_NUM])
{
    if(point == NULL)
    {
        return 0U;
    }

    return ((point[x] != BEACON_POSITION_RECORDER_INVALID_COORD) &&
            (point[y] != BEACON_POSITION_RECORDER_INVALID_COORD)) ? 1U : 0U;
}

static void beacon_position_recorder_fill_invalid(
    float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM])
{
    uint16 index;

    for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
    {
        points[index][x] = BEACON_POSITION_RECORDER_INVALID_COORD;
        points[index][y] = BEACON_POSITION_RECORDER_INVALID_COORD;
    }
}

static uint16 beacon_position_recorder_count_valid(
    const float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM])
{
    uint16 index;
    uint16 count = 0U;

    for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
    {
        if(beacon_position_recorder_point_valid(points[index]) != 0U)
        {
            count++;
        }
    }
    return count;
}

static uint32 beacon_position_recorder_checksum_mix(uint32 checksum, uint32 value)
{
    checksum ^= value;
    checksum = (checksum << 7) | (checksum >> 25);
    checksum += 0x9E3779B9UL;
    return checksum;
}

static uint32 beacon_position_recorder_calc_checksum(
    const beacon_position_recorder_flash_data_t *data)
{
    uint32 checksum = 0x13579BDFUL;
    uint32 value_bits;
    uint16 point;
    uint8 axis;

    if(data == NULL)
    {
        return 0U;
    }

    checksum = beacon_position_recorder_checksum_mix(checksum, data->magic);
    checksum = beacon_position_recorder_checksum_mix(checksum, data->version);
    checksum = beacon_position_recorder_checksum_mix(checksum, data->point_count);
    checksum = beacon_position_recorder_checksum_mix(checksum, data->full);

    for(axis = 0U; axis < BEACON_POSITION_RECORDER_AXIS_NUM; axis++)
    {
        memcpy(&value_bits, &data->position[axis], sizeof(value_bits));
        checksum = beacon_position_recorder_checksum_mix(checksum, value_bits);
    }

    for(point = 0U; point < BEACON_POSITION_RECORDER_MAX_POINTS; point++)
    {
        for(axis = 0U; axis < BEACON_POSITION_RECORDER_AXIS_NUM; axis++)
        {
            memcpy(&value_bits, &data->points[point][axis], sizeof(value_bits));
            checksum = beacon_position_recorder_checksum_mix(checksum, value_bits);
        }
    }

    return checksum;
}

static uint8 beacon_position_recorder_flash_data_valid(
    const beacon_position_recorder_flash_data_t *data)
{
    if((data == NULL) ||
       (data->magic != BEACON_POSITION_RECORDER_FLASH_MAGIC) ||
       (data->version != BEACON_POSITION_RECORDER_FLASH_VERSION) ||
       (data->point_count > BEACON_POSITION_RECORDER_MAX_POINTS) ||
       (data->full > 1U))
    {
        return 0U;
    }

    return (data->checksum == beacon_position_recorder_calc_checksum(data)) ? 1U : 0U;
}

static uint8 beacon_position_recorder_read_flash(
    beacon_position_recorder_flash_data_t *data)
{
    uint32 words;

    if((data == NULL) ||
       (flash_check(0U, BEACON_POSITION_RECORDER_FLASH_PAGE) == 0U))
    {
        return 0U;
    }

    words = (uint32)(sizeof(*data) / sizeof(uint32));
    memset(data, 0, sizeof(*data));
    flash_read_page(0U,
                    BEACON_POSITION_RECORDER_FLASH_PAGE,
                    (uint32 *)data,
                    words);
    return beacon_position_recorder_flash_data_valid(data);
}

static uint8 beacon_position_recorder_write_flash(
    beacon_position_recorder_flash_data_t *data)
{
    beacon_position_recorder_flash_data_t verify;
    uint32 words;

    if(data == NULL)
    {
        return 0U;
    }

    data->magic = BEACON_POSITION_RECORDER_FLASH_MAGIC;
    data->version = BEACON_POSITION_RECORDER_FLASH_VERSION;
    data->checksum = beacon_position_recorder_calc_checksum(data);
    words = (uint32)(sizeof(*data) / sizeof(uint32));

    flash_write_page(0U,
                     BEACON_POSITION_RECORDER_FLASH_PAGE,
                     (const uint32 *)data,
                     words);

    memset(&verify, 0, sizeof(verify));
    flash_read_page(0U,
                    BEACON_POSITION_RECORDER_FLASH_PAGE,
                    (uint32 *)&verify,
                    words);
    return beacon_position_recorder_flash_data_valid(&verify);
}

static void beacon_position_recorder_build_flash_data(
    beacon_position_recorder_flash_data_t *data,
    const float position[BEACON_POSITION_RECORDER_AXIS_NUM],
    const float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM])
{
    uint16 index;

    memset(data, 0, sizeof(*data));
    memcpy(data->position, position, sizeof(data->position));
    beacon_position_recorder_fill_invalid(data->points);

    for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
    {
        if(beacon_position_recorder_point_valid(points[index]) != 0U)
        {
            data->points[index][x] = points[index][x];
            data->points[index][y] = points[index][y];
        }
    }

    data->point_count = beacon_position_recorder_count_valid(data->points);
    data->full = (data->point_count >= BEACON_POSITION_RECORDER_MAX_POINTS) ? 1U : 0U;
}

static void beacon_position_recorder_apply_flash_data(
    const beacon_position_recorder_flash_data_t *data)
{
    if(data == NULL)
    {
        return;
    }

    memcpy(g_beacon_position_recorder.position,
           data->position,
           sizeof(g_beacon_position_recorder.position));
    memcpy(g_beacon_position_recorder.points,
           data->points,
           sizeof(g_beacon_position_recorder.points));
    g_beacon_position_recorder.point_count = data->point_count;
    g_beacon_position_recorder.full = data->full;
    g_beacon_position_recorder.active = 0U;
}

static uint8 beacon_position_recorder_save_current(void)
{
    beacon_position_recorder_flash_data_t data;

    beacon_position_recorder_build_flash_data(&data,
                                               g_beacon_position_recorder.position,
                                               g_beacon_position_recorder.points);
    if(beacon_position_recorder_write_flash(&data) == 0U)
    {
        return 0U;
    }

    beacon_position_recorder_apply_flash_data(&data);
    return 1U;
}

static void beacon_position_recorder_try_save(void)
{
    if((s_save_pending == 0U) || (car_menu_is_runtime_locked() != 0U))
    {
        return;
    }

    (void)beacon_position_recorder_save_current();
    s_save_pending = 0U;
}

static uint8 beacon_position_recorder_ch8_high(void)
{
    return (g_air_crsf_std_ch8 > 0.5f) ? 1U : 0U;
}

static void beacon_position_recorder_record(void)
{
    uint16 index = g_beacon_position_recorder.point_count;

    if(index >= BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        g_beacon_position_recorder.full = 1U;
        return;
    }

    g_beacon_position_recorder.points[index][x] = g_beacon_position_recorder.position[x];
    g_beacon_position_recorder.points[index][y] = g_beacon_position_recorder.position[y];
    g_beacon_position_recorder.point_count++;
    if(g_beacon_position_recorder.point_count >= BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        g_beacon_position_recorder.full = 1U;
    }
}

static void beacon_position_recorder_invalidate_edit_point(uint8 index)
{
    if(index >= BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        return;
    }

    s_edit_points[index][x] = BEACON_POSITION_RECORDER_INVALID_COORD;
    s_edit_points[index][y] = BEACON_POSITION_RECORDER_INVALID_COORD;
}

#define BEACON_DEFINE_INVALID_FUNCTION(number, index) \
    static void beacon_position_recorder_invalid_##number(void) \
    { \
        beacon_position_recorder_invalidate_edit_point(index); \
    }

BEACON_DEFINE_INVALID_FUNCTION(1, 0U)
BEACON_DEFINE_INVALID_FUNCTION(2, 1U)
BEACON_DEFINE_INVALID_FUNCTION(3, 2U)
BEACON_DEFINE_INVALID_FUNCTION(4, 3U)
BEACON_DEFINE_INVALID_FUNCTION(5, 4U)
BEACON_DEFINE_INVALID_FUNCTION(6, 5U)
BEACON_DEFINE_INVALID_FUNCTION(7, 6U)
BEACON_DEFINE_INVALID_FUNCTION(8, 7U)
BEACON_DEFINE_INVALID_FUNCTION(9, 8U)
BEACON_DEFINE_INVALID_FUNCTION(10, 9U)

static void (*const s_invalid_functions[BEACON_POSITION_RECORDER_MAX_POINTS])(void) =
{
    beacon_position_recorder_invalid_1,
    beacon_position_recorder_invalid_2,
    beacon_position_recorder_invalid_3,
    beacon_position_recorder_invalid_4,
    beacon_position_recorder_invalid_5,
    beacon_position_recorder_invalid_6,
    beacon_position_recorder_invalid_7,
    beacon_position_recorder_invalid_8,
    beacon_position_recorder_invalid_9,
    beacon_position_recorder_invalid_10
};

static void beacon_position_recorder_build_menu(void)
{
    uint8 index;
    uint8 initial_index = BEACON_CONFIG_BEACON_COUNT;
    uint8 predata_save_index = BEACON_CONFIG_BEACON_COUNT + 1U;

    if(s_menu_built != 0U)
    {
        return;
    }

    memset(s_beacon_menu, 0, sizeof(s_beacon_menu));
    memset(s_edit_menu, 0, sizeof(s_edit_menu));
    memset(s_point_menu, 0, sizeof(s_point_menu));
    memset(s_edit_param_config, 0, sizeof(s_edit_param_config));
    memset(s_predata_edit_menu, 0, sizeof(s_predata_edit_menu));
    memset(s_predata_point_menu, 0, sizeof(s_predata_point_menu));
    memset(s_predata_param_config, 0, sizeof(s_predata_param_config));

    strcpy(s_beacon_menu[0].name, "C_BeaconRec");
    s_beacon_menu[0].type = MENU_TYPE_FUNCTION;
    s_beacon_menu[0].function = beacon_position_recorder_menu_record;
    strcpy(s_beacon_menu[1].name, "Map");
    s_beacon_menu[1].type = MENU_TYPE_FUNCTION;
    s_beacon_menu[1].function = beacon_position_recorder_menu_map;
    strcpy(s_beacon_menu[2].name, "Edit Recdata");
    s_beacon_menu[2].type = MENU_TYPE_FUNCTION;
    s_beacon_menu[2].function = beacon_position_recorder_menu_edit;
    strcpy(s_beacon_menu[3].name, "Edit Predata");
    s_beacon_menu[3].type = MENU_TYPE_FUNCTION;
    s_beacon_menu[3].function = beacon_position_recorder_menu_edit_predata;

    for(index = 0U; index < BEACON_POSITION_RECORDER_MAX_POINTS; index++)
    {
        snprintf(s_edit_menu[index].name,
                 sizeof(s_edit_menu[index].name),
                 "Beacon%u",
                 (unsigned int)(index + 1U));
        s_edit_menu[index].type = MENU_TYPE_SUBMENU;
        s_edit_menu[index].submenu = s_point_menu[index];

        beacon_position_recorder_setup_xy_config(
            s_edit_param_config[index],
            &s_edit_points[index][x],
            &s_edit_points[index][y],
            -4.0f,
            4.0f,
            0.0f,
            8.0f);

        strcpy(s_point_menu[index][0].name, "X");
        s_point_menu[index][0].type = MENU_TYPE_EXTERNAL_PARAMETER;
        s_point_menu[index][0].external_param = &s_edit_param_config[index][x];
        strcpy(s_point_menu[index][1].name, "Y");
        s_point_menu[index][1].type = MENU_TYPE_EXTERNAL_PARAMETER;
        s_point_menu[index][1].external_param = &s_edit_param_config[index][y];
        strcpy(s_point_menu[index][2].name, "Set Invalid");
        s_point_menu[index][2].type = MENU_TYPE_FUNCTION;
        s_point_menu[index][2].function = s_invalid_functions[index];
    }

    strcpy(s_edit_menu[BEACON_POSITION_RECORDER_MAX_POINTS].name, "Save");
    s_edit_menu[BEACON_POSITION_RECORDER_MAX_POINTS].type = MENU_TYPE_FUNCTION;
    s_edit_menu[BEACON_POSITION_RECORDER_MAX_POINTS].function = beacon_position_recorder_menu_save;

    for(index = 0U; index < BEACON_CONFIG_BEACON_COUNT; index++)
    {
        snprintf(s_predata_edit_menu[index].name,
                 sizeof(s_predata_edit_menu[index].name),
                 "Beacon%u",
                 (unsigned int)(index + 1U));
        s_predata_edit_menu[index].type = MENU_TYPE_SUBMENU;
        s_predata_edit_menu[index].submenu = s_predata_point_menu[index];

        beacon_position_recorder_setup_xy_config(
            s_predata_param_config[index],
            &s_predata_edit.beacons[index].x,
            &s_predata_edit.beacons[index].y,
            -1.0f,
            7.0f,
            -1.0f,
            7.0f);
        strcpy(s_predata_point_menu[index][0].name, "X");
        s_predata_point_menu[index][0].type = MENU_TYPE_EXTERNAL_PARAMETER;
        s_predata_point_menu[index][0].external_param = &s_predata_param_config[index][x];
        strcpy(s_predata_point_menu[index][1].name, "Y");
        s_predata_point_menu[index][1].type = MENU_TYPE_EXTERNAL_PARAMETER;
        s_predata_point_menu[index][1].external_param = &s_predata_param_config[index][y];
    }

    strcpy(s_predata_edit_menu[initial_index].name, "Start Pos");
    s_predata_edit_menu[initial_index].type = MENU_TYPE_SUBMENU;
    s_predata_edit_menu[initial_index].submenu = s_predata_point_menu[initial_index];
    beacon_position_recorder_setup_xy_config(
        s_predata_param_config[initial_index],
        &s_predata_edit.initial_position[x],
        &s_predata_edit.initial_position[y],
        -1.0f,
        7.0f,
        -1.0f,
        7.0f);
    strcpy(s_predata_point_menu[initial_index][0].name, "X");
    s_predata_point_menu[initial_index][0].type = MENU_TYPE_EXTERNAL_PARAMETER;
    s_predata_point_menu[initial_index][0].external_param =
        &s_predata_param_config[initial_index][x];
    strcpy(s_predata_point_menu[initial_index][1].name, "Y");
    s_predata_point_menu[initial_index][1].type = MENU_TYPE_EXTERNAL_PARAMETER;
    s_predata_point_menu[initial_index][1].external_param =
        &s_predata_param_config[initial_index][y];

    strcpy(s_predata_edit_menu[predata_save_index].name, "Save");
    s_predata_edit_menu[predata_save_index].type = MENU_TYPE_FUNCTION;
    s_predata_edit_menu[predata_save_index].function =
        beacon_position_recorder_menu_save_predata;

    s_menu_built = 1U;
}

static int32 beacon_position_recorder_round_to_pixel(float value)
{
    float scaled = value * (float)BEACON_MAP_CELL_PIXELS;

    return (scaled >= 0.0f) ? (int32)(scaled + 0.5f) : (int32)(scaled - 0.5f);
}

static void beacon_position_recorder_configure_map(void)
{
    s_map_origin_x = BEACON_MAP_START_X + BEACON_MAP_CELL_PIXELS;
    s_map_origin_y = BEACON_MAP_BOTTOM_Y - BEACON_MAP_CELL_PIXELS;
    s_map_min_x = -1.0f;
    s_map_max_x = 7.0f;
    s_map_min_y = -1.0f;
    s_map_max_y = 7.0f;
}

static uint8 beacon_position_recorder_map_point_visible(float point_x, float point_y)
{
    return ((point_x >= s_map_min_x) &&
            (point_x <= s_map_max_x) &&
            (point_y >= s_map_min_y) &&
            (point_y <= s_map_max_y)) ? 1U : 0U;
}

static void beacon_position_recorder_draw_marker(uint16 px, uint16 py, uint16 color)
{
    ips114_draw_line((uint16)(px - 2U), py, (uint16)(px + 2U), py, color);
    ips114_draw_line(px, (uint16)(py - 2U), px, (uint16)(py + 2U), color);
}

static void beacon_position_recorder_render_record(void)
{
    char text[32];
    float last_point[BEACON_POSITION_RECORDER_AXIS_NUM];
    uint16 valid_count = beacon_position_recorder_get_count();

    /* 行驶时禁止屏幕刷新，避免 SPI 显示操作占用控制周期。 */
    if((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
    {
        return;
    }

    ips114_set_font(UI_FONT_NORMAL);

    menu_show_text_line(0U, "Beacon Recorder", UI_COLOR_NORMAL);
    sprintf(text, "Active:%u Full:%u",
            (unsigned int)g_beacon_position_recorder.active,
            (unsigned int)g_beacon_position_recorder.full);
    menu_show_text_line(1U, text, UI_COLOR_NORMAL);
    sprintf(text, "Count:%u/%u",
            (unsigned int)valid_count,
            (unsigned int)BEACON_POSITION_RECORDER_MAX_POINTS);
    menu_show_text_line(2U, text, UI_COLOR_NORMAL);
    sprintf(text, "Pos X:%8.3f", (double)g_beacon_position_recorder.position[x]);
    menu_show_text_line(3U, text, UI_COLOR_NORMAL);
    sprintf(text, "Pos Y:%8.3f", (double)g_beacon_position_recorder.position[y]);
    menu_show_text_line(4U, text, UI_COLOR_NORMAL);

    if((valid_count > 0U) &&
       (beacon_position_recorder_get_point(valid_count - 1U, last_point) != 0U))
    {
        sprintf(text, "Last X:%7.3f",
                (double)last_point[x]);
        menu_show_text_line(5U, text, UI_COLOR_NORMAL);
        sprintf(text, "Last Y:%7.3f",
                (double)last_point[y]);
        menu_show_text_line(6U, text, UI_COLOR_NORMAL);
    }
    else
    {
        menu_show_text_line(5U, "Last X: --", UI_COLOR_NORMAL);
        menu_show_text_line(6U, "Last Y: --", UI_COLOR_NORMAL);
    }

    menu_show_text_line(7U, "Hold Back Exit", UI_COLOR_EDITING);
}

static void beacon_position_recorder_render_map(void)
{
    uint8 line;
    uint16 index;
    int32 px;
    int32 py;
    char text[16];

    ips114_clear();
    ips114_set_font(IPS114_6X8_FONT);

    for(line = 0U; line <= BEACON_MAP_GRID_CELLS; line++)
    {
        uint16 grid_x = (uint16)(BEACON_MAP_START_X + (line * BEACON_MAP_CELL_PIXELS));
        ips114_draw_line(grid_x, BEACON_MAP_TOP_Y, grid_x, BEACON_MAP_BOTTOM_Y, RGB565_WHITE);
    }
    for(line = 0U; line <= BEACON_MAP_GRID_CELLS; line++)
    {
        uint16 grid_y = (uint16)(BEACON_MAP_BOTTOM_Y - (line * BEACON_MAP_CELL_PIXELS));
        ips114_draw_line(BEACON_MAP_START_X, grid_y, BEACON_MAP_END_X, grid_y, RGB565_WHITE);
    }

    ips114_draw_line((uint16)s_map_origin_x,
                     BEACON_MAP_TOP_Y,
                     (uint16)s_map_origin_x,
                     BEACON_MAP_BOTTOM_Y,
                     RGB565_GREEN);
    ips114_draw_line(BEACON_MAP_START_X,
                     (uint16)s_map_origin_y,
                     BEACON_MAP_END_X,
                     (uint16)s_map_origin_y,
                     RGB565_GREEN);

    if(s_map_data_valid != 0U)
    {
        for(index = 0U; index < s_map_data.point_count; index++)
        {
            if(beacon_position_recorder_map_point_visible(
                   s_map_data.points[index][x],
                   s_map_data.points[index][y]) == 0U)
            {
                continue;
            }

            px = s_map_origin_x +
                 beacon_position_recorder_round_to_pixel(s_map_data.points[index][x]);
            py = s_map_origin_y -
                 beacon_position_recorder_round_to_pixel(s_map_data.points[index][y]);
            beacon_position_recorder_draw_marker((uint16)px, (uint16)py, RGB565_RED);
            sprintf(text, "%u", (unsigned int)(index + 1U));
            ips114_set_color(RGB565_YELLOW, UI_COLOR_BG);
            ips114_show_string((uint16)(px + 3), (uint16)(py > 7 ? py - 7 : py + 3), text);
        }
    }

    if(beacon_position_recorder_map_point_visible(
           s_map_data.position[x],
           s_map_data.position[y]) != 0U)
    {
        px = s_map_origin_x +
             beacon_position_recorder_round_to_pixel(s_map_data.position[x]);
        py = s_map_origin_y -
             beacon_position_recorder_round_to_pixel(s_map_data.position[y]);
        beacon_position_recorder_draw_marker((uint16)px, (uint16)py, RGB565_CYAN);
        ips114_set_color(RGB565_CYAN, UI_COLOR_BG);
        ips114_show_string((uint16)(px + 3), (uint16)(py > 7 ? py - 7 : py + 3), "S");
    }

    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_show_string(132, 8, "Beacon Map");
    ips114_show_string(132, 24, "Config");
    sprintf(text,
            "Count:%u",
            (unsigned int)((s_map_data_valid != 0U) ? s_map_data.point_count : 0U));
    ips114_show_string(132, 40, text);
    if(s_map_data_valid == 0U)
    {
        ips114_set_color(UI_COLOR_ERROR, UI_COLOR_BG);
        ips114_show_string(132, 56, "No Data");
    }
    ips114_set_color(UI_COLOR_EDITING, UI_COLOR_BG);
    ips114_show_string(132, 112, "Back Exit");
}

static const menu_external_view_config_t s_record_view_config =
{
    beacon_position_recorder_render_record,
    beacon_position_recorder_exit,
    1U,
    1U,
    0U
};

static const menu_external_view_config_t s_map_view_config =
{
    beacon_position_recorder_render_map,
    NULL,
    0U,
    0U,
    0U
};

static void beacon_position_recorder_menu_record(void)
{
    beacon_position_recorder_enter();
    if(menu_enter_external_view(&s_record_view_config) != 0U)
    {
        g_beacon_position_recorder.active = 0U;
    }
}

static void beacon_position_recorder_menu_map(void)
{
    beacon_config_point_t beacon;
    uint16 count;
    uint16 index;

    memset(&s_map_data, 0, sizeof(s_map_data));
    beacon_position_recorder_fill_invalid(s_map_data.points);
    beacon_position_recorder_configure_map();
    beacon_config_get_initial_position(s_map_data.position);
    count = beacon_config_get_count();
    if(count > BEACON_POSITION_RECORDER_MAX_POINTS)
    {
        count = BEACON_POSITION_RECORDER_MAX_POINTS;
    }

    for(index = 0U; index < count; index++)
    {
        if(beacon_config_get_beacon(index, &beacon) == 0U)
        {
            break;
        }
        s_map_data.points[index][x] = beacon.x;
        s_map_data.points[index][y] = beacon.y;
    }
    s_map_data.point_count = index;
    s_map_data_valid = ((index == count) && (count > 0U)) ? 1U : 0U;
    (void)menu_enter_external_view(&s_map_view_config);
}

static void beacon_position_recorder_menu_edit(void)
{
    beacon_position_recorder_flash_data_t data;

    memset(s_edit_position, 0, sizeof(s_edit_position));
    beacon_position_recorder_fill_invalid(s_edit_points);
    if(beacon_position_recorder_read_flash(&data) != 0U)
    {
        memcpy(s_edit_position, data.position, sizeof(s_edit_position));
        memcpy(s_edit_points, data.points, sizeof(s_edit_points));
    }
    menu_enter_submenu(s_edit_menu);
}

static void beacon_position_recorder_menu_save(void)
{
    beacon_position_recorder_flash_data_t data;

    beacon_position_recorder_build_flash_data(&data, s_edit_position, s_edit_points);
    if(beacon_position_recorder_write_flash(&data) == 0U)
    {
        menu_show_error("Beacon Save Fail");
        return;
    }

    memcpy(s_edit_points, data.points, sizeof(s_edit_points));
    beacon_position_recorder_apply_flash_data(&data);
    menu_show_success("Beacon Save OK");
}

static void beacon_position_recorder_menu_edit_predata(void)
{
    beacon_config_get_predata(&s_predata_edit);
    menu_enter_submenu(s_predata_edit_menu);
}

static void beacon_position_recorder_menu_save_predata(void)
{
    if(beacon_config_save_predata(&s_predata_edit) == 0U)
    {
        menu_show_error("Predata Save Fail");
        return;
    }

    beacon_position_recorder_reset_coordinate_consumers();
    menu_show_success("Predata Save OK");
}

void beacon_position_recorder_init(void)
{
    beacon_position_recorder_flash_data_t data;

    memset(&g_beacon_position_recorder, 0, sizeof(g_beacon_position_recorder));
    beacon_position_recorder_fill_invalid(g_beacon_position_recorder.points);
    s_last_ch8_high = 0U;
    s_ch8_synced = 0U;
    s_save_pending = 0U;
    beacon_position_recorder_build_menu();

    if(beacon_position_recorder_read_flash(&data) != 0U)
    {
        beacon_position_recorder_apply_flash_data(&data);
    }
}

void beacon_position_recorder_enter(void)
{
    memset(&g_beacon_position_recorder, 0, sizeof(g_beacon_position_recorder));
    beacon_position_recorder_fill_invalid(g_beacon_position_recorder.points);
    g_beacon_position_recorder.active = 1U;
    car_control_enabled = 0U;
    car_emergency_stop_active = 1U;
    car_forward_target = 0.0f;
    car_yaw_rate_target = 0.0f;

    if(air_comm_car_is_run_data_fresh() != 0U)
    {
        s_last_ch8_high = beacon_position_recorder_ch8_high();
        s_ch8_synced = 1U;
    }
    else
    {
        s_last_ch8_high = 0U;
        s_ch8_synced = 0U;
    }
}

void beacon_position_recorder_exit(void)
{
    uint8 was_active = g_beacon_position_recorder.active;

    g_beacon_position_recorder.active = 0U;
    s_last_ch8_high = 0U;
    s_ch8_synced = 0U;

    if(was_active != 0U)
    {
        s_save_pending = 1U;
        beacon_position_recorder_try_save();
    }
}

void beacon_position_recorder_update_100HZ(void)
{
    uint8 ch8_high;

    if(g_beacon_position_recorder.active == 0U)
    {
        beacon_position_recorder_try_save();
        return;
    }

    g_beacon_position_recorder.position[x] +=
        g_odometer.vel[x] * ODOMETER_UPDATE_DT_S;
    g_beacon_position_recorder.position[y] +=
        g_odometer.vel[y] * ODOMETER_UPDATE_DT_S;

    if(air_comm_car_is_run_data_fresh() == 0U)
    {
        s_ch8_synced = 0U;
        return;
    }

    ch8_high = beacon_position_recorder_ch8_high();
    if(s_ch8_synced == 0U)
    {
        s_last_ch8_high = ch8_high;
        s_ch8_synced = 1U;
        return;
    }

    if((ch8_high != 0U) && (s_last_ch8_high == 0U))
    {
        beacon_position_recorder_record();
    }

    s_last_ch8_high = ch8_high;
}

uint8 beacon_position_recorder_is_active(void)
{
    return g_beacon_position_recorder.active;
}

uint16 beacon_position_recorder_get_count(void)
{
    return beacon_position_recorder_count_valid(g_beacon_position_recorder.points);
}

uint8 beacon_position_recorder_get_point(uint16 index, float point[2])
{
    uint16 slot;
    uint16 valid_index = 0U;

    if((point == NULL) || (index >= beacon_position_recorder_get_count()))
    {
        return 0U;
    }

    for(slot = 0U; slot < BEACON_POSITION_RECORDER_MAX_POINTS; slot++)
    {
        if(beacon_position_recorder_point_valid(
               g_beacon_position_recorder.points[slot]) == 0U)
        {
            continue;
        }
        if(valid_index == index)
        {
            point[x] = g_beacon_position_recorder.points[slot][x];
            point[y] = g_beacon_position_recorder.points[slot][y];
            return 1U;
        }
        valid_index++;
    }

    return 0U;
}

struct menu_item *beacon_position_recorder_get_menu(void)
{
    beacon_position_recorder_build_menu();
    return s_beacon_menu;
}
