/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
/*********************************************************************************************************************
* 菜单核心框架实现文件
* 功能：菜单引擎核心实现，包括菜单导航、按键处理、屏幕渲染、Flash存档管理、参数编辑系统
* 特点：框架稳定，通常不需要修改
*
* 适配：逐飞科技 IPS114 + CYT4BB7 Flash + 统一参数集合存档模式
* 版本：v2.1 (CYT4BB7移植版本)
* 日期：2025年
********************************************************************************************************************/
#include "menu_core.h"
#include "../car_safety.h"
#include <float.h>

//====================================================内部变量====================================================
// 菜单状态变量
static menu_item_t* current_menu = NULL;               // 当前菜单
static menu_item_t* menu_stack[MENU_MAX_DEPTH];        // 菜单堆栈
static uint8_t menu_index_stack[MENU_MAX_DEPTH];       // 返回时恢复的索引
static uint8_t menu_offset_stack[MENU_MAX_DEPTH];      // 返回时恢复的偏移
static uint8_t menu_depth = 0;                         // 当前菜单深度
static uint8_t current_index = 0;                      // 当前选中项索引
static uint8_t current_item_count = 0;                 // 当前菜单项数量
static menu_state_t menu_state = MENU_STATE_NORMAL;    // 菜单状态

// 参数管理变量
static param_config_t param_configs[MENU_MAX_PARAMS];  // 参数配置表
static uint8_t param_count = 0;                        // 已注册参数数量（会在注册时递增）
static uint8_t current_slot = 0;                       // 当前存档号
static uint8_t air_edit_active = 0U;
static uint16_t air_edit_index = 0U;
static float air_edit_value = 0.0f;

// 显示控制变量
static uint8_t need_refresh = 1;                       // 需要刷新标志
static uint8_t display_offset = 0;                     // 显示偏移量（滚动支持）
static uint8_t diag_refresh_divider = 0;               // 动态页面20Hz刷新分频

// 局部刷新优化变量
static refresh_type_t refresh_type = REFRESH_FULL;     // 刷新类型
static uint8_t last_selected_index = 0;                // 上次选中的项索引
#define MENU_DISPLAY_COLUMNS       (30U)
static char display_text_cache[MENU_MAX_VISIBLE_LINES][MENU_DISPLAY_COLUMNS + 1U];
static uint16_t display_color_cache[MENU_MAX_VISIBLE_LINES];
static uint8_t display_line_valid[MENU_MAX_VISIBLE_LINES];

#define MENU_STATUS_SUCCESS_MS       (1000U)
#define MENU_STATUS_ERROR_MS         (2000U)
#define MENU_STATUS_MESSAGE_MS       (2000U)
typedef struct
{
    uint8_t active;
    uint8_t auto_clear;
    uint16_t color;
    uint32_t expire_tick;
    char text[MENU_DISPLAY_COLUMNS + 1U];
} menu_status_message_t;
static menu_status_message_t menu_status_message;
static void menu_show_status_page(const char *msg, uint16_t color);

// 长按加速控制
#define KEY_REPEAT_FIRST_DELAY_TICKS   15   // 第一次重复延迟（10ms单位）
#define KEY_REPEAT_CONT_DELAY_TICKS    8   // 连续重复间隔（10ms单位）
static uint8_t key_hold_ticks[KEY_NUMBER] = {0};        // 按键按下计时
static uint8_t key_repeat_counters[KEY_NUMBER] = {0};   // 长按重复计数器
static uint8_t key_press_consumed[KEY_NUMBER] = {0};    // 是否已经在按下过程中处理
static const gpio_pin_enum menu_key_pins[KEY_NUMBER] = KEY_LIST; // 按键引脚映射
static uint8_t menu_wait_key_release = 0U;
static menu_external_view_config_t external_view_config;

// Flash操作延迟执行
static uint8_t pending_flash_operation = 0;            // 待执行的Flash操作
static uint8_t pending_slot_number = 0;                // 待操作的存档号
#define FLASH_OP_NONE    0
#define FLASH_OP_LOAD    1
#define FLASH_OP_SAVE    2

static uint8_t menu_float_is_finite(float value)
{
    return ((value == value) &&
            (value <= FLT_MAX) &&
            (value >= -FLT_MAX)) ? 1U : 0U;
}

/* 判断菜单项是否为参数类型（本地参数或Air参数） */
static uint8_t menu_is_param_item(const menu_item_t *item)
{
    if(item == NULL)
    {
        return 0U;
    }

    return ((item->type == MENU_TYPE_PARAMETER) ||
            (item->type == MENU_TYPE_AIR_PARAMETER) ||
            (item->type == MENU_TYPE_EXTERNAL_PARAMETER)) ? 1U : 0U;
}

void menu_invalidate_display_cache(void)
{
    memset(display_line_valid, 0, sizeof(display_line_valid));
}

void menu_show_text_line(uint8_t line, const char *text, uint16_t color)
{
    char normalized[MENU_DISPLAY_COLUMNS + 1U];
    char changed[MENU_DISPLAY_COLUMNS + 1U];
    uint8_t column;
    uint8_t start;
    uint8_t length;

    if(line >= MENU_MAX_VISIBLE_LINES)
    {
        return;
    }

    memset(normalized, ' ', MENU_DISPLAY_COLUMNS);
    normalized[MENU_DISPLAY_COLUMNS] = '\0';
    if(text != NULL)
    {
        strncpy(normalized, text, MENU_DISPLAY_COLUMNS);
        normalized[MENU_DISPLAY_COLUMNS] = '\0';
        length = (uint8_t)strlen(normalized);
        while(length < MENU_DISPLAY_COLUMNS)
        {
            normalized[length++] = ' ';
        }
        normalized[MENU_DISPLAY_COLUMNS] = '\0';
    }

    ips114_set_font(UI_FONT_NORMAL);
    ips114_set_color(color, UI_COLOR_BG);

    if((display_line_valid[line] == 0U) ||
       (display_color_cache[line] != color))
    {
        ips114_show_string(0U, (uint16)(line * 16U), normalized);
    }
    else
    {
        column = 0U;
        while(column < MENU_DISPLAY_COLUMNS)
        {
            while((column < MENU_DISPLAY_COLUMNS) &&
                  (display_text_cache[line][column] == normalized[column]))
            {
                column++;
            }
            if(column >= MENU_DISPLAY_COLUMNS)
            {
                break;
            }

            start = column;
            while((column < MENU_DISPLAY_COLUMNS) &&
                  (display_text_cache[line][column] != normalized[column]))
            {
                column++;
            }
            length = (uint8_t)(column - start);
            memcpy(changed, &normalized[start], length);
            changed[length] = '\0';
            ips114_show_string((uint16)(start * 8U),
                               (uint16)(line * 16U),
                               changed);
        }
    }

    memcpy(display_text_cache[line], normalized, sizeof(normalized));
    display_color_cache[line] = color;
    display_line_valid[line] = 1U;
}

static void menu_clear_key_tracking(void)
{
    key_clear_all_state();
    memset(key_hold_ticks, 0, sizeof(key_hold_ticks));
    memset(key_repeat_counters, 0, sizeof(key_repeat_counters));
    memset(key_press_consumed, 0, sizeof(key_press_consumed));
}

static void menu_air_edit_reset(void)
{
    air_edit_active = 0U;
    air_edit_index = 0U;
    air_edit_value = 0.0f;
}

static uint8_t menu_air_edit_begin(uint16_t index)
{
    if((index >= menu_get_air_param_count()) ||
       (menu_air_param_is_available(index) == 0U))
    {
        menu_air_edit_reset();
        return 1U;
    }

    air_edit_index = index;
    air_edit_value = menu_get_air_param_by_index(index);
    air_edit_active = 1U;
    return 0U;
}

static uint8_t menu_air_get_display_value(uint16_t index, float *value)
{
    if((value == NULL) || (index >= menu_get_air_param_count()))
    {
        return 0U;
    }

    if((air_edit_active != 0U) &&
       (air_edit_index == index) &&
       (menu_state == MENU_STATE_EDIT))
    {
        *value = air_edit_value;
        return 1U;
    }

    *value = menu_get_air_param_by_index(index);
    return 1U;
}

static uint8_t menu_air_edit_adjust(const menu_item_t *item, float delta)
{
    const menu_air_param_config_t *config;

    if((item == NULL) || (item->type != MENU_TYPE_AIR_PARAMETER))
    {
        return 1U;
    }

    config = menu_get_air_param_config(item->param_index);
    if(config == NULL)
    {
        return 1U;
    }

    if((air_edit_active == 0U) || (air_edit_index != item->param_index))
    {
        if(menu_air_edit_begin(item->param_index) != 0U)
        {
            return 1U;
        }
    }

    air_edit_value += delta;
    if(air_edit_value < config->min_val)
    {
        air_edit_value = config->min_val;
    }
    if(air_edit_value > config->max_val)
    {
        air_edit_value = config->max_val;
    }
    return 0U;
}

static uint8_t menu_air_edit_commit(const menu_item_t *item)
{
    uint8_t result;

    if((item == NULL) || (item->type != MENU_TYPE_AIR_PARAMETER))
    {
        menu_air_edit_reset();
        return 1U;
    }

    if((air_edit_active == 0U) || (air_edit_index != item->param_index))
    {
        if(menu_air_edit_begin(item->param_index) != 0U)
        {
            return 1U;
        }
    }

    result = menu_air_commit_param_value(item->param_index, air_edit_value);
    menu_air_edit_reset();
    return result;
}

/* 指定Air参数按Back时取消本地编辑，避免未确认值被下发。 */
static uint8_t menu_air_edit_requires_enter(const menu_item_t *item)
{
    const menu_air_param_config_t *config;

    if((item == NULL) || (item->type != MENU_TYPE_AIR_PARAMETER))
    {
        return 0U;
    }

    config = menu_get_air_param_config(item->param_index);
    return ((config != NULL) && (config->enter_confirm_only != 0U)) ? 1U : 0U;
}

/* 获取菜单项对应的参数值（统一处理本地/Air参数） */
static uint8_t menu_get_item_param_value(const menu_item_t *item, float *value)
{
    if((item == NULL) || (value == NULL))
    {
        return 0U;
    }

    if(item->type == MENU_TYPE_PARAMETER)
    {
        if(item->param_index >= param_count)
        {
            return 0U;
        }
        *value = menu_get_param_by_index(item->param_index);
        return 1U;
    }

    if(item->type == MENU_TYPE_AIR_PARAMETER)
    {
        return menu_air_get_display_value(item->param_index, value);
    }

    if((item->type == MENU_TYPE_EXTERNAL_PARAMETER) &&
       (item->external_param != NULL) &&
       (item->external_param->variable != NULL))
    {
        *value = *(item->external_param->variable);
        return 1U;
    }

    return 0U;
}

/* Air或外部枚举参数使用文字显示，普通参数继续使用原有浮点格式。 */
static const char *menu_get_air_enum_label(const menu_item_t *item, float value)
{
    int32_t enum_index;
    float delta;
    const char * const *enum_labels = NULL;
    uint8_t enum_count = 0U;
    const menu_air_param_config_t *config;

    if(item == NULL)
    {
        return NULL;
    }

    if(item->type == MENU_TYPE_AIR_PARAMETER)
    {
        config = menu_get_air_param_config(item->param_index);
        if(config != NULL)
        {
            enum_labels = config->enum_labels;
            enum_count = config->enum_count;
        }
    }
    else if((item->type == MENU_TYPE_EXTERNAL_PARAMETER) &&
            (item->external_param != NULL))
    {
        enum_labels = item->external_param->enum_labels;
        enum_count = item->external_param->enum_count;
    }

    if((enum_labels == NULL) || (enum_count == 0U))
    {
        return NULL;
    }

    enum_index = (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
    if((enum_index < 0) || (enum_index >= (int32_t)enum_count))
    {
        return NULL;
    }

    delta = value - (float)enum_index;
    if(delta < 0.0f)
    {
        delta = -delta;
    }
    if((delta > 0.001f) || (enum_labels[enum_index] == NULL))
    {
        return NULL;
    }

    return enum_labels[enum_index];
}

/* 获取菜单项对应的编辑步进值 */
static uint8_t menu_get_item_param_step(const menu_item_t *item, float *step)
{
    const menu_air_param_config_t *air_config;

    if((item == NULL) || (step == NULL))
    {
        return 0U;
    }

    if(item->type == MENU_TYPE_PARAMETER)
    {
        if(item->param_index >= param_count)
        {
            return 0U;
        }
        *step = param_configs[item->param_index].step;
        return 1U;
    }

    if(item->type == MENU_TYPE_AIR_PARAMETER)
    {
        air_config = menu_get_air_param_config(item->param_index);
        if(air_config == NULL)
        {
            return 0U;
        }
        *step = air_config->step;
        return 1U;
    }

    if((item->type == MENU_TYPE_EXTERNAL_PARAMETER) &&
       (item->external_param != NULL) &&
       (item->external_param->variable != NULL))
    {
        *step = item->external_param->step;
        return 1U;
    }

    return 0U;
}

/* 设置菜单项对应的参数值（自动处理Air参数的dirty标记） */
static uint8_t menu_set_item_param_value(const menu_item_t *item, float value)
{
    if(item == NULL)
    {
        return 1U;
    }

    if(item->type == MENU_TYPE_PARAMETER)
    {
        if(item->param_index >= param_count)
        {
            return 1U;
        }
        menu_set_param_by_index(item->param_index, value);
        return 0U;
    }

    if(item->type == MENU_TYPE_AIR_PARAMETER)
    {
        return menu_set_air_param_by_index(item->param_index, value);
    }

    if((item->type == MENU_TYPE_EXTERNAL_PARAMETER) &&
       (item->external_param != NULL) &&
       (item->external_param->variable != NULL))
    {
        if(value < item->external_param->min_val)
        {
            value = item->external_param->min_val;
        }
        if(value > item->external_param->max_val)
        {
            value = item->external_param->max_val;
        }
        *(item->external_param->variable) = value;
        return 0U;
    }

    return 1U;
}

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint8_t slot_id;
    uint16_t param_count;
    uint32_t data_checksum;
    uint32_t header_checksum;
    char slot_name[16];
} menu_flash_slot_header_t;

/* 获取当前格式存档对应的 Flash 页号。 */
static uint32_t menu_get_slot_page(uint8_t slot);

static uint32_t menu_flash_calc_data_checksum(uint16_t count, uint32_t offset)
{
    uint16_t index;
    uint32_t checksum = 0x2468ACE0UL;

    for(index = 0U; index < count; index++)
    {
        checksum ^= flash_union_buffer[offset + index].uint32_type;
        checksum = (checksum << 7) | (checksum >> 25);
        checksum += (uint32_t)(index + 1U) * 2654435761UL;
    }

    return checksum;
}

static uint32_t menu_flash_calc_header_checksum(const menu_flash_slot_header_t *header)
{
    uint32_t checksum = 0x13579BDFUL;

    if(header == NULL)
    {
        return 0U;
    }

    checksum ^= header->magic;
    checksum = (checksum << 5) | (checksum >> 27);
    checksum ^= ((uint32_t)header->version << 24);
    checksum ^= ((uint32_t)header->slot_id << 16);
    checksum ^= (uint32_t)header->param_count;
    checksum = (checksum << 5) | (checksum >> 27);
    checksum ^= header->data_checksum;

    return checksum;
}

static uint8_t menu_flash_slot_valid(uint8_t slot, menu_flash_slot_header_t *out_header)
{
    uint32_t page_num;
    uint32_t offset;
    menu_flash_slot_header_t *header;

    if((slot >= MENU_SLOT_COUNT) || (param_count == 0U))
    {
        return 0U;
    }

    page_num = menu_get_slot_page(slot);
    if((page_num + MENU_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        return 0U;
    }

    if(flash_check(0U, page_num) == 0U)
    {
        return 0U;
    }

    flash_read_page_to_buffer(0U, page_num, FLASH_PAGE_LENGTH);
    header = (menu_flash_slot_header_t *)flash_union_buffer;
    offset = (uint32_t)((sizeof(menu_flash_slot_header_t) + 3U) / 4U);

    if((header->magic != MENU_MAGIC_NUMBER) ||
       (header->version != MENU_VERSION) ||
       (header->slot_id != slot) ||
       (header->param_count != param_count) ||
       ((offset + header->param_count) > FLASH_PAGE_LENGTH))
    {
        return 0U;
    }

    if(header->data_checksum != menu_flash_calc_data_checksum(header->param_count, offset))
    {
        return 0U;
    }

    if(header->header_checksum != menu_flash_calc_header_checksum(header))
    {
        return 0U;
    }

    if(out_header != NULL)
    {
        *out_header = *header;
    }

    return 1U;
}

static uint32_t menu_get_slot_page(uint8_t slot)
{
    return MENU_SLOT_BASE_PAGE + ((uint32_t)slot * MENU_SLOT_SIZE);
}

//====================================================菜单核心功能====================================================

/**
 * @brief 10ms定时器中断处理函数
 * 注意：此函数需要在用户的中断服务函数中调用
 * 例如：在 pit_ch0_handler() 中调用 menu_timer_handler()
 */
void menu_timer_handler(void)
{
    key_scanner();  // 按键扫描放在定时器中断中
}

void menu_discard_key_events(void)
{
    menu_wait_key_release = 1U;
    menu_clear_key_tracking();
}

void menu_runtime_suspend(void)
{
    menu_air_edit_reset();
    if((menu_state == MENU_STATE_EXTERNAL_VIEW) &&
       (external_view_config.allow_runtime_locked == 0U))
    {
        if(external_view_config.on_exit != NULL)
        {
            external_view_config.on_exit();
        }
        memset(&external_view_config, 0, sizeof(external_view_config));
        menu_state = MENU_STATE_NORMAL;
    }
    else if(menu_state != MENU_STATE_EXTERNAL_VIEW)
    {
        menu_state = MENU_STATE_NORMAL;
    }
    diag_refresh_divider = 0U;
    pending_flash_operation = FLASH_OP_NONE;
    pending_slot_number = 0U;
    memset(&menu_status_message, 0, sizeof(menu_status_message));
    menu_discard_key_events();
}

void menu_runtime_resume(void)
{
    menu_request_refresh(REFRESH_FULL);
}

/**
 * @brief 菜单系统初始化
 */
void menu_init(void)
{
    // 初始化IPS114
    ips114_init();

    // 初始化显示配置 (纯黑背景，白色文字)
    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_set_font(UI_FONT_NORMAL);

    // 初始化Flash存储
    flash_init();

    // 初始化按键
    key_init(10);  // 10ms扫描周期

    // 初始化变量
    menu_depth = 0;
    current_index = 0;
    current_item_count = 0;
    menu_state = MENU_STATE_NORMAL;
    diag_refresh_divider = 0U;
    param_count = 0;
    current_slot = 0;
    display_offset = 0;
    menu_air_edit_reset();
    menu_wait_key_release = 0U;
    memset(&external_view_config, 0, sizeof(external_view_config));
    memset(&menu_status_message, 0, sizeof(menu_status_message));

    // 初始化局部刷新变量
    refresh_type = REFRESH_FULL;
    last_selected_index = 0;
    need_refresh = 1;

    // 初始化Flash操作标志
    pending_flash_operation = FLASH_OP_NONE;
    pending_slot_number = 0;

    // 延时确保系统稳定
    for(volatile uint32_t i = 0; i < 100000; i++);

    // 清除屏幕
    menu_clear_screen();

    // 首次显示菜单
    menu_request_refresh(REFRESH_FULL);
}

/**
 * @brief 菜单任务处理（主循环调用）
 */
void menu_update_100HZ(void)
{
    if((menu_status_message.active != 0U) &&
       (menu_status_message.auto_clear != 0U) &&
       ((int32)(air_comm_car_get_tick() - menu_status_message.expire_tick) >= 0))
    {
        memset(&menu_status_message, 0, sizeof(menu_status_message));
        menu_invalidate_display_cache();
        menu_request_refresh(REFRESH_FULL);
    }

    if(menu_status_message.active == 0U)
    {
        if((car_menu_is_runtime_locked() != 0U) &&
           (menu_external_view_runtime_active() == 0U))
        {
            menu_discard_key_events();
            return;
        }

        menu_process_keys();

        if((car_menu_is_runtime_locked() != 0U) &&
           (menu_external_view_runtime_active() == 0U))
        {
            menu_discard_key_events();
            return;
        }

        if(menu_state == MENU_STATE_EXTERNAL_VIEW)
        {
            if((need_refresh != 0U) && (external_view_config.render != NULL))
            {
                external_view_config.render();
                need_refresh = 0U;
                refresh_type = REFRESH_NONE;
                diag_refresh_divider = 0U;
            }
            else if(external_view_config.refresh_periodic != 0U)
            {
                diag_refresh_divider++;
                if(diag_refresh_divider >= 5U)
                {
                    diag_refresh_divider = 0U;
                    if(external_view_config.render != NULL)
                    {
                        external_view_config.render();
                    }
                }
            }
            return;
        }

        if(menu_state == MENU_STATE_DIAG_VIEW)
        {
            diag_refresh_divider++;
            if(diag_refresh_divider >= 5U)
            {
                diag_refresh_divider = 0U;
                if((current_index < current_item_count) &&
                   (current_menu[current_index].type == MENU_TYPE_DIAG_VIEW) &&
                   (current_menu[current_index].function != NULL))
                {
                    current_menu[current_index].function();
                }
            }
            return;
        }
    }
    else
    {
        menu_discard_key_events();
    }

    // 处理待执行的Flash操作（在主循环中执行，避免在中断中操作Flash）
    if(pending_flash_operation != FLASH_OP_NONE)
    {
        if(car_safety_maintenance_acquire() == 0U)
        {
            menu_show_error("Stop First");
        }
        else
        {
            switch(pending_flash_operation)
            {
            case FLASH_OP_LOAD:
                if(menu_flash_check_slot(pending_slot_number))
                {
                    // 存档有效，加载数据
                    if(menu_flash_load_params(pending_slot_number) != 0U)
                    {
                        menu_show_success("Load OK");
                    }
                    else
                    {
                        menu_show_error("Load Fail");
                    }
                }
                else
                {
                    // 存档无效，显示错误
                    menu_show_error("No Data");
                }
                break;

            case FLASH_OP_SAVE:
                if(menu_flash_save_params(pending_slot_number) != 0U)
                {
                    menu_show_success("Save OK");
                }
                else
                {
                    menu_show_error("Save Fail");
                }
                break;
            }
            car_safety_maintenance_release();
        }
        pending_flash_operation = FLASH_OP_NONE;
        need_refresh = 1;
    }

    if(menu_status_message.active != 0U)
    {
        if(need_refresh != 0U)
        {
            menu_show_status_page(menu_status_message.text, menu_status_message.color);
            need_refresh = 0U;
            refresh_type = REFRESH_NONE;
        }
        return;
    }

    // 仅在需要时刷新显示（使用局部刷新优化）
    if(need_refresh)
    {
        menu_render_current_optimized();
        need_refresh = 0;
    }
}

/**
 * @brief 手动刷新菜单显示（外部调用）
 */
void menu_show(void)
{
    menu_request_refresh(REFRESH_FULL);  // 手动刷新使用全屏刷新
}

//====================================================参数管理====================================================

/**
 * @brief 注册参数
 * @param var 参数变量指针
 * @param step 编辑步进值
 * @param min 最小值
 * @param max 最大值
 */
void menu_register_param(volatile float* var, float step, float min, float max)
{
    if(param_count >= MENU_MAX_PARAMS) return;

    param_configs[param_count].variable = var;
    param_configs[param_count].step = step;
    param_configs[param_count].min_val = min;
    param_configs[param_count].max_val = max;

    param_count++;
}

/**
 * @brief 获取参数数量
 */
uint8_t menu_get_param_count(void)
{
    return param_count;
}

/**
 * @brief 显示系统调试信息
 */
void menu_show_debug_info(void)
{
    char debug_text[32];

    sprintf(debug_text, "Params: %d", param_count);
    menu_show_text_line(0U, debug_text, UI_COLOR_NORMAL);

    sprintf(debug_text, "Slot: %d", current_slot);
    menu_show_text_line(1U, debug_text, UI_COLOR_NORMAL);

    menu_show_text_line(2U, "Flash: Ready", UI_COLOR_NORMAL);

    // 显示前3个参数的实际值
    if(param_count >= 3)
    {
        sprintf(debug_text, "P0: %.2f", *(param_configs[0].variable));
        menu_show_text_line(3U, debug_text, UI_COLOR_NORMAL);

        sprintf(debug_text, "P1: %.2f", *(param_configs[1].variable));
        menu_show_text_line(4U, debug_text, UI_COLOR_NORMAL);

        sprintf(debug_text, "P2: %.2f", *(param_configs[2].variable));
        menu_show_text_line(5U, debug_text, UI_COLOR_NORMAL);
    }
    else
    {
        menu_clear_line(3U);
        menu_clear_line(4U);
        menu_clear_line(5U);
    }

    menu_show_text_line(6U, "Press key to exit", UI_COLOR_NORMAL);
    menu_clear_line(7U);
}

/**
 * @brief 按索引获取参数值
 */
float menu_get_param_by_index(uint8_t index)
{
    if(index >= param_count || param_configs[index].variable == NULL)
        return 0.0f;
    return *(param_configs[index].variable);
}

/**
 * @brief 按索引设置参数值
 */
void menu_set_param_by_index(uint8_t index, float value)
{
    if(index >= param_count || param_configs[index].variable == NULL)
        return;

    if(menu_float_is_finite(value) == 0U)
        return;

    // 限制在范围内
    if(value < param_configs[index].min_val)
        value = param_configs[index].min_val;
    if(value > param_configs[index].max_val)
        value = param_configs[index].max_val;

    *(param_configs[index].variable) = value;
}

//====================================================存档管理====================================================

/**
 * @brief 加载指定存档
 */
void menu_load_slot(uint8_t slot)
{
    if(slot >= MENU_SLOT_COUNT) return;

    // 如果有参数注册，则执行Flash操作
    if(param_count > 0)
    {
        // 设置延迟执行标志
        pending_flash_operation = FLASH_OP_LOAD;
        pending_slot_number = slot;
        current_slot = slot;
    }
    else
    {
        // 参数未注册，直接设置当前存档号
        current_slot = slot;
    }
}

/**
 * @brief 保存到指定存档
 */
void menu_save_slot(uint8_t slot)
{
    if(slot >= MENU_SLOT_COUNT) return;

    // 设置延迟执行标志
    pending_flash_operation = FLASH_OP_SAVE;
    pending_slot_number = slot;
    current_slot = slot;
}

/**
 * @brief 获取当前存档号
 */
uint8_t menu_get_current_slot(void)
{
    return current_slot;
}

//====================================================菜单控制====================================================

/**
 * @brief 设置根菜单
 */
void menu_set_root(menu_item_t* root_menu)
{
    if(root_menu == NULL) return;  // 安全检查

    current_menu = root_menu;
    menu_depth = 0;
    current_index = 0;
    display_offset = 0;

    // 计算菜单项数量
    current_item_count = 0;
    while(current_menu[current_item_count].name[0] != '\0')
    {
        current_item_count++;
        if(current_item_count >= MENU_MAX_ITEMS) break;
    }

    menu_request_refresh(REFRESH_FULL);  // 设置根菜单需要全屏刷新
}

/**
 * @brief 进入子菜单
 */
void menu_enter_submenu(menu_item_t* submenu)
{
    if(menu_depth >= MENU_MAX_DEPTH - 1) return;

    // 保存当前菜单与位置到堆栈
    menu_stack[menu_depth] = current_menu;
    menu_index_stack[menu_depth] = current_index;
    menu_offset_stack[menu_depth] = display_offset;
    menu_depth++;

    // 设置新菜单
    current_menu = submenu;
    current_index = 0;
    display_offset = 0;

    // 计算菜单项数量
    current_item_count = 0;
    while(current_menu[current_item_count].name[0] != '\0')
    {
        current_item_count++;
        if(current_item_count >= MENU_MAX_ITEMS) break;
    }

    menu_request_refresh(REFRESH_FULL);  // 进入子菜单需要全屏刷新
}

/**
 * @brief 返回上级菜单
 */
void menu_return_to_parent(void)
{
    if(menu_depth > 0)
    {
        menu_depth--;
        current_menu = menu_stack[menu_depth];
        current_index = menu_index_stack[menu_depth];
        display_offset = menu_offset_stack[menu_depth];

        // 重新计算菜单项数量
        current_item_count = 0;
        while(current_menu[current_item_count].name[0] != '\0')
        {
            current_item_count++;
            if(current_item_count >= MENU_MAX_ITEMS) break;
        }

        if(current_item_count == 0)
        {
            current_index = 0;
            display_offset = 0;
        }
        else
        {
            if(current_index >= current_item_count)
            {
                current_index = current_item_count - 1;
            }
            if(current_index < display_offset)
            {
                display_offset = current_index;
            }
            if(current_index >= display_offset + 8)
            {
                display_offset = (current_index >= 7) ? (current_index - 7) : 0;
            }
        }
    }
    else
    {
        // 在主菜单时，重置到第一项
        current_index = 0;
        display_offset = 0;
    }

    menu_state = MENU_STATE_NORMAL;
    menu_request_refresh(REFRESH_FULL);  // 返回上级菜单需要全屏刷新
}

/**
 * @brief 重置选择到第一项
 */
void menu_reset_to_first(void)
{
    current_index = 0;
    display_offset = 0;
    menu_state = MENU_STATE_NORMAL;
    menu_request_refresh(REFRESH_FULL);  // 重置到第一项需要全屏刷新
}

uint8_t menu_enter_external_view(const menu_external_view_config_t *config)
{
    if((config == NULL) || (config->render == NULL) ||
       (car_menu_is_runtime_locked() != 0U) ||
       (menu_state != MENU_STATE_NORMAL))
    {
        return 1U;
    }

    external_view_config = *config;
    menu_state = MENU_STATE_EXTERNAL_VIEW;
    diag_refresh_divider = 0U;
    menu_invalidate_display_cache();
    external_view_config.render();
    return 0U;
}

uint8_t menu_external_view_runtime_active(void)
{
    return ((menu_state == MENU_STATE_EXTERNAL_VIEW) &&
            (external_view_config.allow_runtime_locked != 0U)) ? 1U : 0U;
}

static void menu_exit_external_view(uint8_t wait_key_release)
{
    if(external_view_config.on_exit != NULL)
    {
        external_view_config.on_exit();
    }
    memset(&external_view_config, 0, sizeof(external_view_config));
    menu_state = MENU_STATE_NORMAL;
    diag_refresh_divider = 0U;
    if(wait_key_release != 0U)
    {
        menu_wait_key_release = 1U;
        menu_clear_key_tracking();
    }
    menu_request_refresh(REFRESH_FULL);
}

//====================================================按键处理====================================================

/**
 * @brief 处理按键事件（检查按键状态并处理）
 */
void menu_process_keys(void)
{
    uint8_t all_released = 1U;
    uint8_t i;

    if(menu_wait_key_release != 0U)
    {
        for(i = 0U; i < KEY_NUMBER; i++)
        {
            if(gpio_get_level(menu_key_pins[i]) != KEY_RELEASE_LEVEL)
            {
                all_released = 0U;
            }
        }

        menu_clear_key_tracking();
        if(all_released != 0U)
        {
            menu_wait_key_release = 0U;
        }
        return;
    }

    if((menu_state == MENU_STATE_EXTERNAL_VIEW) &&
       (external_view_config.long_back_only != 0U) &&
       (key_get_state(KEY_4) == KEY_LONG_PRESS))
    {
        key_clear_state(KEY_4);
        menu_exit_external_view(1U);
        return;
    }

    // 检测按键按下事件（使用逐飞按键库的状态）
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        if(!key_press_consumed[KEY_1])
        {
            menu_key_handler(KEY_UP);
        }
        key_clear_state(KEY_1);
    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        if(!key_press_consumed[KEY_2])
        {
            menu_key_handler(KEY_DOWN);
        }
        key_clear_state(KEY_2);
    }

    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        menu_key_handler(KEY_ENTER);
        key_clear_state(KEY_3);
    }

    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        menu_key_handler(KEY_BACK);
        key_clear_state(KEY_4);
    }

    // 自定义长按加速处理（基于按键实时电平）
    for(i = 0U; i < 2U; i++)
    {
        if(gpio_get_level(menu_key_pins[i]) != KEY_RELEASE_LEVEL)
        {
            if(key_hold_ticks[i] < 0xFF)
            {
                key_hold_ticks[i]++;
            }

            if(key_hold_ticks[i] == 1)
            {
                menu_key_handler((i == KEY_1) ? KEY_UP : KEY_DOWN);
                key_press_consumed[i] = 1;
                key_repeat_counters[i] = KEY_REPEAT_FIRST_DELAY_TICKS;
            }
            else if(key_repeat_counters[i] > 0)
            {
                key_repeat_counters[i]--;
            }
            else
            {
                menu_key_handler((i == KEY_1) ? KEY_UP : KEY_DOWN);
                key_repeat_counters[i] = KEY_REPEAT_CONT_DELAY_TICKS;
            }
        }
        else
        {
            key_hold_ticks[i] = 0;
            key_repeat_counters[i] = 0;
            key_press_consumed[i] = 0;
        }
    }
}

/**
 * @brief 按键处理
 */
void menu_key_handler(menu_key_t key)
{
    if(current_menu == NULL) return;
    if((car_menu_is_runtime_locked() != 0U) &&
       (menu_external_view_runtime_active() == 0U)) return;

    switch(menu_state)
    {
        case MENU_STATE_NORMAL:
            switch(key)
            {
                case KEY_UP:
                    if(current_index > 0)
                    {
                        current_index--;

                        // 分页逻辑: 如果光标移出当前页,翻到上一页
                        if(current_index < display_offset)
                        {
                            // 整页向上翻
                            if(display_offset >= MENU_MAX_VISIBLE_LINES)
                                display_offset -= MENU_MAX_VISIBLE_LINES;
                            else
                                display_offset = 0;

                            menu_request_refresh(REFRESH_FULL);  // 翻页时全屏刷新
                        }
                        else
                        {
                            menu_request_refresh(REFRESH_SELECTION);  // 页内移动，局部刷新
                        }
                    }
                    else
                    {
                        // 循环到最后一项
                        current_index = current_item_count - 1;

                        // 跳转到最后一页
                        if(current_item_count > MENU_MAX_VISIBLE_LINES)
                        {
                            // 计算最后一页的起始偏移
                            uint8_t last_page_start = ((current_item_count - 1) / MENU_MAX_VISIBLE_LINES) * MENU_MAX_VISIBLE_LINES;
                            display_offset = last_page_start;
                        }
                        else
                        {
                            display_offset = 0;
                        }

                        menu_request_refresh(REFRESH_FULL);  // 跨页跳转，全屏刷新
                    }
                    break;

                case KEY_DOWN:
                    if(current_index < current_item_count - 1)
                    {
                        current_index++;

                        // 分页逻辑: 如果光标移出当前页,翻到下一页
                        if(current_index >= display_offset + MENU_MAX_VISIBLE_LINES)
                        {
                            // 整页向下翻
                            display_offset += MENU_MAX_VISIBLE_LINES;

                            // 防止越界
                            if(display_offset + MENU_MAX_VISIBLE_LINES > current_item_count)
                            {
                                display_offset = ((current_item_count - 1) / MENU_MAX_VISIBLE_LINES) * MENU_MAX_VISIBLE_LINES;
                            }

                            menu_request_refresh(REFRESH_FULL);  // 翻页时全屏刷新
                        }
                        else
                        {
                            menu_request_refresh(REFRESH_SELECTION);  // 页内移动，局部刷新
                        }
                    }
                    else
                    {
                        // 循环到第一项
                        current_index = 0;
                        display_offset = 0;

                        menu_request_refresh(REFRESH_FULL);  // 跨页跳转，全屏刷新
                    }
                    break;

                case KEY_ENTER:
                    if(current_index >= current_item_count) break;  // 安全检查

                    switch(current_menu[current_index].type)
                    {
                        case MENU_TYPE_SUBMENU:
                            if(current_menu[current_index].submenu != NULL)
                                menu_enter_submenu(current_menu[current_index].submenu);
                            break;

                        case MENU_TYPE_FUNCTION:
                            if(current_menu[current_index].function != NULL)
                            {
                                current_menu[current_index].function();
                                menu_request_refresh(REFRESH_FULL);  // 函数执行后全屏刷新
                            }
                            break;

                        case MENU_TYPE_DIAG_VIEW:
                            if(current_menu[current_index].function != NULL)
                            {
                                menu_state = MENU_STATE_DIAG_VIEW;
                                diag_refresh_divider = 0U;
                                current_menu[current_index].function();
                            }
                            break;

                        case MENU_TYPE_AIR_COMMAND:
                            (void)menu_air_command_start(current_menu[current_index].param_index);
                            break;

                        case MENU_TYPE_PARAMETER:
                        case MENU_TYPE_AIR_PARAMETER:
                        case MENU_TYPE_EXTERNAL_PARAMETER:
                            if(menu_is_param_item(&current_menu[current_index]) != 0U)
                            {
                                if((current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER) &&
                                   (menu_can_edit_air_params() == 0U))
                                {
                                    menu_show_error((menu_is_air_connected() == 0U) ?
                                                    "Air Offline" :
                                                    ((car_menu_is_runtime_locked() != 0U) ?
                                                     "Runtime Lock" : "Air Not Ready"));
                                    break;
                                }
                                if((current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER) &&
                                   (menu_air_is_busy() != 0U))
                                {
                                    menu_show_error("Air Busy");
                                    break;
                                }
                                if((current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER) &&
                                   (menu_air_edit_begin(current_menu[current_index].param_index) != 0U))
                                {
                                    menu_show_error("Set Fail");
                                    break;
                                }
                                menu_state = MENU_STATE_EDIT;
                                menu_request_refresh(REFRESH_VALUE);  // 进入编辑模式，只是颜色变化
                            }
                            break;
                    }
                    break;

                case KEY_BACK:
                    if(menu_air_command_is_active() != 0U)
                    {
                        (void)menu_air_command_stop();
                    }
                    else if(menu_depth > 0)
                    {
                        menu_return_to_parent();
                    }
                    else
                    {
                        current_index = 0;
                        display_offset = 0;
                        menu_request_refresh(REFRESH_SELECTION);
                    }
                    break;
            }
            break;

        case MENU_STATE_EDIT:
            if((current_index < current_item_count) &&
               (menu_is_param_item(&current_menu[current_index]) != 0U))
            {
                float current_val = 0.0f;
                float step = 0.0f;

                switch(key)
                {
                    case KEY_UP:
                        if(current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER)
                        {
                            if((menu_get_item_param_step(&current_menu[current_index], &step) != 0U) &&
                               (menu_air_edit_adjust(&current_menu[current_index], step) == 0U))
                            {
                                menu_request_refresh(REFRESH_VALUE);
                            }
                            else
                            {
                                menu_air_edit_reset();
                                menu_state = MENU_STATE_NORMAL;
                                menu_show_error("Set Fail");
                            }
                            break;
                        }

                        if((menu_get_item_param_value(&current_menu[current_index], &current_val) != 0U) &&
                           (menu_get_item_param_step(&current_menu[current_index], &step) != 0U) &&
                           (menu_set_item_param_value(&current_menu[current_index], current_val + step) == 0U))
                        {
                            menu_request_refresh(REFRESH_VALUE);  // 参数值变化，局部刷新
                        }
                        else
                        {
                            menu_state = MENU_STATE_NORMAL;
                            menu_show_error("Set Fail");
                        }
                        break;

                    case KEY_DOWN:
                        if(current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER)
                        {
                            if((menu_get_item_param_step(&current_menu[current_index], &step) != 0U) &&
                               (menu_air_edit_adjust(&current_menu[current_index], -step) == 0U))
                            {
                                menu_request_refresh(REFRESH_VALUE);
                            }
                            else
                            {
                                menu_air_edit_reset();
                                menu_state = MENU_STATE_NORMAL;
                                menu_show_error("Set Fail");
                            }
                            break;
                        }

                        if((menu_get_item_param_value(&current_menu[current_index], &current_val) != 0U) &&
                           (menu_get_item_param_step(&current_menu[current_index], &step) != 0U) &&
                           (menu_set_item_param_value(&current_menu[current_index], current_val - step) == 0U))
                        {
                            menu_request_refresh(REFRESH_VALUE);  // 参数值变化，局部刷新
                        }
                        else
                        {
                            menu_state = MENU_STATE_NORMAL;
                            menu_show_error("Set Fail");
                        }
                        break;

                    case KEY_ENTER:
                        if(current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER)
                        {
                            menu_state = MENU_STATE_NORMAL;
                            (void)menu_air_edit_commit(&current_menu[current_index]);
                            break;
                        }
                        // 确认键：保存参数并退出编辑模式
                        menu_state = MENU_STATE_NORMAL;
                        menu_request_refresh(REFRESH_VALUE);  // 退出编辑模式，只是颜色变化
                        break;

                    case KEY_BACK:
                        if(current_menu[current_index].type == MENU_TYPE_AIR_PARAMETER)
                        {
                            menu_state = MENU_STATE_NORMAL;
                            if(menu_air_edit_requires_enter(&current_menu[current_index]) != 0U)
                            {
                                menu_air_edit_reset();
                                menu_request_refresh(REFRESH_VALUE);
                            }
                            else
                            {
                                (void)menu_air_edit_commit(&current_menu[current_index]);
                            }
                            break;
                        }
                        // 返回键：保存参数并退出编辑模式（与确认键相同）
                        menu_state = MENU_STATE_NORMAL;
                        menu_request_refresh(REFRESH_VALUE);  // 退出编辑模式，只是颜色变化
                        break;
                }
            }
            break;

        case MENU_STATE_DIAG_VIEW:
            if((key == KEY_BACK) || (key == KEY_ENTER))
            {
                menu_state = MENU_STATE_NORMAL;
                diag_refresh_divider = 0U;
                menu_request_refresh(REFRESH_FULL);
            }
            break;

        case MENU_STATE_EXTERNAL_VIEW:
            if((external_view_config.on_key != NULL) &&
               (external_view_config.on_key((uint8_t)key) != 0U))
            {
                break;
            }
            if((key == KEY_BACK) &&
               (external_view_config.long_back_only == 0U))
            {
                menu_exit_external_view(0U);
            }
            break;
    }
}

//====================================================Flash存档实现====================================================

/**
 * @brief 检查存档是否存在（简化版本）
 */
uint8_t menu_flash_check_slot(uint8_t slot)
{
    return menu_flash_slot_valid(slot, NULL);
}

/**
 * @brief 从Flash加载参数，只接受带magic/version/checksum的新格式。
 */
uint8_t menu_flash_load_params(uint8_t slot)
{
    menu_flash_slot_header_t header;
    uint32_t irq_state;
    uint32_t offset;
    uint16_t index;
    float value;

    if(slot >= MENU_SLOT_COUNT || param_count == 0U)
    {
        menu_show_error("Error");
        return 0U;
    }

    if(menu_flash_slot_valid(slot, &header) == 0U)
    {
        menu_show_error("No Data");
        return 0U;
    }

    offset = (uint32_t)((sizeof(menu_flash_slot_header_t) + 3U) / 4U);

    for(index = 0U; index < header.param_count; index++)
    {
        if(param_configs[index].variable != NULL)
        {
            value = flash_union_buffer[offset + index].float_type;
            if(menu_float_is_finite(value) == 0U)
            {
                return 0U;
            }
            if(value < param_configs[index].min_val)
            {
                value = param_configs[index].min_val;
            }
            if(value > param_configs[index].max_val)
            {
                value = param_configs[index].max_val;
            }
            flash_union_buffer[offset + index].float_type = value;
        }
    }

    /* 控制中断只能看到加载前或加载后的完整参数集，不能看到半套新参数。 */
    irq_state = interrupt_global_disable();
    for(index = 0U; index < header.param_count; index++)
    {
        if(param_configs[index].variable != NULL)
        {
            *(param_configs[index].variable) =
                flash_union_buffer[offset + index].float_type;
        }
    }
    interrupt_global_enable(irq_state);

    current_slot = slot;
    return 1U;
}

/**
 * @brief 保存参数到Flash，写入magic/version/checksum防止垃圾值进控制器。
 */
uint8_t menu_flash_save_params(uint8_t slot)
{
    menu_flash_slot_header_t *header;
    uint32_t page_num;
    uint32_t offset;
    uint16_t index;

    if(slot >= MENU_SLOT_COUNT || param_count == 0U)
    {
        menu_show_error("Error");
        return 0U;
    }

    page_num = menu_get_slot_page(slot);
    if((page_num + MENU_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        menu_show_error("Error");
        return 0U;
    }

    flash_buffer_clear();

    header = (menu_flash_slot_header_t *)flash_union_buffer;
    offset = (uint32_t)((sizeof(menu_flash_slot_header_t) + 3U) / 4U);
    if((offset + param_count) > FLASH_PAGE_LENGTH)
    {
        menu_show_error("Error");
        return 0U;
    }

    for(index = 0U; index < param_count; index++)
    {
        if((param_configs[index].variable != NULL) &&
           (menu_float_is_finite(*(param_configs[index].variable)) == 0U))
        {
            menu_show_error("Bad Param");
            return 0U;
        }
    }

    for(index = 0U; index < param_count; index++)
    {
        if(param_configs[index].variable != NULL)
        {
            flash_union_buffer[offset + index].float_type = *(param_configs[index].variable);
        }
    }

    header->magic = MENU_MAGIC_NUMBER;
    header->version = MENU_VERSION;
    header->slot_id = slot;
    header->param_count = param_count;
    header->data_checksum = menu_flash_calc_data_checksum(param_count, offset);
    header->header_checksum = menu_flash_calc_header_checksum(header);
    memset(header->slot_name, 0, sizeof(header->slot_name));
    sprintf(header->slot_name, "Slot%u", (unsigned int)slot);

    (void)flash_write_page_from_buffer(0U, page_num, offset + param_count);

    if(menu_flash_slot_valid(slot, NULL) == 0U)
    {
        return 0U;
    }

    current_slot = slot;
    return 1U;
}

/**
 * @brief 格式化指定存档槽
 */
void menu_flash_format_slot(uint8_t slot)
{
    uint32_t page_num;
    uint8_t index;

    if(slot >= MENU_SLOT_COUNT)
    {
        menu_show_message("Invalid slot");
        return;
    }

    page_num = menu_get_slot_page(slot);
    if((page_num + MENU_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        menu_show_message("Page error");
        return;
    }

    if(car_safety_maintenance_acquire() == 0U)
    {
        menu_show_message("Stop First");
        return;
    }

    for(index = 0U; index < MENU_SLOT_SIZE; index++)
    {
        flash_erase_page(0U, page_num + index);
    }
    car_safety_maintenance_release();

    menu_show_message("Formatted");
}

/**
 * @brief 渲染当前菜单 (分页模式，每页8项)
 */
void menu_render_current(void)
{
    uint8_t i;
    uint8_t display_count = 0U;

    if(current_menu == NULL) return;

    ips114_set_font(UI_FONT_NORMAL);

    if(display_offset < current_item_count)
    {
        display_count = (uint8_t)(current_item_count - display_offset);
        if(display_count > MENU_MAX_VISIBLE_LINES)
        {
            display_count = MENU_MAX_VISIBLE_LINES;
        }
    }

    for(i = 0U; i < MENU_MAX_VISIBLE_LINES; i++)
    {
        if(i < display_count)
        {
            uint8_t item_index = (uint8_t)(display_offset + i);
            uint8_t selected = (item_index == current_index);
            uint8_t editing = (selected && menu_state == MENU_STATE_EDIT);

            menu_render_item(i, &current_menu[item_index], selected, editing);
        }
        else
        {
            menu_clear_line(i);
        }
    }

    if(current_menu[0].type == MENU_TYPE_AIR_COMMAND)
    {
        menu_air_cmd_status_t command_status;
        const char *state_text = "IDLE";

        menu_get_air_command_status(&command_status);
        if(command_status.state == MENU_AIR_CMD_STATE_WAIT_START_ACK)
        {
            state_text = "WAIT ACK";
        }
        else if(command_status.state == MENU_AIR_CMD_STATE_INSTANT_RUNNING)
        {
            state_text = "RUNNING";
        }
        else if(command_status.state == MENU_AIR_CMD_STATE_WAIT_EXIT_ACK)
        {
            state_text = "WAIT EXIT";
        }

        menu_show_text_line(6U, state_text, UI_COLOR_VALUE);
        menu_show_text_line(7U, command_status.last_ack_text, UI_COLOR_VALUE);
    }
}

/**
 * @brief 渲染菜单项 (优化颜色和选中效果)
 */
void menu_render_item(uint8_t line, menu_item_t* item, uint8_t selected, uint8_t editing)
{
    float value = 0.0f;
    float step = 0.0f;
    char display_text[MENU_DISPLAY_COLUMNS + 1U];
    char value_text[20];
    const char *enum_label;
    size_t name_length;
    size_t value_length;
    uint16_t color;

    if((item == NULL) || (line >= MENU_MAX_VISIBLE_LINES)) return;

    if((item->type == MENU_TYPE_AIR_COMMAND) &&
       (menu_air_command_is_running(item->param_index) != 0U))
    {
        color = UI_COLOR_EDITING;
    }
    else if(editing)
    {
        color = UI_COLOR_EDITING;
    }
    else if(selected)
    {
        color = UI_COLOR_SELECTED;
    }
    else
    {
        color = UI_COLOR_NORMAL;
    }

    memset(display_text, ' ', MENU_DISPLAY_COLUMNS);
    display_text[MENU_DISPLAY_COLUMNS] = '\0';
    display_text[0] = (selected && !editing) ? '>' : ' ';

    if(menu_is_param_item(item) != 0U)
    {
        if((item->type == MENU_TYPE_AIR_PARAMETER) &&
           (menu_air_param_is_available(item->param_index) == 0U))
        {
            name_length = strlen(item->name);
            if(name_length > 18U)
            {
                name_length = 18U;
            }
            memcpy(&display_text[1], item->name, name_length);
            memcpy(&display_text[MENU_DISPLAY_COLUMNS - 3U], "N/A", 3U);
            menu_show_text_line(line, display_text, color);
            return;
        }

        if(menu_get_item_param_value(item, &value) == 0U) return;

        name_length = strlen(item->name);
        if(name_length > 18U)
        {
            name_length = 18U;
        }
        memcpy(&display_text[1], item->name, name_length);

        enum_label = menu_get_air_enum_label(item, value);
        if(enum_label != NULL)
        {
            snprintf(value_text, sizeof(value_text), "%s", enum_label);
        }
        else if((menu_get_item_param_step(item, &step) != 0U) && (step < 0.001f))
        {
            snprintf(value_text, sizeof(value_text), "%.4f", (double)value);
        }
        else
        {
            snprintf(value_text, sizeof(value_text), "%.3f", (double)value);
        }
        value_length = strlen(value_text);
        if((value_length > 11U) &&
           ((enum_label == NULL) ||
            ((name_length + value_length + 2U) > MENU_DISPLAY_COLUMNS)))
        {
            value_length = 11U;
        }
        memcpy(&display_text[MENU_DISPLAY_COLUMNS - value_length],
               value_text,
               value_length);
    }
    else
    {
        name_length = strlen(item->name);
        if(name_length > (MENU_DISPLAY_COLUMNS - 1U))
        {
            name_length = MENU_DISPLAY_COLUMNS - 1U;
        }
        memcpy(&display_text[1], item->name, name_length);
    }

    menu_show_text_line(line, display_text, color);
}

/**
 * @brief 显示消息 (已废弃，建议使用专用的显示函数)
 */
static void menu_show_status_page(const char *msg, uint16_t color)
{
    char text[MENU_DISPLAY_COLUMNS + 1U];
    uint8_t line;
    size_t length = (msg != NULL) ? strlen(msg) : 0U;
    size_t start;

    if(length > MENU_DISPLAY_COLUMNS)
    {
        length = MENU_DISPLAY_COLUMNS;
    }
    start = (MENU_DISPLAY_COLUMNS - length) / 2U;
    memset(text, ' ', MENU_DISPLAY_COLUMNS);
    text[MENU_DISPLAY_COLUMNS] = '\0';
    if((msg != NULL) && (length > 0U))
    {
        memcpy(&text[start], msg, length);
    }

    for(line = 0U; line < MENU_MAX_VISIBLE_LINES; line++)
    {
        menu_clear_line(line);
    }
    menu_show_text_line(3U, text, color);
}

static void menu_set_status_message(const char *msg, uint16_t color, uint32_t duration_ms)
{
    memset(&menu_status_message, 0, sizeof(menu_status_message));
    menu_status_message.active = 1U;
    menu_status_message.auto_clear = (duration_ms > 0U) ? 1U : 0U;
    menu_status_message.color = color;
    menu_status_message.expire_tick = air_comm_car_get_tick() + duration_ms;
    if(msg != NULL)
    {
        strncpy(menu_status_message.text, msg, MENU_DISPLAY_COLUMNS);
        menu_status_message.text[MENU_DISPLAY_COLUMNS] = '\0';
    }

    menu_invalidate_display_cache();
    menu_request_refresh(REFRESH_FULL);
}

void menu_show_message(const char* msg)
{
    menu_set_status_message(msg, UI_COLOR_NORMAL, MENU_STATUS_MESSAGE_MS);
    // 普通消息显示2秒后由100Hz任务自动恢复菜单。
}

/**
 * @brief 显示成功消息 (绿色大字体)
 */
void menu_show_success(const char* msg)
{
    menu_set_status_message(msg, UI_COLOR_SUCCESS, MENU_STATUS_SUCCESS_MS);

    // 成功消息显示1秒
}

/**
 * @brief 显示错误消息 (红色大字体)
 */
void menu_show_error(const char* msg)
{
    menu_set_status_message(msg, UI_COLOR_ERROR, MENU_STATUS_ERROR_MS);

    // 错误消息显示2秒
}

/**
 * @brief 显示进度消息 (黄色，不自动清除)
 */
void menu_show_progress(const char* msg)
{
    menu_set_status_message(msg, UI_COLOR_EDITING, 0U);
    // 进度消息不自动清除，需要手动调用其他显示函数
}

/**
 * @brief 设置显示颜色
 */
void menu_set_display_color(uint16_t text_color)
{
    ips114_set_color(text_color, UI_COLOR_BG);
}

/**
 * @brief 清除屏幕
 */
void menu_clear_screen(void)
{
    ips114_clear();
    menu_invalidate_display_cache();
}

//====================================================局部刷新优化====================================================

/**
 * @brief 请求指定类型的刷新
 */
void menu_request_refresh(refresh_type_t type)
{
    if(type > refresh_type) {
        refresh_type = type;  // 优先级：FULL > SELECTION > VALUE > NONE
    }
    need_refresh = 1;
}

/**
 * @brief 清除指定行
 */
void menu_clear_line(uint8_t line)
{
    menu_show_text_line(line, "", UI_COLOR_NORMAL);
}

/**
 * @brief 渲染单个菜单项（用于局部刷新）
 */
void menu_render_single_item(uint8_t item_index)
{
    if(current_menu == NULL || item_index >= current_item_count) return;

    // 计算显示行号
    if(item_index < display_offset ||
       item_index >= display_offset + MENU_MAX_VISIBLE_LINES) return;

    uint8_t line = item_index - display_offset;
    uint8_t selected = (item_index == current_index);
    uint8_t editing = (selected && menu_state == MENU_STATE_EDIT);

    menu_render_item(line, &current_menu[item_index], selected, editing);
}

/**
 * @brief 优化的渲染函数（局部刷新）
 */
void menu_render_current_optimized(void)
{
    if(current_menu == NULL || current_item_count == 0) return;

    switch(refresh_type)
    {
        case REFRESH_FULL:
            // 重新计算当前页面，行缓存只发送实际变化的字符。
            menu_render_current();
            break;

        case REFRESH_SELECTION:
            // 选择项刷新：只重绘上次选中项和当前选中项
            ips114_set_font(UI_FONT_NORMAL);

            // 重绘上次选中的项（清除高亮）
            if(last_selected_index != current_index) {
                menu_render_single_item(last_selected_index);
            }

            // 重绘当前选中的项（添加高亮）
            menu_render_single_item(current_index);
            break;

        case REFRESH_VALUE:
            // 数值或编辑状态刷新：只处理当前选中行。
            ips114_set_font(UI_FONT_NORMAL);
            menu_render_single_item(current_index);
            break;

        case REFRESH_NONE:
        default:
            // 无需刷新
            return;
    }

    // 更新状态
    last_selected_index = current_index;
    refresh_type = REFRESH_NONE;
}

/**
 * @brief 设置需要刷新标志
 */
void menu_set_need_refresh(void)
{
    menu_request_refresh(REFRESH_FULL);
}

/**
 * @brief 显示参数值（内部函数）
 */
void menu_show_param_value(uint8_t x, uint8_t y, uint8_t index, uint8_t editing)
{
    if(index >= param_count) return;

    char value_str[16];
    float value = menu_get_param_by_index(index);
    sprintf(value_str, "%.2f", (double)value);

    if(editing)
    {
        // 编辑模式，添加特殊标记
        char edit_str[20];
        sprintf(edit_str, "*%s*", value_str);
        ips114_show_string(x, y, edit_str);
    }
    else
    {
        ips114_show_string(x, y, value_str);
    }
}
