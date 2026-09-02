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
* 菜单核心框架头文件
* 功能：提供完整的菜单引擎，包括菜单导航、按键处理、屏幕渲染、Flash存档管理、参数编辑系统
* 特点：一旦完成开发，基本不需要修改；提供标准API供用户配置调用；完全独立，不依赖具体菜单内容
*
* 适配：逐飞科技 IPS114 + CYT4BB7 Flash
* 版本：v2.1 (CYT4BB7 移植版本)
* 日期：2025年
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef _MENU_CORE_H_
#define _MENU_CORE_H_

void menu_discard_key_events(void);
void menu_runtime_suspend(void);
void menu_runtime_resume(void);



// 外部变量声明

// 菜单系统配置
#define MENU_MAX_ITEMS          32          // 每个菜单最大选项数
#define MENU_MAX_DEPTH          8           // 最大菜单嵌套层级
#define MENU_MAX_PARAMS         182         // 最大参数数量
#define MENU_MAX_VISIBLE_LINES  8          // 屏幕最大可显示菜单行数

// UI显示配置 (纯净设计，无状态栏标题栏)
#define UI_COLOR_BG             0x0000      // 纯黑背景
#define UI_COLOR_NORMAL         0xFFFF      // 白色正常文字
#define UI_COLOR_SELECTED       0x07E0      // 绿色选中项
#define UI_COLOR_EDITING        0xFFE0      // 黄色编辑状态
#define UI_COLOR_SUCCESS        0x07E0      // 绿色成功状态
#define UI_COLOR_ERROR          0xF800      // 红色错误状态
#define UI_COLOR_VALUE          0x07FF      // 青色参数值

// 字体大小配置（直接使用库定义的枚举类型，避免类型混合警告）
#define UI_FONT_NORMAL              IPS114_8X16_FONT    // 普通字体：菜单项 (8x16像素)
#define UI_FONT_LARGE               IPS114_8X16_FONT    // 大字体：关键提示 (暂用8x16，16x16字体需验证)

// Flash存档安全配置 (CYT4BB7: 96页，每页2KB)
#define MENU_SLOT_COUNT         4           // 存档数量
#define MENU_SLOT_BASE_PAGE     72          // Car菜单存档使用72-79页，避开IMU page 95
#define MENU_SLOT_SIZE          2           // 每个存档占用2页(4KB)
#define MENU_MAGIC_NUMBER       0x5A5A5A5A  // 存档验证魔数
#define MENU_VERSION            0x04        // 参数目录变化后使旧位置式存档失效

// Flash安全边界检查 (CYT4BB7有96页，确保不超出限制)
#define FLASH_SAFE_START_PAGE   MENU_SLOT_BASE_PAGE
#define FLASH_SAFE_END_PAGE     (MENU_SLOT_BASE_PAGE + MENU_SLOT_COUNT * MENU_SLOT_SIZE - 1)
#if (FLASH_SAFE_END_PAGE >= 96)
#error "Flash slot configuration exceeds 96 pages limit!"
#endif

// 菜单项类型
typedef enum {
    MENU_TYPE_SUBMENU = 0,      // 子菜单项
    MENU_TYPE_FUNCTION,         // 函数项
    MENU_TYPE_PARAMETER,        // 参数项
    MENU_TYPE_AIR_PARAMETER,    // Air远程参数项
    MENU_TYPE_EXTERNAL_PARAMETER, // 外部模块浮点参数项
    MENU_TYPE_AIR_COMMAND,      // Air远程命令项
    MENU_TYPE_DIAG_VIEW         // 只读诊断页
} menu_type_t;

// 刷新类型枚举（局部刷新优化）
typedef enum {
    REFRESH_NONE = 0,           // 无需刷新
    REFRESH_VALUE,              // 数值刷新（参数编辑）
    REFRESH_SELECTION,          // 选择项刷新（上下移动）
    REFRESH_FULL                // 页面刷新（仅更新变化内容）
} refresh_type_t;

typedef struct
{
    float *variable;
    float step;
    float min_val;
    float max_val;
    const char * const *enum_labels; // 可选枚举文本，NULL表示显示数值
    uint8_t enum_count;              // 枚举文本数量
} menu_external_param_config_t;

typedef struct
{
    void (*render)(void);
    void (*on_exit)(void);
    uint8_t (*on_key)(uint8_t key);
    uint8_t refresh_periodic;
    uint8_t long_back_only;
    uint8_t allow_runtime_locked;
} menu_external_view_config_t;

// 菜单项结构
typedef struct menu_item {
    char name[32];              // 菜单项名称
    menu_type_t type;           // 菜单项类型
    union {
        struct menu_item* submenu;      // 子菜单指针
        void (*function)(void);         // 函数指针
        uint16_t param_index;          // 参数索引
        menu_external_param_config_t *external_param;
    };
} menu_item_t;

// 参数配置结构
typedef struct {
    volatile float* variable;   // 参数变量指针（可由控制中断读取）
    float step;                 // 编辑步进值
    float min_val;              // 最小值
    float max_val;              // 最大值
} param_config_t;

// 存档参数数据结构
typedef struct {
    float value;                // 参数值
    uint32_t checksum;          // 参数校验和
} menu_param_data_t;

// 存档数据结构 (设计为适合Flash页面大小)
typedef struct {
    uint32_t magic;             // 魔数验证
    uint8_t version;            // 版本号
    uint8_t slot_id;            // 存档ID (0-3)
    uint16_t param_count;       // 参数数量
    uint32_t data_checksum;     // 数据区校验和
    uint32_t header_checksum;   // 头部校验和
    char slot_name[16];         // 存档名称
    menu_param_data_t params[MENU_MAX_PARAMS]; // 参数数据
} menu_save_data_t;

// 菜单状态
typedef enum {
    MENU_STATE_NORMAL = 0,      // 普通浏览模式
    MENU_STATE_EDIT,            // 参数编辑模式
    MENU_STATE_DIAG_VIEW,       // 只读诊断页模式
    MENU_STATE_EXTERNAL_VIEW    // 外部模块页面模式
} menu_state_t;

// 按键定义
typedef enum {
    KEY_UP = 0,                 // Key1 - 上移/增加
    KEY_DOWN,                   // Key2 - 下移/减少
    KEY_ENTER,                  // Key3 - 确认/编辑切换
    KEY_BACK,                   // Key4 - 返回
    KEY_COUNT
} menu_key_t;

// 按键状态
typedef enum {
    KEY_STATE_IDLE = 0,         // 空闲
    KEY_STATE_PRESSED,          // 按下
    KEY_STATE_RELEASED          // 释放
} key_state_t;

// 存档标识
#define SLOT_MAGIC_BASE         0xA5A50000  // 存档魔数基础值

//====================================================菜单核心API====================================================
// 系统初始化
void menu_init(void);                                  // 菜单系统初始化
void menu_update_100HZ(void);                          // 菜单任务处理（主循环调用，处理按键和刷新）
void menu_show(void);                                  // 手动刷新菜单显示

// 定时器处理
void menu_timer_handler(void);                         // 10ms定时器中断处理函数（在用户ISR中调用）

// 参数管理
void menu_register_param(volatile float* var, float step, float min, float max); // 注册参数
uint8_t menu_get_param_count(void);                                        // 获取参数数量
float menu_get_param_by_index(uint8_t index);                             // 按索引获取参数值
void menu_set_param_by_index(uint8_t index, float value);                 // 按索引设置参数值
void menu_show_debug_info(void);                                          // 显示调试信息

// 存档管理
void menu_load_slot(uint8_t slot);                     // 加载指定存档
void menu_save_slot(uint8_t slot);                     // 保存到指定存档
uint8_t menu_get_current_slot(void);                   // 获取当前存档号

// 菜单控制
void menu_set_root(menu_item_t* root_menu);           // 设置根菜单
void menu_enter_submenu(menu_item_t* submenu);        // 进入子菜单
void menu_return_to_parent(void);                     // 返回上级菜单
void menu_reset_to_first(void);                       // 重置选择到第一项
uint8_t menu_enter_external_view(const menu_external_view_config_t *config);
uint8_t menu_external_view_runtime_active(void);

// 显示控制
void menu_set_need_refresh(void);                     // 设置需要刷新标志
void menu_clear_screen(void);                         // 清除屏幕
void menu_show_message(const char* msg);              // 显示消息 (已废弃，使用下面的函数)

// 优化的显示接口 (支持颜色和字体控制)
void menu_show_success(const char* msg);              // 显示成功消息 (绿色大字体)
void menu_show_error(const char* msg);                // 显示错误消息 (红色大字体)
void menu_show_progress(const char* msg);             // 显示进度消息 (黄色)
void menu_set_display_color(uint16_t text_color);     // 设置显示颜色

// 局部刷新优化接口
void menu_request_refresh(refresh_type_t type);       // 请求指定类型的刷新
void menu_render_current_optimized(void);             // 优化的渲染函数（局部刷新）
void menu_clear_line(uint8_t line);                   // 清除指定行
void menu_render_single_item(uint8_t item_index);     // 渲染单个菜单项
void menu_show_text_line(uint8_t line, const char *text, uint16_t color); // 仅更新一行中变化的字符
void menu_invalidate_display_cache(void);             // 外部直接绘图后使文本缓存失效

//====================================================内部接口（用户无需关心）====================================================
// 按键处理内部函数
void menu_process_keys(void);                          // 处理按键事件
void menu_key_handler(menu_key_t key);                 // 按键处理

// Flash存档内部函数
uint8_t menu_flash_check_slot(uint8_t slot);          // 检查存档有效性
uint8_t menu_flash_load_params(uint8_t slot);         // 从Flash加载参数
uint8_t menu_flash_save_params(uint8_t slot);         // 保存参数到Flash
void menu_flash_format_slot(uint8_t slot);            // 格式化存档
void menu_flash_test(void);                           // Flash功能测试

// 显示内部函数
void menu_render_current(void);                       // 渲染当前菜单
void menu_render_item(uint8_t line, menu_item_t* item, uint8_t selected, uint8_t editing);
void menu_show_param_value(uint8_t x, uint8_t y, uint8_t index, uint8_t editing);

#endif
