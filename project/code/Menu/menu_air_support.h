/* Air参数远程同步模块 - 头文件
 *
 * 功能：管理飞机端（Air）的可调参数，通过AirComm无线同步
 * 工作方式：
 *   1. 菜单修改参数 → 标记dirty
 *   2. 100HZ轮询发现dirty → 发送set_param命令给Air
 *   3. Air回复ACK → 清除dirty标记
 * 安全条件：Air在线 且 小车未运行（control_enabled=0或紧急停）时才允许编辑
 * Flash存档：支持4个存档槽（slot 0-3），独立于车端参数存档
 */

#ifndef MENU_AIR_SUPPORT_H
#define MENU_AIR_SUPPORT_H

#include "zf_common_headfile.h"

#define MENU_AIR_MAX_PARAMS                  (384U)
#define MENU_AIR_EXPECTED_PARAM_COUNT        (212U)

#if (MENU_AIR_EXPECTED_PARAM_COUNT > MENU_AIR_MAX_PARAMS)
#error "Air parameter count exceeds storage capacity"
#endif

#define MENU_AIR_SYNC_MODE_IDLE             (0U)
#define MENU_AIR_SYNC_MODE_COMMIT           (1U)
#define MENU_AIR_SYNC_MODE_FULL             (2U)
#define MENU_AIR_SYNC_MODE_DONE             (3U)
#define MENU_AIR_SYNC_MODE_FAIL             (4U)
#define MENU_AIR_SYNC_MODE_PULL             (5U)
#define MENU_AIR_SYNC_MODE_RECOVER          (6U)
#define MENU_AIR_SYNC_REASON_NONE           (0U)
#define MENU_AIR_SYNC_REASON_BOOT           (1U)
#define MENU_AIR_SYNC_REASON_LOAD           (2U)
#define MENU_AIR_SYNC_REASON_MANUAL         (3U)
#define MENU_AIR_SYNC_REASON_COMMIT         (4U)
#define MENU_AIR_SYNC_REASON_BOOT_OVERRIDE  (5U)
#define MENU_AIR_SYNC_REASON_BOOT_SCREEN    (6U)

#define MENU_AIR_CMD_STATE_IDLE             (0U)
#define MENU_AIR_CMD_STATE_WAIT_START_ACK   (1U)
#define MENU_AIR_CMD_STATE_INSTANT_RUNNING  (2U)
#define MENU_AIR_CMD_STATE_WAIT_EXIT_ACK    (3U)
#define MENU_AIR_CMD_INVALID_INDEX          (0xFFU)
#define MENU_AIR_CMD_ACK_TEXT_MAX           (96U)

/* Air参数配置结构体 */
typedef struct
{
    char name[32];          // 参数名（用于AirComm传输）
    float *variable;        // 参数变量指针
    float step;             // 编辑步进值
    float min_val;          // 最小值
    float max_val;          // 最大值
    const char *menu_name;  // 所属菜单名称
    uint8 visible;          // 0=仅从菜单隐藏，仍参与通信与Flash存档
    const char * const *enum_labels; // 可选枚举文本，NULL表示按普通数值显示
    uint8 enum_count;                // 枚举文本数量
    uint8 optional;                  // 1表示旧Air固件缺少该参数时不阻断其余参数同步
    uint8 available;                 // 1表示本次目录拉取已确认远端支持该参数
    uint8 enter_confirm_only;        // 1表示Back取消编辑，仅Enter确认后下发
} menu_air_param_config_t;

/* Air参数同步状态（供诊断页读取） */
typedef struct
{
    uint16 dirty_count;         // 待同步参数数量
    uint8 sending;              // 是否正在发送
    uint16 active_index;        // 当前正在同步的参数索引
    uint16 last_failed_index;   // 上次同步失败的索引
    uint8 last_failed_result;   // 上次失败请求的ACK结果
    uint8 last_failed_status;   // 上次失败请求的ACK状态码
    uint8 last_result;          // 上次ACK结果
    uint8 last_status;          // 上次ACK状态码
    uint32 send_count;          // 总发送次数
    uint32 ok_count;            // 成功次数
    uint32 fail_count;          // 失败次数
    uint8 mode;
    uint8 reason;
    uint32 timeout_count;
} menu_air_sync_status_t;

typedef struct
{
    uint8 state;
    uint8 active_index;
    uint8 last_result;
    uint8 last_status;
    uint32 send_tick;
    uint32 timeout_count;
    char last_ack_text[MENU_AIR_CMD_ACK_TEXT_MAX + 1U];
} menu_air_cmd_status_t;

/* Air参数变量（菜单可调） */
void menu_air_support_init(void);
uint16 menu_get_air_param_count(void);                                                  // 获取参数数量
float menu_get_air_param_by_index(uint16 index);                                        // 按索引读取
uint8 menu_set_air_param_by_index(uint16 index, float value);                           // 按索引设置（自动标记dirty）
const menu_air_param_config_t *menu_get_air_param_config(uint16 index);                 // 获取参数配置
uint8 menu_air_param_is_available(uint16 index);                                        // 远端是否支持指定参数
uint8 menu_is_air_connected(void);                                                      // Air是否在线
uint8 menu_can_edit_air_params(void);                                                   // 是否允许编辑（在线且未运行）
uint8 menu_sync_all_air_params(void);                                                   // 标记所有参数为dirty（触发全量同步）
uint8 menu_air_commit_param(uint16 index);
uint8 menu_air_commit_param_value(uint16 index, float value);
uint8 menu_air_sync_all_start(uint8 reason);
uint8 menu_air_is_busy(void);
void menu_air_stop_param_sync(void);
void menu_air_abort_param_sync_runtime(void);
void menu_air_update_100HZ(void);                                                       // 100HZ同步轮询
void menu_get_air_sync_status(menu_air_sync_status_t *status);                          // 获取同步状态
uint8 menu_load_air_slot(uint8 slot);                                                   // 从Flash加载Air参数存档
uint8 menu_save_air_slot(uint8 slot);                                                   // 保存Air参数到Flash

uint8 menu_air_command_start(uint8 index);
uint8 menu_air_command_stop(void);
void menu_air_command_abort_runtime(void);
void menu_air_command_update_100HZ(void);
uint8 menu_air_command_is_running(uint8 index);
uint8 menu_air_command_is_active(void);
uint8 menu_air_command_get_count(void);
const char *menu_air_command_get_name(uint8 index);
void menu_get_air_command_status(menu_air_cmd_status_t *status);

#endif
