#ifndef _BEACON_POSITION_RECORDER_H_
#define _BEACON_POSITION_RECORDER_H_

#include "zf_common_headfile.h"

#define BEACON_POSITION_RECORDER_AXIS_NUM       (2U)
#define BEACON_POSITION_RECORDER_MAX_POINTS     (10U)
#define BEACON_POSITION_RECORDER_INVALID_COORD  (-99.0f)
#define BEACON_POSITION_RECORDER_FLASH_PAGE     (71U) /* Work Flash 第71页，独占2KB */

typedef struct
{
    float position[BEACON_POSITION_RECORDER_AXIS_NUM];
    float points[BEACON_POSITION_RECORDER_MAX_POINTS][BEACON_POSITION_RECORDER_AXIS_NUM];
    uint16 point_count;
    uint8 active;
    uint8 full;
} beacon_position_recorder_t;

struct menu_item;

extern beacon_position_recorder_t g_beacon_position_recorder;

/* 上电初始化，恢复 Flash 中最近一次有效结果。 */
void beacon_position_recorder_init(void);

/* 进入记录模式：清空上一轮结果，并把当前位置定义为全局坐标 (0, 0)。 */
void beacon_position_recorder_enter(void);

/* 退出记录模式并自动保存；运行锁定时延迟到停车后保存。 */
void beacon_position_recorder_exit(void);

/* 100Hz 更新：按 odometer 全局速度积分，并在 Air CRSF CH8 上升沿记录坐标。 */
void beacon_position_recorder_update_100HZ(void);

/* 返回记录模式是否正在运行。 */
uint8 beacon_position_recorder_is_active(void);

/* 返回记录数据中的有效信标数量。 */
uint16 beacon_position_recorder_get_count(void);

/* 按有效记录顺序读取信标坐标，自动跳过无效槽位。 */
uint8 beacon_position_recorder_get_point(uint16 index, float point[2]);

/* 返回 C_Beacon 子菜单根节点。 */
struct menu_item *beacon_position_recorder_get_menu(void);

#endif
