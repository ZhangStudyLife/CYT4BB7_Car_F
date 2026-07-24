#ifndef _BEACON_CONFIG_H_
#define _BEACON_CONFIG_H_

#include "zf_common_headfile.h"

#define BEACON_CONFIG_BEACON_COUNT (6U)
#define BEACON_CONFIG_FLASH_PAGE   (70U) /* Work Flash 第70页，保存地图配置 */

typedef struct
{
    float x;                        /* 全局 X 坐标 [m]，正=右侧 */
    float y;                        /* 全局 Y 坐标 [m]，正=前方 */
} beacon_config_point_t;

typedef struct
{
    float initial_position[2];
    beacon_config_point_t beacons[BEACON_CONFIG_BEACON_COUNT];
} beacon_config_data_t;

void beacon_config_init(void);
void beacon_config_reset(void);
uint16 beacon_config_get_count(void);
uint8 beacon_config_get_beacon(uint16 index, beacon_config_point_t *beacon);
void beacon_config_get_initial_position(float position[2]);
void beacon_config_get_predata(beacon_config_data_t *data);
/* 保存可编辑地图配置。 */
uint8 beacon_config_save_predata(const beacon_config_data_t *data);

#endif
