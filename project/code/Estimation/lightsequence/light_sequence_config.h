#ifndef LIGHT_SEQUENCE_CONFIG_H
#define LIGHT_SEQUENCE_CONFIG_H

#include "zf_common_headfile.h"

#define LIGHT_SEQUENCE_BEACON_COUNT        (BEACON_CONFIG_BEACON_COUNT) /* 使用Predata地图的信标数量 */
#define LIGHT_SEQUENCE_PRESET_COUNT        (4U)    /* 预设亮灯序列数量 */
#define LIGHT_SEQUENCE_MAX_ROUNDS          (40U)   /* 单套序列最多保存的轮数 */
#define LIGHT_SEQUENCE_MATCH_RADIUS_M      (0.5f)  /* 熄灯事件坐标匹配半径，单位m */
#define LIGHT_SEQUENCE_DUPLICATE_WINDOW_MS (1000U) /* 同一灯号的时间去重窗口，单位ms */

/**
 * @brief 一套预设亮灯序列。
 */
typedef struct
{
    uint8 round_count;                              /* 有效轮数 */
    uint16 round_lights[LIGHT_SEQUENCE_MAX_ROUNDS]; /* 各轮灯号组合，例如16表示1号和6号灯 */
} light_sequence_preset_t;

/* 预设亮灯序列配置。 */
extern const light_sequence_preset_t g_light_sequence_presets[LIGHT_SEQUENCE_PRESET_COUNT];

#endif /* LIGHT_SEQUENCE_CONFIG_H */
