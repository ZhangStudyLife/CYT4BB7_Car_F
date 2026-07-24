#include "light_sequence_config.h"

/*
 * 每轮直接填写亮灯编号，例如16表示1号和6号灯，456表示4号、5号和6号灯。
 * 每轮设置1至3盏灯，灯号范围为1至6，同一轮灯号不能重复。
 * 每套序列均保证同一轮灯号唯一，并且相邻两轮没有相同灯号。
 */
const light_sequence_preset_t g_light_sequence_presets[LIGHT_SEQUENCE_PRESET_COUNT] =
{
    /* 序列1 */
    {6U, {15U, 42U, 36U, 45U, 12U, 46U}},

    /* 序列2 */
    {6U, {12U, 34U, 56U, 14U, 23U, 45U}},

    /* 序列3 */
    {6U, {15U, 46U, 23U, 14U, 56U, 24U}},

    /* 序列4 */
    {6U, {14U, 23U, 46U, 15U, 43U, 21U}},

};
