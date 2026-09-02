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
#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H

#include "zf_common_headfile.h"

#define ENCODER_LEFT_INDEX   (TC_CH58_ENCODER)
#define ENCODER_LEFT_A       (TC_CH58_ENCODER_CH1_P17_3)
#define ENCODER_LEFT_B       (TC_CH58_ENCODER_CH2_P17_4)

#define ENCODER_RIGHT_INDEX  (TC_CH27_ENCODER)
#define ENCODER_RIGHT_A      (TC_CH27_ENCODER_CH1_P19_2)
#define ENCODER_RIGHT_B      (TC_CH27_ENCODER_CH2_P19_3)

typedef struct
{
    encoder_index_enum index;
    encoder_channel1_enum ch1_pin;
    encoder_channel2_enum ch2_pin;
    int16_t count_raw;
    uint8_t invert;
} encoder_data_t;

extern encoder_data_t encoder_left;
extern encoder_data_t encoder_right;

void encoder_control_init(void);
void encoder_update_100HZ(void);
int16_t encoder_get_left_count(void);
int16_t encoder_get_right_count(void);

#endif
