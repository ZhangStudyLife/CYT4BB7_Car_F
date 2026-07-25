/*********************************************************************************************************************
* CYT4BB 编码器控制模块 - 头文件
*
* 文件功能：封装正交编码器的初始化、周期读取、一阶低通滤波
* 模块说明：
*   1. 两轮底盘主映射：左轮使用原左前编码器 M3，右轮使用原右前编码器 M4
*   2. encoder_update_100HZ() 在 10ms 定时中断中调用，读取并清零硬件计数
*   3. count_raw = 速度（单周期脉冲数），count_total = 里程（累计脉冲数）
*   4. 一阶低通滤波器对 count_raw 做平滑，减少脉冲噪声
*   5. 原后轮 M1/M2 编码器暂时保留，供旧信标检测等四轮模块兼容
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef _ENCODER_CONTROL_H_
#define _ENCODER_CONTROL_H_


// ========================== 配置参数 ==========================

/*
 * 一阶低通滤波参数
 * alpha=0.386 在 100Hz 采样下截止频率约 7.92Hz。
 * 旧 Q=0.25/R=4.0 一阶卡尔曼稳态增益约 0.2207，等效截止频率约 3.99Hz。
 */
#define ENCODER_LPF_ALPHA                  (0.386f)


// ========================== 引脚定义 ==========================
// 编码器 M1 硬件引脚（实际对应右后轮位置）
#define ENCODER_M1_INDEX    (TC_CH20_ENCODER)
#define ENCODER_M1_A        (TC_CH20_ENCODER_CH1_P08_1)
#define ENCODER_M1_B        (TC_CH20_ENCODER_CH2_P08_2)

// 编码器 M2 硬件引脚（实际对应左后轮位置）
#define ENCODER_M2_INDEX    (TC_CH07_ENCODER)
#define ENCODER_M2_A        (TC_CH07_ENCODER_CH1_P07_6)
#define ENCODER_M2_B        (TC_CH07_ENCODER_CH2_P07_7)

// 编码器 M3 硬件引脚（实际对应左前轮位置）
#define ENCODER_M3_INDEX    (TC_CH58_ENCODER)
#define ENCODER_M3_A        (TC_CH58_ENCODER_CH1_P17_3)
#define ENCODER_M3_B        (TC_CH58_ENCODER_CH2_P17_4)

// 编码器 M4 硬件引脚（实际对应右前轮位置）
#define ENCODER_M4_INDEX    (TC_CH27_ENCODER)
#define ENCODER_M4_A        (TC_CH27_ENCODER_CH1_P19_2)
#define ENCODER_M4_B        (TC_CH27_ENCODER_CH2_P19_3)

// ========================== 数据结构 ==========================

/* 单个编码器的数据，包含硬件引脚、原始/滤波计数、累计里程 */
typedef struct
{
    encoder_index_enum index;                   // 编码器硬件索引（TC 通道号）
    encoder_channel1_enum ch1_pin;              // A 相引脚
    encoder_channel2_enum ch2_pin;              // B 相引脚

    int16_t count_raw;                          // 本周期原始计数（速度，脉冲/周期）
    float count_filtered;                       // 一阶低通滤波后的计数
    int32_t count_total;                        // 累计计数（用于里程计算）

    int8_t invert;                              // 1=取反计数方向，用于校正接线
} encoder_data_t;

// ========================== 全局变量声明 ==========================

/* 四个轮子的编码器实例，上层可直接读取 count_raw/count_total */
extern encoder_data_t encoder_left_front;       // 左前编码器（对应 M3 通道）
extern encoder_data_t encoder_right_front;      // 右前编码器（对应 M4 通道）
extern encoder_data_t encoder_left_rear;        // 左后编码器（对应 M2 通道）
extern encoder_data_t encoder_right_rear;       // 右后编码器（对应 M1 通道）

// ========================== 函数声明 ==========================

/* 初始化四路编码器硬件（TC 正交解码通道），上电调用一次 */
void encoder_control_init(void);

/*
 * 周期更新：读取并清零四路编码器计数、更新一阶低通滤波和累计里程
 * 调用者：10ms 定时中断（100Hz）
 */
void encoder_update_100HZ(void);

/*
 * 两轮底盘主接口。
 * 逻辑约定：车辆前进时左右轮计数均为正；内部复用已验证的 left_front/right_front 数据。
 */
int16_t encoder_get_left_count(void);
int16_t encoder_get_right_count(void);
float encoder_get_left_filtered_count(void);
float encoder_get_right_filtered_count(void);

/* 以下是四轮原始计数读取（脉冲/周期），给 PID 速度环用 */
int16_t encoder_get_left_front_count(void);
int16_t encoder_get_right_front_count(void);
int16_t encoder_get_left_rear_count(void);
int16_t encoder_get_right_rear_count(void);

/* 一阶低通滤波后的计数，比 raw 更平滑，适合上层路径规划 */
float encoder_get_left_front_filtered_count(void);
float encoder_get_right_front_filtered_count(void);
float encoder_get_left_rear_filtered_count(void);
float encoder_get_right_rear_filtered_count(void);

/* 累计计数（里程），用于位置估计，单位 = 累计脉冲数 */
int32_t encoder_get_left_front_total(void);
int32_t encoder_get_right_front_total(void);
int32_t encoder_get_left_rear_total(void);
int32_t encoder_get_right_rear_total(void);

/* 清零累计计数，开始新任务前调用 */
void encoder_clear_left_front_total(void);
void encoder_clear_right_front_total(void);
void encoder_clear_left_rear_total(void);
void encoder_clear_right_rear_total(void);

#endif // _ENCODER_CONTROL_H_
