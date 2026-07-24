/********************************************************************
 * 文件名  : IMU_Filtter.h
 * 模块    : IMU 输入链路滤波（抗混叠 + 双陷波 + 低通）
 * 位置    : Estimation/Attitude
 * 职责    :
 *   1) 1kHz 原始 IMU 输入 -> 陀螺仪/加速度计多级滤波
 *   2) 输出 g_imufilter_1000hz（姿态解算和里程计使用）
 * 滤波链  :
 *   陀螺仪: AntiAliasLPF(250Hz) -> Notch(161Hz) -> Notch(320Hz) -> LPF(60Hz)
 *   加速度: Notch(161Hz) -> Notch(320Hz) -> LPF(12Hz)
 * 调用方  : IMU_TOP.c IMU_Update_1000HZ()
 ********************************************************************/
#include "zf_common_headfile.h"
#ifndef IMU_FILTTER_H_
#define IMU_FILTTER_H_



#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 采样率 ======================== */
#define IMU_SAMPLE_RATE_HZ         (1000.0f)           /* 输入采样率 1kHz */

/* ======================== 陷波器参数 ======================== */
/* 两级陷波分别针对电机振动的 161Hz 和 320Hz 谐波 */
#define IMU_NOTCH0_HZ              (120.0f)
#define IMU_NOTCH0_BW_HZ           (18.0f)
#define IMU_NOTCH0_Q               (IMU_NOTCH0_HZ / IMU_NOTCH0_BW_HZ)

#define IMU_NOTCH1_HZ              (160.0f)
#define IMU_NOTCH1_BW_HZ           (24.0f)
#define IMU_NOTCH1_Q               (IMU_NOTCH1_HZ / IMU_NOTCH1_BW_HZ)

#define IMU_NOTCH2_HZ              (320.0f)
#define IMU_NOTCH2_BW_HZ           (30.0f)
#define IMU_NOTCH2_Q               (IMU_NOTCH2_HZ / IMU_NOTCH2_BW_HZ)

/* ======================== 低通参数 ======================== */
#define IMU_GYRO_ANTI_ALIAS_LPF_HZ (250.0f)            /* 陀螺仪抗混叠 Butterworth 截止频率 */
#define IMU_GYRO_LPF_HZ            (35.0f)             /* 稳定优先，压掉底盘高频假姿态 */
#define IMU_ACCEL_LPF_HZ           (8.0f)              /* 加速度只保留低频重力参考 */

#define IMU_ACCEL_SHOCK_AXIS_G     (3.0f)              /* 单轴加速度限幅，压制麦轮/齿轮冲击尖峰 */

#define IMU_AXIS_NUM               (3U)                 /* 三轴 */

/* ======================== 二阶 IIR 滤波器状态 ======================== */
/* Transposed Direct Form II 结构，b 为前馈系数，a 为反馈系数 */
typedef struct
{
    float b0, b1, b2;   /* 前馈系数 */
    float a1, a2;        /* 反馈系数（a0 固定为 1） */
    float d1, d2;        /* 延迟状态 */
} IMUBiquad_t;

/* ======================== IMU 六轴输出数据 ======================== */
/* 陀螺仪单位 dps，加速度计单位 g */
typedef struct
{
    float gyrox, gyroy, gyroz;  /* 角速度，单位 dps */
    float accx, accy, accz;     /* 比力（静止约 -1g Down），单位 g */
} imudata_t;

/* ======================== 全局滤波输出 ======================== */
extern imudata_t g_imufilter_1000hz; /* 1kHz 滤波输出，姿态解算/里程计/EKF 使用 */
extern imudata_t g_imudata_500hz;    /* 500Hz 输出（当前与 1kHz 共用同一条链，预留） */
extern imudata_t g_imudata_250hz;    /* 250Hz 输出（当前与 1kHz 共用同一条链，预留） */

/* 初始化全部滤波器（各 IIR 状态清零） */
void IMUFilter_Init(void);

/* 1kHz 更新：输入原始 IMU 数据，经过抗混叠->双陷波->低通，输出到全局结构体 */
/* gx/gy/gz 陀螺仪输入 dps，ax/ay/az 加速度计输入 g */
void IMUFilter_Update(float gx, float gy, float gz,
                      float ax, float ay, float az);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FILTTER_H_ */
