/********************************************************************
 * 文件名  : IMU_TOP.h
 * 模块    : IMU 顶层调度（驱动 + 滤波 + 姿态 + 校准）
 * 位置    : Estimation/Attitude
 * 职责    :
 *   1) 初始化 ICM42688 + 上电自检
 *   2) 1kHz 主循环：读传感器 -> 校准补偿 -> 滤波 -> 姿态解算
 *   3) 对外提供原始 IMU 快照（供校准链读取）
 * 调用方  : main.c / timer ISR（1kHz 中断或主循环调用）
 ********************************************************************/
#include "zf_common_headfile.h"
#ifndef IMU_TOP_H_
#define IMU_TOP_H_




#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    MahonyAhrs_Euler_t euler;
    imudata_t filtered;
    uint32 sample_count;
    uint8 healthy;
} imu_realtime_snapshot_t;

/* ======================== IMU 初始化参数 ======================== */
#define IMU_WARMUP_DISCARD_SAMPLES   (1000U)          /* 暖机丢弃帧数，1kHz 下约 1s，让滤波器稳定 */
#define IMU_UPDATE_DT_SEC            (1.0f / IMU_SAMPLE_RATE_HZ) /* 更新周期，1ms */

/* ======================== 上电自检参数 ======================== */
#define IMU_SELFTEST_SAMPLE_COUNT        (200U)        /* 自检采样数 */
#define IMU_SELFTEST_GYRO_MEAN_MAX_DPS   (8.0f)       /* 静止时陀螺仪平均模长上限，单位 dps */
#define IMU_SELFTEST_ACC_MIN_G           (0.75f)       /* 加速度模长下限，单位 g */
#define IMU_SELFTEST_ACC_MAX_G           (1.25f)       /* 加速度模长上限，单位 g */

/* g_imufilter_1000hz 在 IMU_Filtter.h 中声明，是 1kHz 滤波后输出 */
extern MahonyAhrs_t g_mahony_ahrs;    /* Mahony 姿态解算器状态 */
extern MahonyAhrs_Euler_t g_euler;    /* 当前欧拉角（度），roll/pitch/yaw + sin/cos 缓存 */
extern uint8 g_imu_ready;             /* 1=IMU 初始化与暖机完成；健康状态见 IMU_RuntimeHealthy */

/* 读取当前帧原始 IMU 快照，供校准链使用 */
/* gx/gy/gz 输出陀螺仪角速度 dps（已去零偏），ax/ay/az 输出加速度计比力 g */
extern volatile uint32 g_imu_update_count;
extern volatile uint32 g_imu_read_error_count;
extern volatile uint32 g_imu_processing_error_count;
extern volatile uint32 g_imu_calibration_sample_drop_count;
extern volatile uint16 g_imu_consecutive_read_error_count;
extern volatile uint16 g_imu_max_consecutive_read_error_count;
extern volatile uint8 g_imu_runtime_fault;
extern volatile uint8 g_imu_startup_selftest_fault;

void IMU_GetRawSampleForCalibration(float *gx, float *gy, float *gz,
                                     float *ax, float *ay, float *az);

/* 初始化 IMU 全套：驱动 -> 自检 -> 滤波器 -> 姿态解算器 -> 暖机 */
void IMU_Init_All(void);
/* 1kHz 主更新：读传感器 -> 校准补偿 -> 滤波 -> Mahony 姿态解算 */
uint8 IMU_Update_1000HZ(void);
void IMU_ServicePoll(void);
uint8 IMU_GetRealtimeSnapshot(imu_realtime_snapshot_t *snapshot);
uint8 IMU_RuntimeHealthy(void);
/* 重置 yaw 角为 0，保留 roll/pitch（安全影响：改变航向基准） */
void IMU_ResetYaw(void);
/* 查询 IMU 是否就绪（1=初始化完成且自检通过） */
uint8 IMU_Is_Ready(void);


#ifdef __cplusplus
}
#endif

#endif /* IMU_TOP_H_ */
