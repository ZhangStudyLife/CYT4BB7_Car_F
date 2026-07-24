/********************************************************************
 * 文件名  : Accel_Calibration.h
 * 模块    : 加速度计校准 + IMU 校准子系统
 * 位置    : Estimation/Attitude
 * 职责    :
 *   1) ICM42688 加速度计偏置/缩放/椭球校准
 *   2) 安装旋转矩阵（IMU->机体）
 *   3) 去重力 + 机体系/水平系线性加速度输出（供里程计、EKF 使用）
 *   4) IMU 校准状态机（陀螺仪静态、6面加速度、椭球加速度）
 *   5) Flash 持久化校准参数
 * 调用方  : IMU_TOP.c（1kHz 更新）、菜单/UI（触发校准）
 * 频率    : 1kHz（AccelCalibration_Update_1000HZ）
 ********************************************************************/

#include "zf_common_headfile.h"
#ifndef ACCEL_CALIBRATION_H_
#define ACCEL_CALIBRATION_H_



#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 物理常量与频率 ======================== */
#define ACCEL_CALIBRATION_GRAVITY_MSS            (9.80665f) /* 标准重力加速度，单位 m/s^2 */
#define ACCEL_CALIBRATION_DT_S                   (0.001f)   /* 实时更新周期 1ms，单位 s */
#define ACCEL_CALIBRATION_SAMPLES                (1000U)    /* 静止标定目标样本数，按 1kHz 约 1s */

/* ======================== 静止质量阈值 ======================== */
#define ACCEL_CALIBRATION_STD_G_WARN_MAX         (0.050f)   /* |a| 标准差告警阈值，单位 g */
#define ACCEL_CALIBRATION_STD_G_FAIL_MAX         (0.080f)   /* |a| 标准差失败阈值，单位 g */

/* 比力方向约定：静止平放时 az 约为 -1g（加速度计测量的是比力，不是加速度） */
#define ACCEL_CALIBRATION_STATIC_SPECIFIC_FORCE_SIGN (-1.0f)

/* 水平系投影开关：0=仅 roll/pitch 去倾斜，1=额外做 yaw 旋转 */
#define ACCEL_CALIBRATION_LEVEL_USE_YAW          (0U) /* Keep 0 for odometer; odometer applies yaw itself. */

/* ======================== 机体系方向符号约定 ======================== */
/* 机体系 FRD（Forward-Right-Down）：+X 向前，+Y 向右，+Z 向下 */
#define ACCEL_CALIBRATION_BODY_AXIS_X_FORWARD    (+1.0f)
#define ACCEL_CALIBRATION_BODY_AXIS_Y_RIGHT      (+1.0f)
#define ACCEL_CALIBRATION_BODY_AXIS_Z_DOWN       (+1.0f)
#define ACCEL_CALIBRATION_OUTPUT_DOWN_SIGN       (+1.0f)    /* Down 方向输出为正 */
#define ACCEL_CALIBRATION_OUTPUT_UP_SIGN         (-1.0f)    /* Up 方向输出为负 */

/* ======================== 加速度校准运行态 ======================== */
/* g_accel_calibration 是全局单例，被 odometer、EKF、姿态解算等模块读取 */
typedef struct
{
    bool is_calibrated;              /* 1=已有有效校准参数 */
    uint16_t sample_count;           /* 标定已采集样本计数 */

    /* --- 机体系校准参数（Flash 持久化） --- */
    float accel_bias_g[3];           /* 加速度偏置，单位 g，三轴 */
    float accel_scale[3];            /* 加速度缩放因子（对角矩阵时使用） */
    float accel_corr_matrix[3][3];   /* 加速度 3x3 校正矩阵（椭球拟合时为完整矩阵） */
    uint8_t use_full_matrix;         /* 1=使用完整校正矩阵，0=仅对角缩放 */

    /* --- 每帧传感器数据 --- */
    float accel_raw_body_g[3];       /* 机体系原始加速度（含偏置），单位 g */
    float accel_corrected_body_g[3]; /* 机体系校正后加速度（去偏置+缩放），单位 g */
    float gyro_raw_body_dps[3];      /* 机体系原始角速度，单位 dps（已扣除陀螺仪零偏） */

    /* --- 机体系线性加速度（去重力），供里程计使用 --- */
    float accel_real_body_mps2[3];   /* 机体系线性加速度，单位 m/s^2 */

    /* --- 水平系线性加速度（去倾斜），供 EKF 使用 --- */
    float accel_level_mps2[3];       /* 水平系线性加速度，+X 机头方向，+Y 右侧，+Z Down，单位 m/s^2 */

    /* --- Down 方向加速度输出 --- */
    float accel_down_for_ekf_mps2;   /* 给 EKF 用的 Down 加速度，单位 m/s^2 */
    float accel_down_for_output_mps2;/* 给显示/日志用的 Down 加速度，单位 m/s^2 */

    /* --- 垂直方向积分（Up 为正），可用于简易高度估计 --- */
    float vel_up_mps;                /* 垂直速度，Up 为正，单位 m/s */
    float pos_up_m;                  /* 垂直位置，Up 为正，单位 m */

    /* --- 安装旋转矩阵：传感器坐标系 -> 机体坐标系 --- */
    float imu_to_body[3][3];         /* 旋转矩阵，单位矩阵时表示 IMU 与机体对齐 */
    bool imu_to_body_identity;       /* 快捷判断，避免每次做矩阵乘法 */

    /* --- 运行质量指标（用于 UI 显示和调试） --- */
    float accel_norm_mean_g;         /* |a| 的滑动均值，单位 g */
    float accel_norm_std_g;          /* |a| 的滑动标准差，单位 g */

    float gravity_mps2;              /* 当前重力标称值，通常 9.80665 m/s^2 */
    uint32_t invalid_sample_count;   /* 无效样本累计计数 */
    uint8_t realtime_sample_valid;   /* 1=当前帧数据有效 */
} AccelCalibration_t;

/* 校准参数快照，用于 Load/Get/Flash 之间传递 */
typedef struct
{
    float accel_bias_g[3];           /* 偏置，单位 g */
    float accel_scale[3];            /* 对角缩放因子 */
    float accel_corr_matrix[3][3];   /* 完整校正矩阵 */
    uint8_t use_full_matrix;         /* 1=完整矩阵，0=仅对角 */
    float imu_to_body[3][3];         /* 安装旋转矩阵 */
    float gravity_mps2;              /* 重力标称值 */
} AccelCalibrationParams_t;

/* ======================== Flash 持久化参数 ======================== */
#define IMU_CALIB_FLASH_MAGIC                (0x43414C49UL) /* 校准数据魔数 "CALI" */
#define IMU_CALIB_FLASH_VERSION_V1           (1U)           /* 旧版布局（仅有对角缩放） */
#define IMU_CALIB_FLASH_VERSION              (2U)           /* 当前布局（支持完整校正矩阵） */
#define IMU_CALIB_FLASH_PAGE                 (95U)          /* 存储页号 */

/* ======================== 校准模式枚举 ======================== */
#define IMU_CALIB_STATUS_MODE_IDLE           (0U)           /* 空闲 */
#define IMU_CALIB_STATUS_MODE_GYRO           (1U)           /* 陀螺仪静态校准 */
#define IMU_CALIB_STATUS_MODE_ACC6           (2U)           /* 六面加速度校准 */
#define IMU_CALIB_STATUS_MODE_ALL            (3U)           /* 先陀螺后六面 */
#define IMU_CALIB_STATUS_MODE_ELLIP          (4U)           /* 自动椭球加速度校准 */
#define IMU_CALIB_STATUS_MODE_ELLIP_MANUAL   (5U)           /* 手动椭球加速度校准 */

/* 手动椭球校准子状态 */
#define IMU_CALIB_MANUAL_SUBSTATE_NONE       (0U)           /* 未开始 */
#define IMU_CALIB_MANUAL_SUBSTATE_READY      (1U)           /* 准备态，等待用户触发采集 */
#define IMU_CALIB_MANUAL_SUBSTATE_WAIT_STATIC (2U)          /* 等待静止通过 */
#define IMU_CALIB_MANUAL_SUBSTATE_COLLECTING (3U)           /* 正在采集当前姿态点 */
#define IMU_CALIB_MANUAL_SUBSTATE_SOLVING    (4U)           /* 已停止采点，正在求解 */

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    float gyro_bias_dps[3];
    float accel_bias_g[3];
    float accel_corr_matrix[3][3];
    float imu_to_body[3][3];

    uint32_t reserved[2];
} IMUCalibBlob_t;

typedef struct
{
    uint8_t busy;              /* 当前是否正在执行校准 */
    uint8_t mode;              /* 当前校准模式 */
    uint8_t pose_count;        /* 加速度校准已完成的姿态点数量 */
    uint8_t substate;          /* 手动校准子状态 */
    uint32_t progress_percent; /* 当前校准进度百分比 */
    uint32_t current_samples;  /* 当前子阶段累计样本数 */
    uint32_t target_samples;   /* 当前子阶段目标样本数 */
} IMUCalibStatus_t;

typedef struct
{
    uint8_t valid;                 /* Flash 中是否存在有效校准数据 */
    uint8_t use_full_matrix;       /* 1 表示使用完整 3x3 校正矩阵 */
    uint16_t version;              /* Flash 中校准数据版本 */
    float gyro_bias_dps[3];        /* 陀螺仪零偏，单位 dps */
    float accel_bias_g[3];         /* 加速度偏置，单位 g */
    float accel_corr_matrix[3][3]; /* 加速度 3x3 校正矩阵 */
    float imu_to_body[3][3];       /* IMU 到机体的安装旋转矩阵 */
} IMUCalibFlashInfo_t;

typedef void (*IMUCalibTextSink_t)(const char *text);

/* ======================== 全局对象 ======================== */
extern AccelCalibration_t g_accel_calibration; /* 全局加速度校准运行态，odometer/EKF 读取此对象 */

/* ======================== 加速度校准 API ======================== */
void AccelCalibration_Init(void);                          /* 初始化模块，尝试从 Flash 加载校准参数 */
void AccelCalibration_Reset(void);                         /* 重置运行态（保留校准参数不变） */
bool AccelCalibration_Start(void);                         /* 启动静止自动标定（需静止放置） */
void AccelCalibration_ApplySensorCorrection(float *ax, float *ay, float *az);
                                                           /* 对传感器坐标系原始加速度做安装旋转 + 偏置/缩放补偿，
                                                              输入输出单位 g，由 IMU_Update_1kHz 调用 */
void AccelCalibration_Update_1000HZ(void);                 /* 1kHz 更新：去重力、投影水平系、积分垂直速度 */

void AccelCalibration_SetImuToBodyMatrix(const float matrix[3][3]);     /* 直接设置 IMU->机体旋转矩阵 */
void AccelCalibration_SetImuToBodyEulerDeg(float roll_deg, float pitch_deg, float yaw_deg);
                                                           /* 按欧拉角（度）设置安装旋转 */

/* ======================== 状态查询 API ======================== */
bool AccelCalibration_IsCalibrated(void);                  /* 1=已有有效校准参数 */
uint8_t AccelCalibration_IsRealtimeDataValid(void);        /* 1=当前帧传感器数据有效 */
float AccelCalibration_GetGravityMps2(void);               /* 返回重力标称值，单位 m/s^2 */

/* ======================== 垂直方向 API（Up 为正） ======================== */
float AccelCalibration_GetVerticalAccelUpMps2(void);       /* Up 方向线性加速度，单位 m/s^2 */
float AccelCalibration_GetVerticalVelocityUpMps(void);     /* Up 方向速度（积分），单位 m/s */
float AccelCalibration_GetVerticalPositionUpM(void);       /* Up 方向位置（二次积分），单位 m */

/* ======================== Down 方向 API ======================== */
float AccelCalibration_GetAccelDownMps2(void);             /* Down 方向加速度，单位 m/s^2 */
float AccelCalibration_GetAccelDownForEkfMps2(void);       /* 给 EKF 用的 Down 加速度（滤波后） */
float AccelCalibration_GetAccelDownForOutputMps2(void);    /* 给输出/显示用的 Down 加速度（更平滑） */

/* ======================== 机体系数据 API ======================== */
void AccelCalibration_GetBodyAccelMps2(float *ax, float *ay, float *az);
                                                           /* 机体系线性加速度（去重力），单位 m/s^2 */
void AccelCalibration_GetBodyGyroDps(float *gx, float *gy, float *gz);
                                                           /* 机体系角速度（已去零偏），单位 dps */
void AccelCalibration_GetCorrectedSpecificForceG(float *ax_g, float *ay_g, float *az_g);
                                                           /* 机体系校正后比力，单位 g（含重力，静止约 -1g Down） */

/* ======================== 水平系数据 API ======================== */
void AccelCalibration_GetLevelAccelMps2(float *ax_level, float *ay_level, float *az_level);
                                                           /* 水平系线性加速度（去倾斜+重力），单位 m/s^2 */
void AccelCalibration_GetBodyLevelAccelNoYawMps2(float *ax_forward, float *ay_right);
void AccelCalibration_GetHorizontalAccelMps2(float *ax_h, float *ay_h);
                                                           /* 水平面 X/Y 线性加速度，单位 m/s^2 */
void AccelCalibration_RotateImuToBody(const float vec_sensor[3], float vec_body[3]);
                                                           /* 传感器坐标系 -> 机体坐标系向量旋转 */

/* ======================== 参数加载/读取 ======================== */
bool AccelCalibration_LoadParams(const AccelCalibrationParams_t *params); /* 加载校准参数到运行态 */
void AccelCalibration_GetParams(AccelCalibrationParams_t *params);       /* 读取当前校准参数 */

/* ======================== IMU 校准状态机 API ======================== */
void IMUCalib_Init(void);                                  /* 初始化校准子系统，尝试从 Flash 恢复 */
void IMUCalib_Update_1000HZ(void);                         /* 1kHz 状态机更新，由 IMU_Update 调用 */
uint8_t IMUCalib_LoadFromFlashAndApply(void);              /* 从 Flash 加载并应用 */
uint8_t IMUCalib_SaveCurrentToFlash(void);                 /* 保存当前参数到 Flash */
uint8_t IMUCalib_ClearFlash(void);                         /* 擦除 Flash 中的校准数据 */
uint8_t IMUCalib_IsBusy(void);                             /* 1=校准正在进行 */
void IMUCalib_SetTextSink(IMUCalibTextSink_t sink);        /* 设置文本输出回调（WiFi 串口等） */
uint8_t IMUCalib_ReadFlashInfo(IMUCalibFlashInfo_t *info); /* 读取 Flash 校准信息 */

/* ======================== 校准启动命令 ======================== */
uint8_t IMUCalib_StartGyro(void);                          /* 启动陀螺仪静态校准（需静止） */
uint8_t IMUCalib_StartAccel(void);                         /* 启动自动椭球加速度校准 */
uint8_t IMUCalib_StartAccelManual(void);                   /* 启动手动椭球加速度校准 */
uint8_t IMUCalib_ManualCollect(void);                      /* 手动模式：触发一次姿态点采集 */
uint8_t IMUCalib_ManualStop(void);                         /* 手动模式：停止采点，开始求解 */
void IMUCalib_GetStatus(IMUCalibStatus_t *status);         /* 读取当前校准状态 */
void IMUCalib_CommandPoll(void);                           /* 轮询调试串口校准命令 */

#ifdef __cplusplus
}
#endif

#endif /* ACCEL_CALIBRATION_H_ */
