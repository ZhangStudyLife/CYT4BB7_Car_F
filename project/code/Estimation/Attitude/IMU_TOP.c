/********************************************************************
 * 文件名  : IMU_TOP.c
 * 模块    : IMU 顶层调度实现
 * 位置    : Estimation/Attitude
 * 数据流  :
 *   ICM42688_Get_Data()
 *     -> s_imu_raw_calib_1000hz（原始快照，给校准用）
 *     -> AccelCalibration_ApplySensorCorrection（安装旋转+偏置补偿）
 *     -> IMUFilter_Update（抗混叠->陷波->低通，输出 g_imufilter_1000hz）
 *     -> IMUCalib_Update / AccelCalibration_Update（校准状态机）
 *     -> MahonyAhrs_Update（姿态四元数）
 *     -> MahonyAhrs_GetEulerDegrees（输出欧拉角到 g_euler）
 ********************************************************************/

#include "IMU_TOP.h"
#define IMU_DEG_TO_RAD (0.017453292519943295f)
#define IMU_RUNTIME_FAULT_THRESHOLD  (5U)
#define IMU_RUNTIME_RECOVERY_SAMPLES (20U)

/* ======================== IMU 全局状态 ======================== */
MahonyAhrs_t g_mahony_ahrs;       /* Mahony 姿态解算器状态 */
MahonyAhrs_Euler_t g_euler;       /* 当前姿态欧拉角（单位: 度） */
uint8 g_imu_ready = 0U;           /* 1=IMU 初始化与暖机完成；健康状态见 IMU_RuntimeHealthy */
static imudata_t s_imu_raw_calib_1000hz = {0}; /* 当前帧原始 IMU 快照，供校准链读取 */
static uint8 s_imu_initializing = 0U;           /* 1=正在初始化中（暖机阶段） */
/* ======================== 本地工具函数 ======================== */
volatile uint32 g_imu_update_count = 0U;
volatile uint32 g_imu_read_error_count = 0U;
volatile uint32 g_imu_processing_error_count = 0U;
volatile uint32 g_imu_calibration_sample_drop_count = 0U;
volatile uint16 g_imu_consecutive_read_error_count = 0U;
volatile uint16 g_imu_max_consecutive_read_error_count = 0U;
volatile uint8 g_imu_runtime_fault = 0U;
volatile uint8 g_imu_startup_selftest_fault = 0U;

static volatile uint32 s_imu_raw_sequence = 0U;
static uint32 s_imu_calibration_serviced_sequence = 0U;
static uint16 s_imu_recovery_success_count = 0U;
static volatile uint32 s_imu_snapshot_sequence = 0U;
static imu_realtime_snapshot_t s_imu_realtime_snapshot = {0};

static uint8 IMU_CopyRawSample(imudata_t *sample, uint32 *sequence);

static void IMU_PublishRawSample(void)
{
    s_imu_raw_sequence++;
    __DMB();
    s_imu_raw_calib_1000hz.gyrox = ICM42688.gyro_x;
    s_imu_raw_calib_1000hz.gyroy = ICM42688.gyro_y;
    s_imu_raw_calib_1000hz.gyroz = ICM42688.gyro_z;
    s_imu_raw_calib_1000hz.accx = ICM42688.acc_x;
    s_imu_raw_calib_1000hz.accy = ICM42688.acc_y;
    s_imu_raw_calib_1000hz.accz = ICM42688.acc_z;
    __DMB();
    s_imu_raw_sequence++;
}

static void IMU_PublishRealtimeSnapshot(void)
{
    s_imu_snapshot_sequence++;
    __DMB();
    s_imu_realtime_snapshot.euler = g_euler;
    s_imu_realtime_snapshot.filtered = g_imufilter_1000hz;
    s_imu_realtime_snapshot.sample_count = g_imu_update_count;
    s_imu_realtime_snapshot.healthy =
        ((g_imu_ready != 0U) &&
         (g_imu_runtime_fault == 0U) &&
         (g_imu_startup_selftest_fault == 0U)) ? 1U : 0U;
    __DMB();
    s_imu_snapshot_sequence++;
}

static void IMU_RecordFrameFailure(void)
{
    s_imu_recovery_success_count = 0U;
    if (g_imu_consecutive_read_error_count < 0xFFFFU)
    {
        g_imu_consecutive_read_error_count++;
    }
    if (g_imu_consecutive_read_error_count >
        g_imu_max_consecutive_read_error_count)
    {
        g_imu_max_consecutive_read_error_count =
            g_imu_consecutive_read_error_count;
    }
    if (g_imu_consecutive_read_error_count >= IMU_RUNTIME_FAULT_THRESHOLD)
    {
        g_imu_runtime_fault = 1U;
    }
}

static void IMU_RecordReadFailure(void)
{
    g_imu_read_error_count++;
    IMU_RecordFrameFailure();
}

static void IMU_RecordProcessingFailure(void)
{
    g_imu_processing_error_count++;
    IMU_RecordFrameFailure();
    /* 非有限值意味着计算链已失效，不等待连续 5 帧，立即禁止输出。 */
    g_imu_runtime_fault = 1U;
}

static void IMU_RecordReadSuccess(void)
{
    g_imu_consecutive_read_error_count = 0U;
    if (g_imu_runtime_fault != 0U)
    {
        if (s_imu_recovery_success_count < IMU_RUNTIME_RECOVERY_SAMPLES)
        {
            s_imu_recovery_success_count++;
        }
        if (s_imu_recovery_success_count >= IMU_RUNTIME_RECOVERY_SAMPLES)
        {
            g_imu_runtime_fault = 0U;
            s_imu_recovery_success_count = 0U;
        }
    }
    else
    {
        s_imu_recovery_success_count = 0U;
    }
}

static uint8 IMU_IsFiniteFloat(float value)
{
    if (value != value)
    {
        return 0U;
    }

    if ((value > 1000000.0f) || (value < -1000000.0f))
    {
        return 0U;
    }

    return 1U;
}

/*
 * 函数功能: 读取当前 1kHz 周期内供校准使用的原始 IMU 物理量快照。
 * 输入参数:
 *   gx, gy, gz - 输出陀螺仪原始角速度，单位 dps；已做符号映射并扣除陀螺仪零偏
 *   ax, ay, az - 输出加速度计原始比力，单位 g；仅做量程换算与符号映射
 * 输出参数/返回值:
 *   通过指针返回当前帧原始 IMU 快照；空指针会被忽略
 */
void IMU_GetRawSampleForCalibration(float *gx, float *gy, float *gz,
                                    float *ax, float *ay, float *az)
{
    imudata_t sample;
    uint32 sequence;

    if (IMU_CopyRawSample(&sample, &sequence) == 0U)
    {
        return;
    }

    if (gx != NULL)
    {
        *gx = sample.gyrox;
    }
    if (gy != NULL)
    {
        *gy = sample.gyroy;
    }
    if (gz != NULL)
    {
        *gz = sample.gyroz;
    }
    if (ax != NULL)
    {
        *ax = sample.accx;
    }
    if (ay != NULL)
    {
        *ay = sample.accy;
    }
    if (az != NULL)
    {
        *az = sample.accz;
    }
}

uint8 IMU_GetRealtimeSnapshot(imu_realtime_snapshot_t *snapshot)
{
    uint32 sequence_after;
    uint32 sequence_before;

    if (snapshot == NULL)
    {
        return 0U;
    }

    for (;;)
    {
        sequence_before = s_imu_snapshot_sequence;
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }
        __DMB();
        *snapshot = s_imu_realtime_snapshot;
        __DMB();
        sequence_after = s_imu_snapshot_sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U))
        {
            break;
        }
    }

    snapshot->healthy = IMU_RuntimeHealthy();
    return 1U;
}

uint8 IMU_RuntimeHealthy(void)
{
    return ((g_imu_ready != 0U) &&
            (g_imu_runtime_fault == 0U) &&
            (g_imu_startup_selftest_fault == 0U)) ? 1U : 0U;
}

/*
 * 上电自检：读取短窗口（200 帧）数据做基础健康检查
 * 检查项：
 *   1) 传感器数据是否有限（无 NaN/异常大值）
 *   2) 静止时陀螺仪平均模长是否 < 8 dps
 *   3) 加速度模长均值是否在 0.75~1.25g 范围内
 * 返回值：1=通过（可安全使用），0=失败（传感器异常，死循环）
 * 调用要求：调用时需静止放置
 */
static uint8 IMU_Startup_SelfCheck(void)
{
    uint32 i;
    float gyro_abs_sum = 0.0f;
    float acc_mag_sum = 0.0f;

    for (i = 0U; i < IMU_SELFTEST_SAMPLE_COUNT; i++)
    {
        float gyro_abs;
        float acc_mag;

        if (ICM42688_Get_Data() == 0U)
        {
            return 0U;
        }

        if ((0U == IMU_IsFiniteFloat(ICM42688.gyro_x)) ||
            (0U == IMU_IsFiniteFloat(ICM42688.gyro_y)) ||
            (0U == IMU_IsFiniteFloat(ICM42688.gyro_z)) ||
            (0U == IMU_IsFiniteFloat(ICM42688.acc_x)) ||
            (0U == IMU_IsFiniteFloat(ICM42688.acc_y)) ||
            (0U == IMU_IsFiniteFloat(ICM42688.acc_z)))
        {
            return 0U;
        }

        gyro_abs = sqrtf(ICM42688.gyro_x * ICM42688.gyro_x +
                         ICM42688.gyro_y * ICM42688.gyro_y +
                         ICM42688.gyro_z * ICM42688.gyro_z);

        acc_mag = sqrtf(ICM42688.acc_x * ICM42688.acc_x +
                       ICM42688.acc_y * ICM42688.acc_y +
                       ICM42688.acc_z * ICM42688.acc_z);

        gyro_abs_sum += gyro_abs;
        acc_mag_sum += acc_mag;

        system_delay_us(ICM42688_SAMPLE_INTERVAL_US);
    }

    gyro_abs_sum /= (float)IMU_SELFTEST_SAMPLE_COUNT;
    acc_mag_sum /= (float)IMU_SELFTEST_SAMPLE_COUNT;

    if (gyro_abs_sum > IMU_SELFTEST_GYRO_MEAN_MAX_DPS)
    {
        return 0U;
    }

    if ((acc_mag_sum < IMU_SELFTEST_ACC_MIN_G) || (acc_mag_sum > IMU_SELFTEST_ACC_MAX_G))
    {
        return 0U;
    }

    return 1U;
}

/* ======================== IMU 初始化 ======================== */
/* 调用后 IMU 进入就绪状态（g_imu_ready=1），可以开始 1kHz 更新 */
void IMU_Init_All(void)
{
    uint32 i;
    uint8 startup_ok;

    g_imu_ready = 0U;
    s_imu_initializing = 1U;
    s_imu_raw_calib_1000hz = (imudata_t){0};
    g_imu_update_count = 0U;
    g_imu_read_error_count = 0U;
    g_imu_processing_error_count = 0U;
    g_imu_calibration_sample_drop_count = 0U;
    g_imu_consecutive_read_error_count = 0U;
    g_imu_max_consecutive_read_error_count = 0U;
    g_imu_runtime_fault = 0U;
    g_imu_startup_selftest_fault = 0U;
    s_imu_raw_sequence = 0U;
    s_imu_calibration_serviced_sequence = 0U;
    s_imu_recovery_success_count = 0U;
    s_imu_snapshot_sequence = 0U;
    s_imu_realtime_snapshot = (imu_realtime_snapshot_t){0};

    /* 步骤1: 初始化 ICM42688 驱动（SPI 通信、量程配置等） */
    ICM42688_Init(&ICM42688_CONFIG);

    /* 步骤2: 上电自检（必须静止放置，失败则死循环） */
    startup_ok = IMU_Startup_SelfCheck();
    g_imu_startup_selftest_fault = (startup_ok == 0U) ? 1U : 0U;
    if (startup_ok == 0U)
    {
        printf("IMU startup self-check failed.\r\n");
    }

    /* 步骤3: 初始化上层滤波与姿态解算器 */
    IMUFilter_Init();
    MahonyAhrs_Init(&g_mahony_ahrs);
    g_euler.roll = 0.0f;
    g_euler.pitch = 0.0f;
    g_euler.yaw = 0.0f;
    g_euler.sin_roll = 0.0f;
    g_euler.cos_roll = 1.0f;
    g_euler.sin_pitch = 0.0f;
    g_euler.cos_pitch = 1.0f;

    /* 步骤4: 暖机，丢弃前 1000 帧（约 1s），让滤波器内部状态收敛 */
    for (i = 0U; i < IMU_WARMUP_DISCARD_SAMPLES; i++)
    {
        IMU_Update_1000HZ();
        system_delay_us(ICM42688_SAMPLE_INTERVAL_US);
    }

    g_imu_ready = 1U;
    s_imu_initializing = 0U;
    s_imu_calibration_serviced_sequence = s_imu_raw_sequence;
    IMU_PublishRealtimeSnapshot();
}

/* 1kHz 主更新，由定时器中断或主循环调用 */
uint8 IMU_Update_1000HZ(void)
{
    const float dt_s = IMU_UPDATE_DT_SEC;
    float ahrs_gx;
    float ahrs_gy;
    float ahrs_gz;
    float ahrs_ax;
    float ahrs_ay;
    float ahrs_az;
    MahonyAhrs_Euler_t new_euler;
    if ((0U == g_imu_ready) && (0U == s_imu_initializing))
    {
        return 0U;
    }
    /* 1. 原始数据 (gyro已去零偏) */
    if (ICM42688_Get_Data() == 0U)
    {
        IMU_RecordReadFailure();
        return 0U;
    }

    /* 缓存当前帧原始 IMU 物理量，供校准流程直接读取 */
    IMU_PublishRawSample();

    /* 2. 加速度计校准前置（传感器坐标系） */
    float cal_ax = ICM42688.acc_x;
    float cal_ay = ICM42688.acc_y;
    float cal_az = ICM42688.acc_z;
    AccelCalibration_ApplySensorCorrection(&cal_ax, &cal_ay, &cal_az);

    /* 3. 校准后数据送入滤波器 */
    IMUFilter_Update(ICM42688.gyro_x, ICM42688.gyro_y, ICM42688.gyro_z,
                     cal_ax, cal_ay, cal_az);

    /* 4. 仅保留固定耗时的实时加速度处理；校准状态机由主循环服务。 */
    AccelCalibration_Update_1000HZ();

    /* 5. 姿态解算统一使用 1kHz 滤波 IMU 输出 */
    ahrs_gx = g_imufilter_1000hz.gyrox;
    ahrs_gy = g_imufilter_1000hz.gyroy;
    ahrs_gz = g_imufilter_1000hz.gyroz;
    ahrs_ax = g_imufilter_1000hz.accx;
    ahrs_ay = g_imufilter_1000hz.accy;
    ahrs_az = g_imufilter_1000hz.accz;

    if ((0U == IMU_IsFiniteFloat(ahrs_gx)) ||
        (0U == IMU_IsFiniteFloat(ahrs_gy)) ||
        (0U == IMU_IsFiniteFloat(ahrs_gz)) ||
        (0U == IMU_IsFiniteFloat(ahrs_ax)) ||
        (0U == IMU_IsFiniteFloat(ahrs_ay)) ||
        (0U == IMU_IsFiniteFloat(ahrs_az)))
    {
        IMU_RecordProcessingFailure();
        return 0U;
    }

    MahonyAhrs_Update(
        &g_mahony_ahrs,
        ahrs_gx, ahrs_gy, ahrs_gz,
        ahrs_ax, ahrs_ay, ahrs_az,
        dt_s);

    /* 步骤5: 从四元数提取欧拉角（单位: 度），缓存到 g_euler 供全局使用 */
    new_euler = MahonyAhrs_GetEulerDegrees(&g_mahony_ahrs);
    if ((0U == IMU_IsFiniteFloat(new_euler.roll)) ||
        (0U == IMU_IsFiniteFloat(new_euler.pitch)) ||
        (0U == IMU_IsFiniteFloat(new_euler.yaw)) ||
        (0U == IMU_IsFiniteFloat(new_euler.sin_roll)) ||
        (0U == IMU_IsFiniteFloat(new_euler.cos_roll)) ||
        (0U == IMU_IsFiniteFloat(new_euler.sin_pitch)) ||
        (0U == IMU_IsFiniteFloat(new_euler.cos_pitch)))
    {
        IMU_RecordProcessingFailure();
        return 0U;
    }
    g_euler = new_euler;
    g_imu_update_count++;
    IMU_RecordReadSuccess();
    IMU_PublishRealtimeSnapshot();

    // wifi_justfloat(tick_1000us_cnt,ICM42688.gyro_x, ICM42688.gyro_y, ICM42688.gyro_z,
    // 				ICM42688.acc_x, ICM42688.acc_y, ICM42688.acc_z,
    // 				g_imufilter_1000hz.gyrox, g_imufilter_1000hz.gyroy, g_imufilter_1000hz.gyroz,
    // 				g_imufilter_1000hz.accx, g_imufilter_1000hz.accy, g_imufilter_1000hz.accz,
    // 				g_euler.roll, g_euler.pitch, g_euler.yaw);

    return 1U;
}

static uint8 IMU_CopyRawSample(imudata_t *sample, uint32 *sequence)
{
    uint32 sequence_after;
    uint32 sequence_before;

    if ((sample == NULL) || (sequence == NULL))
    {
        return 0U;
    }

    for (;;)
    {
        sequence_before = s_imu_raw_sequence;
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }
        __DMB();
        *sample = s_imu_raw_calib_1000hz;
        __DMB();
        sequence_after = s_imu_raw_sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U))
        {
            *sequence = sequence_after;
            return 1U;
        }
    }
}

void IMU_ServicePoll(void)
{
    imudata_t sample;
    uint32 published_sequence;
    uint32 sequence_delta;

    if (g_imu_ready == 0U)
    {
        return;
    }

    if (IMU_CopyRawSample(&sample, &published_sequence) == 0U)
    {
        return;
    }

    if (published_sequence == s_imu_calibration_serviced_sequence)
    {
        return;
    }

    sequence_delta = published_sequence - s_imu_calibration_serviced_sequence;
    if (sequence_delta > 2U)
    {
        g_imu_calibration_sample_drop_count += (sequence_delta / 2U) - 1U;
    }

    IMUCalib_Update_1000HZ(&sample);
    /* 只确认进入本次处理前捕获的序号；处理中到达的新帧留给下次轮询。 */
    s_imu_calibration_serviced_sequence = published_sequence;
}

/* 重置 yaw 为 0，保留当前 roll/pitch 不变
 * 安全影响：改变航向基准，里程计的 yaw 投影也会基于新零点
 * 使用场景：启动时/手动校准航向 */
void IMU_ResetYaw(void)
{
    float roll_rad;
    float pitch_rad;
    float cr;
    float sr;
    float cp;
    float sp;

    roll_rad = IMU_IsFiniteFloat(g_euler.roll) ? (g_euler.roll * IMU_DEG_TO_RAD) : 0.0f;
    pitch_rad = IMU_IsFiniteFloat(g_euler.pitch) ? (g_euler.pitch * IMU_DEG_TO_RAD) : 0.0f;

    cr = cosf(roll_rad * 0.5f);
    sr = sinf(roll_rad * 0.5f);
    cp = cosf(pitch_rad * 0.5f);
    sp = sinf(pitch_rad * 0.5f);

    g_mahony_ahrs.q0 = cr * cp;
    g_mahony_ahrs.q1 = sr * cp;
    g_mahony_ahrs.q2 = cr * sp;
    g_mahony_ahrs.q3 = -sr * sp;
    g_euler = MahonyAhrs_GetEulerDegrees(&g_mahony_ahrs);
    IMU_PublishRealtimeSnapshot();
}

uint8 IMU_Is_Ready(void)
{
    return IMU_RuntimeHealthy();
}
