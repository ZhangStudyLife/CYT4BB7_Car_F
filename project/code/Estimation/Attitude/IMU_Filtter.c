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
/********************************************************************
 * 文件名 : IMU_Filtter.c
 * 模块   : IMU 输入链路滤波实现
 * 链路   :
 *   Gyro : AntiAlias(250Hz LPF) -> Notch(120Hz) -> Notch(160Hz) -> Notch(320Hz) -> LPF(35Hz)
 *   Acc  : Median3 -> ShockLimit -> LPF(8Hz) -> LPF(8Hz)
 ********************************************************************/

#include "IMU_Filtter.h"

#define IMU_FILTER_PI  (3.14159265359f)

imudata_t g_imufilter_1000hz;
imudata_t g_imudata_500hz;
imudata_t g_imudata_250hz;

static struct
{
    IMUBiquad_t gyro_anti_alias_lpf[IMU_AXIS_NUM];
    IMUBiquad_t gyro_notch0[IMU_AXIS_NUM];
    IMUBiquad_t gyro_notch1[IMU_AXIS_NUM];
    IMUBiquad_t gyro_notch2[IMU_AXIS_NUM];
    IMUBiquad_t gyro_lpf[IMU_AXIS_NUM];

    IMUBiquad_t accel_lpf0[IMU_AXIS_NUM];
    IMUBiquad_t accel_lpf1[IMU_AXIS_NUM];
    float accel_hist[IMU_AXIS_NUM][3];
    uint8_t accel_hist_count;

    uint8_t initialized;
} s_filt;

static float IMUBiquad_Apply(IMUBiquad_t *f, float in)
{
    float out = f->b0 * in + f->d1;

    f->d1 = f->b1 * in - f->a1 * out + f->d2;
    f->d2 = f->b2 * in - f->a2 * out;

    return out;
}

static float IMU_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static float IMU_Median3(float a, float b, float c)
{
    if (a > b)
    {
        float tmp = a;
        a = b;
        b = tmp;
    }

    if (b > c)
    {
        float tmp = b;
        b = c;
        c = tmp;
    }

    if (a > b)
    {
        b = a;
    }

    return b;
}

static void IMUBiquad_InitLPF(IMUBiquad_t *f, float fs, float fc)
{
    float w0;
    float sw0;
    float cw0;
    float alpha;
    float a0;

    w0 = 2.0f * IMU_FILTER_PI * fc / fs;
    sw0 = sinf(w0);
    cw0 = cosf(w0);
    alpha = sw0 / (2.0f * 0.70710678f);
    a0 = 1.0f + alpha;

    f->b0 = (1.0f - cw0) * 0.5f / a0;
    f->b1 = (1.0f - cw0) / a0;
    f->b2 = (1.0f - cw0) * 0.5f / a0;
    f->a1 = (-2.0f * cw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

static void IMUBiquad_InitNotch(IMUBiquad_t *f, float fs, float fc, float q)
{
    float w0;
    float sw0;
    float cw0;
    float alpha;
    float a0;

    w0 = 2.0f * IMU_FILTER_PI * fc / fs;
    sw0 = sinf(w0);
    cw0 = cosf(w0);
    alpha = sw0 / (2.0f * q);
    a0 = 1.0f + alpha;

    f->b0 = 1.0f / a0;
    f->b1 = (-2.0f * cw0) / a0;
    f->b2 = 1.0f / a0;
    f->a1 = (-2.0f * cw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

void IMUFilter_Init(void)
{
    uint8_t axis;

    for (axis = 0U; axis < IMU_AXIS_NUM; axis++)
    {
        IMUBiquad_InitLPF(&s_filt.gyro_anti_alias_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_ANTI_ALIAS_LPF_HZ);
        IMUBiquad_InitNotch(&s_filt.gyro_notch0[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH0_HZ, IMU_NOTCH0_Q);
        IMUBiquad_InitNotch(&s_filt.gyro_notch1[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH1_HZ, IMU_NOTCH1_Q);
        IMUBiquad_InitNotch(&s_filt.gyro_notch2[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH2_HZ, IMU_NOTCH2_Q);
        IMUBiquad_InitLPF(&s_filt.gyro_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_LPF_HZ);

        IMUBiquad_InitLPF(&s_filt.accel_lpf0[axis], IMU_SAMPLE_RATE_HZ, IMU_ACCEL_LPF_HZ);
        IMUBiquad_InitLPF(&s_filt.accel_lpf1[axis], IMU_SAMPLE_RATE_HZ, IMU_ACCEL_LPF_HZ);
        s_filt.accel_hist[axis][0] = 0.0f;
        s_filt.accel_hist[axis][1] = 0.0f;
        s_filt.accel_hist[axis][2] = 0.0f;
    }

    s_filt.initialized = 0U;
    s_filt.accel_hist_count = 0U;
    g_imufilter_1000hz = (imudata_t){0};
    g_imudata_500hz = (imudata_t){0};
    g_imudata_250hz = (imudata_t){0};
}

void IMUFilter_Update(float gx, float gy, float gz,
                      float ax, float ay, float az)
{
    float gyro_in[IMU_AXIS_NUM];
    float accel_in[IMU_AXIS_NUM];
    float gyro_out[IMU_AXIS_NUM];
    float accel_out[IMU_AXIS_NUM];
    uint8_t axis;

    gyro_in[0] = gx;
    gyro_in[1] = gy;
    gyro_in[2] = gz;
    accel_in[0] = ax;
    accel_in[1] = ay;
    accel_in[2] = az;

    if (0U == s_filt.initialized)
    {
        for (axis = 0U; axis < IMU_AXIS_NUM; axis++)
        {
            s_filt.accel_hist[axis][0] = accel_in[axis];
            s_filt.accel_hist[axis][1] = accel_in[axis];
            s_filt.accel_hist[axis][2] = accel_in[axis];
        }

        g_imufilter_1000hz = (imudata_t){gx, gy, gz, ax, ay, az};
        g_imudata_500hz = g_imufilter_1000hz;
        g_imudata_250hz = g_imufilter_1000hz;
        s_filt.initialized = 1U;
        s_filt.accel_hist_count = 1U;
        return;
    }

    for (axis = 0U; axis < IMU_AXIS_NUM; axis++)
    {
        float gyro_stage_aa;
        float gyro_stage0;
        float gyro_stage1;
        float gyro_stage2;
        float accel_median;
        float accel_limited;
        float accel_stage0;

        gyro_stage_aa = IMUBiquad_Apply(&s_filt.gyro_anti_alias_lpf[axis], gyro_in[axis]);
        gyro_stage0 = IMUBiquad_Apply(&s_filt.gyro_notch0[axis], gyro_stage_aa);
        gyro_stage1 = IMUBiquad_Apply(&s_filt.gyro_notch1[axis], gyro_stage0);
        gyro_stage2 = IMUBiquad_Apply(&s_filt.gyro_notch2[axis], gyro_stage1);
        gyro_out[axis] = IMUBiquad_Apply(&s_filt.gyro_lpf[axis], gyro_stage2);

        s_filt.accel_hist[axis][2] = s_filt.accel_hist[axis][1];
        s_filt.accel_hist[axis][1] = s_filt.accel_hist[axis][0];
        s_filt.accel_hist[axis][0] = accel_in[axis];

        if (s_filt.accel_hist_count < 3U)
        {
            accel_median = accel_in[axis];
        }
        else
        {
            accel_median = IMU_Median3(s_filt.accel_hist[axis][0],
                                       s_filt.accel_hist[axis][1],
                                       s_filt.accel_hist[axis][2]);
        }

        accel_limited = IMU_ClampFloat(accel_median, -IMU_ACCEL_SHOCK_AXIS_G, IMU_ACCEL_SHOCK_AXIS_G);
        accel_stage0 = IMUBiquad_Apply(&s_filt.accel_lpf0[axis], accel_limited);
        accel_out[axis] = IMUBiquad_Apply(&s_filt.accel_lpf1[axis], accel_stage0);
    }

    if (s_filt.accel_hist_count < 3U)
    {
        s_filt.accel_hist_count++;
    }

    g_imufilter_1000hz.gyrox = gyro_out[0];
    g_imufilter_1000hz.gyroy = gyro_out[1];
    g_imufilter_1000hz.gyroz = gyro_out[2];
    g_imufilter_1000hz.accx = accel_out[0];
    g_imufilter_1000hz.accy = accel_out[1];
    g_imufilter_1000hz.accz = accel_out[2];

    g_imudata_500hz = g_imufilter_1000hz;
    g_imudata_250hz = g_imufilter_1000hz;
}
