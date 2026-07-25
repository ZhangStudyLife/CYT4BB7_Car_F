#include "odometer.h"
#include "Estimation/Attitude/Accel_Calibration.h"

#include <math.h>

#define ODOMETER_DEG_TO_RAD (0.017453292519943295f)
#define ODOMETER_PI (3.14159265358979323846f)
#define ODOMETER_TWO_PI (6.28318530717958647692f)

odometer_data_t g_odometer = {0};

static float s_yaw_zero_rad;
static float s_accel_bias[ODOMETER_AXIS_NUM];
static float s_imu_vel[ODOMETER_AXIS_NUM];
static float s_encoder_vel[ODOMETER_AXIS_NUM];
static float s_prev_encoder_vel[ODOMETER_AXIS_NUM];
static uint8 s_yaw_ready;
static uint8 s_accel_bias_ready;
static uint8 s_slip_hold_ticks;

static float odometer_normalize_angle(float angle)
{
    if (!isfinite(angle))
    {
        return 0.0f;
    }

    angle = fmodf(angle, ODOMETER_TWO_PI);
    if (angle > ODOMETER_PI)
    {
        angle -= ODOMETER_TWO_PI;
    }
    else if (angle < -ODOMETER_PI)
    {
        angle += ODOMETER_TWO_PI;
    }

    return angle;
}

static float odometer_norm(const float value[ODOMETER_AXIS_NUM])
{
    return sqrtf((value[x] * value[x]) + (value[y] * value[y]));
}

static float odometer_yaw_delta_rad(void)
{
    float yaw_now_rad = g_euler.yaw * ODOMETER_DEG_TO_RAD;

    if (0U == s_yaw_ready)
    {
        s_yaw_zero_rad = yaw_now_rad;
        s_yaw_ready = 1U;
    }

    return odometer_normalize_angle(s_yaw_zero_rad - yaw_now_rad);
}

float odometer_get_heading_rad(void)
{
    return odometer_yaw_delta_rad();
}

static void odometer_body_to_horizontal(const float body[ODOMETER_AXIS_NUM],
                                        float horizontal[ODOMETER_AXIS_NUM])
{
    float yaw_delta_rad = odometer_yaw_delta_rad();
    float cos_yaw = cosf(yaw_delta_rad);
    float sin_yaw = sinf(yaw_delta_rad);

    horizontal[x] = (cos_yaw * body[x]) - (sin_yaw * body[y]);
    horizontal[y] = (sin_yaw * body[x]) + (cos_yaw * body[y]);
}

static void odometer_update_accel(void)
{
    float body_acc[ODOMETER_AXIS_NUM];
    float acc_forward;
    float acc_right;

    AccelCalibration_GetBodyLevelAccelNoYawMps2(&acc_forward, &acc_right);
    body_acc[x] = acc_right;
    body_acc[y] = acc_forward;
    odometer_body_to_horizontal(body_acc, g_odometer.acc);

    if (0U == s_accel_bias_ready)
    {
        s_accel_bias[x] = g_odometer.acc[x];
        s_accel_bias[y] = g_odometer.acc[y];
        s_accel_bias_ready = 1U;
    }

    g_odometer.acc[x] -= s_accel_bias[x];
    g_odometer.acc[y] -= s_accel_bias[y];
}

void odometer_init(void)
{
    odometer_reset();
}

static void odometer_reset_state(const float initial_position[ODOMETER_AXIS_NUM])
{
    g_odometer = (odometer_data_t){0};
    g_odometer.position[x] = initial_position[x];
    g_odometer.position[y] = initial_position[y];
    s_yaw_zero_rad = 0.0f;
    s_accel_bias[x] = 0.0f;
    s_accel_bias[y] = 0.0f;
    s_imu_vel[x] = 0.0f;
    s_imu_vel[y] = 0.0f;
    s_encoder_vel[x] = 0.0f;
    s_encoder_vel[y] = 0.0f;
    s_prev_encoder_vel[x] = 0.0f;
    s_prev_encoder_vel[y] = 0.0f;
    s_yaw_ready = 0U;
    s_accel_bias_ready = 0U;
    s_slip_hold_ticks = 0U;
}

void odometer_reset(void)
{
    beacon_config_data_t map_data;

    beacon_config_get_predata(&map_data);
    odometer_reset_state(map_data.initial_position);
}

/**
 * @brief 立即应用fixator生成的待修正位置。
 * @param 无。
 * @return 无。
 */
void odometer_apply_pending_fix(void)
{
#if (ODOMETER_BEACON_FIXATOR_ENABLE != 0U)
    float fixed_position[ODOMETER_AXIS_NUM];

    if(fixator_get_position_fix(fixed_position) != 0U)
    {
        g_odometer.position[x] = fixed_position[x];
        g_odometer.position[y] = fixed_position[y];
    }
#endif
}

void odometer_update_100HZ(void)
{
    float body_vel[ODOMETER_AXIS_NUM];
    float left_count = encoder_get_left_filtered_count();
    float right_count = encoder_get_right_filtered_count();
    float left_mps = 0.0f;
    float right_mps = 0.0f;

    /*
     * 两轮差速底盘没有车体横移自由度。
     * 左右编码器正值均表示车体前进，各自使用独立的每米脉冲标定值换算，
     * 再取平均值得到车体前向速度。航向变化由IMU yaw负责坐标变换。
     */
    if((isfinite(wheel_left_count_per_meter)) &&
       (wheel_left_count_per_meter > 0.0f))
    {
        left_mps =
            left_count / wheel_left_count_per_meter / ODOMETER_UPDATE_DT_S;
    }
    if((isfinite(wheel_right_count_per_meter)) &&
       (wheel_right_count_per_meter > 0.0f))
    {
        right_mps =
            right_count / wheel_right_count_per_meter / ODOMETER_UPDATE_DT_S;
    }

    body_vel[x] = 0.0f;
    body_vel[y] = (left_mps + right_mps) * 0.5f;

    g_odometer.body_vel[x] = body_vel[x];
    g_odometer.body_vel[y] = body_vel[y];
    odometer_body_to_horizontal(body_vel, s_encoder_vel);
    odometer_update_accel();

    if ((odometer_norm(s_encoder_vel) < ODOMETER_STATIC_ENCODER_SPEED_MPS) &&
        (odometer_norm(g_odometer.acc) < ODOMETER_STATIC_ACCEL_MPS2))
    {
        s_accel_bias[x] += ODOMETER_ACCEL_BIAS_ALPHA_STATIC * g_odometer.acc[x];
        s_accel_bias[y] += ODOMETER_ACCEL_BIAS_ALPHA_STATIC * g_odometer.acc[y];
        s_imu_vel[x] = 0.0f;
        s_imu_vel[y] = 0.0f;
        g_odometer.body_vel[x] = 0.0f;
        g_odometer.body_vel[y] = 0.0f;
        s_slip_hold_ticks = 0U;
    }
    else
    {
        float innovation[ODOMETER_AXIS_NUM];
        float enc_acc_x = (s_encoder_vel[x] - s_prev_encoder_vel[x]) / ODOMETER_UPDATE_DT_S;
        float enc_acc_y = (s_encoder_vel[y] - s_prev_encoder_vel[y]) / ODOMETER_UPDATE_DT_S;
        float diff_x = enc_acc_x - g_odometer.acc[x];
        float diff_y = enc_acc_y - g_odometer.acc[y];
        float diff_mag = sqrtf((diff_x * diff_x) + (diff_y * diff_y));
        float alpha;

        innovation[x] = s_encoder_vel[x] - s_imu_vel[x];
        innovation[y] = s_encoder_vel[y] - s_imu_vel[y];

        if ((odometer_norm(innovation) > ODOMETER_SLIP_INNOVATION_THRESH) &&
            (diff_mag > ODOMETER_SLIP_ACCEL_DIFF_THRESH))
        {
            s_slip_hold_ticks = ODOMETER_SLIP_HOLD_TICKS;
        }

        alpha = (s_slip_hold_ticks > 0U) ? ODOMETER_SLIP_BLEND_ALPHA : ODOMETER_ENCODER_BLEND_ALPHA;
        s_imu_vel[x] += alpha * (s_encoder_vel[x] - s_imu_vel[x]);
        s_imu_vel[y] += alpha * (s_encoder_vel[y] - s_imu_vel[y]);

        if (s_slip_hold_ticks > 0U)
        {
            s_slip_hold_ticks--;
        }
    }

    s_prev_encoder_vel[x] = s_encoder_vel[x];
    s_prev_encoder_vel[y] = s_encoder_vel[y];

    g_odometer.vel[x] = s_imu_vel[x];
    g_odometer.vel[y] = s_imu_vel[y];
    g_odometer.position[x] += g_odometer.vel[x] * ODOMETER_UPDATE_DT_S;
    g_odometer.position[y] += g_odometer.vel[y] * ODOMETER_UPDATE_DT_S;

    odometer_apply_pending_fix();

    wifi_core_Poll();

}

void odometer_update_1000HZ(void)
{
    odometer_update_accel();

    s_imu_vel[x] += g_odometer.acc[x] * ODOMETER_IMU_UPDATE_DT_S;
    s_imu_vel[y] += g_odometer.acc[y] * ODOMETER_IMU_UPDATE_DT_S;

    g_odometer.vel[x] = s_imu_vel[x];
    g_odometer.vel[y] = s_imu_vel[y];
}
