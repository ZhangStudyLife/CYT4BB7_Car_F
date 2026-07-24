/**
 * @file wireless_control.c
 * @brief 无线遥控控制实现
 *
 * 安全逻辑链：
 *   receiver_online=0 OR ch5=上 OR (ch6!=遥控 AND ch6!=mode2) → 强制急停
 *   其他情况 → 解析摇杆值输出速度指令
 *
 * CH5 开关：下 = 使能，上 = 禁止（急停）
 * CH6 开关：上 = 遥控模式，下 = mode2 占位模式，中间 = 禁止
 */

#include "wireless_control.h"


wireless_control_state_t g_wireless_control_state = {0};

/**
 * @brief 将 SBUS 轴值按比例缩放到输出限幅
 * @param axis_value SBUS 标准化轴值（-1000~+1000）
 * @param output_limit 输出限幅值
 * @return 缩放后的速度值
 * 公式：result = axis_value * output_limit / 1000
 */
static float wireless_scale_axis_to_limit(int16 axis_value, float output_limit)
{
    return ((float)axis_value * output_limit) / 1000.0f;
}

/**
 * @brief 清除所有运动目标值
 * 急停或 mode2 占位模式下调用，确保不会残留上一次的遥控指令
 */
static void wireless_clear_targets(void)
{
    uint8 index;

    g_wireless_control_state.forward_speed = 0;
    g_wireless_control_state.strafe_speed = 0;
    g_wireless_control_state.rotate_speed = 0.0f;

    for(index = 0U; index < WIRELESS_CONTROL_WHEEL_COUNT; index++)
    {
        g_wireless_control_state.wheel_target[index] = 0;
        g_wireless_control_state.wheel_pwm[index] = 0;
    }
}

/**
 * @brief 强制进入急停状态
 * 谁调用：安全条件不满足时、初始化时
 * 效果：
 *   - control_enabled=0, emergency_stop_active=1
 *   - 清除所有模式请求标志
 *   - 清除所有运动目标值
 */
static void wireless_force_estop(void)
{
    g_wireless_control_state.control_enabled = 0U;
    g_wireless_control_state.emergency_stop_active = 1U;
    g_wireless_control_state.remote_mode_requested = 0U;
    g_wireless_control_state.mode2_requested = 0U;
    g_wireless_control_state.mode_request_valid = 0U;
    wireless_clear_targets();
}

/**
 * @brief 从 SBUS 状态快照通道值到本模块
 * @param sbus SBUS 状态指针
 */
static void wireless_snapshot_sbus(const sbus_state_t *sbus)
{
    uint8 index;

    for(index = 0U; index < WIRELESS_CONTROL_CHANNEL_COUNT; index++)
    {
        g_wireless_control_state.raw_channel[index] = sbus->raw_channel[index];
        g_wireless_control_state.std_channel[index] = sbus->std_channel[index];
    }
}

/**
 * @brief 初始化无线控制模块
 * 启动时默认急停，需要手动拨动 CH5 开关才能解除
 */
void wireless_control_init(void)
{
    wireless_force_estop();
}

/**
 * @brief 25Hz 更新入口
 * 调用频率：25Hz（每 40ms）
 *
 * 详细流程：
 *   1. 快照 SBUS 通道值
 *   2. 判断 CH5 使能开关（下=使能）
 *   3. 判断 CH6 模式开关（上=遥控, 下=mode2）
 *   4. 三个安全条件不满足任一 → 强制急停返回
 *   5. mode2 占位模式 → 清除遥控目标值
 *   6. 遥控模式 → 解析 CH1/CH2/CH4 摇杆为速度指令
 */
void wireless_control_update_25HZ(void)
{
    const sbus_state_t *sbus;
    uint8 ch5_enabled;
    uint8 ch6_remote_mode;
    uint8 ch6_mode2_request;
    float manual_rotate_speed;

    sbus = sbus_get_state();
    wireless_snapshot_sbus(sbus);
    g_wireless_control_state.receiver_online = sbus->receiver_online;

    /* CH5: 总使能开关，拨到"下"才使能控制 */
    ch5_enabled = ((0U != sbus->channel_valid[SBUS_CH5]) &&
                   (SBUS_STD_SWITCH_DOWN == sbus->std_channel[SBUS_CH5])) ? 1U : 0U;

    /* CH6: 模式开关，"上"=遥控模式 */
    ch6_remote_mode = ((0U != sbus->channel_valid[SBUS_CH6]) &&
                       (SBUS_STD_SWITCH_UP == sbus->std_channel[SBUS_CH6])) ? 1U : 0U;

    /* CH6: 模式开关，"下"=mode2 占位模式 */
    ch6_mode2_request = ((0U != sbus->channel_valid[SBUS_CH6]) &&
                         (SBUS_STD_SWITCH_DOWN == sbus->std_channel[SBUS_CH6])) ? 1U : 0U;

    /* ===== 安全检查：任一条件不满足 → 急停 ===== */
    if((0U == g_wireless_control_state.receiver_online) ||
       (0U == ch5_enabled) ||
       ((0U == ch6_remote_mode) && (0U == ch6_mode2_request)))
    {
        wireless_force_estop();
        return;
    }

    /* 通过安全检查，清除急停标志 */
    g_wireless_control_state.control_enabled = 1U;
    g_wireless_control_state.emergency_stop_active = 0U;
    g_wireless_control_state.remote_mode_requested = ch6_remote_mode;
    g_wireless_control_state.mode2_requested = ch6_mode2_request;
    g_wireless_control_state.mode_request_valid = 1U;

    /* mode2 占位模式：清零遥控目标 */
    if(0U != ch6_mode2_request)
    {
        wireless_clear_targets();
        return;
    }

    /* ===== 遥控模式：解析摇杆值 ===== */
    /* CH2 → 前后速度 */
    g_wireless_control_state.forward_speed =
        (int16)wireless_scale_axis_to_limit(sbus->std_channel[SBUS_CH2], (float)MAX_CONTROL_SPEED);
    /* CH1 → 左右平移速度 */
    g_wireless_control_state.strafe_speed =
        (int16)wireless_scale_axis_to_limit(sbus->std_channel[SBUS_CH1], (float)MAX_CONTROL_SPEED);
    /* CH4 → 旋转角速度 */
    manual_rotate_speed =
        wireless_scale_axis_to_limit(sbus->std_channel[SBUS_CH4], MAX_ANGULAR_SPEED);

    /* 旋转死区过滤：极小值清零，防止摇杆漂移导致自旋 */
    if(car_math_absf(manual_rotate_speed) < 0.001f)
    {
        manual_rotate_speed = 0.0f;
    }

    g_wireless_control_state.rotate_speed = manual_rotate_speed;
}

/**
 * @brief 获取当前控制状态
 * 返回值：指向 g_wireless_control_state 的只读指针
 * 谁用：运动控制、模式切换等模块
 */
const wireless_control_state_t *wireless_control_get_state(void)
{
    return &g_wireless_control_state;
}
