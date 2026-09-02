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
#include "car_safety.h"
#include "zf_common_interrupt.h"
#include <math.h>

#define CAR_SAFETY_UPDATE_PERIOD_S                 (0.01f)

/* 目标归零后等待300 ms，再用300 ms窗口判断速度是否至少下降10%。 */
#define CAR_SAFETY_STOP_TARGET_MAX                 (5.0f)
#define CAR_SAFETY_STOP_MOVING_SPEED_MIN           (20.0f)
#define CAR_SAFETY_STOP_GRACE_CYCLES               (30U)
#define CAR_SAFETY_STOP_TREND_CYCLES               (30U)
#define CAR_SAFETY_STOP_MIN_DECEL_RATIO            (0.10f)

/* 电机输出超过60%，但目标有效且反馈接近零持续500 ms，判定堵转。 */
#define CAR_SAFETY_STALL_OUTPUT_MIN                (4800.0f)
#define CAR_SAFETY_STALL_TARGET_MIN                (50.0f)
#define CAR_SAFETY_STALL_SPEED_MAX                 (5.0f)
#define CAR_SAFETY_STALL_CYCLES                    (50U)

/* 同方向角速度不低于350°/s累计1080°，或不低于500°/s持续2500 ms。 */
#define CAR_SAFETY_ROTATION_RATE_MIN_DPS           (350.0f)
#define CAR_SAFETY_ROTATION_ANGLE_LIMIT_DEG        (1080.0f)
#define CAR_SAFETY_EXTREME_RATE_MIN_DPS            (500.0f)
#define CAR_SAFETY_EXTREME_RATE_CYCLES             (250U)
#define CAR_SAFETY_ROTATION_RESET_CYCLES           (10U)

typedef struct
{
    volatile uint8 output_allowed;
    uint8 rearm_ready;
    uint8 previous_switch_on;
    volatile car_safety_fault_e fault;

    uint16 stop_grace_cycles;
    uint16 stop_trend_cycles;
    float stop_reference_speed;

    uint16 left_stall_cycles;
    uint16 right_stall_cycles;

    int8 rotation_direction;
    uint16 rotation_below_rate_cycles;
    uint16 rotation_reverse_cycles;
    uint16 extreme_rate_cycles;
    float rotation_accumulated_deg;
} car_safety_state_t;

static car_safety_state_t s_car_safety;
static volatile uint8 s_car_motion_scheduler_started = 0U;
static volatile uint8 s_car_maintenance_request = 0U;

static void car_safety_reset_stop_monitor(void)
{
    s_car_safety.stop_grace_cycles = 0U;
    s_car_safety.stop_trend_cycles = 0U;
    s_car_safety.stop_reference_speed = 0.0f;
}

static void car_safety_reset_rotation_monitor(void)
{
    s_car_safety.rotation_direction = 0;
    s_car_safety.rotation_below_rate_cycles = 0U;
    s_car_safety.rotation_reverse_cycles = 0U;
    s_car_safety.extreme_rate_cycles = 0U;
    s_car_safety.rotation_accumulated_deg = 0.0f;
}

static void car_safety_reset_motion_monitors(void)
{
    car_safety_reset_stop_monitor();
    s_car_safety.left_stall_cycles = 0U;
    s_car_safety.right_stall_cycles = 0U;
    car_safety_reset_rotation_monitor();
}

static void car_safety_trip(car_safety_fault_e fault)
{
    if (s_car_safety.fault == CAR_SAFETY_FAULT_NONE)
    {
        s_car_safety.fault = fault;
    }

    s_car_safety.output_allowed = 0U;
    s_car_safety.rearm_ready = 0U;
    car_safety_reset_motion_monitors();
}

static float car_safety_max_abs(float first, float second)
{
    first = fabsf(first);
    second = fabsf(second);
    return (first > second) ? first : second;
}

static void car_safety_monitor_stop(const car_safety_input_t *input)
{
    float target_peak = car_safety_max_abs(input->left_target_speed,
                                            input->right_target_speed);
    float speed_peak = car_safety_max_abs(input->left_feedback_speed,
                                           input->right_feedback_speed);

    if ((target_peak > CAR_SAFETY_STOP_TARGET_MAX) ||
        (speed_peak <= CAR_SAFETY_STOP_MOVING_SPEED_MIN))
    {
        car_safety_reset_stop_monitor();
        return;
    }

    if (s_car_safety.stop_grace_cycles < CAR_SAFETY_STOP_GRACE_CYCLES)
    {
        s_car_safety.stop_grace_cycles++;
        if (s_car_safety.stop_grace_cycles == CAR_SAFETY_STOP_GRACE_CYCLES)
        {
            s_car_safety.stop_reference_speed = speed_peak;
            s_car_safety.stop_trend_cycles = 0U;
        }
        return;
    }

    s_car_safety.stop_trend_cycles++;
    if (s_car_safety.stop_trend_cycles < CAR_SAFETY_STOP_TREND_CYCLES)
    {
        return;
    }

    if (speed_peak >=
        s_car_safety.stop_reference_speed *
        (1.0f - CAR_SAFETY_STOP_MIN_DECEL_RATIO))
    {
        car_safety_trip(CAR_SAFETY_FAULT_STOP_NOT_DECELERATING);
        return;
    }

    s_car_safety.stop_reference_speed = speed_peak;
    s_car_safety.stop_trend_cycles = 0U;
}

static uint8 car_safety_stall_condition(float target,
                                         float feedback,
                                         float motor_output)
{
    return ((fabsf(target) >= CAR_SAFETY_STALL_TARGET_MIN) &&
            (fabsf(feedback) <= CAR_SAFETY_STALL_SPEED_MAX) &&
            (fabsf(motor_output) >= CAR_SAFETY_STALL_OUTPUT_MIN)) ? 1U : 0U;
}

static void car_safety_monitor_stall(const car_safety_input_t *input)
{
    if (car_safety_stall_condition(input->left_target_speed,
                                   input->left_feedback_speed,
                                   input->left_motor_output) != 0U)
    {
        if (s_car_safety.left_stall_cycles < CAR_SAFETY_STALL_CYCLES)
        {
            s_car_safety.left_stall_cycles++;
        }
    }
    else
    {
        s_car_safety.left_stall_cycles = 0U;
    }

    if (car_safety_stall_condition(input->right_target_speed,
                                   input->right_feedback_speed,
                                   input->right_motor_output) != 0U)
    {
        if (s_car_safety.right_stall_cycles < CAR_SAFETY_STALL_CYCLES)
        {
            s_car_safety.right_stall_cycles++;
        }
    }
    else
    {
        s_car_safety.right_stall_cycles = 0U;
    }

    if (s_car_safety.left_stall_cycles >= CAR_SAFETY_STALL_CYCLES)
    {
        car_safety_trip(CAR_SAFETY_FAULT_LEFT_MOTOR_STALL);
    }
    else if (s_car_safety.right_stall_cycles >= CAR_SAFETY_STALL_CYCLES)
    {
        car_safety_trip(CAR_SAFETY_FAULT_RIGHT_MOTOR_STALL);
    }
}

static void car_safety_monitor_rotation(const car_safety_input_t *input)
{
    float rate_abs = fabsf(input->gyroz_dps);
    int8 direction = (input->gyroz_dps >= 0.0f) ? 1 : -1;

    if (rate_abs >= CAR_SAFETY_EXTREME_RATE_MIN_DPS)
    {
        if (s_car_safety.extreme_rate_cycles < CAR_SAFETY_EXTREME_RATE_CYCLES)
        {
            s_car_safety.extreme_rate_cycles++;
        }
    }
    else
    {
        s_car_safety.extreme_rate_cycles = 0U;
    }

    if (s_car_safety.extreme_rate_cycles >= CAR_SAFETY_EXTREME_RATE_CYCLES)
    {
        car_safety_trip(CAR_SAFETY_FAULT_EXTREME_ROTATION);
        return;
    }

    if (rate_abs < CAR_SAFETY_ROTATION_RATE_MIN_DPS)
    {
        if (s_car_safety.rotation_below_rate_cycles < CAR_SAFETY_ROTATION_RESET_CYCLES)
        {
            s_car_safety.rotation_below_rate_cycles++;
        }
        if (s_car_safety.rotation_below_rate_cycles >= CAR_SAFETY_ROTATION_RESET_CYCLES)
        {
            s_car_safety.rotation_direction = 0;
            s_car_safety.rotation_reverse_cycles = 0U;
            s_car_safety.rotation_accumulated_deg = 0.0f;
        }
        return;
    }

    s_car_safety.rotation_below_rate_cycles = 0U;

    if (s_car_safety.rotation_direction == 0)
    {
        s_car_safety.rotation_direction = direction;
    }
    else if (direction != s_car_safety.rotation_direction)
    {
        if (s_car_safety.rotation_reverse_cycles < CAR_SAFETY_ROTATION_RESET_CYCLES)
        {
            s_car_safety.rotation_reverse_cycles++;
        }
        if (s_car_safety.rotation_reverse_cycles >= CAR_SAFETY_ROTATION_RESET_CYCLES)
        {
            s_car_safety.rotation_direction = direction;
            s_car_safety.rotation_reverse_cycles = 0U;
            s_car_safety.rotation_accumulated_deg = 0.0f;
        }
        return;
    }

    s_car_safety.rotation_reverse_cycles = 0U;
    s_car_safety.rotation_accumulated_deg +=
        rate_abs * CAR_SAFETY_UPDATE_PERIOD_S;

    if (s_car_safety.rotation_accumulated_deg >=
        CAR_SAFETY_ROTATION_ANGLE_LIMIT_DEG)
    {
        car_safety_trip(CAR_SAFETY_FAULT_CONTINUOUS_ROTATION);
    }
}

void car_safety_init(void)
{
    s_car_safety.output_allowed = 0U;
    s_car_safety.rearm_ready = 0U;
    s_car_safety.previous_switch_on = 0U;
    s_car_safety.fault = CAR_SAFETY_FAULT_NONE;
    s_car_motion_scheduler_started = 0U;
    s_car_maintenance_request = 0U;
    car_safety_reset_motion_monitors();
}

void car_safety_set_motion_scheduler_started(void)
{
    s_car_motion_scheduler_started = 1U;
}

uint8 car_safety_maintenance_acquire(void)
{
    uint32 irq_state;
    uint8 acquired = 0U;

    irq_state = interrupt_global_disable();
    if ((s_car_maintenance_request == 0U) &&
        ((s_car_motion_scheduler_started == 0U) ||
         (s_car_safety.output_allowed == 0U)))
    {
        s_car_maintenance_request = 1U;
        acquired = 1U;
    }
    interrupt_global_enable(irq_state);
    return acquired;
}

void car_safety_maintenance_release(void)
{
    s_car_maintenance_request = 0U;
}

uint8 car_safety_is_maintenance_requested(void)
{
    return s_car_maintenance_request;
}

void car_safety_update_100HZ(const car_safety_input_t *input)
{
    uint8 switch_rising_edge;

    if (input == NULL)
    {
        return;
    }

    switch_rising_edge = ((input->run_switch_on != 0U) &&
                          (s_car_safety.previous_switch_on == 0U)) ? 1U : 0U;
    s_car_safety.previous_switch_on = input->run_switch_on;

    /* 失联时输入会被置低，不能把这个低位当作人工复位。 */
    if (input->link_up == 0U)
    {
        car_safety_trip(CAR_SAFETY_FAULT_REMOTE_LOSS);
        return;
    }

    if (input->imu_healthy == 0U)
    {
        car_safety_trip(CAR_SAFETY_FAULT_IMU_LOSS);
        return;
    }

    if (input->maintenance_active != 0U)
    {
        car_safety_trip(CAR_SAFETY_FAULT_MAINTENANCE_ACTIVE);
        return;
    }

    /* 开关关闭始终总停；链路正常时才允许记录人工复位准备。 */
    if (input->run_switch_on == 0U)
    {
        s_car_safety.output_allowed = 0U;
        s_car_safety.rearm_ready = 1U;
        car_safety_reset_motion_monitors();
        return;
    }

    if (s_car_safety.output_allowed == 0U)
    {
        if ((switch_rising_edge != 0U) &&
            (s_car_safety.rearm_ready != 0U))
        {
            s_car_safety.fault = CAR_SAFETY_FAULT_NONE;
            s_car_safety.output_allowed = 1U;
            s_car_safety.rearm_ready = 0U;
            car_safety_reset_motion_monitors();
        }
        return;
    }

    // car_safety_monitor_stop(input);
    if (s_car_safety.output_allowed == 0U)
    {
        return;
    }

    // car_safety_monitor_stall(input);
    if (s_car_safety.output_allowed == 0U)
    {
        return;
    }

    // car_safety_monitor_rotation(input);
}

uint8 car_safety_is_output_allowed(void)
{
    return s_car_safety.output_allowed;
}

car_safety_fault_e car_safety_get_fault(void)
{
    return s_car_safety.fault;
}
