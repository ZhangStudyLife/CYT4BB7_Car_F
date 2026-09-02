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
#ifndef PID_CORE_H
#define PID_CORE_H

#include <stdint.h>

/*
 * PID 系数均为每采样周期的离散系数：
 *   位置式 I[k] = I[k-1] + Ki * e[k]
 *   位置式 D[k] = Kd * (e[k] - e[k-1])
 * 调用接口不传入 dt，控制周期变化后应重新整定 Ki、Kd。
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float kff;
    float ff_static;
    float ff_deadband;
    float ff_transition;
    float external_ff_term;

    float i_limit;
    float output_min;
    float output_max;
    float incremental_limit;

    float integral;
    float error;
    float prev_error;
    float prev_prev_error;
    float prev_meas;
    float prev_sp;
    uint8_t initialized;

    /* 调试与遥测量。增量式PID中 P/I/D 为本周期输出增量。 */
    float p_term;
    float i_term;
    float d_term;
    float ff_term;
    float incremental_output;
    float output;
    float sp_rate;
} pid_t;

/** 初始化离散PID，不需要控制周期 dt。 */
void PID_Init(pid_t *pid, float kp, float ki, float kd, float kff,
              float i_limit);

/** 配置最终输出限幅。 */
void PID_SetOutputLimits(pid_t *pid, float output_min, float output_max);

/** 配置与目标方向一致的静态前馈绝对值。 */
void PID_SetStaticFeedforward(pid_t *pid, float ff_static);

/** 配置目标前馈死区和平滑过渡上限，过渡上限不大于死区时仅使用死区。 */
void PID_SetFeedforwardTransition(pid_t *pid, float deadband,
                                  float transition);

/** 配置外部动态前馈，并与目标前馈共同经过死区、平滑和最终输出限幅。 */
void PID_SetExternalFeedforward(pid_t *pid, float external_ff);

/** 配置增量式输出增量或受限位置式最终PWM步进，非正值表示不限幅。 */
void PID_SetIncrementLimit(pid_t *pid, float incremental_limit);

/** 位置式离散PID：可通过将某项系数设为0实现P、PI或PD控制。 */
float PID_Update(pid_t *pid, float setpoint, float measurement);

/** 增量式离散PID。 */
float PID_UpdateIncremental(pid_t *pid, float setpoint, float measurement);

/** Mode4/Mode5专用的独立积分位置式PID，带最终PWM步进和条件抗饱和。 */
float PID_UpdatePositionLimited(pid_t *pid, float setpoint,
                                float measurement);

/** 清零PID历史状态和输出。 */
void PID_Reset(pid_t *pid);

#endif /* PID_CORE_H */
