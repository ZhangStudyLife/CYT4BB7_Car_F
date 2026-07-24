/*********************************************************************************************************************
* CYT4BB 麦克纳姆轮惯性导航系统 - 7段S曲线速度规划模块 (实现文件)
*
* @file    s_curve_planner.c
* @brief   基于Jerk受限的7段S曲线轨迹规划的完整实现
*
* @details
* 算法基于Biagiotti & Melchiorri《Trajectory Planning for Automatic Machines and Robots》
* 第3章和2023年全国大学生智能汽车竞赛一等奖代码
*
* @author  Claude Code (基于范总国赛代码重构)
* @date    2025-12-10
********************************************************************************************************************/

#include "s_curve_planner.h"

// ========================== 物理约束上限 (安全保护) ==========================
#define MAX_VELOCITY 5000.0f             // 5 m/s
#define MAX_ACCELERATION 10000.0f        // 10 m/s²
#define MAX_JERK 100000.0f               // 100 m/s³

// ========================== 内部辅助函数 ==========================

/**
 * @brief 取较大值
 */
/**
 * @brief gamma迭代求解vlim (当vmax不能达到时)
 *
 * @details
 * 使用二分法求解以下方程:
 *   s_total = Ta*(v0+vlim)/2 + Td*(vlim+v1)/2 = distance
 *
 * 其中:
 *   Ta, Td 是 vlim 的函数
 *
 * @param planner  规划器实例
 * @param q0       起始位置 (归一化为0)
 * @param q1       结束位置 (已归一化为正值)
 * @param v0       起始速度
 * @param v1       结束速度
 * @param v_max    最大速度约束
 * @param a_max    最大加速度约束
 * @param j_max    最大Jerk约束
 * @return         实际最大速度vlim
 */
static float solve_vlim_iterative(s_curve_planner_t *planner,
                                    float q0, float q1,
                                    float v0, float v1,
                                    float v_max, float a_max, float j_max)
{
    float vlim_min = car_math_maxf(v0, v1);
    float vlim_max = v_max;
    float vlim = (vlim_min + vlim_max) * 0.5f;

    float Tj1_temp, Tj2_temp, Ta_temp, Td_temp;
    float alim_a_temp, alim_d_temp;

    int max_iter = (int)s_curve_max_iter;
    for (int iter = 0; iter < max_iter; iter++) {
        // 计算加速段参数
        if ((vlim - v0) * j_max < a_max * a_max) {
            Tj1_temp = sqrtf((vlim - v0) / j_max);
            alim_a_temp = j_max * Tj1_temp;
            Ta_temp = 2.0f * Tj1_temp;
        } else {
            Tj1_temp = a_max / j_max;
            alim_a_temp = a_max;
            Ta_temp = Tj1_temp + (vlim - v0) / a_max;
        }

        // 计算减速段参数
        if ((vlim - v1) * j_max < a_max * a_max) {
            Tj2_temp = sqrtf((vlim - v1) / j_max);
            alim_d_temp = j_max * Tj2_temp;
            Td_temp = 2.0f * Tj2_temp;
        } else {
            Tj2_temp = a_max / j_max;
            alim_d_temp = a_max;
            Td_temp = Tj2_temp + (vlim - v1) / a_max;
        }

        // 计算总位移
        float s_accel = Ta_temp * (v0 + vlim) * 0.5f;
        float s_decel = Td_temp * (vlim + v1) * 0.5f;
        float s_calc = s_accel + s_decel;

        // 检查收敛
        if (fabsf(s_calc - q1) < s_curve_conv_tol) {
            planner->Tj1 = Tj1_temp;
            planner->Tj2 = Tj2_temp;
            planner->Ta = Ta_temp;
            planner->Td = Td_temp;
            planner->alim_accel = alim_a_temp;
            planner->alim_decel = -alim_d_temp;
            return vlim;
        }

        // 二分迭代
        if (s_calc > q1) {
            vlim_max = vlim;
        } else {
            vlim_min = vlim;
        }

        vlim = (vlim_min + vlim_max) * 0.5f;
    }

    // 超过最大迭代次数，保存最后一次计算结果
    planner->Tj1 = Tj1_temp;
    planner->Tj2 = Tj2_temp;
    planner->Ta = Ta_temp;
    planner->Td = Td_temp;
    planner->alim_accel = alim_a_temp;
    planner->alim_decel = -alim_d_temp;
    return vlim;
}

/**
 * @brief 计算t时刻的目标速度 (内部函数)
 *
 * @details
 * 7段速度公式:
 *   Phase 1 [0, Tj1]:           v = v0 + jmax*t²/2
 *   Phase 2 [Tj1, Ta-Tj1]:      v = v0 + alim_a*(t - Tj1/2)
 *   Phase 3 [Ta-Tj1, Ta]:       v = vlim - jmax*(Ta-t)²/2
 *   Phase 4 [Ta, Ta+Tv]:        v = vlim
 *   Phase 5 [Ta+Tv, Ta+Tv+Tj2]: v = vlim - jmax*(t-Ta-Tv)²/2
 *   Phase 6 [Ta+Tv+Tj2, T-Tj2]: v = vlim + alim_d*(Tj2/2 + (t-Ta-Tv-Tj2))
 *   Phase 7 [T-Tj2, T]:         v = v1 + jmax*(T-t)²/2
 *
 * @param planner 规划器实例
 * @param t       当前时间 (s)
 * @param v_out   输出速度 (mm/s, 已包含sigma)
 * @return        当前阶段 (1-7), 0表示结束
 */
static uint8_t s_curve_get_velocity(s_curve_planner_t *planner, float t, float *v_out)
{
    // 边界检查
    if (t < 0.0f) {
        *v_out = planner->v0 * planner->sigma;
        return 0;
    }
    if (t >= planner->T) {
        *v_out = planner->v1 * planner->sigma;
        return 0;
    }

    float v = 0.0f;
    uint8_t phase = 0;

    // 判断当前阶段并计算速度
    if (t >= 0.0f && t < planner->Tj1) {
        // Phase 1: Jerk加速 (a: 0 → amax)
        phase = 1;
        v = planner->v0 + planner->j_max * t * t * 0.5f;

    } else if (t >= planner->Tj1 && t < planner->Ta - planner->Tj1) {
        // Phase 2: 匀加速 (a: amax)
        phase = 2;
        float dt = t - planner->Tj1;
        float v_at_Tj1 = planner->v0 + planner->j_max * planner->Tj1 * planner->Tj1 * 0.5f;
        v = v_at_Tj1 + planner->alim_accel * dt;

    } else if (t >= planner->Ta - planner->Tj1 && t < planner->Ta) {
        // Phase 3: Jerk减速 (a: amax → 0)
        phase = 3;
        float dt_from_end = planner->Ta - t;
        v = planner->vlim - planner->j_max * dt_from_end * dt_from_end * 0.5f;

    } else if (t >= planner->Ta && t < planner->Ta + planner->Tv) {
        // Phase 4: 匀速 (v: vlim)
        phase = 4;
        v = planner->vlim;

    } else if (t >= planner->Ta + planner->Tv &&
               t < planner->Ta + planner->Tv + planner->Tj2) {
        // Phase 5: Jerk减速 (a: 0 → -amax)
        phase = 5;
        float dt = t - planner->Ta - planner->Tv;
        v = planner->vlim - planner->j_max * dt * dt * 0.5f;

    } else if (t >= planner->Ta + planner->Tv + planner->Tj2 &&
               t < planner->T - planner->Tj2) {
        // Phase 6: 匀减速 (a: -amax)
        phase = 6;
        float dt = t - planner->Ta - planner->Tv - planner->Tj2;
        float v_at_phase5_end = planner->vlim - planner->j_max * planner->Tj2 * planner->Tj2 * 0.5f;
        v = v_at_phase5_end + planner->alim_decel * dt;  // alim_decel为负

    } else if (t >= planner->T - planner->Tj2 && t < planner->T) {
        // Phase 7: Jerk回零 (a: -amax → 0)
        phase = 7;
        float dt_from_end = planner->T - t;
        v = planner->v1 + planner->j_max * dt_from_end * dt_from_end * 0.5f;
    } else {
        // 超出范围,返回结束速度
        v = planner->v1;
        phase = 0;
    }

    // 应用sigma方向
    *v_out = v * planner->sigma;

    return phase;
}

// ========================== 对外接口函数 ==========================

/**
 * @brief 初始化7段S曲线速度规划器
 *
 * @see 函数原型在头文件中有详细说明
 */
int s_curve_planner_init(s_curve_planner_t *planner,
                          float distance,
                          float v_max,
                          float a_max,
                          float j_max)
{
    // ========== [1] 参数验证 ==========
    if (planner == NULL) {
        return -1;
    }
    if (v_max <= 0.0f || a_max <= 0.0f || j_max <= 0.0f) {
        return -1;
    }

    // 参数限幅 (安全保护)
    if (v_max > MAX_VELOCITY) v_max = MAX_VELOCITY;
    if (a_max > MAX_ACCELERATION) a_max = MAX_ACCELERATION;
    if (j_max > MAX_JERK) j_max = MAX_JERK;

    // 保存输入参数
    planner->v_max = v_max;
    planner->a_max = a_max;
    planner->j_max = j_max;

    // 设置默认边界条件
    if (planner->v0 != planner->v0) planner->v0 = 0.0f;  // NaN检查
    if (planner->v1 != planner->v1) planner->v1 = 0.0f;

    // ========== [2] sigma归一化处理双向运动 ==========
    if (distance >= 0.0f) {
        planner->sigma = 1.0f;
    } else {
        planner->sigma = -1.0f;
        distance = -distance;  // 取绝对值
    }

    // ========== [3] 投影v0/v1到运动方向 (修复双向运动bug) ==========
    // v0/v1从物理坐标系投影到归一化坐标系，确保后续计算正确
    planner->v0 = planner->v0 * planner->sigma;
    planner->v1 = planner->v1 * planner->sigma;

    float q0 = 0.0f;
    float q1 = distance;
    float v0 = planner->v0;  // 已投影到运动方向
    float v1 = planner->v1;

    // ========== [4] 最小位移检查 ==========
    if (q1 < s_curve_min_dist) {
        // 位移过短,标记为立即完成
        planner->flag = 1;
        planner->is_running = 0;
        planner->is_finished = 1;
        planner->v_target = 0.0f;
        planner->vlim = 0.0f;
        planner->T = 0.0f;
        planner->Tv = 0.0f;
        planner->Ta = 0.0f;
        planner->Td = 0.0f;
        planner->Tj1 = 0.0f;
        planner->Tj2 = 0.0f;
        planner->time_current = 0.0f;
        planner->phase = 0;
        return 0;  // 成功初始化但不运行
    }

    // ========== [5] 判断amax能否达到 (加速段) ==========
    float Tj1, Ta, alim_accel;

    if ((v_max - v0) * j_max < a_max * a_max) {
        // amax不能达到 (Jerk受限)
        Tj1 = sqrtf((v_max - v0) / j_max);
        Ta = 2.0f * Tj1;
        alim_accel = j_max * Tj1;
    } else {
        // amax能达到
        Tj1 = a_max / j_max;
        Ta = Tj1 + (v_max - v0) / a_max;
        alim_accel = a_max;
    }

    // ========== [6] 判断amax能否达到 (减速段) ==========
    float Tj2, Td, alim_decel;

    if ((v_max - v1) * j_max < a_max * a_max) {
        // amax不能达到 (Jerk受限)
        Tj2 = sqrtf((v_max - v1) / j_max);
        Td = 2.0f * Tj2;
        alim_decel = j_max * Tj2;
    } else {
        // amax能达到
        Tj2 = a_max / j_max;
        Td = Tj2 + (v_max - v1) / a_max;
        alim_decel = a_max;
    }

    // ========== [7] 判断vmax能否达到 ==========
    float s_accel = Ta * (v0 + v_max) * 0.5f;
    float s_decel = Td * (v_max + v1) * 0.5f;
    float Tv, vlim;

    if (s_accel + s_decel <= q1) {
        // Case 1: vmax能达到
        vlim = v_max;
        Tv = (q1 - s_accel - s_decel) / v_max;

        // 保存结果
        planner->Tj1 = Tj1;
        planner->Tj2 = Tj2;
        planner->Ta = Ta;
        planner->Td = Td;
        planner->Tv = Tv;
        planner->vlim = vlim;
        planner->alim_accel = alim_accel;
        planner->alim_decel = -alim_decel;  // 减速为负
        planner->T = Ta + Tv + Td;

    } else {
        // Case 2: vmax不能达到,gamma迭代求解
        vlim = solve_vlim_iterative(planner, q0, q1, v0, v1, v_max, a_max, j_max);
        Tv = 0.0f;

        planner->Tv = Tv;
        planner->vlim = vlim;
        planner->T = planner->Ta + planner->Td;
    }

    // ========== [8] 边界检查 ==========
    if (planner->Ta <= 0.0f || planner->Td <= 0.0f || planner->T <= 0.0f) {
        // 异常情况,标记为降级处理
        planner->flag = 1;
        planner->is_running = 0;
        planner->is_finished = 1;
        planner->v_target = 0.0f;
        return 0;
    }

    // ========== [9] 初始化状态 ==========
    planner->time_current = 0.0f;
    planner->phase = 0;
    planner->v_target = v0 * planner->sigma;
    planner->is_running = 1;
    planner->is_finished = 0;
    planner->flag = 0;

    return 0;  // 成功
}

/**
 * @brief 更新7段S曲线速度规划器
 *
 * @see 函数原型在头文件中有详细说明
 */
void s_curve_planner_step(s_curve_planner_t *planner, float dt)
{
    if (planner == NULL || planner->is_running == 0) {
        return;
    }

    // 时间推进
    planner->time_current += dt;

    // 计算当前速度
    planner->phase = s_curve_get_velocity(planner, planner->time_current, &planner->v_target);

    // 检查是否完成
    if (planner->time_current >= planner->T - 0.001f || planner->phase == 0) {
        planner->is_finished = 1;
        planner->is_running = 0;
        planner->v_target = planner->v1 * planner->sigma;
    }
}
