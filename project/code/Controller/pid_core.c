#include "pid_core.h"

#define PID_DEFAULT_OUTPUT_LIMIT (1000000000.0f)

static float pid_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float pid_clampf(float value, float min_value, float max_value)
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

static float pid_feedforward(const pid_t *pid, float setpoint)
{
    float setpoint_abs = pid_absf(setpoint);
    float feedforward = pid->kff * setpoint + pid->external_ff_term;
    float transition_scale = 1.0f;

    if (setpoint_abs <= pid->ff_deadband)
    {
        return 0.0f;
    }

    if (setpoint > 0.0f)
    {
        feedforward += pid->ff_static;
    }
    else if (setpoint < 0.0f)
    {
        feedforward -= pid->ff_static;
    }

    if ((pid->ff_transition > pid->ff_deadband) &&
        (setpoint_abs < pid->ff_transition))
    {
        transition_scale = (setpoint_abs - pid->ff_deadband) /
                           (pid->ff_transition - pid->ff_deadband);
    }

    return feedforward * transition_scale;
}

void PID_Init(pid_t *pid, float kp, float ki, float kd, float kff,
              float i_limit)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->ff_static = 0.0f;
    pid->ff_deadband = 0.0f;
    pid->ff_transition = 0.0f;
    pid->i_limit = pid_absf(i_limit);
    pid->output_min = -PID_DEFAULT_OUTPUT_LIMIT;
    pid->output_max = PID_DEFAULT_OUTPUT_LIMIT;
    pid->incremental_limit = 0.0f;
    PID_Reset(pid);
}

void PID_SetOutputLimits(pid_t *pid, float output_min, float output_max)
{
    if (pid == 0)
    {
        return;
    }

    if (output_min <= output_max)
    {
        pid->output_min = output_min;
        pid->output_max = output_max;
    }
    else
    {
        pid->output_min = output_max;
        pid->output_max = output_min;
    }

    pid->output = pid_clampf(pid->output, pid->output_min, pid->output_max);
}

void PID_SetStaticFeedforward(pid_t *pid, float ff_static)
{
    if (pid != 0)
    {
        pid->ff_static = pid_absf(ff_static);
    }
}

void PID_SetFeedforwardTransition(pid_t *pid, float deadband,
                                  float transition)
{
    if (pid != 0)
    {
        pid->ff_deadband = pid_absf(deadband);
        pid->ff_transition = pid_absf(transition);
    }
}

void PID_SetExternalFeedforward(pid_t *pid, float external_ff)
{
    if (pid != 0)
    {
        pid->external_ff_term = external_ff;
    }
}

void PID_SetIncrementLimit(pid_t *pid, float incremental_limit)
{
    if (pid != 0)
    {
        pid->incremental_limit = pid_absf(incremental_limit);
    }
}

float PID_Update(pid_t *pid, float setpoint, float measurement)
{
    float new_error;
    float integral_candidate;
    float output_unsaturated;
    float output_saturated;

    if (pid == 0)
    {
        return 0.0f;
    }

    new_error = setpoint - measurement;
    pid->error = new_error;
    pid->p_term = pid->kp * new_error;
    pid->i_term = pid->integral;

    if (pid->initialized != 0U)
    {
        pid->d_term = pid->kd * (new_error - pid->prev_error);
        pid->sp_rate = setpoint - pid->prev_sp;
    }
    else
    {
        pid->d_term = 0.0f;
        pid->sp_rate = 0.0f;
    }

    pid->ff_term = pid_feedforward(pid, setpoint);
    integral_candidate = pid_clampf(pid->integral + pid->ki * new_error,
                                    -pid->i_limit, pid->i_limit);
    output_unsaturated = pid->p_term + integral_candidate +
                         pid->d_term + pid->ff_term;
    output_saturated = pid_clampf(output_unsaturated,
                                  pid->output_min, pid->output_max);

    /* 输出饱和且误差继续推动饱和时冻结积分。 */
    if (((output_unsaturated > pid->output_max) && (new_error > 0.0f)) ||
        ((output_unsaturated < pid->output_min) && (new_error < 0.0f)))
    {
        integral_candidate = pid->integral;
        output_unsaturated = pid->p_term + integral_candidate +
                             pid->d_term + pid->ff_term;
        output_saturated = pid_clampf(output_unsaturated,
                                      pid->output_min, pid->output_max);
    }

    pid->integral = integral_candidate;
    pid->i_term = pid->integral;
    pid->output = output_saturated;
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = new_error;
    pid->prev_meas = measurement;
    pid->prev_sp = setpoint;
    pid->initialized = 1U;

    return pid->output;
}

float PID_UpdateIncremental(pid_t *pid, float setpoint, float measurement)
{
    float new_error;
    float delta_output;
    float correction_candidate;
    float output_unsaturated;

    if (pid == 0)
    {
        return 0.0f;
    }

    new_error = setpoint - measurement;
    pid->error = new_error;
    pid->p_term = pid->kp * (new_error - pid->prev_error);
    pid->i_term = pid_clampf(pid->ki * new_error,
                             -pid->i_limit, pid->i_limit);
    pid->d_term = pid->kd * (new_error - 2.0f * pid->prev_error +
                             pid->prev_prev_error);
    pid->ff_term = pid_feedforward(pid, setpoint);
    pid->sp_rate = (pid->initialized != 0U) ?
                   (setpoint - pid->prev_sp) : 0.0f;

    delta_output = pid->p_term + pid->i_term + pid->d_term;
    if (pid->incremental_limit > 0.0f)
    {
        delta_output = pid_clampf(delta_output,
                                  -pid->incremental_limit,
                                  pid->incremental_limit);
    }

    correction_candidate = pid->incremental_output + delta_output;
    output_unsaturated = pid->ff_term + correction_candidate;

    /* 最终输出饱和时，撤销继续推动饱和的积分增量。 */
    if (((output_unsaturated > pid->output_max) && (pid->i_term > 0.0f)) ||
        ((output_unsaturated < pid->output_min) && (pid->i_term < 0.0f)))
    {
        delta_output -= pid->i_term;
        pid->i_term = 0.0f;
        if (pid->incremental_limit > 0.0f)
        {
            delta_output = pid_clampf(delta_output,
                                      -pid->incremental_limit,
                                      pid->incremental_limit);
        }
        correction_candidate = pid->incremental_output + delta_output;
        output_unsaturated = pid->ff_term + correction_candidate;
    }

    pid->output = pid_clampf(output_unsaturated,
                             pid->output_min, pid->output_max);
    pid->incremental_output = pid->output - pid->ff_term;
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = new_error;
    pid->prev_meas = measurement;
    pid->prev_sp = setpoint;
    pid->initialized = 1U;

    return pid->output;
}

void PID_Reset(pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->prev_meas = 0.0f;
    pid->prev_sp = 0.0f;
    pid->initialized = 0U;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->ff_term = 0.0f;
    pid->external_ff_term = 0.0f;
    pid->incremental_output = 0.0f;
    pid->output = 0.0f;
    pid->sp_rate = 0.0f;
}
