#include "pid.h"
#include <stddef.h>

static inline float val_limit_float(float x, float min, float max)
{
    if (x > max)
    {
        return max;
    }
    else if (x < min)
    {
        return min;
    }
    else
    {
        return x;
    }
}

void state_reset(PidInfo *pid)
{
    pid->target = 0;
    pid->measure = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->output = 0;

    pid->p_out = 0;
    pid->i_out = 0;
    pid->d_out = 0;
}

float pid_calculate(PidInfo *pid, float target, float measure)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->target = target;
    pid->measure = measure;
    pid->error = target - measure;

    // p out
    pid->p_out = pid->kp * pid->error;
    // i out with limit
    pid->i_out += pid->ki * pid->error;
    pid->i_out = val_limit_float(pid->i_out, -pid->i_limit, pid->i_limit);
    // d out
    pid->d_out = pid->kd * (pid->error - pid->last_error);

    // total output with limit
    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output = val_limit_float(pid->output, -pid->out_limit, pid->out_limit);

    // update error record
    pid->last_error = pid->error;

    return pid->output;
}
