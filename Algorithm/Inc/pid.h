#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct
{
    // pid parameters
    float kp;
    float ki;
    float kd;
    float i_limit;   // integral limit
    float out_limit; // output limit

    // realtime info
    float target;
    float measure;
    float error;
    float last_error;
    float output;

    // for debug
    float p_out;
    float i_out;
    float d_out;
} PidInfo;

void pid_init(PidInfo *pid, float kp, float ki, float kd, float i_limit, float out_limnit);
float pid_calculate(PidInfo *pid, float target, float measure);
void state_reset(PidInfo *pid);

#endif // __PID_H__
