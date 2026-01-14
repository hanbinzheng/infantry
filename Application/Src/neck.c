#include "pid.h"
#include "motor.h"
#include "neck.h"


#ifndef PI
#define PI (3.14159265358979f)
#endif

// control frequency: 1000hz
// can 1, control identifier: 0X2FF
// gimbal yaw (data[0], data[1]) can id 5, feedback identifier 0x209

/*
 **************************************************************************
 * parameters
 **************************************************************************
 */
PidInfo pid_yaw_v2v = { // gimbal yaw gm6020 velocity to voltage pid (motors[4])
    .kp = 4.0f,
    .ki = 0.2f,
    .kd = 0.0f,
    .i_limit = 5.0f,
    .out_limit = 24.0f, // voltage limit 24V
};

PidInfo pid_yaw_p2v = { // gimbal yaw gm6020 position to velocity pid (motors[4])
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .i_limit = 0.0f,
    .out_limit = 10.0f, // velocity limit 15 rad / s
};


/*
 **************************************************************************
 * usedful functions
 **************************************************************************
 */
static inline float val_limit_float(float x, float min, float max)
{
    if (x > max) {return max;}
    else if (x < min) {return min;}
    else {return x;}
}

// to create best error due to mod 2 pi problem
static inline float optimal_error(float target, float measure)
{
    float raw_diff = target - measure;

    if (raw_diff >= PI) {measure += 2 * PI;}
    else if (raw_diff <= -PI) {measure -= 2 * PI;}

    return measure;
}

// gimbal gm6020 velocity to voltage control
float gimbal_yaw_v2v_control(float target_velocity, float measure_velocity)
{
    float command;
    // linear mapping and pid control
    command = 0.8 * target_velocity + pid_calculate(&pid_yaw_v2v, target_velocity, measure_velocity);
    command = val_limit_float(command, -24.0f, 24.0f);
    return command;
}

// gimbal gm6020 position to voltage control
float gimbal_yaw_p2v_control(float target_position, float measure_position, float measure_velocity)
{
    float command, command_velocity;
    measure_position = optimal_error(target_position, measure_position);
    command_velocity = pid_calculate(&pid_yaw_p2v, target_position, measure_position);
    command = gimbal_yaw_v2v_control(command_velocity, measure_velocity);
    command = val_limit_float(command, -24.0, 24.0);
    return command;
}

/*
 **************************************************************************
 * application neck task
 **************************************************************************
 */
void neck_task(void){
    ;
}
