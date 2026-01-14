#include "head.h"
#include "pid.h"
#include "motor.h"

// control frequency: 1000Hz
// can3, control identifier 0x1FF
// 1. pitch (data[0], data[1]): GM6020, can id 1, feedback identifier 0x208
// 2. friction left (data[2], data[3]): M3508, can id 6, feedback identifier 0x206
// 3. friction right (data[2], data[3]): M3508, can id 7, feedback identifier 0x207
// 4. trigger (data[2], data[3]): M2006, can id 8, feedback identifier 0x208


/*
 **************************************************************************
 * parameters
 **************************************************************************
 */
PidInfo pid_pitch_v2v = { // gimbal pitch gm6020 velocity to voltage pid (motors[5])
    .kp = 1.3f,
    .ki = 0.001f,
    .kd = 0.0f,
    .i_limit = 0.5f,
    .out_limit = 10.0f,
};

PidInfo pid_pitch_p2v = { // gimbal pitch gm6020 position to velocity pid (motors[5]
    .kp = 13.5f,
    .ki = 0.0f,
    .kd = 0.0f,
    .i_limit = 0.0f,
    .out_limit = 10.0f, // velocity limit 10 rad/s
};

PidInfo pid_friction_l_v2c = { // friction left m3508 velocity to current pid (motors[6])
    .kp = 0.14f,
    .ki = 0.0005f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20A
};

PidInfo pid_friction_r_v2c = { // friction right m3508 velocity to current pid (motors[6])
    .kp = 0.14f,
    .ki = 0.0005f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20A
};

PidInfo pid_trigger_v2c = { // trigger m2006 velocity to current pit (motors[8])
    .kp = 0.02f,
    .ki = 0.001f,
    .kd = 0.0f,
    .i_limit = 0.5f,
    .out_limit = 10.0f, // current limit 10A
};

/*
 **************************************************************************
 * useful functions
 **************************************************************************
 */
// pitch gm6020 velocity to voltage control
float gimbal_pitch_v2v_control(float target_velocity, float measure_velocity)
{
    float command, linear_scale;
    int raw_angle = motors[GIMBAL_PITCH].raw_angle;

    // ugly linear mapping control
    if (target_velocity >= 0) // target_velocity >= 0
    {
        if (raw_angle <= 1000) // 666 ~ 1000
        {
            linear_scale = 2.5;
        }
        else if (raw_angle <= 1475) // 1000 ~ 1475
        {
            linear_scale = 2.0;
        }
        else if (raw_angle <= 1800) // 1475 ~ 1800
        {
            linear_scale = 1.5;
        }
        else // 1800 ~ 2190
        {
            linear_scale = 1.0;
        }
    }
    else // target_velocity <= 0
    {
        if (raw_angle <= 1600) // 666 ~ 1600
        {
            linear_scale = -0.3;
        }
        else if (raw_angle <= 1950) // 1600 ~ 1950
        {
            linear_scale = 1.0;
        }
        else // 1950 ~ 2190
        {
            linear_scale = 1.5;
        }
    }
    command = linear_scale * target_velocity + pid_calculate(&pid_pitch_v2v, target_velocity, measure_velocity);
    return command;
}

// gimbal pitch gm6020 position to velocity control
float gimbal_pitch_p2v_control(float target_position, float measure_position, float measure_velocity)
{
    float command_velocity = pid_calculate(&pid_pitch_p2v, target_position, measure_position);
    float command = gimbal_pitch_v2v_control(command_velocity, measure_velocity);
    return command;
}

/*
void armor_booster_set_velocity(float v_friction_l,
                                float v_friction_r, float v_trigger, float *current_command)
{
    float measure_friction_l, measure_friction_r, measure_trigger;
    measure_friction_l = motors[FRICTION_L].velocity;
    measure_friction_r = motors[FRICTION_R].velocity;
    measure_trigger = motors[TRIGGER].velocity;

    current_command[FRICTION_L - 6] = pid_calculate(&pid_friction_l_v2c, v_friction_l, measure_friction_l);
    current_command[FRICTION_R - 6] = pid_calculate(&pid_friction_r_v2c, v_friction_r, measure_friction_r);
    current_command[TRIGGER - 6] = pid_calculate(&pid_trigger_v2c, v_trigger, measure_trigger);
}
*/

/*
 **************************************************************************
 * application head task
 **************************************************************************
 */
 void head_task(void){
    ;
}
