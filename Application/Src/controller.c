#include "pid.h"
#include "motor.h"
#include "controller.h"

#ifndef PI
#define PI (3.14159265358979f)
#endif

#define M3508_REDUCTION_RATIO (19.0f)
#define M2006_REDUCTION_RATIO (36.0f)

/*
 **************************************************************************
 * parameters
 **************************************************************************
 */
PidInfo pid_fr_v2c = {
    // chassis front right m3508 velocity to current pid (motors[0])
    .kp = 0.045f,
    .ki = 0.00114514f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20.0A
};

PidInfo pid_fl_v2c = {
    // chassis front left m3508 velocity to current pid (motors[1])
    .kp = 0.045f,
    .ki = 0.00114514f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20.0A
};

PidInfo pid_bl_v2c = {
    // chassis back left m3508 velocity to current pid (motors[2])
    .kp = 0.045f,
    .ki = 0.00066666f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20.0A
};

PidInfo pid_br_v2c = {
    // chassis back right m3508 velocity to current pid (motors[3])
    .kp = 0.045f,
    .ki = 0.00066666f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20.0A
};

PidInfo pid_pitch_v2v = {
    // gimbal pitch gm6020 velocity to voltage pid (motors[5])
    .kp = 1.3f,
    .ki = 0.001f,
    .kd = 0.0f,
    .i_limit = 0.5f,
    .out_limit = 10.0f,
};

PidInfo pid_pitch_p2v = {
    // gimbal pitch gm6020 position to velocity pid (motors[5]
    .kp = 13.5f,
    .ki = 0.0f,
    .kd = 0.0f,
    .i_limit = 0.0f,
    .out_limit = 10.0f, // velocity limit 10 rad/s
};

PidInfo pid_friction_l_v2c = {
    // friction left m3508 velocity to current pid (motors[6])
    .kp = 0.14f,
    .ki = 0.0005f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20A
};

PidInfo pid_friction_r_v2c = {
    // friction right m3508 velocity to current pid (motors[6])
    .kp = 0.14f,
    .ki = 0.0005f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20A
};

PidInfo pid_trigger_v2c = {
    // trigger m2006 velocity to current pit (motors[8])
    .kp = 0.02f,
    .ki = 0.001f,
    .kd = 0.0f,
    .i_limit = 0.5f,
    .out_limit = 10.0f, // current limit 10A
};

PidInfo pid_yaw_v2v = {
    // gimbal yaw gm6020 velocity to voltage pid (motors[4])
    .kp = 4.0f,
    .ki = 0.2f,
    .kd = 0.0f,
    .i_limit = 5.0f,
    .out_limit = 24.0f, // voltage limit 24V
};

PidInfo pid_yaw_p2v = {
    // gimbal yaw gm6020 position to velocity pid (motors[4])
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .i_limit = 0.0f,
    .out_limit = 10.0f, // velocity limit 15 rad / s
};

/*
 **************************************************************************
 * value limit function and pitch velocity control function
 **************************************************************************
 */
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
    return x;
}

static inline float optimal_measure(float target, float measure)
{
    float raw_diff = target - measure;

    if (raw_diff >= PI)
    {
        measure += 2 * PI;
    }
    else if (raw_diff <= -PI)
    {
        measure -= 2 * PI;
    }

    return measure;
}

static inline float gimbal_pitch_v2v_control(float target_vel, float measure_vel)
{
    float command, linear_scale = 0;
    int raw_angle = motors[GIMBAL_PITCH].raw_angle;

    // ugly linear mapping fucntion
    if (target_vel >= 0)
    { // target velocity >= 0
        if (raw_angle <= 1000)
        { // 666 ~ 1000
            linear_scale = 2.5;
        }
        else if (raw_angle <= 1475)
        { // 1000 ~ 1475
            linear_scale = 2.0;
        }
        else if (raw_angle <= 1800)
        { // 1475 ~ 1800
            linear_scale = 1.5;
        }
        else
        { // 1800 ~ 2190
            linear_scale = 1.0;
        }
    }
    else
    { // target velocity <= 0
        if (raw_angle <= 1600)
        { // 666 ~ 1600
            linear_scale = -0.3;
        }
        else if (raw_angle <= 1950)
        { // 1600 ~ 1950
            linear_scale = 1.0;
        }
        else
        { // 1950 ~ 2190
            linear_scale = 1.5;
        }
    }

    command = linear_scale * target_vel + pid_calculate(&pid_pitch_v2v, target_vel, measure_vel);
    return val_limit_float(command, -24.0, 24.0); // voltage limit 24.0 v
}

/*
 **************************************************************************
 * exposed interfaces
 **************************************************************************
 */
void set_body_velocity(float v_fr, float v_fl, float v_bl, float v_br)
{
    // get raw m3508 velocity target
    v_fr = v_fr * M3508_REDUCTION_RATIO;
    v_fl = v_fl * M3508_REDUCTION_RATIO;
    v_bl = v_bl * M3508_REDUCTION_RATIO;
    v_br = v_br * M3508_REDUCTION_RATIO;

    // calculate current command
    float c_fr = pid_calculate(&pid_fr_v2c, v_fr, motors[CHASSIS_FR].velocity); // front right
    float c_fl = pid_calculate(&pid_fl_v2c, v_fl, motors[CHASSIS_FL].velocity); // front left
    float c_bl = pid_calculate(&pid_bl_v2c, v_bl, motors[CHASSIS_BL].velocity); // back left
    float c_br = pid_calculate(&pid_br_v2c, v_br, motors[CHASSIS_BR].velocity); // back right

    // set current command
    motor_set_body_current(c_fr, c_fl, c_bl, c_br);
}

void set_neck_position(float pos_target, float pos_measure, float v_measure)
{
    float command, command_vel;

    pos_measure = optimal_measure(pos_target, pos_measure);
    command_vel = pid_calculate(&pid_yaw_p2v, pos_target, pos_measure);

    // linear mapping and pid for velocity control
    command = 0.8 * command_vel + pid_calculate(&pid_yaw_v2v, command_vel, v_measure);
    command = val_limit_float(command, -24.0, 24.0); // voltage limit 24.0

    motor_set_neck_voltage(command);
}

void set_head_command(float pos_pitch_target, float pos_pitch_measure,
                      float vel_pitch_measure, float v_fric_l, float v_fric_r, float v_trigger)
{
    // pitch position to voltage control
    float pitch_command_vel = pid_calculate(&pid_pitch_p2v, pos_pitch_target, pos_pitch_measure);
    float command_pitch = gimbal_pitch_v2v_control(pitch_command_vel, vel_pitch_measure);

    // friction left and friction velocity to current control, without velocity reduction
    float command_fric_l = pid_calculate(&pid_friction_l_v2c, v_fric_l, motors[FRICTION_L].velocity);
    float command_fric_r = pid_calculate(&pid_friction_r_v2c, v_fric_r, motors[FRICTION_R].velocity);

    // trigger velocity to current control, without reduction
    v_trigger = M2006_REDUCTION_RATIO * v_trigger;
    float command_trigger = pid_calculate(&pid_trigger_v2c, v_trigger, motors[TRIGGER].velocity);

    // set command
    motor_set_head_command(command_pitch, command_fric_l, command_fric_r, command_trigger);
}
