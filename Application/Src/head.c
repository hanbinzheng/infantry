#include "head.h"
#include "motor.h"
#include "dbus.h"
#include "controller.h"

#ifndef PI
#define PI (3.14159265358979f)
#endif

#define V_FRICTION_L (-300.0f) // in rad
#define V_FRICTION_R (300.0f)
#define V_TRIGGER (3.0f) // in rad

#define PITCH_HALF_ANGLE ((2190.f - 670.0f) / 8192.0f / 2.0f * 2 * PI)
#define GET_POSITION_FROM_ANGLE(angle) (((float)angle - 1430.0f) / 8192.0f * 2 * PI)
#define FREQUENCY_HEAD (1000.0f)
#define PITCH_SENSITIVITY (6.0f)

/*
 **************************************************************************
 * helper function
 **************************************************************************
 */
static inline void limit_pitch_target(float *pos_pitch_target)
{
    if (*pos_pitch_target > PITCH_HALF_ANGLE)
    {
        *pos_pitch_target = PITCH_HALF_ANGLE;
    }
    else if (*pos_pitch_target < -PITCH_HALF_ANGLE)
    {
        *pos_pitch_target = -PITCH_HALF_ANGLE;
    }
}

/*
 **************************************************************************
 * application head task
 **************************************************************************
 */
void head_task(void)
{
    if (dbus_data.sw1 == SW_UP)
    { // close the head
        motor_set_head_command(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    // get pitch position target and position measure
    static float pos_pitch_target, pos_pitch_measure, vel_pitch_measure, v_fric_l, v_fric_r, v_trigger;
    pos_pitch_target -= ((dbus_data.rs_x * PITCH_HALF_ANGLE / FREQUENCY_HEAD) * PITCH_SENSITIVITY);
    limit_pitch_target(&pos_pitch_target);
    pos_pitch_measure = GET_POSITION_FROM_ANGLE(motors[GIMBAL_PITCH].raw_angle);

    // get pitch velocity measure
    vel_pitch_measure = motors[GIMBAL_PITCH].velocity;

    if (dbus_data.wheel > 1024)
    {
        v_fric_l = V_FRICTION_L;
        v_fric_r = V_FRICTION_R;
        v_trigger = V_TRIGGER;
    }
    else if (dbus_data.wheel < 1024)
    {
        v_trigger = -V_TRIGGER;
    }
    else
    {
        v_fric_l = 0.0f;
        v_fric_r = 0.0f;
        v_trigger = 0.0f;
    }

    set_head_command(pos_pitch_target, pos_pitch_measure, vel_pitch_measure, v_fric_l, v_fric_r, v_trigger);
}
