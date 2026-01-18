#include "pid.h"
#include "motor.h"
#include "neck.h"
#include "imu.h"
#include "dbus.h"
#include "controller.h"

#ifndef PI
#define PI (3.14159265358979f)
#endif

#define RIGHT_FORWARD_ANGLE (3406.0f) // int, but transformed into float
#define TOTAL_ANGLE_NUMBER (8192.0f)

#define FREQUENCY 1000.0f

static inline float get_yaw_pos_from_motor(void)
{
    float angle = (motors[GIMBAL_YAW].raw_angle - RIGHT_FORWARD_ANGLE) / TOTAL_ANGLE_NUMBER;
    angle = angle * 2 * PI;

    // make sure the angle is within - PI ~ PI
    if (angle > PI)
    {
        angle -= 2 * PI;
    }
    else if (angle < -PI)
    {
        angle += 2 * PI;
    }

    return angle;
}

static inline void get_right_target(float *target)
{
    if (*target >= PI)
    {
        *target -= 2 * PI;
    }
    else if (*target <= -PI)
    {
        *target += 2 * PI;
    }
}

/*
 **************************************************************************
 * application neck task
 **************************************************************************
 */
void neck_task(void)
{
    static float pos_target = 0;
    if (dbus_data.sw1 == SW_UP) // turn down the infantry
    {
        motor_set_neck_voltage(0.0f);
        return;
    }

    pos_target += (-dbus_data.rs_y / FREQUENCY) * 2 * PI; // - rs_y
    get_right_target(&pos_target);
    float pos_measure = get_yaw_pos_from_motor();
    float vel_measure = motors[GIMBAL_YAW].velocity;

    set_neck_position(pos_target, pos_measure, vel_measure);
}
