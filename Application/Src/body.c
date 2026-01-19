#include "body.h"
#include "kinematics.h"
#include "dbus.h"
#include "imu.h"
#include "motor.h"
#include "controller.h"

/*
/2   1\
   ^    main control
\3   4/
battery
*/

#define VELOCITY_SCALE (2.0f)

#define WHEEL_RADIUS (0.10f) // in meters
#define SIGN_V_FR (-1.0f)    // front right velocity sign
#define SIGN_V_FL (1.0f)     // front left velocity sign
#define SIGN_V_BL (1.0f)     // back left velocity sign
#define SIGN_V_BR (-1.0f)    // back right velocity sign

static inline void linear_omni_motion(float32_t v_x, float32_t v_y)
{
    float32_t v_chassis[2] = {v_x, v_y};
    float32_t v_wheels[4];
    kine_omni_decomposition(v_chassis, v_wheels);

    set_body_velocity(
        SIGN_V_FR * v_wheels[0] / WHEEL_RADIUS,
        SIGN_V_FL * v_wheels[1] / WHEEL_RADIUS,
        SIGN_V_BL * v_wheels[2] / WHEEL_RADIUS,
        SIGN_V_BR * v_wheels[3] / WHEEL_RADIUS);
}

/*
 **************************************************************************
 * application body task
 **************************************************************************
 */

void safe_mode(void)
{
    float32_t v_x = dbus_data.ls_x * VELOCITY_SCALE;
    float32_t v_y = -dbus_data.ls_y * VELOCITY_SCALE;

    linear_omni_motion(v_x, v_y);
}

void body_task(void)
{
    if (dbus_data.sw1 == SW_UP) // turn down the infantry
    {
        motor_set_body_current(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    safe_mode();
}
