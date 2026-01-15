#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "arm_math.h"

/*
chassis frame and layout
      +x
   /2    1\
+y              main control
   \3    4/

    battery
*/

// omni chassis kinematics decomposition
void kine_omni_decomposition(float32_t v_chassis[2], float32_t v_wheels[4]);

// kinematics for gimbal-follow mode
void kine_gimbal_follow(float32_t yaw_angle, float32_t v_gimbal_frame[2], float32_t v_chassis_frame[2]);

#endif // __KINEMATICS_H__
