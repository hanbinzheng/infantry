#include "kinematics.h"

#ifndef SQRT_2
#define SQRT_2 1.41421356237f
#endif

void kine_omni_decomposition(float32_t v_chassis[2], float32_t v_wheels[4]){
    // v_chassis: (vc_x, vc_y)
    // v_wheels: (vw_1, vw_2, vw_3, vw_4), be careful about positive direction of v wheels

    float32_t v24 = ( v_chassis[0] - v_chassis[1]) / SQRT_2;
    float32_t v13 = ( v_chassis[0] + v_chassis[1]) / SQRT_2;
    v_wheels[0] = v13;
    v_wheels[1] = v24;
    v_wheels[2] = v13;
    v_wheels[3] = v24;
}

void kine_gimbal_follow(float32_t yaw_angle, float32_t v_gimbal_frame[2], float32_t v_chassis_frame[2]) {
    // [vc_x] = [  cos(yaw) - sin(yaw) ] [vg_x]
    // [vc_y]   [  sin(yaw)   cos(yaw) ] [vg_y]
    float32_t cos_yaw = arm_cos_f32(yaw_angle);
    float32_t sin_yaw = arm_sin_f32(yaw_angle);
    v_chassis_frame[0] = cos_yaw * v_gimbal_frame[0] - sin_yaw * v_gimbal_frame[1];
    v_chassis_frame[1] = sin_yaw * v_gimbal_frame[0] + cos_yaw * v_gimbal_frame[1];
}

