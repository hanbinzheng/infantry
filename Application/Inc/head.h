#ifndef __HEAD_H__
#define __HEAD_H__

#include <stdint.h>

// application head task
void head_task(void);

// useful functions
void set_head_target(float v_fric_l, float v_fric_r, float v_trig, float p_pitch);
float gimbal_pitch_v2v_control(float target_velocity, float measure_velocity);
float gimbal_pitch_p2v_control(float target_position, float measure_position, float measure_velocity);

#endif // __HEAD_H__
