#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

void set_body_velocity(float v_fr, float v_fl, float v_bl, float v_br);
void set_neck_position(float pos_target, float pos_measure, float v_measure);
void set_head_command(float pos_pitch_target, float pos_pitch_measure,
                      float vel_pitch_measure, float v_fric_l, float v_fric_r, float v_trigger);

#endif // __CONTROLLER_H__
