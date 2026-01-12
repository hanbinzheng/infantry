#include "body.h"
#include "fdcan.h"
#include "bsp_fdcan.h"
#include "motor.h"
#include "pid.h"

#define M3508_REDUCTION_RATIO (19.0f)

// chassis front right m3508 velocity to current pid (motors[0])
PidInfo pid_fr_v2c = {
    .kp = 0.045f,
    .ki = 0.00114514f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20.0A
};

// chassis front left m3508 velocity to current pid (motors[1])
PidInfo pid_fl_v2c = {
    .kp = 0.045f,
    .ki = 0.00114514f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20.0A
};

// chassis back left m3508 velocity to current pid (motors[2])
PidInfo pid_bl_v2c = {
    .kp = 0.045f,
    .ki = 0.00066666f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20.0A
};

// chassis back right m3508 velocity to current pid (motors[3])
PidInfo pid_br_v2c = {
    .kp = 0.045f,
    .ki = 0.00066666f,
    .kd = 0.0f,
    .i_limit = 2.0f,
    .out_limit = 20.0f, // current limit 20.0A
};

static inline void set_chassis_command(
    int16_t c_fr, int16_t c_fl, int16_t c_bl, int16_t c_br)
{
    // set current command
    motors[CHASSIS_FR].command = c_fr; // front right
    motors[CHASSIS_FL].command = c_fl; // front left
    motors[CHASSIS_BL].command = c_bl; // back left
    motors[CHASSIS_BR].command = c_br; // back right

    // convert float command into int
    motors[CHASSIS_FR].command_int = M3508_CURRENT_FLOAT_TO_INT(c_fr);
    motors[CHASSIS_FL].command_int = M3508_CURRENT_FLOAT_TO_INT(c_fl);
    motors[CHASSIS_BL].command_int = M3508_CURRENT_FLOAT_TO_INT(c_bl);
    motors[CHASSIS_BR].command_int = M3508_CURRENT_FLOAT_TO_INT(c_br);

    // prepare data
    uint8_t data[8];
    data[0] = (motors[CHASSIS_FR].command_int >> 8) & 0xFF;
    data[1] = motors[CHASSIS_FR].command_int & 0xFF;
    data[2] = (motors[CHASSIS_FL].command_int >> 8) & 0xFF;
    data[3] = motors[CHASSIS_FL].command_int & 0xFF;
    data[4] = (motors[CHASSIS_BL].command_int >> 8) & 0xFF;
    data[5] = motors[CHASSIS_BL].command_int & 0xFF;
    data[6] = (motors[CHASSIS_BR].command_int >> 8) & 0xFF;
    data[7] = motors[CHASSIS_BR].command_int & 0xFF;

    // send command message
    BSP_FDCAN_TxMessage(&hfdcan1, 0x200, data);
}

void set__body_target(float v_fr, float v_fl, float v_bl, float v_br)
{
    // get m3508 raw data
    v_fr = v_fr * M3508_REDUCTION_RATIO;
    v_fl = v_fl * M3508_REDUCTION_RATIO;
    v_bl = v_bl * M3508_REDUCTION_RATIO;
    v_br = v_br * M3508_REDUCTION_RATIO;

    // calculate current command
    int16_t c_fr = pid_calculate(&pid_fr_v2c, v_fr, motors[CHASSIS_FR].velocity); // front right
    int16_t c_fl = pid_calculate(&pid_fl_v2c, v_fl, motors[CHASSIS_FL].velocity); // front left
    int16_t c_bl = pid_calculate(&pid_bl_v2c, v_bl, motors[CHASSIS_BL].velocity); // back left
    int16_t c_br = pid_calculate(&pid_br_v2c, v_br, motors[CHASSIS_BR].velocity); // back right

    // set current command
    set_chassis_command(c_fr, c_fl, c_bl, c_br);
}
