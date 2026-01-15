#include "motor.h"
#include "bsp_fdcan.h"
#include "fdcan.h"

#define RPM_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 60.0f)     // rpm to rad/s
#define ANGLE_TO_RADS(value) ((float)(value) * 2 * 3.14159265359f / 8192.0f) // angle unit to rad

// GM6020
#define GM6020_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 16384.0f / 3.0f)) // -3A~0~3A, -16384~0~16384
#define GM6020_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 3.0f / 16384.0f)
#define GM6020_VOLTAGE_FLOAT_TO_INT(value) ((int16_t)((value) * 25000.0f / 24.0f)) // -24v~0~24v, -25000~0~25000
// #define GM6020_VOLTAGE_FLOAT_TO_INT(value) (int16_t)((value)) // -25000~0~25000

// M3508
#define M3508_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 16384.0f / 20.0f)) // -20A~0~20A, -16384~0~16384
#define M3508_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 20.0f / 16384.0f)

// M2006
#define M2006_CURRENT_FLOAT_TO_INT(value) ((int16_t)((value) * 10000.0f / 10.0f)) // -10A~0~10A, -10000~0~10000
#define M2006_CURRENT_INT_TO_FLOAT(value) ((float)(value) * 10.0f / 10000.0f)

/*
 **************************************************************************
 * global variables
 **************************************************************************
 */
MotorInfo motors[TOTAL_MOTOR_NUM] = {
    // CAN1 motors
    [CHASSIS_FR] = {.type = M3508},
    [CHASSIS_FL] = {.type = M3508},
    [CHASSIS_BL] = {.type = M3508},
    [CHASSIS_BR] = {.type = M3508},
    [GIMBAL_YAW] = {.type = GM6020},

    // CAN3 motors
    [FRICTION_L] = {.type = M3508},
    [FRICTION_R] = {.type = M3508},
    [TRIGGER] = {.type = M2006},
    [GIMBAL_PITCH] = {.type = GM6020},
};

/*
 **************************************************************************
 * motor init and data interpretation
 **************************************************************************
 */

void reset_motor_info(MotorInfo *motor)
{
    motor->raw_velocity = 0;
    motor->raw_current = 0;
    motor->raw_angle = 0;
    motor->temperature = 0;
    motor->angle = 0.0f;
    motor->velocity = 0.0f;
    motor->current = 0.0f;
}

void motor_init(void)
{
    for (int i = 0; i < TOTAL_MOTOR_NUM; i++)
    {
        reset_motor_info(&motors[i]);
    }
}

void motor_data_interpret(uint8_t *buff, MotorInfo *motor)
{
    // interpret feedback raw data
    motor->raw_angle = (buff[0] << 8) | buff[1];
    motor->raw_velocity = (buff[2] << 8) | buff[3];
    motor->raw_current = (buff[4] << 8) | buff[5];
    motor->temperature = buff[6];

    // convert to physical values
    motor->angle = ANGLE_TO_RADS(motor->raw_angle);
    motor->velocity = RPM_TO_RADS(motor->raw_velocity);
    switch (motor->type)
    {
    case M3508:
        motor->current = M3508_CURRENT_INT_TO_FLOAT(motor->raw_current);
        break;
    case M2006:
        motor->current = M2006_CURRENT_INT_TO_FLOAT(motor->raw_current);
        break;
    case GM6020:
        motor->current = GM6020_CURRENT_INT_TO_FLOAT(motor->raw_current);
        break;
    default:
        break;
    }
}

/*
 **************************************************************************
 * motor control interface
 **************************************************************************
 */
void motor_set_body_current(float c_fr, float c_fl, float c_bl, float c_br)
{
    // current command: front right, front left, back left, back right
    // convert float command into int
    int16_t c_fr_int = M3508_CURRENT_FLOAT_TO_INT(c_fr);
    int16_t c_fl_int = M3508_CURRENT_FLOAT_TO_INT(c_fl);
    int16_t c_bl_int = M3508_CURRENT_FLOAT_TO_INT(c_bl);
    int16_t c_br_int = M3508_CURRENT_FLOAT_TO_INT(c_br);

    // prepare data
    uint8_t data[8] = {0};
    data[0] = (c_fr_int >> 8) & 0xFF;
    data[1] = c_fr_int & 0xFF;
    data[2] = (c_fl_int >> 8) & 0xFF;
    data[3] = c_fl_int & 0xFF;
    data[4] = (c_bl_int >> 8) & 0xFF;
    data[5] = c_bl_int & 0xFF;
    data[6] = (c_br_int >> 8) & 0xFF;
    data[7] = c_br_int & 0xFF;

    // send command message
    BSP_FDCAN_TxMessage(&hfdcan1, 0x200, data);
}

void motor_set_neck_voltage(float v_yaw)
{
    // voltage command: gimbal yaw
    // convert float command into int
    int16_t v_yaw_int = GM6020_VOLTAGE_FLOAT_TO_INT(v_yaw);

    // prepare data
    uint8_t data[8] = {0};
    data[0] = (v_yaw_int >> 8) & 0xFF;
    data[1] = v_yaw_int & 0xFF;

    // send command message
    BSP_FDCAN_TxMessage(&hfdcan1, 0x2FF, data);
}

void motor_set_head_command(float v_pitch, float c_friction_left, float c_friction_right, float v_trigger)
{
    // command: gimbal pitch voltage, friction left current, friction right current, trigger current
    // convert float command into int
    int16_t v_pitch_int = GM6020_VOLTAGE_FLOAT_TO_INT(v_pitch);
    int16_t c_friction_left_int = M3508_CURRENT_FLOAT_TO_INT(c_friction_left);
    int16_t c_friction_right_int = M3508_CURRENT_FLOAT_TO_INT(c_friction_right);
    int16_t c_trigger_int = M2006_CURRENT_FLOAT_TO_INT(v_trigger);

    // prepare data
    uint8_t data[8] = {0};
    data[0] = (v_pitch_int >> 8) & 0xFF;
    data[1] = v_pitch_int & 0xFF;
    data[2] = (c_friction_left_int >> 8) & 0xFF;
    data[3] = c_friction_left_int & 0xFF;
    data[4] = (c_friction_right_int >> 8) & 0xFF;
    data[5] = c_friction_right_int & 0xFF;
    data[6] = (c_trigger_int >> 8) & 0xFF;
    data[7] = c_trigger_int & 0xFF;

    // send command message
    BSP_FDCAN_TxMessage(&hfdcan3, 0x1FF, data);
}
