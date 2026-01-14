#include "motor.h"

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
    motor->command = 0.0f;
    motor->command_int = 0;
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
