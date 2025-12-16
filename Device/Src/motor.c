#include "motor.h"

MotorInfo motors[TOTAL_MOTOR_NUM] = {
    // CAN1 motors
    [CHASSIS_FR] = {.type = M3508},
    [CHASSIS_FL] = {.type = M3508}, 
    [CHASSIS_BL] = {.type = M3508}, 
    [CHASSIS_BR] = {.type = M3508}, 
    [GIMBAL_YAW] = {.type = GM6020}, 

    // CAN2 motors
    [FRICTION_L] = {.type = M3508}, 
    [FRICTION_R] = {.type = M3508}, 
    [TRIGGER] = {.type = M2006}, 
    [GIMBAL_PITCH] = {.type = GM6020}, 
};

void motor_data_interpret(uint8_t *buff, MotorInfo *motor)
{
    motor->angle = (buff[0] << 8) | buff[1];
    motor->velocity = (buff[2] << 8) | buff[3];
    motor->current = (buff[4] << 8) | buff[5];
    motor->temperature = buff[6];
}
