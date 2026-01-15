#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

typedef enum
{
    M3508,
    M2006,
    GM6020, // voltage pid motor
} MotorType;

typedef struct
{
    int16_t raw_velocity; // rpm
    int16_t raw_current;
    uint16_t raw_angle; // 0~8191: 0° ~ 360°
    int8_t temperature;

    float angle;    // rad
    float velocity; // rad/s
    float current;  // A

    MotorType type;
} MotorInfo;

typedef enum
{
    // can1: 4 x M3508 + 1 x GM6020
    CHASSIS_FR = 0, // CAN1 ID 1, M3508, Identifier 0x201
    CHASSIS_FL,     // CAN1 ID 2, M3508, Identifier 0x202
    CHASSIS_BL,     // CAN1 ID 3, M3508, Identifier 0x203
    CHASSIS_BR,     // CAN1 ID 4, M3508, Identifier 0x204
    GIMBAL_YAW,     // CAN1 ID 5, GM6020, Identifier 0x209

    // can3: 2 x M3508 + 1 x GM6020 + 1 x M2006
    GIMBAL_PITCH, // CAN3 ID 1, GM6020, Identifier 0x205
    FRICTION_L,   // CAN3 ID 6, M3508, Identifier 0x206
    FRICTION_R,   // CAN3 ID 7, M3508, Identifier 0x207
    TRIGGER,      // CAN3 ID 8, M2006, Identifier 0x208

    TOTAL_MOTOR_NUM // 9 motors in total
} Motor_Index;

void motor_init(void);
void motor_data_interpret(uint8_t *buff, MotorInfo *motor);

// motor control interface
void motor_set_body_current(float c_fr, float c_fl, float c_bl, float c_br);
void motor_set_neck_voltage(float v_yaw);
void motor_set_head_command(float v_pitch, float c_friction_left, float c_friction_right, float v_trigger);

// global variables
extern MotorInfo motors[TOTAL_MOTOR_NUM];

#endif //__MOTOR_H__
