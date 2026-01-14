#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

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

typedef enum
{
    M3508,
    M2006,
    GM6020, // voltage pid motor
} MotorType;

typedef struct
{
    // command info, current or voltage ( only for GM6020 )
    float command;       // for pid control
    int16_t command_int; // raw command to can tx

    // state info
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


// global variables
extern MotorInfo motors[TOTAL_MOTOR_NUM];

#endif //__MOTOR_H__
