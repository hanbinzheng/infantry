#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

typedef enum {
    M3508,
    M2006,
    GM6020,  // voltage pid motor
} MotorType;

typedef struct {
    // command info
    int16_t   target_current;
    
    // state info
    int16_t velocity;
    // float velocity_rad;

    int16_t current;
    int8_t temperature;

    uint16_t  angle;  // 0~8191: 0° ~ 360°
    // float total_angle_rad;  // total angle, in radians
    
    uint16_t last_angle;
    MotorType type;
} MotorInfo;

typedef enum {
    // can1: 4 x M3508 + 1 x GM6020
    CHASSIS_FR = 0,  // CAN1 ID 1, M3508, Identifier 0x201
    CHASSIS_FL,  // CAN1 ID 2, M3508, Identifier 0x202
    CHASSIS_BL,  // CAN1 ID 3, M3508, Identifier 0x203
    CHASSIS_BR,  // CAN1 ID 4, M3508, Identifier 0x204
    GIMBAL_YAW,  // CAN1 ID 5, GM6020, Identifier 0x209
    
    // can3: 2 x M3508 + 1 x GM6020 + 1 x M2006
    // we use can 3 because can3 is the only 4 pin can on dm-mc02
    GIMBAL_PITCH,  // CAN2 ID 1, GM6020, Identifier 0x205
    FRICTION_L,  // CAN2 ID 6, M3508, Identifier 0x206
    FRICTION_R,  // CAN2 ID 7, M3508, Identifier 0x207
    TRIGGER,  // CAN2 ID 8, M2006, Identifier 0x208
    
    TOTAL_MOTOR_NUM  // 9 motors in total 
} Motor_Index;

void motor_data_interpret(uint8_t *buff, MotorInfo *motor);
extern MotorInfo motors[TOTAL_MOTOR_NUM];

#endif //__MOTOR_H__
