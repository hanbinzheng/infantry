#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>
#include "quaternion.h"
#include "mahony.h"

typedef struct
{
    float accel[3]; // x, y, z
    float temp;
    float gyro[3]; // x, y, z
} ImuRawData;

typedef struct
{
    Quaternion q;

    // angle position data
    float angle_roll;
    float angle_pitch;
    float angle_yaw;

    // angle velocity data
    float velocity_roll;
    float velocity_pitch;
    float velocity_yaw;
} ImuData;

uint8_t imu_init(void);
// void imu_get_data(ImuRawData *data);
void imu_update(void);

typedef enum
{
    BMI088_NO_ERROR = 0x00,

    BMI088_GYRO_RANGE_ERROR = 0x01,     //
    BMI088_GYRO_BANDWIDTH_ERROR = 0x02, //
    BMI088_GYRO_LPM1_ERROR = 0x03,      //

    BMI088_ACC_RANGE_ERROR = 0x04,    //
    BMI088_ACC_CONF_ERROR = 0x05,     //
    BMI088_ACC_PWR_CTRL_ERROR = 0x06, //
    BMI088_ACC_PWR_CONF_ERROR = 0x07, //

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80, //
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,  //
    BMI088_NO_SENSOR = 0xFF,             //
} bmi_error_type;

// global variables
extern ImuRawData imu_raw_data;
extern MahonyFilter mahony_filter;
extern ImuData imu_data;

#endif // __IMU_H__
