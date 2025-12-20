#ifndef __IMU_H__
#define __IMU_H__

#include <stdint.h>

typedef struct {
    float accel[3];
    float temp;
    float gyro[3];
} ImuData;

uint8_t imu_init(void);
void get_imu_data(ImuData * data);

typedef enum {
    BMI088_NO_ERROR = 0x00,

    BMI088_GYRO_RANGE_ERROR = 0x01, //
    BMI088_GYRO_BANDWIDTH_ERROR = 0x02, //
    BMI088_GYRO_LPM1_ERROR = 0x03, //

    BMI088_ACC_RANGE_ERROR = 0x04, //
    BMI088_ACC_CONF_ERROR = 0x05, //
    BMI088_ACC_PWR_CTRL_ERROR = 0x06, //
    BMI088_ACC_PWR_CONF_ERROR = 0x07, //

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80, //
    BMI088_SELF_TEST_GYRO_ERROR = 0x40, //
    BMI088_NO_SENSOR = 0xFF, //
} bmi_error_type;

#endif // __IMU_H__
