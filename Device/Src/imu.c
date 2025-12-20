#include "imu.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "bmi088_reg.h"

#ifndef PI
#define PI (3.14159265358979f)
#endif

#define G (9.81f)

#define GYRO_SENSITIVITY_1000 32.768f
#define ACCEL_SENSITIVITY_3 10922.66667f

#define _PI_OVER_180 (3.14159265358979f / 180.0f)
#define _180_OVER_PI (180.0f / 3.14159265358979f)

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_COM_WAIT_SENSOR_TIME 150  // communication wait time: 150us
#define BMI088_LONG_DELAY_TIME 100  // long delay time: 100ms

// basic read write functions
static inline void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
    SET_CS_ACCEL_LOW();
    SPI2_Transfer_Byte(reg & 0x7F);
    SPI2_Transfer_Byte(data);
    SET_CS_ACCEL_HIGH();
}

static inline void bmi088_accel_read_single_reg(uint8_t reg, uint8_t *data) {
    SET_CS_ACCEL_LOW();
    SPI2_Transfer_Byte(reg | 0x80);
    SPI2_Transfer_Byte(0x55);  // dummy write
    *data = SPI2_Transfer_Byte(0x55);
    SET_CS_ACCEL_HIGH();
}

static inline void bmi088_accel_read_multi_reg(uint8_t reg, uint8_t *rx_buf) {
    SET_CS_ACCEL_LOW();
    SPI2_Transfer_Byte(reg | 0x80);
    SPI2_Transfer_Byte(0x55); // dummy write
    for (uint8_t i = 0; i < 6; i++) {
        rx_buf[i] = SPI2_Transfer_Byte(0x55);
    }
    SET_CS_ACCEL_HIGH();
}

static inline void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t data) {
    SET_CS_GYRO_LOW();
    SPI2_Transfer_Byte(reg & 0x7F);
    SPI2_Transfer_Byte(data);
    SET_CS_GYRO_HIGH();
}

static inline void bmi088_gyro_read_single_reg(uint8_t reg, uint8_t *data) {
    SET_CS_GYRO_LOW();
    SPI2_Transfer_Byte(reg | 0x80);
    *data = SPI2_Transfer_Byte(0x55);
    SET_CS_GYRO_HIGH();
}

static inline void bmi088_gyro_read_multi_reg(uint8_t reg, uint8_t *rx_buf) {
    SET_CS_GYRO_LOW();
    SPI2_Transfer_Byte(reg | 0x80);
    for (uint8_t i = 0; i < 6; i++) {
        rx_buf[i] = SPI2_Transfer_Byte(0x55);
    }
    SET_CS_GYRO_HIGH();
}


// initialize imu
uint8_t bmi088_accel_init(void)
{
    // configure, set registers and define corresponding errors
    uint8_t BMI088_Acc_Init_Config[4][3] = {
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_CONF, BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR}
    };
    static uint8_t read_value;

    // software reset, required
    bmi088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    Delay_ms(BMI088_LONG_DELAY_TIME);
    bmi088_accel_write_single_reg(BMI088_ACC_PWR_CONF,BMI088_ACC_PWR_ACTIVE_MODE);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check commiunication is normal after reset
    bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &read_value);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (read_value != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;//**?
    }

    // write register
    for (uint8_t i = 0; i < 4; i++) {
        bmi088_accel_write_single_reg(BMI088_Acc_Init_Config[i][0], BMI088_Acc_Init_Config[i][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        bmi088_accel_read_single_reg(BMI088_Acc_Init_Config[i][0], &read_value);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (read_value != BMI088_Acc_Init_Config[i][1]) {
            return BMI088_Acc_Init_Config[i][2];  // return if error occurs
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t BMI088_Gyro_Init_Config[3][3] = {
            {BMI088_GYRO_RANGE, BMI088_GYRO_1000, BMI088_GYRO_RANGE_ERROR},
            {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
            {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR}
    };
    static uint8_t read_value;

    bmi088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    Delay_ms(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal after reset
    bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &read_value);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (read_value != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // configure registers
    for (uint8_t i = 0; i < 3; i++) {
        bmi088_gyro_write_single_reg(BMI088_Gyro_Init_Config[i][0], BMI088_Gyro_Init_Config[i][1]);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        bmi088_gyro_read_single_reg(BMI088_Gyro_Init_Config[i][0], &read_value);
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (read_value != BMI088_Gyro_Init_Config[i][1]) {
            return BMI088_Gyro_Init_Config[i][2];  // return if error occurs
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t imu_init(void)
{
    uint8_t state = BMI088_NO_ERROR;
    //uint8_t state = BMI088_NO_ERROR;
    state |= bmi088_gyro_init();
    state |= bmi088_accel_init();
    
    return state;
}


// data reading functions
void bmi088_get_temperature(uint8_t* tempbuff)
{
    uint8_t bmi_tx_byte, len = 2;
    SET_CS_ACCEL_LOW();
    bmi_tx_byte = ((BMI088_TEMP_M) | 0x80);

    SPI2_Transfer_Byte(bmi_tx_byte);
    SPI2_Transfer_Byte(bmi_tx_byte);  // dummy byte

    while (len != 0) {
        tempbuff[2 - len] = SPI2_Transfer_Byte(bmi_tx_byte);
        len--;
    }

    SET_CS_ACCEL_HIGH();
}

void get_imu_data(ImuData * data) {
    uint8_t gyro_buff[6];
    uint8_t accel_buff[6];
    uint8_t temp_buff[2];
    int16_t tmp;

    // read gyro data
    bmi088_gyro_read_multi_reg(BMI088_GYRO_X_L, gyro_buff);
    tmp = (int16_t)((gyro_buff[1] << 8) | gyro_buff[0]);
    data->gyro[0] = ((float)tmp / GYRO_SENSITIVITY_1000) * _PI_OVER_180;  // in radians
    tmp = (int16_t)((gyro_buff[3] << 8) | gyro_buff[2]);
    data->gyro[1] = ((float)tmp / GYRO_SENSITIVITY_1000) * _PI_OVER_180; 
    tmp = (int16_t)((gyro_buff[5] << 8) | gyro_buff[4]);
    data->gyro[2] = ((float)tmp / GYRO_SENSITIVITY_1000) * _PI_OVER_180;

    // read accel data
    bmi088_accel_read_multi_reg(BMI088_ACCEL_XOUT_L, accel_buff);
    tmp = (int16_t)((accel_buff[1] << 8) | accel_buff[0]);
    data->accel[0] = ((float)tmp / ACCEL_SENSITIVITY_3) * G;
    tmp = (int16_t)((accel_buff[3] << 8) | accel_buff[2]);
    data->accel[1] = ((float)tmp / ACCEL_SENSITIVITY_3) * G;
    tmp = (int16_t)((accel_buff[5] << 8) | accel_buff[4]);
    data->accel[2] = ((float)tmp / ACCEL_SENSITIVITY_3) * G;

    // read temperature data
    bmi088_get_temperature(temp_buff);
    tmp = (int16_t)((temp_buff[0] << 3) | (temp_buff[1] >> 5));
    if (tmp > 1023) {
        tmp -= 2048;
    }
    data->temp = (float)tmp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}
