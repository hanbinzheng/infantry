#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__

#include "main.h"
#include "gpio.h"

#define SET_CS_ACCEL_LOW() HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_RESET)
#define SET_CS_ACCEL_HIGH() HAL_GPIO_WritePin(CS_ACCEL_GPIO_Port, CS_ACCEL_Pin, GPIO_PIN_SET)
#define SET_CS_GYRO_LOW() HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_RESET)
#define SET_CS_GYRO_HIGH() HAL_GPIO_WritePin(CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIO_PIN_SET)

void BSP_GPIO_Init(void);

#endif // __BSP_GPIO_H__
