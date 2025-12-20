#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include <stdint.h>

void BSP_SPI_Init(void);
uint8_t SPI2_Transfer_Byte(uint8_t Tx_Data);

#endif // __BSP_SPI_H__