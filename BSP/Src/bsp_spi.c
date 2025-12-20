#include "bsp_spi.h"
#include "spi.h"

// initialize spi
void BSP_SPI_Init(void)
{
    return;  // reserved
}

// returns the spi receive data after transmiting the specified data
uint8_t SPI2_Transfer_Byte(uint8_t Tx_Data)
{
	uint8_t Rx_Data = 0;
	HAL_SPI_TransmitReceive(&hspi2, &Tx_Data, &Rx_Data, 1, 100);
    return Rx_Data;
}
