#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

void BSP_FDCAN_Init(void);
HAL_StatusTypeDef BSP_FDCAN_TxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t StdId, uint8_t *pData);

#endif // __BSP_FDCAN_H__
