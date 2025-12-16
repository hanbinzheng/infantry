#include "fdcan.h"
#include "motor.h"
#include "bsp_fdcan.h"
#include "stm32h7xx_hal.h"

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// initialize FDCAN peripheral
void FDCAN1_Init()
{
    // confitgure can filter
    FDCAN_FilterTypeDef FDCAN1_FilterConfig;
    FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN1_FilterConfig.FilterIndex = 0;
    FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_FilterConfig.FilterID1 = 0x00000000;
    FDCAN1_FilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    // activate fdcan receive interrupt
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    // start fdcan peripheral
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

void FDCAN3_Init()
{
    // confitgure can filter
    FDCAN_FilterTypeDef FDCAN3_FilterConfig;
    FDCAN3_FilterConfig.IdType = FDCAN_STANDARD_ID;
    FDCAN3_FilterConfig.FilterIndex = 0;
    FDCAN3_FilterConfig.FilterType = FDCAN_FILTER_MASK;
    FDCAN3_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    FDCAN3_FilterConfig.FilterID1 = 0x00000000;
    FDCAN3_FilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,
                                     FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    // activate fdcan receive interrupt
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

    // start fdcan peripheral
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
    }
}

void BSP_FDCAN_Init(void)
{
    FDCAN1_Init();
    FDCAN3_Init();
}

// FDCAN Transmit Function
HAL_StatusTypeDef BSP_FDCAN_TxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t StdId, uint8_t *pData)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    // configure TxHeader
    TxHeader.Identifier = StdId;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // dji-can always use 8 bytes
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // standard can
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // transmit can message
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, pData);
}

FDCAN_RxHeaderTypeDef RxHeader1; // for can 1
FDCAN_RxHeaderTypeDef RxHeader3; // for can 3
uint8_t RxData1[8];
uint8_t RxData3[8];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // receive message
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, RxData1) != HAL_OK)
    {
        return;
    }

    // interpret message
    switch (RxHeader1.Identifier)
    {
    case 0x201: // chassis front right motor
        motor_data_interpret(RxData1, &motors[CHASSIS_FR]);
        break;
    case 0x202: // chassis front left motor
        motor_data_interpret(RxData1, &motors[CHASSIS_FL]);
        break;
    case 0x203: // chassis back left motor
        motor_data_interpret(RxData1, &motors[CHASSIS_BL]);
        break;
    case 0x204: // chassis back right motor
        motor_data_interpret(RxData1, &motors[CHASSIS_BR]);
        break;
    case 0x209: // gimbal yaw motor
        motor_data_interpret(RxData1, &motors[GIMBAL_YAW]);
        break;
    default:
        break;
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    // receive message
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader3, RxData3) != HAL_OK)
    {
        return;
    }

    // interpret message
    switch (RxHeader3.Identifier)
    {
    case 0x205: // gimbal pitch motor
        motor_data_interpret(RxData3, &motors[GIMBAL_PITCH]);
        break;
    case 0x206: // friction left motor
        motor_data_interpret(RxData3, &motors[FRICTION_L]);
        break;
    case 0x207: // friction right motor
        motor_data_interpret(RxData3, &motors[FRICTION_R]);
        break;
    case 0x208: // trigger motor
        motor_data_interpret(RxData3, &motors[TRIGGER]);
        break;
    default:
        break;
    }
}
