#include "main.h"
#include "usart.h"
#include "bsp_usart.h"
#include "dbus.h"

static void USART5_RxDMA_DoubleBuffer_Init(
    UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength);
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);

// reference:
// https://zhuanlan.zhihu.com/p/720966722

// general usart init
void BSP_USART_Init(void)
{
    // initialize usart5 for dbus
    USART5_RxDMA_DoubleBuffer_Init(
        &huart5, (uint32_t *)DbusRxBuf[0], (uint32_t *)DbusRxBuf[1], DBUS_RX_BUF_NUM);
}


// initialize USART5 RX DMA double buffer
static void USART5_RxDMA_DoubleBuffer_Init(
    UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;  // UART IDLE reception mode
    huart->RxEventType = HAL_UART_RXEVENT_IDLE;
    huart->RxXferSize = DataLength;  // receive data length
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);  // enable DMA
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  // enable IDLE interrupt

    // configure DMA double buffer mode
    HAL_DMAEx_MultiBufferStart(huart->hdmarx, 
        (uint32_t)&huart->Instance->RDR, (uint32_t)DstAddress, (uint32_t)SecondMemAddress, DataLength);
 }

// rewrite HAL UART RX Event callback function
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart5) {
        USER_USART5_RxHandler(huart, Size);
    }
}

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    __HAL_DMA_DISABLE(huart->hdmarx);  // disable DMA

    // check DMA current buffer
    if(((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET) {
        // change DMA buffer and reset NDTR
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_RX_BUF_NUM);  // stm32h7xx_hal_uart.c, line 2392

        // interpret data if full frame received
        if(Size == DBUS_FRAME_LENGTH){
            dbus_data_interpret(DbusRxBuf[0], &dbus_data);
        }
    } else {
        // change buffer and reset NDTR
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_RX_BUF_NUM);

        // interpret data if full frame received
        if(Size == DBUS_FRAME_LENGTH) {
            dbus_data_interpret(DbusRxBuf[1], &dbus_data);
        }
    }

    __HAL_DMA_ENABLE(huart->hdmarx);  // enable DMA
}
