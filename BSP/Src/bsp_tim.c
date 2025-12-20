#include "bsp_tim.h"
#include "tim.h"

void BSP_TIM_Init(void)
{
    // enable tim2
    HAL_TIM_Base_Start(&htim2);

    // leave for future control use
}

void Delay_us(uint16_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while((__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}

void Delay_ms(uint16_t ms)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while((__HAL_TIM_GET_COUNTER(&htim2) - start) < (ms * 1000));
}
