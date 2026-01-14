#include "bsp_tim.h"
#include "tim.h"

void BSP_TIM_Init(void)
{
    // enable tim2, tim4, tim5, tim12, tim15
    HAL_TIM_Base_Start(&htim2);     // accurate 1us and 1ms
    HAL_TIM_Base_Start_IT(&htim4);  // imu update, 1000hz
    HAL_TIM_Base_Start_IT(&htim5);  // neck control, 1000hz
    HAL_TIM_Base_Start_IT(&htim12); // head control, 1000hz
    HAL_TIM_Base_Start_IT(&htim15); // chassis control, 125hz      Instance = TIM15
}

void Delay_us(uint16_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < us)
    {
        ;
    }
    return;
}

void Delay_ms(uint16_t ms)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while ((__HAL_TIM_GET_COUNTER(&htim2) - start) < (ms * 1000))
    {
        ;
    }
    return;
}
