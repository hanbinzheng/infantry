#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__

#include <stdint.h>

void BSP_TIM_Init(void);
void Delay_us(uint16_t us);
void Delay_ms(uint16_t ms);

#endif // __BSP_TIM_H__