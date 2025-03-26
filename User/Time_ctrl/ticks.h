#include "stdint.h"



void SysTick_Handler(void);
void HAL_InitTick(uint32_t TickPriority);
uint32_t HAL_GetTick(void);
void feed_dog(void); //看门狗相关
