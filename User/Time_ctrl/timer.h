#ifndef __TIMER_H
#define __TIMER_H

#include <sys.h>	 

extern double now_time;

void TIM4_Init(void);

void TIM5_Init(void);
void Update_Time(void) ;

#endif
