#ifndef __IWDG_H
#define __IWDG_H
#include "stm32f4xx.h"
void iwdgInit(u8 prer,u16 rlr);
void iwdgFeed(void);

#endif /* __IWDG_H */