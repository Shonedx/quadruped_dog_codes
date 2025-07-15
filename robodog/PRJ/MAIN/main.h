#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f4xx.h"
#include "rc_spi.h"

extern u8 rx_buffer[NRF_PAYLOAD_LENGTH]; // 接收数据缓冲区
extern volatile u8 start;			//开始标志量
 
#endif// _MAIN_H