#ifndef _RC_NRF24L01_H
#define _RC_NRF24L01_H
#include "stm32f4xx.h"

void nrfSetRxMode(void) ;
void nrfSetTxMode(void);
u8 nrfRxPacketIrq(u8 *rxbuf); 
u8 nrfTxPacketIrq(u8 *txbuf);
u8 nrfRxPacketRolling(u8 *rxbuf) ;

#endif