#ifndef _RC_H
#define _RC_H
#include "nrf24l01.h"
#include "gaitparams.h"
#define FORMAL_DATAS_LENGTH (NRF_PAYLOAD_LENGTH)-1
typedef enum CtrlState
{
	CS_NONE,
	CS_INIT,
	CS_MAIN,
	CS_PRE_JUMP,
	CS_EXE_JUMP,
	CS_HEIGHT,
	CS_QUIT,
}CtrlState_t; //控制类状态机
typedef enum TranslateState
{
	TRANS_DISABLE,
	TRANS_ENABLE,
}TranslateState_t; //控制类状态机
typedef enum IdleState
{
	STOP,
	NORMAL,
}IdleState_t; //控制类状态机
typedef enum JumpState
{
	IDLE, 
	BEND,//俯身 step1
	LEAN,//倾斜 step2
	EXE, //执行 step3
}JumpState_t; //控制类状态机

typedef enum ConnectState
{
	UNCONNECTED,
	CONNECTED
}ConnectState_t; //控制类状态机

extern uint16_t formal_datas[FORMAL_DATAS_LENGTH];

void RC_MotionCtrl(void);
void trans_rx_buffer_to_formal_datas(void);
uint8_t if_in_normal_range(uint16_t value, uint16_t min, uint16_t max) ;
void RC_StepLengthCtrl(GaitParams *gaitparams);

#endif