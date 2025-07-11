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
}CtrlState_t; //������״̬��
typedef enum TranslateState
{
	TRANS_DISABLE,
	TRANS_ENABLE,
}TranslateState_t; //������״̬��
typedef enum IdleState
{
	STOP,
	NORMAL,
}IdleState_t; //������״̬��
typedef enum JumpState
{
	IDLE, 
	BEND,//���� step1
	LEAN,//��б step2
	EXE, //ִ�� step3
}JumpState_t; //������״̬��

typedef enum ConnectState
{
	UNCONNECTED,
	CONNECTED
}ConnectState_t; //������״̬��

extern uint16_t formal_datas[FORMAL_DATAS_LENGTH];

void RC_MotionCtrl(void);
void trans_rx_buffer_to_formal_datas(void);
uint8_t if_in_normal_range(uint16_t value, uint16_t min, uint16_t max) ;
void RC_StepLengthCtrl(GaitParams *gaitparams);

#endif