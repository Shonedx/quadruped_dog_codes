#ifndef _RC_H
#define _RC_H
#include "nrf24l01.h"
#include "gaitparams.h"
typedef enum CtrlState
{
	CS_NONE,
	CS_INIT,
	CS_MAIN,
	CS_SLOPE,
	CS_JUMP_1,
	CS_JUMP_2,
	CS_HEIGHT,
	CS_QUIT,
}CtrlState_t;//ctrl state

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
	ANGLE_SET,
	JUMP1,
	JUMP2,
}JumpState_t; //������״̬��

typedef enum ConnectState
{
	UNCONNECTED,
	CONNECTED
}ConnectState_t; //������״̬��

typedef struct SlopeCtrlState
{
	uint8_t slope_walk;
	uint8_t slope_jump_on;
	uint8_t slope_jump_off;
}SlopeCtrlState_t;
/* 按键ID枚举定义 */
typedef enum {
    KY_Back = 0,   // 返回键
    KY_Enter,      // 确认键
    KY_UP,         // 上方向键
    KY_DOWN,       // 下方向键
    KY_LEFT,       // 左方向键
    KY_RIGHT,      // 右方向键
    KY_Y,          // Y功能键
    KY_A,
    KY_X,           
    KY_B,           // B功能键 
} Keys;

void RC_MotionCtrl(void);
void normalizeDatas(void);
uint8_t if_in_normal_range(uint16_t value, uint16_t min, uint16_t max) ;
void RC_StepLengthCtrl(GaitParams *gaitparams);

#endif