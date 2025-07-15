#ifndef _RC_H
#define _RC_H
#include "stm32f4xx.h"
#include "gaitparams.h"

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
}TranslateState_t; //平移

typedef enum IdleState
{
	STOP,
	NORMAL,
}IdleState_t; //原地踏步

typedef enum JumpState
{
	IDLE, 
	BEND,//俯身 step1
	LEAN,//倾斜 step2
	EXE, //执行 step3
}JumpState_t; //跳跃状态

extern TranslateState_t translate_state;
extern IdleState_t idle_state;
extern CtrlState_t ctrl_state;
extern JumpState_t jump_state;

extern u8 pre_height; // 预设高度
extern u8 pre_angle; // 预设角度

//0-4096
extern u16 rc_left_x;	//中点 2075
extern u16 rc_left_y;	//中点 2030
extern u16 rc_right_x;	//中点 2050
extern u16 rc_right_y;	//中点 2020
//中点值 middle 小于中点部分为左 or 上
extern u16 rc_left_x_md;	
extern u16 rc_left_y_md;	
extern u16 rc_right_x_md;	
extern u16 rc_right_y_md;	

void formalDataFromRxBuffer(void) ;//转换接收的数据为标准格式
u8 judgeNumInRange(u16 value, u16 min, u16 max); //判断某个数是否在一定范围
void rcMotionCtrl(void) ;//切换机器狗运动状态的控制器 
void rcSetStepLength(GaitParams_t *gait_params); //步长控制


#endif