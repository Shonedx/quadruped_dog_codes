#ifndef GaitParams_H
#define GaitParams_H
#include "stdint.h"
typedef enum MotionState
{
	MS_NORMAL,
	MS_TRANSLATE_LEFT,
	MS_TRANSLATE_RIGHT,
	MS_STOP,
}MotionState_t; //机器狗运行状态

//typedef enum 
//{
//	Idle=0,
//	Normal,
//	Translate_Left,
//	Translate_Right,
//	Jump,
//	Stop,
//}State; //机器狗运行状态

typedef struct
{
    double Up_Amp; //Amplitude 上升幅度
    double Down_Amp; //下降高度
    double stanceheight; //站立高度
    double steplength;  //迈腿步长
    double freq; //频率
    double swingpercent; //摆动期占比
    double gaitoffset;  //各腿的相位
    int i; //腿序号
    double x_offset; //x初始值
} GaitParams;

extern GaitParams  gait_params[][4];
extern MotionState_t current_motion_state;

void Set_StandHeight(GaitParams *gaitparams,uint8_t height);
void Set_StepLength(GaitParams *gait_params);
#endif


