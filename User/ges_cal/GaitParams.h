#ifndef GaitParams_H
#define GaitParams_H
#include "Allheaderfile.h"
typedef enum 
{
	Idle=0,
	Forward, 
	Back,
	Turn_Left,
	Turn_Right,
	Translate_Left,
	Translate_Right,
	Jump,
	Stop,
}State; //机器狗运行状态


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

extern GaitParams  gaitparams[][4];
extern State currentstate;

#endif