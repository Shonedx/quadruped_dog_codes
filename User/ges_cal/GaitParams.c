#include "GaitParams.h"
State currentstate;

GaitParams gaitparams[][4] = {
  //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
  //	上升幅度 下降高度 站立高度 迈腿步长 频率 摆动期占比 各腿的相位 腿序号 x初始值
{//原地踏步（待机）状态 0
	{ 3.5, 0.2, StandHeight, 0, Freq*1.0, 0.25, 0, 0,X_OFFSET},
	{ 3.5, 0.2, StandHeight, 0, Freq*1.0, 0.25, 0.5, 1,X_OFFSET},
	{ 3.5, 0.2, StandHeight, 0, Freq*1.0, 0.25, 0.5, 2,X_OFFSET},
	{ 3.5, 0.2, StandHeight, 0, Freq*1.0, 0.25, 0, 3,X_OFFSET},
},
  {//前进状态 1
	{ 4.0, 0.2, StandHeight, 5, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq, 0.25, 0, 3,X_OFFSET},
},
  {//后退状态 2
	{ 4.0, 0.2, StandHeight, -5, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.2, StandHeight,-5, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5, Freq, 0.25, 0, 3,X_OFFSET},
},
{//左转弯状态 3
	{ 4.0, 0.2, StandHeight, -5, Freq*1.2, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq*1.2, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5, Freq*1.2, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq*1.2, 0.25, 0, 3,X_OFFSET},
},
{//右转弯状态 4
	{ 4.0, 0.2, StandHeight,5, Freq*1.2, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5,Freq*1.2, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.2, StandHeight, 5, Freq*1.2, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.2, StandHeight, -5, Freq*1.2, 0.25, 0, 3,X_OFFSET},
},

{//停止 5
	{ 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
{//左平移状态 6
	{ 2, 0.2, StandHeight+2, 0, Freq, 0.25, 0.5, 0,X_OFFSET},
	{ 2, 0.2, StandHeight-2, 0, Freq, 0.25, 0, 1,X_OFFSET},
	{ 2, 0.2, StandHeight+2, 0, Freq ,0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.2, StandHeight-2, 0, Freq, 0.25, 0, 3,X_OFFSET},
},
{//右平移状态 7
	{ 2, 0.2, StandHeight-2, 0, Freq, 0.25, 0.5, 0,X_OFFSET},
	{ 2, 0.2, StandHeight+2, 0, Freq, 0.25, 0, 1,X_OFFSET},
	{ 2, 0.2, StandHeight-2, 0, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.2, StandHeight+2, 0, Freq, 0.25, 0, 3,X_OFFSET},
},

////crouch

{//前进状态 1+7
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0, 0,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0, 3,X_OFFSET},
},
  {//后退状态 2+7
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0, 0,X_OFFSET},
	{ 2, 0.1, CrouchHeight,-4, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0, 3,X_OFFSET},
},
{//左转弯状态 3+7
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0, 0,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0, 3,X_OFFSET},
},
{//右转弯状态 4+7
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0, 0,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 2, 0.1, CrouchHeight, 4, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 2, 0.1, CrouchHeight, -4, Freq, 0.25, 0, 3,X_OFFSET},
},
{//停止状态 5+7
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 3,X_OFFSET},
},

//Higher
{//前进状态 1+12
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight,8, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
  {//后退状态 2+12
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight,-8, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
{//左转弯状态 3+12
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
{//右转弯状态 4+12
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, 8, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ 4.0, 0.1, HeigherHeight, -8, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
{//停止状态 5+12
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
 };