#include "GaitParams.h"
State currentstate;

GaitParams gaitparams[][4] = {
  //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
  //	上升幅度 下降高度 站立高度 迈腿步长 频率 摆动期占比 各腿的相位 腿序号 x初始值
{//原地踏步（待机）状态 0
	{ 4, 0.2, StandHeight, 0, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq, 0.25, 0, 3,X_OFFSET},
},
  {//前进状态 1
	{ 5, 0.2, StandHeight, 10, Freq, 0.25, 0, 0,X_OFFSET},
	{ 5, 0.2, StandHeight, 10, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 5, 0.2, StandHeight, 10, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 5, 0.2, StandHeight, 10, Freq, 0.25, 0, 3,X_OFFSET},
},
  {//后退状态 2
	{ 5, 0.2, StandHeight, -10, Freq, 0.25, 0, 0,X_OFFSET},
	{ 5, 0.2, StandHeight,-10, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 5, 0.2, StandHeight, -10, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 5, 0.2, StandHeight, -10, Freq, 0.25, 0, 3,X_OFFSET},
},
{//左转弯状态 3
	{ 4, 0.2, StandHeight, -8, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight, 8, Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight, -8, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight, 8, Freq, 0.25, 0, 3,X_OFFSET},
},
{//右转弯状态 4
	{ 4, 0.2, StandHeight, 8, Freq, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight, -8,Freq, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight, 8, Freq, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight, -8, Freq, 0.25, 0, 3,X_OFFSET},
},

{//停止 5
	{ 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
{//左平移状态 6
	{ 4, 0.2, StandHeight, 0, Freq+0.5, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight-3.5, 0, Freq+0.5, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq+0.5, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight-3.5, 0, Freq+0.5, 0.25, 0, 3,X_OFFSET},
},
{//右平移状态 7
	{ 4, 0.2, StandHeight-3.5, 0, Freq+0.5, 0.25, 0, 0,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq+0.5, 0.25, 0.5, 1,X_OFFSET},
	{ 4, 0.2, StandHeight-3.5, 0, Freq+0.5, 0.25, 0.5, 2,X_OFFSET},
	{ 4, 0.2, StandHeight, 0, Freq+0.5, 0.25, 0, 3,X_OFFSET},
},

////Climbing
//{//原地踏步（待机）状态 0+6
//	{ 3, 0.1, StandHeight, 0, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight, 0, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, 0, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, 0, 3, 0.25, 0, 3,X_OFFSET},
//},
//{//前进状态 1+6
//	{ 3, 0.1, StandHeight, 3, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight, 3, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, 3, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, 3, 3, 0.25, 0, 3,X_OFFSET},
//},
//  {//后退状态 2+6
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight,-4, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0, 3,X_OFFSET},
//},
//{//左转弯状态 3+6
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight, 4, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, 4, 3, 0.25, 0, 3,X_OFFSET},
//},
//{//右转弯状态 4+6
//	{ 3, 0.1, StandHeight, 4, 3, 0.25, 0, 0,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0.5, 1,X_OFFSET},
//	{ 3, 0.1, StandHeight, 4, 3, 0.25, 0.5, 2,X_OFFSET},
//	{ 3, 0.1, StandHeight, -4, 3, 0.25, 0, 3,X_OFFSET},
//},
 };