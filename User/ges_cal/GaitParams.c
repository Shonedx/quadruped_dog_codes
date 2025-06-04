#include "GaitParams.h"
State currentstate;
const float step_length=8; //�������߲���
const float higher_step_length=10; //�������߲���
const float crouch_step_length=4; //�¶����߲���
const float trun_step_length=6; //ת�䲽��
const float up_amp=4;
const float higher_up_amp=5;
const float crouch_up_amp=2.5;
GaitParams gaitparams[][4] = {
  //	Up_Amp Down_Amp stanceheight steplength freq swingpercent gaitoffset i x_offset
	//	上摆幅度 下摆幅度
//{//idle ״̬ 0
//	{ up_amp, 0.2, StandHeight, 0, Freq*1.0, 0.35, 0, 0,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, 0, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, 0, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, 0, Freq*1.0, 0.35, 0, 3,X_OFFSET},
//},
//  {//前进״̬ 1
//	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0, 0,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0.5, 1,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0.5, 2,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0, 3,X_OFFSET},
//},
//  {//后退״̬ 2
//	{ up_amp, 0.2, StandHeight, -step_length, Freq, 0.35, 0, 0,X_OFFSET},
//	{ up_amp, 0.2, StandHeight,-step_length, Freq, 0.35, 0.5, 1,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, -step_length, Freq, 0.35, 0.5, 2,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, -step_length, Freq, 0.35, 0, 3,X_OFFSET},
//},
//{//左转3
//	{ up_amp, 0.2, StandHeight, -trun_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, trun_step_length, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, -trun_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, trun_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
//},
//{//右转4
//	{ up_amp, 0.2, StandHeight,trun_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, -trun_step_length,Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
//	{ up_amp, 0.2, StandHeight,trun_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
//	{ up_amp, 0.2, StandHeight, -trun_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
//},
 {//Normal 0
	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0, 0,X_OFFSET},
	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0.5, 1,X_OFFSET},
	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0.5, 2,X_OFFSET},
	{ up_amp, 0.2, StandHeight, step_length, Freq, 0.35, 0, 3,X_OFFSET},
},
{//停止	1
	{ 0, 0, StandHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, StandHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
{//左平移 2
	{ up_amp, 0.2, StandHeight-1, 0, Freq*1.5, 0.5, 0.5, 0,X_OFFSET},
	{ up_amp, 0.2, StandHeight+3, 0, Freq*1.5, 0.5, 0, 1,X_OFFSET},
	{ up_amp, 0.2, StandHeight-1, 0, Freq*1.5 ,0.5, 0.5, 2,X_OFFSET},
	{ up_amp, 0.2, StandHeight+3, 0, Freq*1.5, 0.5, 0, 3,X_OFFSET},
},
{//右平移 3
	{ up_amp, 0.2, StandHeight+3, 0, Freq*1.5, 0.5, 0.5, 0,X_OFFSET},
	{ up_amp, 0.2, StandHeight-1, 0, Freq*1.5, 0.5, 0, 1,X_OFFSET},
	{ up_amp, 0.2, StandHeight+3, 0, Freq*1.5, 0.5, 0.5, 2,X_OFFSET},
	{ up_amp, 0.2, StandHeight-1, 0, Freq*1.5, 0.5, 0, 3,X_OFFSET},
},

//crouch

//{//前进״̬ 1+7
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
//},
//  {//后退״̬ 2+7
//	{ crouch_up_amp, 0.1, CrouchHeight, -crouch_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight,-crouch_step_length, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, -crouch_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, -crouch_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
//},
//{//左转3+7
//	{ crouch_up_amp, 0.1, CrouchHeight, -crouch_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, -crouch_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
//},
//{//右转4+7
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, -crouch_step_length, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
//	{ crouch_up_amp, 0.1, CrouchHeight, -crouch_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
//},
{// Normal 0+4
	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 0,X_OFFSET},
	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 1,X_OFFSET},
	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0.5, 2,X_OFFSET},
	{ crouch_up_amp, 0.1, CrouchHeight, crouch_step_length, Freq*1.0, 0.35, 0, 3,X_OFFSET},
},
{//停止 1+4
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, CrouchHeight, 0, 0, 0, 0, 3,X_OFFSET},
},

//Higher
//{//前进״̬ 1+12
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 0,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight,higher_step_length, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 3,X_OFFSET},
//},
//  {//后退״̬ 2+12
//	{ higher_up_amp, 0.1, HeigherHeight, -higher_step_length, Freq*0.8, 0.25, 0, 0,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight,-higher_step_length, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, -higher_step_length, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, -higher_step_length, Freq*0.8, 0.25, 0, 3,X_OFFSET},
//},
//{//左转3+12
//	{ higher_up_amp, 0.1, HeigherHeight, -higher_step_length, Freq*0.8, 0.25, 0, 0,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, -higher_step_length, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 3,X_OFFSET},
//},
//{//右转4+12
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 0,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, -higher_step_length, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
//	{ higher_up_amp, 0.1, HeigherHeight, -higher_step_length, Freq*0.8, 0.25, 0, 3,X_OFFSET},
//},
{//Normal 0+6
	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 0,X_OFFSET},
	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0.5, 1,X_OFFSET},
	{ higher_up_amp, 0.1, HeigherHeight,higher_step_length, Freq*0.8, 0.25, 0.5, 2,X_OFFSET},
	{ higher_up_amp, 0.1, HeigherHeight, higher_step_length, Freq*0.8, 0.25, 0, 3,X_OFFSET},
},
{//停止 1+6
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 0,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 1,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 2,X_OFFSET},
	{ 0, 0, HeigherHeight, 0, 0, 0, 0, 3,X_OFFSET},
},
 };

 void Set_StepLength(GaitParams *gaitparams)
 {
	if(left_x<=100&&left_x>=-100)
	{
		//left legs
		gaitparams[0].steplength=(float)left_y/660.0f*step_length;
		gaitparams[2].steplength=(float)left_y/660.0f*step_length;
		//right legs
		gaitparams[1].steplength=(float)left_y/660.0f*step_length;
		gaitparams[3].steplength=(float)left_y/660.0f*step_length;
	}
	else
	{
		//left legs
		gaitparams[0].steplength=(float)left_x/660.0f*step_length;
		gaitparams[2].steplength=(float)left_x/660.0f*step_length;
		//right legs
		gaitparams[1].steplength=-(float)left_x/660.0f*step_length;
		gaitparams[3].steplength=-(float)left_x/660.0f*step_length;
	}
	if(left_x<=100
		&&left_x>=-100
		&&left_y<=100
		&&left_y>=-100)
	{
		//left legs
		gaitparams[0].steplength*=0;
		gaitparams[2].steplength*=0;
		//right legs
		gaitparams[1].steplength*=0;
		gaitparams[3].steplength*=0;
	}
	
 }