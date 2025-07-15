#include "GaitParams.h"
#include "rc.h"
#include "motion_ctrl.h"



static float constrain(float value, float min, float max) { //限幅函数
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

GaitParams_t gait_params[][4] = {
  //	up_amp Down_Amp stance_height step_length freq swingpercent gaitoffset i X_Offset
	//	上摆幅度 下摆幅度
	{//Normal 0
		{ UpAmp, 0.2, StandHeight, StepLength, Freq, 0.35, 0, 0,X_Offset},
		{ UpAmp, 0.2, StandHeight, StepLength, Freq, 0.35, 0.5, 1,X_Offset},
		{ UpAmp, 0.2, StandHeight, StepLength, Freq, 0.35, 0.5, 2,X_Offset},
		{ UpAmp, 0.2, StandHeight, StepLength, Freq, 0.35, 0, 3,X_Offset},
	},
	{//停止	1
		{ 0, 0, StandHeight, 0, 0, 0, 0, 0,X_Offset},
		{ 0, 0, StandHeight, 0, 0, 0, 0, 1,X_Offset},
		{ 0, 0, StandHeight, 0, 0, 0, 0, 2,X_Offset},
		{ 0, 0, StandHeight, 0, 0, 0, 0, 3,X_Offset},
	},
	{//左平移 2
		{ UpAmp, 0.2, StandHeight-2.0, 0, Freq*1.0, 0.25, 0.5, 0,X_Offset},
		{ UpAmp, 0.2, StandHeight+2.0, 0, Freq*1.0, 0.25, 0, 1,X_Offset},
		{ UpAmp, 0.2, StandHeight-2.0, 0, Freq*1.0 ,0.25, 0.5, 2,X_Offset},
		{ UpAmp, 0.2, StandHeight+2.0, 0, Freq*1.0, 0.25, 0, 3,X_Offset},
	},
	{//右平移 3
		{ UpAmp, 0.2, StandHeight+2.0, 0, Freq*1.0, 0.25, 0.5, 0,X_Offset},
		{ UpAmp, 0.2, StandHeight-2.0, 0, Freq*1.0, 0.25, 0, 1,X_Offset},
		{ UpAmp, 0.2, StandHeight+2.0, 0, Freq*1.0, 0.25, 0.5, 2,X_Offset},
		{ UpAmp, 0.2, StandHeight-2.0, 0, Freq*1.0, 0.25, 0, 3,X_Offset},
	},
 };

void setStandHeight(GaitParams_t *gait_params,u8 tar_height)
{
	//left legs
	gait_params[0].stance_height=tar_height;
	gait_params[2].stance_height=tar_height;
	//right legs
	gait_params[1].stance_height=tar_height;
	gait_params[3].stance_height=tar_height;
}
//步长随着高度变化而变化
void setStepLength(GaitParams_t *gait_params)
{
	//left legs
	gait_params[0].step_length=gait_params[0].stance_height*0.8;
	gait_params[2].step_length=gait_params[2].stance_height*0.8;
	//right legs
	gait_params[1].step_length=gait_params[1].stance_height*0.8;
	gait_params[3].step_length=gait_params[3].stance_height*0.8;

	
}
//上摆幅度随着高度变化而变化
void setUpAmp(GaitParams_t *gait_params)
{
	//left legs
	gait_params[0].up_amp=0.25*gait_params[0].stance_height;
	gait_params[2].up_amp=0.25*gait_params[2].stance_height;
	//right legs
	gait_params[1].up_amp=0.25*gait_params[1].stance_height;
	gait_params[3].up_amp=0.25*gait_params[3].stance_height;
}
