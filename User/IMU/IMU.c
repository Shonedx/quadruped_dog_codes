#include "imu.h"
#include "ges_cal.h"
#include "gaitparams.h"
#include "Allheaderfile.h"
#include "pid.h"
// R(roll)=	[1	,0			,0			]
//			[0	,cos(roll)	,-sin(roll)	]
//			[0	,sin(roll)	,cos(roll)	]	

// R(pitch)=[cos(pitch)	,0	,sin(pitch)	]
//			[0			,1	,0			]
//			[-sin(pitch),0	,cos(pitch)	]	
//R_Rotate=	R(roll)*R(pitch)=	[cos(pitch),0,sin(pitch)]
//								[sin(roll)*sin(pitch)，cos(roll),-sin(roll)*cos(roll)]
// 								[-sin(pitch)*cos(roll),sin(roll),cos(roll)*cos(pitch)]
//[x_offset,0,standheight]*R_rotate=[x_offset*cos(pitch)-standheight*sin(pitch)*cos(roll),（强制为零，因为这个坐标系用不到),x*sin(pitch)+standheight*cos(roll)*cos(pitch)]
//一般设置x_offset为0
double standheight[4] = {StandHeight,StandHeight,StandHeight ,StandHeight};
 double  x_offset[4] = { 0,0,0,0 };
void IMU_Pos_Cal(double yaw, double pitch, double roll)
{
	for (int i = 0; i < 4; i++)
	{
		standheight[i] =
			StandHeight * cos(pitch) * cos(roll);
		x_offset[i] =
			StandHeight * sin(pitch)*cos(roll);
		constrain(standheight[i], 12, 28);
	}
}
static float leg_length_for_pos_cal[4]={StandHeight,StandHeight,StandHeight,StandHeight}; 
static void imuCalRollPos(float roll)
{
	//两腿间距离270mm
	float ds=sin(roll)*27.5f/2.0f;
	leg_length_for_pos_cal[1]=StandHeight+ds;
	leg_length_for_pos_cal[3]=StandHeight+ds;
	leg_length_for_pos_cal[0]=StandHeight-ds;
	leg_length_for_pos_cal[2]=StandHeight-ds;

}
static void imuCalPitchPos(float pitch)
{
	for(int i=0;i<4;i++)
	{
		standheight[i]=
			leg_length_for_pos_cal[i]*cos(pitch);
		x_offset[i]=
			leg_length_for_pos_cal[i]*sin(pitch);
	}
}
void calPosByRollNdPitch(float roll,float pitch)
{
	imuCalRollPos(roll);
	imuCalPitchPos(pitch);
}
void setPostureOffset(void)
 {
	for (int j = 0; j < 4; j++) //只调normal步态的
	{
		gait_params[0][j].stanceheight = standheight[j];
		gait_params[0][j].x_offset = x_offset[j];
		gait_params[1][j].stanceheight = standheight[j];
		gait_params[1][j].x_offset = x_offset[j];
	}
	/* printf("standheight:%f,%f,%f,%f\n", standheight[0], standheight[1], standheight[2], standheight[3]);*/
 }


 


