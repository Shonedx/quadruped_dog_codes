#include "imu.h"
#include "math.h"

 double standheight[4] = {StandHeight,StandHeight,StandHeight ,StandHeight};
 double  x_offset[4] = { 0,0,0,0 };
void IMU_Pos_Cal(double yaw, double pitch, double roll)
{
	for (int i = 0; i < 4; i++)
	{
		standheight[i] =
			StandHeight * cos(pitch) * cos(roll);
		x_offset[i] =
			StandHeight * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch));
		constrain(standheight[i], 12, 28);
	}
}
 void Set_Standheight_Offset(void)
 {
	 for (int i = 0; i < 8; i++)
	 {
		 for (int j = 0; j < 4; j++)
		 {
			 gaitparams[i][j].stanceheight = standheight[j];
			 gaitparams[i][j].x_offset = x_offset[j];
		 }
	 }
	/* printf("standheight:%f,%f,%f,%f\n", standheight[0], standheight[1], standheight[2], standheight[3]);*/
 }


 


