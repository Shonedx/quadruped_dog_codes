#include "Allheaderfile.h"
#include "math.h"
#include "ges_cal.h"


	double b1[3],a1[3],r1[3],r1_[3],	t1[3],t1_[3];
	double b2[3],a2[3],r2[3],r2_[3],	t2[3],t2_[3];
	double b3[3],a3[3],r3[3],r3_[3],	t3[3],t3_[3];
	double b4[3],a4[3],r4[3],r4_[3],	t4[3],t4_[3];
	double R[3][3];
	
	double Pitch[3],Roll[3],roll,pitch;



extern PidTypeDef Yaw_PID_Loop; //PID相关
extern PidTypeDef Roll_PID_Loop;
extern PidTypeDef Pitch_PID_Loop;
 
void IMU_PID_Init()
{
	Pitch_PID_Loop.Kp=1.2;
	Pitch_PID_Loop.Ki=0.01;
	Pitch_PID_Loop.Kd=1.2;
	Roll_PID_Loop.Kp=1.2;
	Roll_PID_Loop.Ki=0.01;
	Roll_PID_Loop.Kd=1.2;
}
double IMU_PID( PidTypeDef *pid,double target,double Now_Point)//位置式 for imu
{
    double Now_Error,d_Error;
	Now_Error=target-Now_Point;
	pid->error[0]+=Now_Error;
	//积分限幅
	constrain(pid->error[0],-20,20);
	d_Error=Now_Error-pid->error[1];
    pid->error[1]=Now_Error;
	pid->out=-(pid->Kp*Now_Error+pid->Ki*pid->error[0]+pid->Kd*d_Error);
	//速度目标值的限幅
	constrain(pid->out,-45/180*PI,45/180*PI);
	return pid->out;
}
double IMU_Output_Cal(double *angle,double A)
{
//	double output;
//	angle[2]=angle[1];
//	angle[1]=angle[0];
//	angle[0]=A;
//	if((fabs(angle[0])-fabs(angle[1]))>2.5/180*PI)
//	{
//		output=0;
//	}
//	else 
//	{
//		output=angle[0];
//	}
//	return output ;
	
}
void Limit(double *t, double min ,double max)
{
	double temp =sqrt((t[0]*t[0])+(t[1]*t[1])+(t[2]*t[2]));
	if(temp<min)
	{
		t[0]=t[0]*min/temp;
		t[1]=t[1]*min/temp;
		t[2]=t[2]*min/temp;
	}
	else if(temp>max)
	{
		t[0]=t[0]*max/temp;
		t[1]=t[1]*max/temp;
		t[2]=t[2]*max/temp;
	}
}
void Limit_double(double t, double min ,double max)
{
	
	if(t<min)
	{
		t=min;
	}
	else if(t>max)
	{
		t=max;
	}
}
//extern Detached_Params Detached_params;
//IMU初始化
void IMU_Control(void)
{
//	pitch=IMU_Output_Cal(&Pitch[0],Euler.pitch);
//	roll=IMU_Output_Cal(&Roll[0],Euler.roll);
	pitch=Euler.pitch;
	roll=0;
//	pitch=IMU_PID(&Pitch_PID_Loop,0,Euler.pitch);
//	roll=IMU_PID(&Roll_PID_Loop,0,Euler.roll);
	/*****腿一*****/
	b1[0]=0;	b1[1]=15.5;b1[2]=0;
	a1[0]=21.5;	a1[1]=15.5;a1[2]=15;
	r1[0]=21.5;	r1[1]=15.5;r1[2]=0;
	t1[0]=0;	t1[1]=0;	t1[2]=-15;
	/*****腿二*****/
	b2[0]=0;	b2[1]=-15.5;b2[2]=0;
	a2[0]=21.5;	a2[1]=-15.5;a2[2]=15;
	r2[0]=21.5;	r2[1]=-15.5;r2[2]=0;
	t2[0]=0;	t2[1]=0;	t2[2]=-15;
	/*****腿三*****/
	b3[0]=0;	b3[1]=15.5;	b3[2]=0;
	a3[0]=-21.5;a3[1]=15.5;	a3[2]=15;
	r3[0]=-21.5;r3[1]=15.5;	r3[2]=0;
	t3[0]=0;	t3[1]=0;	t3[2]=-15;
	/*****腿四*****/
	b4[0]=0;	b4[1]=-15.5;b4[2]=0;
	a4[0]=-21.5;a4[1]=-15.5;a4[2]=15;
	r4[0]=-21.5;r4[1]=-15.5;r4[2]=0;
	t4[0]=0;	t4[1]=0;	t4[2]=-15;
	/*****旋转矩阵*****/
	R[0][0]=cos(pitch);				R[0][1]=0;			R[0][2]=sin(pitch);
	R[1][0]=sin(roll)*sin(pitch);	R[1][1]=cos(roll);	R[1][2]=-sin(roll)*cos(pitch);
	R[2][0]=-cos(roll)*sin(pitch);	R[2][1]=sin(roll);	R[2][2]=cos(roll)*cos(pitch);
	/**********/
	r1_[0]=(cos(pitch)*r1[0]+sin(roll)*sin(pitch)*r1[1]-cos(roll)*sin(pitch)*r1[2])*15.5/(cos(roll)*r1[1]+sin(roll)*r1[2]);
	r1_[1]=15.5;
	r1_[2]=(sin(pitch)*r1[0]-sin(roll)*cos(pitch)*r1[1]+cos(roll)*cos(pitch)*r1[2])*15.5/(cos(roll)*r1[1]+sin(roll)*r1[2]);
	/**********/
	r2_[0]=(cos(pitch)*r2[0]+sin(roll)*sin(pitch)*r2[1]-cos(roll)*sin(pitch)*r2[2])*-15.5/(cos(roll)*r2[1]+sin(roll)*r2[2]);
	r2_[1]=-15.5;
	r2_[2]=(sin(pitch)*r2[0]-sin(roll)*cos(pitch)*r2[1]+cos(roll)*cos(pitch)*r2[2])*-15.5/(cos(roll)*r2[1]+sin(roll)*r2[2]);
	/**********/
	r3_[0]=(cos(pitch)*r3[0]+sin(roll)*sin(pitch)*r3[1]-cos(roll)*sin(pitch)*r3[2])*15.5/(cos(roll)*r3[1]+sin(roll)*r3[2]);
	r3_[1]=15.5;
	r3_[2]=(sin(pitch)*r3[0]-sin(roll)*cos(pitch)*r3[1]+cos(roll)*cos(pitch)*r3[2])*15.5/(cos(roll)*r3[1]+sin(roll)*r3[2]);
	/**********/
	r4_[0]=(cos(pitch)*r4[0]+sin(roll)*sin(pitch)*r4[1]-cos(roll)*sin(pitch)*r4[2])*-15.5/(cos(roll)*r4[1]+sin(roll)*r4[2]);
	r4_[1]=-15.5;
	r4_[2]=(sin(pitch)*r4[0]-sin(roll)*cos(pitch)*r4[1]+cos(roll)*cos(pitch)*r4[2])*-15.5/(cos(roll)*r4[1]+sin(roll)*r4[2]);
	/*****计算得出的每个腿的最终向量*****/
	t1_[0]=r1_[0]-a1[0];t1_[1]=r1_[1]-a1[1];t1_[2]=r1_[2]-a1[2];
	t2_[0]=r2_[0]-a2[0];t2_[1]=r2_[1]-a2[1];t2_[2]=r2_[2]-a2[2];
	t3_[0]=r3_[0]-a3[0];t3_[1]=r3_[1]-a3[1];t3_[2]=r3_[2]-a3[2];
	t4_[0]=r4_[0]-a4[0];t4_[1]=r4_[1]-a4[1];t4_[2]=r4_[2]-a4[2];
	Limit(&t1_[0],10,29);
	Limit(&t2_[0],10,29);
	Limit(&t3_[0],10,29);
	Limit(&t4_[0],10,29);
	constrain(t1_[0],-15,15);
	constrain(t2_[0],-15,15);
	constrain(t3_[0],-15,15);
	constrain(t4_[0],-15,15);
}

