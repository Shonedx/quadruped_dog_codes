#ifndef __POSTURE_CALCULATION_H
#define __POSTURE_CALCULATION_H
#include "Allheaderfile.h"
#include "sys.h"

#define L1 11.0 //14
#define L2 20.0 //30
#define L3 14.0
#define L4 30.0         //两大腿与两小腿长度
#define xa 3.5f            
#define xb -3.5f        //xa,xb为两电机点坐标,xa=-xb=motor_length/2(轴心距/2)
#define PI 3.1415926f  //Π,需要转换为f,使其可参与float值运算
#define val 180.0/PI  //弧度制转换参数
#define AngleRaito 436.926337 //24年11.20 这个没用到
#define Gaito     3591/187//3508转子比,角度环控制时需要将希望的角度乘上这个
#define t_length  0.0023//时间步长 0.0025
#define t_length_w  0.0020 //0.0020

/***/
#define X_OFFSET 0.0f
#define Freq 1.5f
#define StepLenthMin 2.0f
#define StandHeight 18.0f
#define LegLenthMax 30.0f //实际上离最长腿还有一定裕度 //IMU相关，下同
#define LegLenthMin 11.0f //有略微挤压髋关节


typedef struct
{
    float x;
    float z;
    float L;
    float psai1;
    float fai1;
    float theta1;
    float theta2;
} Leg; //和腿部运动有关

extern Leg legs[4];
extern  int Jump_State;

 void Gait(void);

void CartesianToTheta_Cycloid(Leg *leg);

void CartesianToTheta_Cycloid_All_Legs(void);

void Angle_Setting_Cycloid(int Legid);  // Moveleg 里被调用

void Moveleg(void);

void AllLeg_Set_angle(int target_angle, int offset);

void Motor_Auto_Run(void); //驱动电机


#endif
