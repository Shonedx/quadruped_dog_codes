#ifndef _MOTION_CTRL_H
#define _MOTION_CTRL_H
#include "gaitparams.h"

#define L1 12.0f //14
#define L2 24.0f //30

#define PI 3.1415f  
#define RotorRatio     3591/187//3508转子比,角度环控制时需要将希望的角度乘上这个

/***/
#define X_Offset 0.0f //横坐标偏移值
#define Freq 1.5f //运动频率
#define StepLenthMin (2.0f)
#define StandHeight (20.0f)
#define MinHeight (14.0f)
#define MaxHeight (30.0f)
#define StepLength (16.0f)//正常站立高度时的步长
#define UpAmp   (5.0f) //腿的上摆

typedef enum MotionState
{
	MS_NORMAL,
	MS_TRANSLATE_LEFT,
	MS_TRANSLATE_RIGHT,
	MS_STOP,
}MotionState_t; //机器狗运行状态

typedef struct LegParameter
{
    float x;
    float z;
    float L;
    float psai1;
    float fai1;
    float theta1;
    float theta2;
} LegParameter_t; //和腿部运动有关

extern MotionState_t current_motion_state;


void sinTrajectory(float t, GaitParams_t gait_params); //正弦轨迹逆解
void motionStateCtrl(void);
void gaitCtrl(float t);
void transXYToAngle(LegParameter_t *leg);
void transXYToAngle_SUM(void) ;
void setMotionOutAngle(void) ;


#endif //_MOTION_CTRL_H
