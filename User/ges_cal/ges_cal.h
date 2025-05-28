#ifndef __GES_CAL_H
#define __GES_CAL_H
#include "Allheaderfile.h"
#include "sys.h"

#define L1 8.0f //14
#define L2 12.0f //30

#define xa 3.5f            
#define xb -3.5f        //xa,xbΪ�����������,xa=-xb=motor_length/2(���ľ�/2)
#define PI 3.1415f  //��,��Ҫת��Ϊf,ʹ��ɲ���floatֵ����
#define val 180.0f/PI  //������ת������
#define AngleRaito 436.926337 //24��11.20 ���û�õ�
#define Gaito     3591/187//3508ת�ӱ�,�ǶȻ�����ʱ��Ҫ��ϣ���ĽǶȳ������
#define t_length  0.0023//ʱ�䲽�� 0.0025
#define t_length_w  0.0020 //0.0020

/***/
#define X_OFFSET 0.0f
#define Freq 1.5f
#define StepLenthMin 2.0f
#define StandHeight 10.0f //max 19.5 min 5
#define CrouchHeight 7.0f
#define HeigherHeight 13.0f
#define LegLenthMax 19.5.0f 
#define LegLenthMin 5.0f 


typedef struct
{
    float x;
    float z;
    float L;
    float psai1;
    float fai1;
    float theta1;
    float theta2;
} Leg; //���Ȳ��˶��й�

extern Leg legs[4];
extern  int Jump_State;

void Gait(double t);

void CartesianToTheta_Cycloid(Leg *leg);

void CartesianToTheta_Cycloid_All_Legs(void);

void Angle_Setting_Cycloid(int Legid);  // Moveleg �ﱻ����

void Moveleg(void);

void AllLeg_Set_angle(int target_angle, int offset);

void Motor_Auto_Run(void); //�������

void Motor_Set_Current(void);

void Stand_Init(void);

#endif
