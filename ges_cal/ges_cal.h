#ifndef __POSTURE_CALCULATION_H
#define __POSTURE_CALCULATION_H
#include "Allheaderfile.h"
#include "sys.h"

#define L1 11.0 //14
#define L2 20.0 //30
#define L3 14.0
#define L4 30.0         //����������С�ȳ���
#define xa 3.5f            
#define xb -3.5f        //xa,xbΪ�����������,xa=-xb=motor_length/2(���ľ�/2)
#define PI 3.1415926f  //��,��Ҫת��Ϊf,ʹ��ɲ���floatֵ����
#define val 180.0/PI  //������ת������
#define AngleRaito 436.926337 //24��11.20 ���û�õ�
#define Gaito     3591/187//3508ת�ӱ�,�ǶȻ�����ʱ��Ҫ��ϣ���ĽǶȳ������
#define t_length  0.0023//ʱ�䲽�� 0.0025
#define t_length_w  0.0020 //0.0020

/***/
#define X_OFFSET 0.0f
#define Freq 1.5f
#define StepLenthMin 2.0f
#define StandHeight 18.0f
#define LegLenthMax 30.0f //ʵ��������Ȼ���һ��ԣ�� //IMU��أ���ͬ
#define LegLenthMin 11.0f //����΢��ѹ�Źؽ�


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

 void Gait(void);

void CartesianToTheta_Cycloid(Leg *leg);

void CartesianToTheta_Cycloid_All_Legs(void);

void Angle_Setting_Cycloid(int Legid);  // Moveleg �ﱻ����

void Moveleg(void);

void AllLeg_Set_angle(int target_angle, int offset);

void Motor_Auto_Run(void); //�������


#endif
