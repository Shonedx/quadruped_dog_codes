#ifndef __GES_CAL_H
#define __GES_CAL_H


#define L1 12.0f //14
#define L2 24.0f //30

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
#define Freq 2.5f //1.5-2.5
#define StepLenthMin 2.0f
#define StandHeight (20.0f) //max 30  min 12
#define MIN_HEIGHT (14.0f)
#define MAX_HEIGHT (35.0f)
//pid
#define SPEED_P 12
#define SPEED_I 0.1
#define SPEED_D 0.01
#define POS_P 15
#define POS_I 0.01
#define POS_D 0.5

#include "stm32f4xx.h"
#include "gaitparams.h"

typedef struct
{
    float x;
    float z;
    float L;
    float psai1;
    float fai1;
    float theta1;
    float theta2;
    float p;
    float prev_t;
} Leg; //���Ȳ��˶��й�

typedef struct  Rotate_Stretch
{
    float rotate_angle; //旋转角度
    float stretch_length; //伸展长度
    
    float rotate_time; //计数最大时间 0-1
    float stretch_time; 
    
    float rotate_freq; //计数频率
    float stretch_freq; 

    float rotate_count; //计数
    float stretch_count; 

    float rotate_prev_t[2]; //上一次计数时间
    float stretch_prev_t[2]; 
    
    float rotate_k; //比例系数 
    float stretch_k; //比例系数
    
    //完成置位
    u8 flag;
}Rotate_Stretch_t; //配合rotateAndStretch函数使用


extern Leg legs[4];


void motion_state_ctrl(void);

void Gait(double t);

void CartesianToTheta_Cycloid(Leg *leg);

void CartesianToTheta_Cycloid_All_Legs(void);

void Angle_Setting_Cycloid(int Legid);  // Moveleg �ﱻ����

void Moveleg(void);

void AllLeg_Set_angle(int target_angle, int offset);

void Motor_Auto_Run(void); //�������

void Motor_Set_Current(void);

void Stand_Init(void);

u8 rotateAndStretch(float t,  float rotate_angle, float stretch_length
                    ,float original_length,float original_angle
                    ,Leg *leg,Rotate_Stretch_t *rs); //函数内部会将flag置位

void updatePrevTime(float t);

#endif
