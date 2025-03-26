#ifndef __PID_H
#define __PID_H
#include <sys.h>
#include "usart.h"
//#include "robocon.h"
#include "Allheaderfile.h"
#define PWM_MAX 15000
#define PWM_MIN -15000
#define TIAOSPEED 7000
#define QIANTIAOSPEED 8000
 
 enum PID_MODE //pidλ�û����ٶȻ���ѡ��
{
    PID_POSITION = 0,
    PID_DELTA    = 1
};

typedef struct
{
    uint8_t mode;           // ����ģʽ��PID_POSITION �� PID_DELTA

    float Kp;               // ��������
    float Ki;               // ��������
    float Kd;               // ΢������

    float max_out;          // ������
    float max_iout;         // ���������

    float set;              // �趨ֵ
    float fdb;              // ����ֵ

    float out;              // �����
    float Pout;             // ���������
    float Iout;             // ���������
    float Dout;             // ΢�������
    float Dbuf[3];          // ΢�����
    float error[3];         // ����
} PidTypeDef;

typedef struct
{
    int angle;              // ��ǰ�Ƕ�
    int POS_ABS;            // ���ԽǶ�
    int target_angle;       // Ŀ��Ƕ�
	int angle_out;
    short current_output;   // �����������
	
	int current_received;	//���յĵ���
	int speed;				//��ǰ�ٶ�
	int target_speed;		//Ŀ���ٶ�
	
	int last_ecd; 			//��һ�λ�е�Ƕ�
	int ecd;				//���ת�ӻ�е�Ƕȷ�Χ��0~8191����Ӧ360��
} MOTOR_3508_MSG;

typedef struct
{
    MOTOR_3508_MSG ID1;				//�����ؽṹ��
    MOTOR_3508_MSG ID2;
    MOTOR_3508_MSG ID3;
    MOTOR_3508_MSG ID4;
    MOTOR_3508_MSG ID5;
    MOTOR_3508_MSG ID6;
    MOTOR_3508_MSG ID7;
    MOTOR_3508_MSG ID8;
} MOTOR_3508;

typedef struct
{
    PidTypeDef M3508_SPEED_ID1; //�ٶȻ�pid
    PidTypeDef M3508_SPEED_ID2;
    PidTypeDef M3508_SPEED_ID3;
    PidTypeDef M3508_SPEED_ID4;
    PidTypeDef M3508_SPEED_ID5;
    PidTypeDef M3508_SPEED_ID6;
    PidTypeDef M3508_SPEED_ID7;
    PidTypeDef M3508_SPEED_ID8;

    PidTypeDef M3508_POS_ID1;	//�Ƕ��й�
    PidTypeDef M3508_POS_ID2;
    PidTypeDef M3508_POS_ID3;
    PidTypeDef M3508_POS_ID4;
    PidTypeDef M3508_POS_ID5;
    PidTypeDef M3508_POS_ID6;
    PidTypeDef M3508_POS_ID7;
    PidTypeDef M3508_POS_ID8;

    PidTypeDef M3508_STAND_ID1;	//stand��̬λ�û�pid
    PidTypeDef M3508_STAND_ID2;
    PidTypeDef M3508_STAND_ID3;
    PidTypeDef M3508_STAND_ID4;
    PidTypeDef M3508_STAND_ID5;
    PidTypeDef M3508_STAND_ID6;
    PidTypeDef M3508_STAND_ID7;
    PidTypeDef M3508_STAND_ID8;
} PidMsg;						//���pid�ṹ��

typedef struct
{
	float motorangle1;
	float motorangle2;
	float motorangle3;
	float motorangle4;
	float motorangle5;
	float motorangle6;
	float motorangle7;
	float motorangle8;
}Dog_Motor_Angle;  			//���ȵ���Ƕ�

//����������pid.c�ļ���������
extern PidMsg pidmsg;
extern MOTOR_3508 motor_3508;
extern Dog_Motor_Angle Leg_angle;

extern int Stand_ON_OFF; //վ��״̬�������ȱ�־������RC_Command.c�б�ʹ�ã�
extern int Crouch_ON_OFF;//����״̬�������ȱ�־������RC_Command.c�б�ʹ�ã�
/***********************************/
void ChangeTheGainOfPID_KP_KI_KD(float sp_kp, float sp_ki, float sp_kd, float pos_kp, float pos_ki, float pos_kd); //��δ�ã���ͬ
void LegPID_Change(u8 LegId, float sp_kp, float sp_ki, float sp_kd, float pos_kp, float pos_ki, float pos_kd);

//���������ԽǶȣ����㴦��
void Motor_Angle_Cal_1(float T);
void Motor_Angle_Cal_2(float T);
void Motor_Angle_Cal_3(float T);
void Motor_Angle_Cal_4(float T);
void Motor_Angle_Cal_5(float T);
void Motor_Angle_Cal_6(float T);
void Motor_Angle_Cal_7(float T);
void Motor_Angle_Cal_8(float T);



void PID_Init(PidMsg *pid); //pidmsg�еĸ����ṹ���Ա��ʼ��

float PID_Calc( PidTypeDef *pid,MOTOR_3508_MSG *motor_temp); // �ٶȻ�PID��ʽ

float PID_Cal_STAND( PidTypeDef *PID ,float get);//������̬λ�û�

float PID_Cal_POSITION(PidTypeDef *PID, float get);//PIDλ�û�

void Target_Pos_Setting(PidTypeDef *pid,float target); //��AllLeg_Set_angle��ges_cal.c�У������ù�

void SetPoint(PidTypeDef *pid,float target);//IMUģ�����

void PID_Setting(PidTypeDef *pid,float kp,float kd,float ki); //����KP,KD,KI

void PID_Pos_Setting(PidTypeDef* pid,float kp,float kd,float ki);

void ChangeTheGainOfPID_KP_KI_KD(float sp_kp,float sp_ki,float sp_kd,float pos_kp,float pos_ki,float pos_kd);

void LegPID_Change(u8 LegId,float sp_kp,float sp_kd,float sp_ki,float pos_kp,float pos_kd,float pos_ki);

float constrain(float value, float min, float max); 


	
	
#endif
