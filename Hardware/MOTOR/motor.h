#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"
typedef struct
{
    int current_angle;              // ��ǰ�Ƕ�
    int absolute_angle;            // ���ԽǶ�
	
    int target_angle;       // Ŀ��Ƕ�
	int output_angle;
	
	int16_t output_current;   // �����������
	int received_current;	//���յĵ���
	
	int current_speed;				//��ǰ�ٶ�
	int target_speed;		//Ŀ���ٶ�
	
	int last_ecd; 			//��һ�λ�е�Ƕ�
	int ecd;				//���ת�ӻ�е�Ƕȷ�Χ��0~8191����Ӧ360��

	float pos_old;
	float pos;
	
} Motor_Property; //������Խṹ��

typedef struct
{
    Motor_Property ID[8];				//�����ؽṹ��
} Motors; //�ýṹ������װ�˸����������

extern Motors motors;

#include "Allheaderfile.h"
 //���ԽǶȼ��㺯��
void Motor_Absolute_Angle_Cal(Motor_Property *motor, float T, uint8_t index); 

void Set_Max_Output_SL(int max_out);//SL Speed Loop
void Set_Max_Output_PL( int max_out); //PL Postion Loop
void setMaxSLForSingleLeg(int max_out,uint8_t leg_id);
void setMaxPLForSingleLeg(int max_out,uint8_t leg_id);

void Set_Motor_Target_Angle(int i); //����ǶȻ����

void Set_Motor_Target_Speed(int i); // �ǶȻ������Ϊ�ٶȻ�����

void Set_Motor_Output_Current(int i); //���õ�����������ĵ�����С

void Load_Data_To_Canbuf(int i); //װ�ض�Ӧ���ݵ�������
void SetZeroToCanBuf(int i);

/**********����������ݸ����*****************/
void Can1_Send_Msg_to_Motor(void); 
void Can2_Send_Msg_to_Motor(void);
/********************************************/

void Motor_Auto_Run(void); //�������
void runSingleMotor(int i);
void runSingleLeg(int i);

#endif


