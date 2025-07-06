#ifndef __MOTOR_H
#define __MOTOR_H

typedef struct
{
    int current_angle;              // 当前角度
    int absolute_angle;            // 绝对角度
	
    int target_angle;       // 目标角度
	int output_angle;
	
    short output_current;   // 最终输出电流
	int received_current;	//接收的电流
	
	int current_speed;				//当前速度
	int target_speed;		//目标速度
	
	int last_ecd; 			//上一次机械角度
	int ecd;				//获得转子机械角度范围是0~8191，对应360度
	
} Motor_Property; //电机属性结构体

typedef struct
{
    Motor_Property ID[8];				//电机相关结构体
} Motors; //该结构体用来装八个电机的属性

extern Motors motors;

#include "Allheaderfile.h"
 //绝对角度计算函数
void Motor_Absolute_Angle_Cal(Motor_Property *motor, float T, uint8_t index); 

//设置速度环最大输出
void Set_Max_Output_SL(int max_out);//SL Speed Loop

//设置角度环最大输出
void Set_Max_Output_PL( int max_out); //PL Postion Loop

void Set_Motor_Target_Angle(int i); //计算角度环输出

void Set_Motor_Target_Speed(int i); // 角度环输出作为速度环输入

void Set_Motor_Output_Current(int i); //设置电调输出给电机的电流大小

void Load_Data_To_Canbuf(int i); //装载对应数据到缓存区
void SetZeroToCanBuf(int i);

/**********电调发送数据给电机*****************/
void Can1_Send_Msg_to_Motor(void); 
void Can2_Send_Msg_to_Motor(void);
/********************************************/

void Motor_Auto_Run(void); //驱动电机



#endif


