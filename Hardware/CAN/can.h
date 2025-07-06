#ifndef __CAN_H
#define __CAN_H	 
#include "motor.h"
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
	CAN_CHASSIS_ALL_ID_2 = 0x1FF,	 
} can_msg_id_e;//��id�ŵ�ַ

	
//CAN1����RX0�ж�ʹ��
extern u8 canbuf[8];
extern u8 canbuf2[8];


#define CAN1_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.			
#define CAN2_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.	
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
u8 Can_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);

u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������
u8 CAN2_Send_Msg(u8* msg,u8 len);						//��������

u8 CAN1_Receive_Msg(u8 *buf);							//��������
u8 CAN2_Receive_Msg(u8 *buf);							//��������

void Set_Datas_From_Motor(Motor_Property * motors,u8 * buf);

void M3508_ALL_ZERO_SET(void);



#endif



























