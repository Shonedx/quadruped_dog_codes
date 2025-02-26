

#include "Allheaderfile.h"
#include "ticks.h"

//CAN_1 : PA11(RX)   PA12(TX)        CAN_2 : PB12(RX)      PB13(TX)
//ң�����:PA3
//������:PB10(RX)    PB11(TX)



int feed=1; //ι��
int start=0;			//��ʼ��־��
int reset=1;			//ң����������ʱreset��1 
int left_push_stick,right_push_stick; //ң�������ϣ������Ƹ˱�����ʼ��

int main()
{
	HAL_InitTick(15);//��ʼ��hal�йأ���ΪҪ�õ�HAL_GetTick����
	
	delay_init();    
//	uart_init(9600);	//������������
	uart3_init(9600);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	LED_Init();					//LED 
	remote_control_init();
	KEY_Init(); 				

	IWDG_Init(4,500);  //Tout=((4*2^prer)*rlr)/32 (ms)  1s
	TIM5_Init(5000-1,84-1);				//5ms  
	TIM4_Init(65535,0);					//0.8ms
	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal);//CAN1 1Mbps 
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal);//CAN2 1Mbps 
	
	PID_Init(&pidmsg);	
	IMU_PID_Init();

	ChangeTheGainOfPID_KP_KI_KD(8.2,0.3,4.81,9.1,0.3,1.82);

	while(1) //������
	{
		//ִ��Զ�̿����߼�
		Remote_Cmd();
	}
}
 
