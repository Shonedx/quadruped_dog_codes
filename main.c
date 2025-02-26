

#include "Allheaderfile.h"
#include "ticks.h"

//CAN_1 : PA11(RX)   PA12(TX)        CAN_2 : PB12(RX)      PB13(TX)
//遥控输出:PA3
//陀螺仪:PB10(RX)    PB11(TX)



int feed=1; //喂狗
int start=0;			//开始标志量
int reset=1;			//遥控器在中央时reset赋1 
int left_push_stick,right_push_stick; //遥控器左上，右上推杆变量初始化

int main()
{
	HAL_InitTick(15);//初始化hal有关，因为要用到HAL_GetTick函数
	
	delay_init();    
//	uart_init(9600);	//调试陀螺仪用
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

	while(1) //主程序
	{
		//执行远程控制逻辑
		Remote_Cmd();
	}
}
 
