
#include "Allheaderfile.h"
#include "ticks.h"
#include	"led.h"
#include	"can.h"
#include	"pid.h"
#include 	"motor.h"
#include	"IMU.h"
#include	"timer.h"
#include    "OLED.h"
#include 	"nrf24l01.h"

#include	"RemoteControl_Init.h"
#include	"RC_Command.h"
#include    "RC.h"

#include	"IWDG.h"

#include	"ges_cal.h"
#include 	"GaitParams.h"
#include 	"Jump.h"

#include	"main_params.h"
//CAN_1 : PA11(RX)   PA12(TX)        CAN_2 : PB12(RX)      PB13(TX)
//遥控输出:PA3
//陀螺仪:PB10(RX)    PB11(TX)

//test=============
char ctrlstate_names[7][15]=
{
	{"CS_NONE"},
	{"CS_INIT"},
	{"CS_MAIN"},
	{"CS_PRE_JUMP"},
	{"CS_EXE_JUMP"},
	{"CS_HEIGHT"},
	{"CS_QUIT"},
};

char current_motion_state_names[4][20]=
{
	{"MS_NORMAL"},
	{"MS_TRANSLATE_LEFT"},
	{"MS_TRANSLATE_RIGHT"},
	{"MS_STOP"},
};
//===============

// 接收数据缓冲区
uint8_t rx_buffer[NRF_PAYLOAD_LENGTH]={0};
extern uint16_t formal_datas[FORMAL_DATAS_LENGTH];
extern MotionState_t current_motion_state;
extern CtrlState_t ctrl_state;
extern TranslateState_t translate_state;
extern IdleState_t idle_state;
extern uint8_t pre_angle;
extern uint8_t setted_height;
extern uint8_t jump_state;


int feed=1; //喂狗
int start=0;			//开始标志量
int reset=1;			//遥控器在中央时reset赋1 
int left_push_stick,right_push_stick; //遥控器左上，右上推杆变量初始化

extern uint16_t rc_left_x,rc_left_y,rc_right_x,rc_right_y;


uint16_t test=0;
int main()
{
	
	HAL_InitTick(0);//初始化hal有关，因为要用到HAL_GetTick函数
	delay_init();    //delay初始化
	USART1_Init();	//串口调试
	USART3_Init();  //陀螺仪数据接收
	LED_Init();		//LED
/********远程控制*********/	
//	remote_control_init();
	NRF24L01_Init();
	NRF24L01_Set_RX_Mode();
/*************************/	
//	KEY_Init(); 				
	IWDG_Init(IWDG_Prescaler_32, 2000); 
	TIM5_Init(); //5 ms
	TIM4_Init(); //1 ms					
	Can_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal); //can
	PID_Init();		//pid控制
	OLED_Init(); 	//oled控制
	
	ChangeTheGainOfPID_KP_KI_KD(5.1,0.3,1.81,5.1,0.3,1.81);

/********非初始化区域*********/	
	OLED_NewFrame();
	OLED_PrintASCIIString(0,0,"Init Done",&afont16x8,OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	delay_ms(1000);	
	
	while(1) //主程序
	{
//		test++;
		trans_rx_buffer_to_formal_datas();
		//============测试代码================		
				OLED_NewFrame();
				OLED_PrintASCIIString(0,0,ctrlstate_names[ctrl_state],&afont16x8,OLED_COLOR_NORMAL); //ctrl state
//				OLED_PrintASCIINum(0,16,rc_left_x,4,&afont16x8,OLED_COLOR_NORMAL);
//				OLED_PrintASCIINum(40,16,rc_left_y,4,&afont16x8,OLED_COLOR_NORMAL);
//				OLED_PrintASCIINum(0,32,rc_right_x,4,&afont16x8,OLED_COLOR_NORMAL);
//				OLED_PrintASCIINum(40,32,rc_right_y,4,&afont16x8,OLED_COLOR_NORMAL);
				OLED_PrintASCIINum(0,16,(uint8_t)translate_state,2,&afont16x8,OLED_COLOR_NORMAL); //平移
				OLED_PrintASCIINum(20,16,(uint8_t)idle_state,2,&afont16x8,OLED_COLOR_NORMAL);  //停止或原地踏步
				OLED_PrintASCIINum(40,16,(uint8_t)jump_state,2,&afont16x8,OLED_COLOR_NORMAL); //跳跃状态
				OLED_PrintASCIINum(0,32,(uint8_t)setted_height,2,&afont16x8,OLED_COLOR_NORMAL); // 高度
				OLED_PrintASCIINum(50,32,(uint8_t)pre_angle,2,&afont16x8,OLED_COLOR_NORMAL);  // 倾斜角度

				OLED_PrintASCIIString(0,48,current_motion_state_names[current_motion_state],&afont16x8,OLED_COLOR_NORMAL);
				OLED_ShowFrame();
		//============测试代码================		
		//========================主程序代码=================================
				motion_state_ctrl();	
		feed_dog();
	}
}


