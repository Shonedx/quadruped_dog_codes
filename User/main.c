
#include "Allheaderfile.h"
#include "ticks.h"
#include	"led.h"
#include	"can.h"
#include	"pid.h"
#include 	"motor.h"
//×ËÌ¬½âËã
#include	"IMU.h"
#include 	"kalman.h"
//
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

#include    "stdio.h"
#include 	"../DEFINE/define_file.h"
//CAN_1 : PA11(RX)   PA12(TX)        CAN_2 : PB12(RX)      PB13(TX)

//æ˜¾ç¤ºå¯¹åº”å­—ç?¦ä¸²=============
char ctrlstate_names[7][15]=
{
	{"CS_NONE"},
	{"CS_INIT"},
	{"CS_MAIN"},
	{"CS_JUMP_1"},
	{"CS_JUMP_2"},
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

uint8_t rx_buffer[NRF_PAYLOAD_LENGTH]={0};
extern MotionState_t current_motion_state;
extern CtrlState_t ctrl_state;
extern bool imu_state;
extern TranslateState_t translate_state;
extern IdleState_t idle_state;
extern uint8_t pre_angle;
extern uint8_t setted_height;
extern uint8_t jump_state;
extern JumpParameter_t jump1_struct;
extern JumpParameter_t jump2_struct;

int feed=1; 
int start=0;			
int reset=1;			
int left_push_stick,right_push_stick; 

extern uint16_t rc_left_x,rc_left_y,rc_right_x,rc_right_y;
extern uint64_t timer;
extern Motors motors;

extern MovingAverageFilter_t yaw_filter;
extern MovingAverageFilter_t roll_filter;
extern MovingAverageFilter_t pitch_filter;
extern float yaw;
extern float pitch;
extern float roll;

extern Adjust_Euler_Angle adjust_euler_agl;//pid µ÷Õû¹ýºóµÄÅ·À­½Ç
extern volatile IMU_RxBuffer imu_rx; 
int main()
{
	
	HAL_InitTick(0);
	delay_init();    
	USART1_Init();	
	USART3_Init(); 
	LED_Init();		//LED
/********remote ctrl*********/	
//	remote_control_init();
	NRF24L01_Init();
	NRF24L01_Set_RX_Mode();
/*************************/	
//	KEY_Init(); 				
	IWDG_Init(IWDG_Prescaler_32, 2000); 
	TIM5_Init(); //5 ms
	TIM4_Init(); //1 ms					
	Can_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal); //can
	PID_Init();		//pid
	OLED_Init(); 	//oled

	//jump1
	jumpInit(&jump1_struct,1);
	//jump2
	jumpInit(&jump2_struct,2);

	ChangeTheGainOfPID_KP_KI_KD(SPEED_P,SPEED_I,SPEED_D,POS_P,POS_I,POS_D);
//moving average filter
	// movAveInit(&yaw_filter,10);
	// movAveInit(&roll_filter,10);
	// movAveInit(&pitch_filter,10);
/********oled init*********/	
	OLED_NewFrame();
	OLED_PrintASCIIString(0,0,"Init Done",&afont16x8,OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	delay_ms(1000);	
	
	while(1) 
	{
		NRF24L01_RxPacket_Polling(rx_buffer);
		// if(imu_rx.packet_ready) 
		// {
		// 	CopeSerial3Data((unsigned char)imu_rx.buffer);
		// 	imu_rx.packet_ready = false;
		// }
		// usart1TxDateToVofa(motors.ID[5].target_angle,motors.ID[5].absolute_angle,motors.ID[5].target_speed,motors.ID[5].current_speed,motors.ID[5].ecd/8192.0f*360.0f);
		usart1TxDateToVofa(legs[0].x,legs[0].z,legs[2].x,legs[2].z,0);
		//usart1TxDateToVofa(jump2_count[0],jump2_count[1],jump2_count[2],jump2_count[3],0);
		//usart1TxDateToVofa(Euler.roll,Euler.pitch,adjust_euler_agl.roll,adjust_euler_agl.pitch,0); 
		normalizeDatas();
		char now_time_str_buffer[20];
		sprintf(now_time_str_buffer, "%.2f", timer/1000.0f); 
		//============oled================		
		OLED_NewFrame();
		OLED_PrintASCIIString(0,0,ctrlstate_names[ctrl_state],&afont16x8,OLED_COLOR_NORMAL); //ctrl state
		// OLED_PrintASCIINum(0,16,rc_left_x,4,&afont16x8,OLED_COLOR_NORMAL);
		// OLED_PrintASCIINum(40,16,rc_left_y,4,&afont16x8,OLED_COLOR_NORMAL);
		// OLED_PrintASCIINum(0,32,rc_right_x,4,&afont16x8,OLED_COLOR_NORMAL);
		// OLED_PrintASCIINum(40,32,rc_right_y,4,&afont16x8,OLED_COLOR_NORMAL);
		OLED_PrintASCIIString(0,16,"JUMP:",&afont16x8,OLED_COLOR_NORMAL); //ctrl state
		OLED_PrintASCIINum(40,16,(uint8_t)jump_state,2,&afont16x8,OLED_COLOR_NORMAL); //ï¿½ï¿½Ô¾×´Ì¬
		OLED_PrintASCIIString(56,16,"IMU:",&afont16x8,OLED_COLOR_NORMAL); //ctrl state
		OLED_PrintASCIINum(88,16,(uint8_t)imu_state,2,&afont16x8,OLED_COLOR_NORMAL); //ï¿½ï¿½Ô¾×´Ì¬
		// OLED_PrintASCIINum(0,32,(uint8_t)setted_height,2,&afont16x8,OLED_COLOR_NORMAL); // ï¿½ß¶ï¿½
		// OLED_PrintASCIINum(50,32,(uint8_t)pre_angle,2,&afont16x8,OLED_COLOR_NORMAL);  // ï¿½ï¿½Ð±ï¿½Ç¶ï¿½
		OLED_PrintASCIIString(0,32,now_time_str_buffer,&afont16x8,OLED_COLOR_NORMAL); // ï¿½Ë¶ï¿½×´Ì¬
		OLED_PrintASCIIString(0,48,current_motion_state_names[current_motion_state],&afont16x8,OLED_COLOR_NORMAL);
		OLED_ShowFrame();
		//============================		
		//========================mainctrl =================================
		motion_state_ctrl();	
		feed_dog();
	}
	// movAveFree(&yaw_filter);
	// movAveFree(&pitch_filter);
	// movAveFree(&roll_filter);

}


