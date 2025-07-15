//BSP
#include	"can.h"
#include	"IMU.h"
#include	"timer.h"
#include    "OLED.h"
#include    "oled_iic.h"
#include 	"rc_nrf24l01.h"
#include    "rc_spi.h"
#include    "delay.h"
#include    "usart.h"
#include	"IWDG.h"
#include	"led.h"
//APP
#include    "RC.h"
#include    "pid.h"
#include    "kalman.h"
#include    "motion_ctrl.h"
#include    "motor.h"
//others
#include   "stm32f4xx.h"
#include   "stm32f4xx_iwdg.h"
#include   "stm32f4xx_can.h"




//CAN_1 : PA11(RX)   PA12(TX)        CAN_2 : PB12(RX)      PB13(TX)
//遥控输出:PA3
//陀螺仪:PB10(RX)    PB11(TX)

//用来显示对应数据的字符串集
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
u8 rx_buffer[NRF_PAYLOAD_LENGTH]={0};

extern MotionState_t current_motion_state;
extern CtrlState_t ctrl_state;
extern TranslateState_t translate_state;
extern IdleState_t idle_state;
extern u8 pre_angle;
extern u8 setted_height;
extern u8 jump_state;


u8 feed=1; //喂狗
volatile u8 start=0;			//开始标志量
u8 reset=1;			//遥控器在中央时reset赋1 



u16 test=0;
int main()
{
	
	delay_init();    //delay初始化
	USART1_Init();	//串口调试
	USART3_Init();  //陀螺仪数据接收
	ledInit();		//LED
	//rc
	spiInit();
	nrfSetRxMode();
	//
	iwdgInit(IWDG_Prescaler_8, 125); 
	
	tim5Init(); //5 ms
	tim4Init(); //1 ms					
	
	canInit();
	pidInit();		//pid控制
	oledInit(); 	//oled控制
	
	setSpeedPosPid(5.1,0.3,1.81,5.1,0.3,1.81);

/********非初始化区域*********/	
	oledNewFrame();
	oledPrintStringASCII(0,0,"Init Done",&afont16x8,OLED_COLOR_NORMAL);
	oledShowFrame();
	delay_ms(1000);	
	
	while(1) //主程序
	{
//		test++;
		formalDataFromRxBuffer();
		//============测试代码================		
				oledNewFrame();
				oledPrintStringASCII(0,0,ctrlstate_names[ctrl_state],&afont16x8,OLED_COLOR_NORMAL); //ctrl state
//				oledPrintNumASCII(0,16,rc_left_x,4,&afont16x8,OLED_COLOR_NORMAL);
//				oledPrintNumASCII(40,16,rc_left_y,4,&afont16x8,OLED_COLOR_NORMAL);
//				oledPrintNumASCII(0,32,rc_right_x,4,&afont16x8,OLED_COLOR_NORMAL);
//				oledPrintNumASCII(40,32,rc_right_y,4,&afont16x8,OLED_COLOR_NORMAL);
				oledPrintNumASCII(0,16,(u8)translate_state,2,&afont16x8,OLED_COLOR_NORMAL); //平移
				oledPrintNumASCII(20,16,(u8)idle_state,2,&afont16x8,OLED_COLOR_NORMAL);  //停止或原地踏步
				oledPrintNumASCII(40,16,(u8)jump_state,2,&afont16x8,OLED_COLOR_NORMAL); //跳跃状态
				oledPrintNumASCII(0,32,(u8)setted_height,2,&afont16x8,OLED_COLOR_NORMAL); // 高度
				oledPrintNumASCII(50,32,(u8)pre_angle,2,&afont16x8,OLED_COLOR_NORMAL);  // 倾斜角度

				oledPrintStringASCII(0,48,current_motion_state_names[current_motion_state],&afont16x8,OLED_COLOR_NORMAL);
				oledShowFrame();
		//============测试代码================		
		//========================主程序代码=================================
				motionStateCtrl();	
		feedDog();
	}
}


