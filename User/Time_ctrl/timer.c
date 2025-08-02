
#include "Allheaderfile.h"
#include "ticks.h"
#include "timer.h"
#include "nrf24l01.h"
#include "RC.h"
#include "led.h"
#include "jump.h"
#include "sys.h"
#include "Kalman.h"
#include "pid.h"
#include "IMU.h"
#include "usart.h"
#include "ges_cal.h"
extern uint8_t rx_buffer[NRF_PAYLOAD_LENGTH];
extern int start;
extern CtrlState_t ctrl_state;
uint64_t timer=0;
 static void Update_Time(void) //更新时间
 {
	    timer++;
 }

void TIM4_Init(void) //1 ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;   
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;   
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); 

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM4, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;             
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;     
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);
}
void TIM5_Init(void) //2 ms
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=2000-1;  
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM5,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
extern MovingAverageFilter_t yaw_filter;
extern MovingAverageFilter_t roll_filter;
extern MovingAverageFilter_t pitch_filter; //滤波
void TIM4_IRQHandler() 
{  
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
	{
		Update_Time();
		// NRF24L01_RxPacket_IRQ(rx_buffer);
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
extern JumpState_t jump_state;
extern JumpParameter_t jump1_struct;
extern JumpParameter_t jump2_struct;
extern uint8_t imu_state;
extern CtrlState_t ctrl_state;
float yaw,pitch,roll;
extern IMU_Euler_Angle_Pid imu_euler_angle_pid;
Adjust_Euler_Angle adjust_euler_agl;//pid 调整过后的欧拉角
extern Euler_t Euler;
extern uint8_t init_done;
void TIM5_IRQHandler(void) 
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
	{		
		if(init_done)
		{
			//获得陀螺仪欧拉角
			// yaw=movAveUpdate(&yaw_filter,Euler.yaw);
			roll=movAveUpdate(&roll_filter,Euler.roll);
			pitch=movAveUpdate(&pitch_filter,Euler.pitch);
			adjust_euler_agl.pitch=IMU_pidCal(&imu_euler_angle_pid.Pitch,pitch,0);
			adjust_euler_agl.roll=IMU_pidCal(&imu_euler_angle_pid.Roll,roll,0);
			if(start==1)
			{		
				if (ctrl_state != CS_JUMP_1&& ctrl_state!=CS_JUMP_2&&imu_state) //when imu enable
				{
					calPosByRollNdPitch(adjust_euler_agl.roll,adjust_euler_agl.pitch);
					setPostureOffset();
				}
				if(ctrl_state!=CS_JUMP_1&&ctrl_state!=CS_JUMP_2&&ctrl_state!=CS_SLOPE)
				{	
					Gait(timer/1000.0f);
				}
				else if(ctrl_state==CS_SLOPE&&ctrl_state!=CS_JUMP_1&&ctrl_state!=CS_JUMP_2)
				{
					slopeCtrl(timer/1000.0f);
				}
				else 
				{
					if(ctrl_state==CS_JUMP_1)
					{
						jumpCtrl(timer/1000.0f,&jump1_struct);
					}
					else if(ctrl_state==CS_JUMP_2)
					{
						jumpCtrl(timer/1000.0f,&jump2_struct);
					}
				}
			}
			else if(start==0&&ctrl_state==CS_QUIT)
			{
				for(int i=0;i<8;i++)
				{
					SetZeroToCanBuf(i);
				}
				Can1_Send_Msg_to_Motor();
				Can2_Send_Msg_to_Motor();
			}
			else if(!start&&ctrl_state==CS_INIT)
			{
				Motor_Auto_Run();
			}
		}
		else if(!init_done&&ctrl_state==CS_INIT)
		{
			dogInit();
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update); 
	}
}
extern int feed; 

void feed_dog(void)
{
	if(feed)
	{
		IWDG_Feed();	
		LED1 =0;
	}
	else
	{
		LED1 = 1;
	}

}


