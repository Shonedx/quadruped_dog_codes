

#include "Allheaderfile.h"
#include "ticks.h"
#include "timer.h"
double now_time=0;
const double dt=0.007; //0.007

 static void Update_Time(void) //��ȡ��ǰʱ�䣬ͨ������һ��dtֵ������ʱ��
 {
		now_time+=dt;
		now_time=fmod(now_time,1000000);
 }
//Ft=168Mhz/4*ʱ�ӷ�Ƶ

void TIM4_Init(void) //10 ms
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. ʹ�� TIM4 ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // 2. ���� TIM4 ʵ�� 1ms �ж�
    //    ���� TIM4 ʱ�� (TIMxCLK) = 84MHz (PCLK1=42MHz, APB1 Ԥ��Ƶ�� > 1)
    //    �������ж�Ƶ�� = 1ms (1000Hz)
    //    ���㹫ʽ��(Ԥ��Ƶ�� + 1) * (���� + 1) = TIM4_ʱ��Ƶ�� / ����Ƶ��
    //    (Ԥ��Ƶ�� + 1) * (���� + 1) = 84,000,000 Hz / 1000 Hz = 84000

    //    Ϊ��ʵ�� 1us (1MHz) �ļ�����������
    //    Ԥ��Ƶ�� + 1 = 84,000,000 / 1,000,000 = 84  => Ԥ��Ƶ�� = 83
    //    Ȼ��Ϊ��ʵ�� 1ms (1000us) ���жϣ�
    //    ���� + 1 = 1000 => ���� = 999

    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;   
    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;   
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ��ʱ�ӷ�Ƶ

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); // ��ʼ�� TIM4

    // 3. ʹ�� TIM4 �����ж�
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    // 4. ������ʱ��
    TIM_Cmd(TIM4, ENABLE);

    // 5. ���� NVIC ������ TIM4 �ж�
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;             // TIM4 ȫ���ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
}
void TIM5_Init(void) //5 ms
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=5000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM5,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM4_IRQHandler() //�������߼�
{  
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) !=RESET)
	{
		Remote_Cmd();
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
static int start_stand_flag=0;

void TIM5_IRQHandler(void) //����ʱ�� 5ms
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET)
	{		
		Update_Time();
		if(start==1)
		{		
//			if (currentstate != Jump&& ctrl_state!=Start_Ctrl)
//			 {
//				 IMU_Pos_Cal(0, Euler.pitch, Euler.roll);
//				 Set_Standheight_Offset();
//			 }
			Gait(now_time); //ִ�в�̬�߼�
		}
		feed_dog();
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //����жϱ�־λ
	}
}


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


