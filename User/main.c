
#include "Allheaderfile.h"
#include "ticks.h"

//CAN_1 : PA11(RX)   PA12(TX)        CAN_2 : PB12(RX)      PB13(TX)
//ң�����:PA3
//������:PB10(RX)    PB11(TX)

// NRF24L01 ����ӳ�䵽STM32F407��GPIO
GPIO_TypeDef* NRF_CE_Port = GPIOB;
uint16_t      NRF_CE_Pin = GPIO_Pin_0;
GPIO_TypeDef* NRF_CSN_Port = GPIOB;
uint16_t      NRF_CSN_Pin = GPIO_Pin_1;
GPIO_TypeDef* NRF_IRQ_Port = GPIOC;
uint16_t      NRF_IRQ_Pin = GPIO_Pin_5;
GPIO_TypeDef* NRF_MOSI_Port = GPIOA; // SPI1
uint16_t      NRF_MOSI_Pin = GPIO_Pin_7;
GPIO_TypeDef* NRF_MISO_Port = GPIOA; // SPI1
uint16_t      NRF_MISO_Pin = GPIO_Pin_6;
GPIO_TypeDef* NRF_SCK_Port = GPIOA;  // SPI1
uint16_t      NRF_SCK_Pin = GPIO_Pin_5;

// �������ݻ�����
uint8_t rx_buffer[NRF_PAYLOAD_LENGTH]={0};

uint8_t rx_result=-1;


int feed=1; //ι��
int start=0;			//��ʼ��־��
int reset=1;			//ң����������ʱreset��1 
int left_push_stick,right_push_stick; //ң�������ϣ������Ƹ˱�����ʼ��
int crouch_flag=0;
int higher_flag=0;

int main()
{
	
	HAL_InitTick(0);//��ʼ��hal�йأ���ΪҪ�õ�HAL_GetTick����
	
	delay_init();    
//	USART1_Init();	//������������
	USART3_Init();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	LED_Init();					//LED 
	remote_control_init();
//	KEY_Init(); 				

	IWDG_Init(4,500);  //Tout=((4*2^prer)*rlr)/32 (ms)  1s
	TIM5_Init();				//5 ms
	TIM4_Init();					//1ms
	
//	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal);//CAN1 1Mbps 
//	
//	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal);//CAN2 1Mbps 
	
	Can_Init(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_4tq,6,CAN_Mode_Normal);
	
	PID_Init();	
	
	ChangeTheGainOfPID_KP_KI_KD(5.1,0.3,1.81,5.1,0.3,1.81);
	
//============���Դ���================
	   // SPI��GPIO��ʼ�� (�������ͨ�ó�ʼ������)
//    NRF24L01_Init_Common();
//	OLED_Init();
    // ����Ϊ����ģʽ
//    NRF24L01_Set_RX_Mode();
//============================
//	OLED_NewFrame();
//	OLED_PrintASCIIString(0,0,"Init Achive",&afont16x8,OLED_COLOR_NORMAL);
//	OLED_ShowFrame();
//	delay_ms(1000);	
//============���Դ���================

	while(1) //������
	{
//============���Դ���================		
//		OLED_NewFrame();
//		OLED_PrintASCIIString(0,48,"While Running",&afont16x8,OLED_COLOR_NORMAL);
//		
//		if(rx_result)
//		{
//			OLED_PrintASCIIString(0,0,"Received ",&afont16x8,OLED_COLOR_NORMAL);
//		}
//		else 
//		{
//			OLED_PrintASCIIString(0,0,"ERROR ",&afont16x8,OLED_COLOR_NORMAL);
//			
//		}
////		if( NRF24L01_Read_Reg(FIFO_STATUS)&0x01)
////		memset(rx_buffer,0,NRF_PAYLOAD_LENGTH);
////		OLED_PrintASCIIString(0,16,"RX buffer:",&afont16x8,OLED_COLOR_NORMAL);
////		OLED_PrintASCIIString(0,32,(char*)rx_buffer,&afont16x8,OLED_COLOR_NORMAL);
//		OLED_PrintASCIINum(0,16,(uint16_t)rx_buffer[11]<<8|((uint16_t)rx_buffer[10]),4,&afont16x8,OLED_COLOR_NORMAL);
//		OLED_PrintASCIINum(40,16,(uint16_t)rx_buffer[13]<<8|((uint16_t)rx_buffer[12]),4,&afont16x8,OLED_COLOR_NORMAL);
//		OLED_PrintASCIINum(0,32,(uint16_t)rx_buffer[15]<<8|((uint16_t)rx_buffer[14]),4,&afont16x8,OLED_COLOR_NORMAL);
//		OLED_PrintASCIINum(40,32,(uint16_t)rx_buffer[17]<<8|((uint16_t)rx_buffer[16]),4,&afont16x8,OLED_COLOR_NORMAL);
//		
//		delay_ms(50);
//        // �������������һ����С����ʱ�����Ƴ���ȡ�������������CPU����
//		OLED_ShowFrame();
//============���Դ���================		
   
		
//========================���������=================================
		if(ctrl_state==Crouch_Ctrl)
			crouch_flag=7;
		else
			crouch_flag=0;
		if(ctrl_state==Higher_Ctrl)
			higher_flag=12;
		else
			higher_flag=0;
		switch(ctrl_state)
		{
			case Initial_Ctrl://��ʼ״̬ 
			
			M3508_ALL_ZERO_SET();
			start=0;
			if_idle=0; //̤��״̬��־λ��1λ̤����0Ϊֹͣ
           break;
			
			case Start_Ctrl://��ʼ��������
			start=1;
			if_idle=0;
//			if(start_stand_flag==0)
//			{
//				Stand_Init();
//			}
			Ctrl_Cmd();
			break;
			
			case Main_Ctrl://������
			if_idle=1;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Stop_Ctrl:// ֹͣ
			if_idle=0;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Jump_Ctrl_1:// ��Ծ
			if_idle=0;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Jump_Ctrl_2:// ��Ծ
			if_idle=0;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Crouch_Ctrl:// �¶�
			if_idle=0;
			start=1;
			Ctrl_Cmd();
			break;
			
			case Higher_Ctrl:// 
			if_idle=0;
			start=1;
			Ctrl_Cmd();
			break;

		}
//		if(ctrl_state!=Initial_Ctrl&&ctrl_state!=Start_Ctrl)
//			start_stand_flag=1;

		
	}
}

// ==============================================================================
// �ⲿ�жϷ����� (PC5)
// ==============================================================================
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(NRF_EXTI_LINE) != RESET) // ����Ƿ���PC5�ϵ��ж�
    {
//		 rx_result =NRF24L01_RxPacket(rx_buffer);
		
        EXTI_ClearITPendingBit(NRF_EXTI_LINE); // ���MCU�ⲿ�жϱ�־
    }
}