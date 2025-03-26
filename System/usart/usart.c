#include "sys.h"
#include "usart.h"
#include <string.h> // �����ַ���������
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "IMU.h"	
////////////////////////////////////////////////////////////////////////////////// 	

#define pi 3.1415926f

#if SYSTEM_SUPPORT_OS
#include "includes.h"					  
#endif
// ȫ�ֱ���
stcAccel_t	stcAccel;
stcAngle_t stcAngle;
Euler_t Euler;
Accel_t Accel;
static unsigned char TxBuffer[256];
static unsigned char TxCounter = 0;
static unsigned char count = 0;

#if 1
#pragma import(__use_no_semihosting)             
                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
  
void _sys_exit(int x) 
{ 
	x = x; 
} 


int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR & 0x40) == 0); 
	USART1->DR = (u8) ch;      
	return ch;
}
#endif




// USART1��ʼ��
void uart_init(u32 bound) {
  	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); 
	
	// USART1����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USART1, &USART_InitStructure); 
	
  USART_Cmd(USART1, ENABLE);  
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	

#endif
	
}

// USART3��ʼ��
void uart3_init(u32 bound) { // IMU��Ӧ����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // GPIOB10 USART3
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // GPIOB11 USART3
	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	 
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
	USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);    
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	USART_ClearFlag(USART3, USART_FLAG_TC);	
	USART_Cmd(USART3, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// USART1�жϴ���
void USART1_IRQHandler(void) {                	
#if SYSTEM_SUPPORT_OS 		
	OSIntEnter();    
#endif
	uint8_t Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		Res = USART_ReceiveData(USART1);
		USART_SendData(USART1,Res);
		// ������յ�������
		// ������������Ĵ����߼�
	}
#if SYSTEM_SUPPORT_OS 	
	OSIntExit();  											 
#endif
} 

// USART3�жϴ���
void USART3_IRQHandler(void) {
#if SYSTEM_SUPPORT_OS 		
	OSIntEnter();    
#endif
	unsigned char data;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		data = USART3->DR;
		CopeSerial3Data(data);
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
#if SYSTEM_SUPPORT_OS 	
	OSIntExit();  											 
#endif
}

// �����ַ���USART1
void UART1_Put_Char(unsigned char DataToSend) {
	while ((USART1->SR & 0x80) == 0); // �ȴ����ͼĴ���Ϊ��
	USART1->DR = DataToSend;
}

// �����ַ�����USART1
void UART1_Put_String(unsigned char *Str) {
	while(*Str) {
		if(*Str == '\r') UART1_Put_Char(0x0d);
		else if(*Str == '\n') UART1_Put_Char(0x0a);
		else UART1_Put_Char(*Str);
		Str++;
	}
}

// ����USART3���յ�������
void CopeSerial3Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	char debug_str[50];
	ucRxBuffer[ucRxCnt++] = ucData;	// ���յ������ݴ��뻺������
	if (ucRxBuffer[0] != 0x55) // ����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt = 0;
		return;
	}
	if (ucRxCnt < 11) { return; } // ���ݲ���11�����򷵻�
	else
	{
		switch (ucRxBuffer[1]) // �ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ����
		{
			case 0x53:
				memcpy(&stcAngle, &ucRxBuffer[2], 8);
				Euler.roll = (float)stcAngle.Angle[0] / 32768.0 *pi;
				Euler.pitch = (float)stcAngle.Angle[1] / 32768.0 * pi;
				Euler.yaw = (float)stcAngle.Angle[2] / 32768.0 * pi;
				// ��ӡ������Ϣ��USART1
				//Ҫ���������ǽǶȵ�ʱ�����Щע�ͽ����
//				sprintf(debug_str, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", Euler.roll, Euler.pitch, Euler.yaw);
//				sprintf(debug_str, "samples:%.2f, %.2f, %.2f\n",Euler.roll, Euler.pitch, Euler.yaw);
//				UART1_Put_String((unsigned char *)debug_str);
//				debug_str[50]=0;
				/*******************/
				break;
			case 0x51: // ���ٶ�����
				memcpy(&stcAccel, &ucRxBuffer[2], 8);
				Accel.x = (float)stcAccel.Accel[0] / 32768.0 * 16.0f; // ���������ٶ�Ϊ��16g
				Accel.y = (float)stcAccel.Accel[1] / 32768.0 * 16.0f;
				Accel.z = (float)stcAccel.Accel[2] / 32768.0 * 16.0f;
				break;
			default:
				break;
		}
		ucRxCnt = 0; // ��ջ�����
	}
}
