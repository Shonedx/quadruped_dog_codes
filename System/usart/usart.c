#include "sys.h"
#include "usart.h"
#include <string.h> // 引入字符串处理函数
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "IMU.h"	
////////////////////////////////////////////////////////////////////////////////// 	

#define pi 3.1415926f

#if SYSTEM_SUPPORT_OS
#include "includes.h"					  
#endif
// 全局变量
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
	x = x; //这一行是一个空操作，可能只是为了避免编译器警告参数 x 未被使用
} 


int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR & 0x40) == 0); 
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

// USART1初始化
void USART1_Init(void) {
  	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); 
	
	// USART1配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	USART_InitStructure.USART_BaudRate = 115200;
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
// USART1中断处理
void USART1_IRQHandler(void) {                	
#if SYSTEM_SUPPORT_OS 		
	OSIntEnter();    
#endif
	uint8_t Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		Res = USART_ReceiveData(USART1);
		USART_SendData(USART1,Res);
		// 处理接收到的数据
		// 这里可以添加你的处理逻辑
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
#if SYSTEM_SUPPORT_OS 	
	OSIntExit();  											 
#endif
}
// USART3初始化
void USART3_Init(void) { // IMU对应的串口
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
	 
	USART_InitStructure.USART_BaudRate = 115200;
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

 

// USART3中断处理
void USART3_IRQHandler(void) {
#if SYSTEM_SUPPORT_OS 		
	OSIntEnter();    
#endif
	unsigned char data;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		data = USART3->DR;
		CopeSerial3Data(data); //处理imu串口的数据
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
#if SYSTEM_SUPPORT_OS 	
	OSIntExit();  											 
#endif
}

// 发送字符到USART1
void UART1_Put_Char(unsigned char DataToSend) {
	while ((USART1->SR & 0x80) == 0); // 等待发送寄存器为空
	USART1->DR = DataToSend;
}

// 发送字符串到USART1
void UART1_Put_String(unsigned char *Str) {
	while(*Str) {
		if(*Str == '\r') UART1_Put_Char(0x0d);
		else if(*Str == '\n') UART1_Put_Char(0x0a);
		else UART1_Put_Char(*Str);
		Str++;
	}
}

// 处理USART3接收到的数据
void CopeSerial3Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	char debug_str[50];
	ucRxBuffer[ucRxCnt++] = ucData;	// 将收到的数据存入缓冲区中
	if (ucRxBuffer[0] != 0x55) // 数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt = 0;
		return;
	}
	if (ucRxCnt < 11) { return; } // 数据不满11个，则返回
	else
	{
		switch (ucRxBuffer[1]) // 判断数据是哪种数据，然后将其拷贝到对应的结构体中
		{
			case 0x53:
				memcpy(&stcAngle, &ucRxBuffer[2], 8);
				Euler.roll = (float)stcAngle.Angle[0] / 32768.0 *pi;
				Euler.pitch = (float)stcAngle.Angle[1] / 32768.0 * pi;
				Euler.yaw = (float)stcAngle.Angle[2] / 32768.0 * pi;
				// 打印调试信息到USART1
				//要测试陀螺仪角度的时候把这些注释解除掉
//				sprintf(debug_str, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", Euler.roll, Euler.pitch, Euler.yaw);
//				sprintf(debug_str, "samples:%.2f, %.2f, %.2f\n",Euler.roll, Euler.pitch, Euler.yaw);
//				UART1_Put_String((unsigned char *)debug_str);
//				debug_str[50]=0;
				/*******************/
				break;
			case 0x51: // 加速度数据
				memcpy(&stcAccel, &ucRxBuffer[2], 8);
				Accel.x = (float)stcAccel.Accel[0] / 32768.0 * 16.0f; // 假设最大加速度为±16g
				Accel.y = (float)stcAccel.Accel[1] / 32768.0 * 16.0f;
				Accel.z = (float)stcAccel.Accel[2] / 32768.0 * 16.0f;
				break;
			default:
				break;
		}
		ucRxCnt = 0; // 清空缓存区
	}
}
#define UART4_TX_AF_GPIO_PIN GPIO_Pin_0
#define UART4_RX_AF_GPIO_PIN GPIO_Pin_1
/**
  * @brief  配置 UART4 (PA0 TX, PA1 RX)
  * @param  无
  * @retval 无
  */
void UART4_Init(void) //用于串口打印数据调试
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure; // UART 也是使用 USART_InitTypeDef 结构体

    // 1. 使能 UART4 和相关 GPIO 端口的时钟
    // UART4 挂载在 APB1 总线上
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    // GPIOA 挂载在 AHB1 总线上
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // 2. 配置 UART4 的 GPIO 引脚
    // 配置 PA0 为 UART4_TX (复用推挽输出)
    GPIO_InitStructure.GPIO_Pin = UART4_TX_AF_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // 复用功能模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // 上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     // 50MHz 速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置 PA1 为 UART4_RX (复用输入)
    GPIO_InitStructure.GPIO_Pin = UART4_RX_AF_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;          // 复用功能模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      // 无上下拉 (或 GPIO_PuPd_UP)
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 将 GPIO 引脚连接到 UART4 的复用功能
    // STM32F4系列需要这一步来明确复用功能映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4); // PA0 连接到 UART4_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); // PA1 连接到 UART4_RX

    // 4. 配置 UART4 参数
    USART_InitStructure.USART_BaudRate = 115200;                // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 8位数据位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;         // 无校验位
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 使能接收和发送
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_Init(UART4, &USART_InitStructure); // 注意这里是 UART4

    // 5. 使能 UART4 外设
    USART_Cmd(UART4, ENABLE); // 注意这里是 UART4
}
/**
  * @brief  UART4 中断服务程序
  * @param  无
  * @retval 无
  */
void UART4_IRQHandler(void)
{
    // 检查是否是接收非空中断
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
      
        USART_ClearITPendingBit(UART4, USART_IT_RXNE); // 清除中断挂起位 (某些中断类型需要手动清除)
    }
}