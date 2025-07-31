#include "usart.h"
#include <string.h> // 用于字符串操作
#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "IMU.h"
#include "RC.h"
#include "can.h"
#include "motor.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dma2d.h"
#define pi 3.1415926f // 定义圆周率常量

#if SYSTEM_SUPPORT_OS
#include "includes.h" // 包含操作系统相关头文件
#endif

// 全局变量，用于存储IMU加速度和角度数据
stcAccel_t stcAccel; // 加速度数据结构体
stcAngle_t stcAngle; // 角度数据结构体
Euler_t Euler;       // 欧拉角结构体
Accel_t Accel;       // 加速度结构体

// 初始化USART1，用于串口通信
void USART1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能GPIOA和USART1外设时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // 配置GPIOA的引脚9和10为USART1的复用功能
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // RX

    // 配置GPIOA引脚（PA9、PA10）为复用模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 50MHz速度
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置USART1参数
    USART_InitStructure.USART_BaudRate = 115200;                 // 波特率115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8位数据长度
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;          // 无奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 使能接收和发送
    USART_Init(USART1, &USART_InitStructure);

    // 使能USART1
    USART_Cmd(USART1, ENABLE);

#if EN_USART1_RX
    // 使能USART1接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // 配置USART1中断的NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // 子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);
#endif
}

// 通过USART1发送单个字符
void UART1_Put_Char(unsigned char DataToSend) {
    // 等待发送数据寄存器为空
    while ((USART1->SR & 0x80) == 0);
    USART1->DR = DataToSend; // 发送数据
}

// 通过USART1发送字符串
void UART1_Put_String(unsigned char *Str) {
    while (*Str) {
        // 处理回车符
        if (*Str == '\r') UART1_Put_Char(0x0d);
        // 处理换行符
        else if (*Str == '\n') UART1_Put_Char(0x0a);
        // 发送普通字符
        else UART1_Put_Char(*Str);
        Str++;
    }
}

void usart1TxDateToVofa(float ch1, float ch2, float ch3,float ch4,float ch5) {
    char buffer[100];
    // 格式化为JustFloat格式：浮点数用逗号分隔，保留2位小数，结尾为\r\n
    sprintf(buffer, "%.2f,%.2f,%.2f,%.2f,%.2f\r\n", ch1, ch2, ch3,ch4,ch5);
    UART1_Put_String((unsigned char *)buffer);
}
extern uint8_t setted_height;   // 外部声明：设定高度
extern CtrlState_t ctrl_state;  // 外部声明：控制状态
extern uint16_t test;           // 外部声明：测试变量

// USART1接收中断处理函数
void USART1_IRQHandler(void) {
    uint8_t Res;
    // 检查是否为接收数据中断
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        Res = USART_ReceiveData(USART1); // 读取接收到的数据
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); // 清除中断标志
    }
}
//usart3
#define RX_BUFFER_SIZE 256
uint8_t RxBuffer[RX_BUFFER_SIZE]; // DMA接收缓冲区

// 初始化USART3 DMA接收
void USART3_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    // 1. 使能DMA1时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    
    // 2. 配置DMA接收(USART3_RX使用DMA1 Stream1 Channel4)
    DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR; // 外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;       // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;           // 传输方向:外设到内存
    DMA_InitStructure.DMA_BufferSize = RX_BUFFER_SIZE;               // 缓冲区大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不递增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         // 内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度:字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        // 内存数据宽度:字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                 // 循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;             // 高优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         // 禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;     // 单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
    
    // 3. 使能DMA流
    DMA_Cmd(DMA1_Stream1, ENABLE);
    
    // 4. 配置USART3 DMA接收使能
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    
    // 5. 使能USART3空闲中断
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
}
// 初始化USART3，用于IMU通信
void USART3_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能USART3和GPIOB外设时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // 配置GPIOB的引脚10和11为USART3的复用功能
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); // TX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); // RX

    // 配置GPIOB引脚（PB10、PB11）为复用模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 50MHz速度
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置USART3参数
    USART_InitStructure.USART_BaudRate = 115200;                 // 波特率115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8位数据长度
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;          // 无奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 使能接收和发送
    USART_Init(USART3, &USART_InitStructure);

    // 配置USART3中断
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE); // 禁用发送中断
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE); // 使能接收中断
    USART3_DMA_Init();
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    // 清除发送完成标志
    USART_ClearFlag(USART3, USART_FLAG_TC);
    // 使能USART3
    USART_Cmd(USART3, ENABLE);

    // 配置USART3中断的NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;        // 子优先级4
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);
}
void USART3_IRQHandler(void) {
    // 检查空闲中断
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) {
        USART_ReceiveData(USART3); // 清除IDLE标志
        
        // 计算接收到的数据长度
        uint16_t dataLength = RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
        
        // 处理接收到的数据
        if(dataLength > 0) {
            // 这里调用你的数据处理函数
            for(uint16_t i = 0; i < dataLength; i++) {
                CopeSerial3Data(RxBuffer[i]);
            }
        }
        
        // 循环模式下DMA会自动继续接收，无需重新配置
    }
}
// // USART3接收中断处理函数
// void USART3_IRQHandler(void) {
//     unsigned char data;
//     // 检查是否为接收数据中断
//     if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
//         data = USART3->DR; // 读取接收到的数据
//         CopeSerial3Data(data); // 处理IMU串口数据
//         USART_ClearITPendingBit(USART3, USART_IT_RXNE); // 清除中断标志
//     }
// }

// 处理USART3接收到的IMU数据
void CopeSerial3Data(unsigned char ucData) {
    static unsigned char ucRxBuffer[250]; // 接收数据缓冲区
    static unsigned char ucRxCnt = 0;     // 接收数据计数器
    char debug_str[50];                   // 调试字符串缓冲区

    ucRxBuffer[ucRxCnt++] = ucData; // 将接收到的数据存入缓冲区

    // 检查数据帧头是否为0x55，若不是则清空计数器重新开始
    if (ucRxBuffer[0] != 0x55) {
        ucRxCnt = 0;
        return;
    }

    // 数据不足11字节时返回，等待更多数据
    if (ucRxCnt < 11) {
        return;
    }

    // 根据数据类型处理数据帧
    switch (ucRxBuffer[1]) {
        case 0x53: // 角度数据
            memcpy(&stcAngle, &ucRxBuffer[2], 8); // 复制角度数据到结构体
            // 将角度数据转换为弧度
            Euler.roll = (float)stcAngle.Angle[0] / 32768.0 * pi;
            Euler.pitch = (float)stcAngle.Angle[1] / 32768.0 * pi;
            Euler.yaw = (float)stcAngle.Angle[2] / 32768.0 * pi;
            // 调试代码：通过USART1打印角度信息（已注释）
            /*
            sprintf(debug_str, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", Euler.roll, Euler.pitch, Euler.yaw);
            UART1_Put_String((unsigned char *)debug_str);
            */
            break;
        case 0x51: // 加速度数据
            memcpy(&stcAccel, &ucRxBuffer[2], 8); // 复制加速度数据到结构体
            // 将加速度数据转换为g单位（最大16g）
            Accel.x = (float)stcAccel.Accel[0] / 32768.0 * 16.0f;
            Accel.y = (float)stcAccel.Accel[1] / 32768.0 * 16.0f;
            Accel.z = (float)stcAccel.Accel[2] / 32768.0 * 16.0f;
            break;
        default:
            break;
    }
    ucRxCnt = 0; // 清空缓冲区计数器
}

#define UART4_TX_AF_GPIO_PIN GPIO_Pin_0 // UART4 TX引脚
#define UART4_RX_AF_GPIO_PIN GPIO_Pin_1 // UART4 RX引脚

// 初始化UART4，用于数据打印
void UART4_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 使能UART4和GPIOA外设时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // 配置PA0为UART4 TX（发送引脚）
    GPIO_InitStructure.GPIO_Pin = UART4_TX_AF_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 50MHz速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置PA1为UART4 RX（接收引脚）
    GPIO_InitStructure.GPIO_Pin = UART4_RX_AF_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // 无上下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 将GPIO引脚映射到UART4
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4); // PA0映射为UART4 TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); // PA1映射为UART4 RX

    // 配置UART4参数
    USART_InitStructure.USART_BaudRate = 115200;                 // 波特率115200
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8位数据长度
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      // 1位停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;          // 无奇偶校验
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 使能接收和发送
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控制
    USART_Init(UART4, &USART_InitStructure);

    // 使能UART4
    USART_Cmd(UART4, ENABLE);
}

// UART4接收中断处理函数
void UART4_IRQHandler(void) {
    // 检查是否为接收数据中断
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {
        USART_ClearITPendingBit(UART4, USART_IT_RXNE); // 清除中断标志
    }
}