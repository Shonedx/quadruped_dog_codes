#include "mpu6050.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_exti.h"   // 添加 EXTI 头文件
#include "stm32f4xx_syscfg.h" // 添加 SYSCFG 头文件
#include "misc.h"             // 添加 NVIC 相关定义

#include <stdio.h> // 用于 printf 调试，如果不需要可以移除

// 你的延时函数（例如基于 SysTick）的声明，需要在其他地方实现
extern void delay_ms(uint32_t ms);

// 全局偏移量定义
int16_t Accel_X_Offset, Accel_Y_Offset, Accel_Z_Offset;
int16_t Gyro_X_Offset, Gyro_Y_Offset, Gyro_Z_Offset;

// MPU6050 数据就绪标志，volatile 关键字确保编译器不会优化对它的访问
volatile uint8_t MPU6050_DataReady = 0;


/**
  * @brief  初始化 STM32 的 I2C3 外设 (PA8 SCL, PC9 SDA)
  * @param  无
  * @retval 无
  */
void I2C3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    // 1. 使能 I2C3 和相关 GPIO 端口的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // PA8 属于 GPIOA
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // PC9 属于 GPIOC

    // 2. 配置 I2C3 的 GPIO 引脚
    // SCL (PA8) 配置为复用开漏输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;        // 开漏输出 (I2C 需要外部上拉电阻)
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      // 无上下拉 (外部上拉电阻更重要)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // SDA (PC9) 配置为复用开漏输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 3. 将 GPIO 引脚连接到 I2C3 的复用功能
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3); // PA8 连接到 I2C3_SCL
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3); // PC9 连接到 I2C3_SDA

    // 4. 配置 I2C3 参数
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;             // I2C 模式
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;     // 占空比 (标准模式 50%，快速模式 33%)
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;              // STM32 自己的地址 (不作为从机时可任意设)
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;            // 使能应答
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7位地址模式
    I2C_InitStructure.I2C_ClockSpeed = 100000;             // I2C 时钟速度 (100kHz 标准模式)
    I2C_Init(I2C3, &I2C_InitStructure);

    // 5. 使能 I2C3 外设
    I2C_Cmd(I2C3, ENABLE);
}


/**
  * @brief  向 I2C3 设备写入一个字节数据
  * @param  SlaveAddress: 从机地址 (8位)
  * @param  RegAddress: 寄存器地址
  * @param  Data: 要写入的数据
  * @retval 无
  */
void I2C_WriteByte(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t Data)
{
    while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY)); // 等待总线空闲

    I2C_GenerateSTART(I2C3, ENABLE); // 发送起始条件
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT)); // 等待 EV5: 主机模式选择完成

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Transmitter); // 发送从机地址 (写模式)
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // 等待 EV6: 从机地址发送完成，从机应答

    I2C_SendData(I2C3, RegAddress); // 发送寄存器地址
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTING)); // 等待 EV8: 数据字节发送中

    I2C_SendData(I2C3, Data); // 发送数据
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // 等待 EV8_2: 数据字节发送完成

    I2C_GenerateSTOP(I2C3, ENABLE); // 发送停止条件
}

/**
  * @brief  从 I2C3 设备读取一个字节数据
  * @param  SlaveAddress: 从机地址 (8位)
  * @param  RegAddress: 寄存器地址
  * @retval 读取到的数据
  */
uint8_t I2C_ReadByte(uint8_t SlaveAddress, uint8_t RegAddress)
{
    uint8_t Data;

    while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C3, ENABLE);
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Transmitter); // 发送从机地址 (写模式)
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C3, RegAddress); // 发送要读取的寄存器地址
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(I2C3, ENABLE); // 再次发送起始条件 (重复起始)
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Receiver); // 发送从机地址 (读模式)
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    I2C_AcknowledgeConfig(I2C3, DISABLE); // 接收最后一个字节前发送 NACK
    I2C_GenerateSTOP(I2C3, ENABLE);      // 接收最后一个字节后发送停止条件

    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED)); // 等待 EV7: 接收到数据
    Data = I2C_ReceiveData(I2C3); // 读取数据

    I2C_AcknowledgeConfig(I2C3, ENABLE); // 恢复 ACK 模式 (为下次通信准备)

    return Data;
}

/**
  * @brief  从 I2C3 设备读取多个字节数据
  * @param  SlaveAddress: 从机地址 (8位)
  * @param  RegAddress: 寄存器地址
  * @param  pBuffer: 存储数据的缓冲区指针
  * @param  NumByteToRead: 要读取的字节数
  * @retval 无
  */
void I2C_ReadBytes(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t* pBuffer, uint16_t NumByteToRead)
{
    while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C3, ENABLE);
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C3, RegAddress);
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    I2C_GenerateSTART(I2C3, ENABLE); // 重复起始
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    // 循环读取数据
    while (NumByteToRead)
    {
        if (NumByteToRead == 1) // 最后一个字节
        {
            I2C_AcknowledgeConfig(I2C3, DISABLE); // 发送 NACK
            I2C_GenerateSTOP(I2C3, ENABLE);      // 发送停止条件
        }

        while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED)); // 等待接收到数据
        *pBuffer = I2C_ReceiveData(I2C3); // 读取数据
        pBuffer++;
        NumByteToRead--;
    }

    I2C_AcknowledgeConfig(I2C3, ENABLE); // 恢复 ACK 模式
}


/**
  * @brief  配置 MPU6050 内部寄存器，使能数据就绪中断
  * @param  无
  * @retval 无
  */
void MPU6050_ConfigInterrupt(void)
{
    // 配置 INT_PIN_CFG 寄存器 (0x37)
    // bit[7] ACTL: 1 = 中断输出低电平有效 (默认), 0 = 高电平有效
    // bit[6] LATCH_INT_EN: 0 = 中断脉冲 (默认), 1 = 锁存中断 (需要读取 INT_STATUS 清除)
    // 保持默认 0x00，即低电平有效，脉冲模式
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x00);

    // 配置 INT_ENABLE 寄存器 (0x38)
    // bit[0] DATA_RDY_EN: 1 = 使能数据就绪中断 (推荐使用)
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_INT_ENABLE, 0x01); // 使能数据就绪中断
}


/**
  * @brief  配置 STM32 的外部中断线 (PC1) 用于 MPU6050 的 INT 引脚
  * @param  无
  * @retval 无
  */
void MPU6050_EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能 GPIOC 时钟 (PC1) 和 SYSCFG 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // 使能 SYSCFG 时钟

    // 2. 配置 PC1 为输入模式 (连接 MPU6050_INT)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // PC1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // 输入模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉输入 (因为 MPU6050 INT 默认是低电平有效)
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 3. 将 PC1 连接到 EXTI Line1
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);

    // 4. 配置 EXTI Line1 参数
    EXTI_InitStructure.EXTI_Line = EXTI_Line1; // EXTI Line1
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // 中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿触发 (因为 MPU6050 中断是低电平有效)
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; // 使能 EXTI Line1
    EXTI_Init(&EXTI_InitStructure);

    // 5. 配置 NVIC (中断控制器)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; // EXTI Line1 对应中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级 (最高优先级)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);
}


/**
  * @brief  初始化 MPU6050 (包括 I2C 配置和内部中断使能)
  * @param  无
  * @retval 1: 初始化成功, 0: 初始化失败
  */
uint8_t MPU6050_Init(void)
{
    uint8_t MPU_ID;

    // 1. 检查 MPU6050 的 ID (WHO_AM_I 寄存器)
    MPU_ID = I2C_ReadByte(MPU6050_ADDRESS, MPU6050_WHO_AM_I);
    if (MPU_ID != 0x68) // MPU6050 的 WHO_AM_I 寄存器值应该是 0x68
    {
        #ifdef DEBUG_PRINTF // 如果定义了 DEBUG_PRINTF，才使用 printf
        printf("MPU6050 WHO_AM_I Error! ID: 0x%X\r\n", MPU_ID);
        #endif
        return 0; // 初始化失败
    }

    // 2. 解除睡眠模式，选择时钟源
    // PWR_MGMT_1 寄存器 (0x6B): bit[7] DEVICE_RESET, bit[6] SLEEP
    // 写入 0x00，选择内部 8MHz 晶振作为时钟源（默认），并唤醒 MPU6050
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);

    // 3. 设置采样率分频 (Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV))
    // 假设 Gyro Output Rate 是 1KHz (默认)，设置 0x07 (即分频 8) 得到 125Hz 采样率
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x07);

    // 4. 配置数字低通滤波器 (DLPF)
    // CONFIG 寄存器 (0x1A): 设置 DLPF_CFG
    // 设置 DLPF 为 6 (对应加速度计 5Hz 带宽，陀螺仪 5Hz 带宽)
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);

    // 5. 配置陀螺仪量程 (GYRO_CONFIG 寄存器 0x1B)
    // 0x00: +/- 250 deg/s (默认)
    // 0x08: +/- 500 deg/s
    // 0x10: +/- 1000 deg/s
    // 0x18: +/- 2000 deg/s
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x00); // 设置为 +/- 250 deg/s

    // 6. 配置加速度计量程 (ACCEL_CONFIG 寄存器 0x1C)
    // 0x00: +/- 2g (默认)
    // 0x08: +/- 4g
    // 0x10: +/- 8g
    // 0x18: +/- 16g
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x00); // 设置为 +/- 2g

    // 7. 配置 MPU6050 内部中断 (使能数据就绪中断)
    MPU6050_ConfigInterrupt();

    return 1; // 初始化成功
}


/**
  * @brief  读取 MPU6050 的原始数据
  * @param  pRawData: 指向 MPU6050_RawData_TypeDef 结构体的指针，用于存储读取到的数据
  * @retval 无
  */
void MPU6050_ReadRawData(MPU6050_RawData_TypeDef *pRawData)
{
    uint8_t Rx_Buffer[14]; // 14个字节：加速度计(6) + 温度(2) + 陀螺仪(6)

    // 从 MPU6050_ACCEL_XOUT_H (0x3B) 开始连续读取 14 个字节
    I2C_ReadBytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, Rx_Buffer, 14);

    // 组合高位和低位字节，得到 16 位的原始数据
    pRawData->Accel_X_RAW = (Rx_Buffer[0] << 8) | Rx_Buffer[1];
    pRawData->Accel_Y_RAW = (Rx_Buffer[2] << 8) | Rx_Buffer[3];
    pRawData->Accel_Z_RAW = (Rx_Buffer[4] << 8) | Rx_Buffer[5];
    pRawData->Temp_RAW    = (Rx_Buffer[6] << 8) | Rx_Buffer[7];
    pRawData->Gyro_X_RAW  = (Rx_Buffer[8] << 8) | Rx_Buffer[9];
    pRawData->Gyro_Y_RAW  = (Rx_Buffer[10] << 8) | Rx_Buffer[11];
    pRawData->Gyro_Z_RAW  = (Rx_Buffer[12] << 8) | Rx_Buffer[13];
}

/**
  * @brief  对 MPU6050 进行简单的零偏校准
  * 将 MPU6050 静置，采集多次数据取平均作为偏移量。
  * @param  无
  * @retval 无
  */
void MPU6050_Calibration(void)
{
    MPU6050_RawData_TypeDef RawData;
    long Accel_X_Sum = 0, Accel_Y_Sum = 0, Accel_Z_Sum = 0;
    long Gyro_X_Sum = 0, Gyro_Y_Sum = 0, Gyro_Z_Sum = 0;
    uint16_t i;
    const uint16_t num_samples = 1000; // 采样次数

    #ifdef DEBUG_PRINTF
    printf("MPU6050 Calibration started. Please keep the sensor steady...\r\n");
    #endif

    for (i = 0; i < num_samples; i++)
    {
        MPU6050_ReadRawData(&RawData);
        Accel_X_Sum += RawData.Accel_X_RAW;
        Accel_Y_Sum += RawData.Accel_Y_RAW;
        Accel_Z_Sum += RawData.Accel_Z_RAW;
        Gyro_X_Sum += RawData.Gyro_X_RAW;
        Gyro_Y_Sum += RawData.Gyro_Y_RAW;
        Gyro_Z_Sum += RawData.Gyro_Z_RAW;
        delay_ms(1); // 延时 1ms，确保每次采样都取到新数据
    }

    // 计算平均值作为偏移量
    Accel_X_Offset = Accel_X_Sum / num_samples;
    Accel_Y_Offset = Accel_Y_Sum / num_samples;
    Accel_Z_Offset = Accel_Z_Sum / num_samples;

    Gyro_X_Offset = Gyro_X_Sum / num_samples;
    Gyro_Y_Offset = Gyro_Y_Sum / num_samples;
    Gyro_Z_Offset = Gyro_Z_Sum / num_samples;

    #ifdef DEBUG_PRINTF
    printf("MPU6050 Calibration finished. Offsets:\r\n");
    printf("Accel_X_Offset: %d, Accel_Y_Offset: %d, Accel_Z_Offset: %d\r\n", Accel_X_Offset, Accel_Y_Offset, Accel_Z_Offset);
    printf("Gyro_X_Offset: %d, Gyro_Y_Offset: %d, Gyro_Z_Offset: %d\r\n", Gyro_X_Offset, Gyro_Y_Offset, Gyro_Z_Offset);
    #endif
}