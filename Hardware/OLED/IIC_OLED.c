#include "IIC_OLED.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"

/**
  * @brief  初始化I2C1外设及相关的GPIO引脚，用于驱动OLED屏幕。
  * 引脚：PB6 (SCL), PB7 (SDA)
  * 模式：主模式，快速模式 (400KHz)
  * @param  无
  * @retval 无
  */
void I2C1_OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    // 1. 使能 GPIO 和 I2C 时钟
    RCC_AHB1PeriphClockCmd(OLED_I2Cx_PORT_CLK, ENABLE); // 使能 GPIOB 时钟
    RCC_APB1PeriphClockCmd(OLED_I2Cx_CLK, ENABLE);      // 使能 I2C1 时钟

    // 2. 配置 I2C GPIO 引脚 (SCL, SDA)
    // SCL 和 SDA 引脚需要配置为复用功能、开漏输出、上拉
    GPIO_InitStructure.GPIO_Pin = OLED_I2Cx_SCL_PIN | OLED_I2Cx_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      // 开漏输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 速度
    GPIO_Init(OLED_I2Cx_PORT, &GPIO_InitStructure);

    // 3. 连接 GPIO 引脚到对应的 I2C 复用功能
    GPIO_PinAFConfig(OLED_I2Cx_PORT, OLED_I2Cx_SCL_SOURCE, OLED_I2Cx_SCL_AF);
    GPIO_PinAFConfig(OLED_I2Cx_PORT, OLED_I2Cx_SDA_SOURCE, OLED_I2Cx_SDA_AF);

    // 4. I2C 外设配置
    I2C_DeInit(OLED_I2Cx); // 先复位I2C外设，确保干净初始化
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;              // I2C 模式
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;      // 占空比，快速模式下常用
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;               // 主机模式下可随意设置，或者设置为0
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;             // 允许应答
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7位地址模式
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;           // I2C 时钟速度 (400KHz)

    I2C_Init(OLED_I2Cx, &I2C_InitStructure); // 初始化 I2C1
    I2C_Cmd(OLED_I2Cx, ENABLE);             // 使能 I2C1 外设
}

/**
  * @brief  通过硬件I2C发送一个字节到OLED
  * @param  DeviceAddress OLED的I2C地址 (0x78)
  * @param  Command_Type 命令/数据类型 (0x00 为命令, 0x40 为数据)
  * @param  Data 要发送的字节数据
  * @retval 无
  */
void I2C1_WriteByte(uint8_t DeviceAddress, uint8_t Command_Type, uint8_t Data)
{
    // 1. 等待总线空闲
    while(I2C_GetFlagStatus(OLED_I2Cx, I2C_FLAG_BUSY));

    // 2. 发送起始信号
    I2C_GenerateSTART(OLED_I2Cx, ENABLE);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)); // 等待EV5事件：总线处于主模式，发出起始信号

    // 3. 发送从机地址 (写操作)
    I2C_Send7bitAddress(OLED_I2Cx, DeviceAddress, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // 等待EV6事件：地址已发送，进入发送模式

    // 4. 发送控制字节 (命令或数据标志)
    I2C_SendData(OLED_I2Cx, Command_Type);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)); // 等待EV8事件：数据已发送，数据寄存器空

    // 5. 发送数据字节
    I2C_SendData(OLED_I2Cx, Data);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // 等待EV8_2事件：数据发送完成，接收到ACK

    // 6. 发送停止信号
    I2C_GenerateSTOP(OLED_I2Cx, ENABLE);
}

/**
  * @brief  通过硬件I2C发送多字节数据到OLED
  * @param  DeviceAddress OLED的I2C地址 (0x78)
  * @param  Command_Type 命令/数据类型 (0x00 为命令, 0x40 为数据)
  * @param  Data 指向要发送数据的指针
  * @param  Length 要发送的数据长度
  * @retval 无
  */
void I2C1_WriteMultiByte(uint8_t DeviceAddress, uint8_t Command_Type, const uint8_t* Data, uint16_t Length)
{
    // 1. 等待总线空闲
    while(I2C_GetFlagStatus(OLED_I2Cx, I2C_FLAG_BUSY));

    // 2. 发送起始信号
    I2C_GenerateSTART(OLED_I2Cx, ENABLE);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    // 3. 发送从机地址 (写操作)
    I2C_Send7bitAddress(OLED_I2Cx, DeviceAddress, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // 4. 发送控制字节 (命令或数据标志)
    I2C_SendData(OLED_I2Cx, Command_Type);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    // 5. 循环发送所有数据字节
    for (uint16_t i = 0; i < Length; i++)
    {
        I2C_SendData(OLED_I2Cx, Data[i]);
        // 这里可以根据实际情况等待 EV8 事件，或者通过检查 TXE 标志位
        // 如果是连续发送大量数据，可以在循环外等待最后一次 EV8_2
        while(!I2C_GetFlagStatus(OLED_I2Cx, I2C_FLAG_TXE)); // 等待发送缓冲区空
    }

    // 6. 等待最后一个字节完全发送并确认
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // 等待EV8_2事件

    // 7. 发送停止信号
    I2C_GenerateSTOP(OLED_I2Cx, ENABLE);
}

/**
  * @brief  发送命令到OLED
  * @param  Command 要发送的命令字节
  * @retval 无
  */
void I2C1_WriteCommand(uint8_t Command)
{
    I2C1_WriteByte(OLED_I2C_ADDRESS, 0x00, Command); // 0x00 表示后续字节是命令
}

/**
  * @brief  发送数据到OLED
  * @param  Data 要发送的数据字节
  * @retval 无
  */
void I2C1_WriteData(uint8_t Data)
{
    I2C1_WriteByte(OLED_I2C_ADDRESS, 0x40, Data); // 0x40 表示后续字节是数据
}