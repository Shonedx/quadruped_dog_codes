#include "IIC_OLED.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"

/**
  * @brief  ��ʼ��I2C1���輰��ص�GPIO���ţ���������OLED��Ļ��
  * ���ţ�PB6 (SCL), PB7 (SDA)
  * ģʽ����ģʽ������ģʽ (400KHz)
  * @param  ��
  * @retval ��
  */
void I2C1_OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    // 1. ʹ�� GPIO �� I2C ʱ��
    RCC_AHB1PeriphClockCmd(OLED_I2Cx_PORT_CLK, ENABLE); // ʹ�� GPIOB ʱ��
    RCC_APB1PeriphClockCmd(OLED_I2Cx_CLK, ENABLE);      // ʹ�� I2C1 ʱ��

    // 2. ���� I2C GPIO ���� (SCL, SDA)
    // SCL �� SDA ������Ҫ����Ϊ���ù��ܡ���©���������
    GPIO_InitStructure.GPIO_Pin = OLED_I2Cx_SCL_PIN | OLED_I2Cx_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // ���ù���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      // ��©���
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // ����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // �ٶ�
    GPIO_Init(OLED_I2Cx_PORT, &GPIO_InitStructure);

    // 3. ���� GPIO ���ŵ���Ӧ�� I2C ���ù���
    GPIO_PinAFConfig(OLED_I2Cx_PORT, OLED_I2Cx_SCL_SOURCE, OLED_I2Cx_SCL_AF);
    GPIO_PinAFConfig(OLED_I2Cx_PORT, OLED_I2Cx_SDA_SOURCE, OLED_I2Cx_SDA_AF);

    // 4. I2C ��������
    I2C_DeInit(OLED_I2Cx); // �ȸ�λI2C���裬ȷ���ɾ���ʼ��
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;              // I2C ģʽ
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;      // ռ�ձȣ�����ģʽ�³���
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;               // ����ģʽ�¿��������ã���������Ϊ0
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;             // ����Ӧ��
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7λ��ַģʽ
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;           // I2C ʱ���ٶ� (400KHz)

    I2C_Init(OLED_I2Cx, &I2C_InitStructure); // ��ʼ�� I2C1
    I2C_Cmd(OLED_I2Cx, ENABLE);             // ʹ�� I2C1 ����
}

/**
  * @brief  ͨ��Ӳ��I2C����һ���ֽڵ�OLED
  * @param  DeviceAddress OLED��I2C��ַ (0x78)
  * @param  Command_Type ����/�������� (0x00 Ϊ����, 0x40 Ϊ����)
  * @param  Data Ҫ���͵��ֽ�����
  * @retval ��
  */
void I2C1_WriteByte(uint8_t DeviceAddress, uint8_t Command_Type, uint8_t Data)
{
    // 1. �ȴ����߿���
    while(I2C_GetFlagStatus(OLED_I2Cx, I2C_FLAG_BUSY));

    // 2. ������ʼ�ź�
    I2C_GenerateSTART(OLED_I2Cx, ENABLE);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)); // �ȴ�EV5�¼������ߴ�����ģʽ��������ʼ�ź�

    // 3. ���ʹӻ���ַ (д����)
    I2C_Send7bitAddress(OLED_I2Cx, DeviceAddress, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // �ȴ�EV6�¼�����ַ�ѷ��ͣ����뷢��ģʽ

    // 4. ���Ϳ����ֽ� (��������ݱ�־)
    I2C_SendData(OLED_I2Cx, Command_Type);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)); // �ȴ�EV8�¼��������ѷ��ͣ����ݼĴ�����

    // 5. ���������ֽ�
    I2C_SendData(OLED_I2Cx, Data);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // �ȴ�EV8_2�¼������ݷ�����ɣ����յ�ACK

    // 6. ����ֹͣ�ź�
    I2C_GenerateSTOP(OLED_I2Cx, ENABLE);
}

/**
  * @brief  ͨ��Ӳ��I2C���Ͷ��ֽ����ݵ�OLED
  * @param  DeviceAddress OLED��I2C��ַ (0x78)
  * @param  Command_Type ����/�������� (0x00 Ϊ����, 0x40 Ϊ����)
  * @param  Data ָ��Ҫ�������ݵ�ָ��
  * @param  Length Ҫ���͵����ݳ���
  * @retval ��
  */
void I2C1_WriteMultiByte(uint8_t DeviceAddress, uint8_t Command_Type, const uint8_t* Data, uint16_t Length)
{
    // 1. �ȴ����߿���
    while(I2C_GetFlagStatus(OLED_I2Cx, I2C_FLAG_BUSY));

    // 2. ������ʼ�ź�
    I2C_GenerateSTART(OLED_I2Cx, ENABLE);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

    // 3. ���ʹӻ���ַ (д����)
    I2C_Send7bitAddress(OLED_I2Cx, DeviceAddress, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // 4. ���Ϳ����ֽ� (��������ݱ�־)
    I2C_SendData(OLED_I2Cx, Command_Type);
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    // 5. ѭ���������������ֽ�
    for (uint16_t i = 0; i < Length; i++)
    {
        I2C_SendData(OLED_I2Cx, Data[i]);
        // ������Ը���ʵ������ȴ� EV8 �¼�������ͨ����� TXE ��־λ
        // ������������ʹ������ݣ�������ѭ����ȴ����һ�� EV8_2
        while(!I2C_GetFlagStatus(OLED_I2Cx, I2C_FLAG_TXE)); // �ȴ����ͻ�������
    }

    // 6. �ȴ����һ���ֽ���ȫ���Ͳ�ȷ��
    while(!I2C_CheckEvent(OLED_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // �ȴ�EV8_2�¼�

    // 7. ����ֹͣ�ź�
    I2C_GenerateSTOP(OLED_I2Cx, ENABLE);
}

/**
  * @brief  �������OLED
  * @param  Command Ҫ���͵������ֽ�
  * @retval ��
  */
void I2C1_WriteCommand(uint8_t Command)
{
    I2C1_WriteByte(OLED_I2C_ADDRESS, 0x00, Command); // 0x00 ��ʾ�����ֽ�������
}

/**
  * @brief  �������ݵ�OLED
  * @param  Data Ҫ���͵������ֽ�
  * @retval ��
  */
void I2C1_WriteData(uint8_t Data)
{
    I2C1_WriteByte(OLED_I2C_ADDRESS, 0x40, Data); // 0x40 ��ʾ�����ֽ�������
}