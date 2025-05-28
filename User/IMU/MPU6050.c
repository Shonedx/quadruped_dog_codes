#include "mpu6050.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_exti.h"   // ��� EXTI ͷ�ļ�
#include "stm32f4xx_syscfg.h" // ��� SYSCFG ͷ�ļ�
#include "misc.h"             // ��� NVIC ��ض���

#include <stdio.h> // ���� printf ���ԣ��������Ҫ�����Ƴ�

// �����ʱ������������� SysTick������������Ҫ�������ط�ʵ��
extern void delay_ms(uint32_t ms);

// ȫ��ƫ��������
int16_t Accel_X_Offset, Accel_Y_Offset, Accel_Z_Offset;
int16_t Gyro_X_Offset, Gyro_Y_Offset, Gyro_Z_Offset;

// MPU6050 ���ݾ�����־��volatile �ؼ���ȷ�������������Ż������ķ���
volatile uint8_t MPU6050_DataReady = 0;


/**
  * @brief  ��ʼ�� STM32 �� I2C3 ���� (PA8 SCL, PC9 SDA)
  * @param  ��
  * @retval ��
  */
void I2C3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    // 1. ʹ�� I2C3 ����� GPIO �˿ڵ�ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // PA8 ���� GPIOA
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // PC9 ���� GPIOC

    // 2. ���� I2C3 �� GPIO ����
    // SCL (PA8) ����Ϊ���ÿ�©���
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;        // ��©��� (I2C ��Ҫ�ⲿ��������)
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      // �������� (�ⲿ�����������Ҫ)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // SDA (PC9) ����Ϊ���ÿ�©���
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 3. �� GPIO �������ӵ� I2C3 �ĸ��ù���
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3); // PA8 ���ӵ� I2C3_SCL
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3); // PC9 ���ӵ� I2C3_SDA

    // 4. ���� I2C3 ����
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;             // I2C ģʽ
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;     // ռ�ձ� (��׼ģʽ 50%������ģʽ 33%)
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;              // STM32 �Լ��ĵ�ַ (����Ϊ�ӻ�ʱ��������)
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;            // ʹ��Ӧ��
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // 7λ��ַģʽ
    I2C_InitStructure.I2C_ClockSpeed = 100000;             // I2C ʱ���ٶ� (100kHz ��׼ģʽ)
    I2C_Init(I2C3, &I2C_InitStructure);

    // 5. ʹ�� I2C3 ����
    I2C_Cmd(I2C3, ENABLE);
}


/**
  * @brief  �� I2C3 �豸д��һ���ֽ�����
  * @param  SlaveAddress: �ӻ���ַ (8λ)
  * @param  RegAddress: �Ĵ�����ַ
  * @param  Data: Ҫд�������
  * @retval ��
  */
void I2C_WriteByte(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t Data)
{
    while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY)); // �ȴ����߿���

    I2C_GenerateSTART(I2C3, ENABLE); // ������ʼ����
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT)); // �ȴ� EV5: ����ģʽѡ�����

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Transmitter); // ���ʹӻ���ַ (дģʽ)
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // �ȴ� EV6: �ӻ���ַ������ɣ��ӻ�Ӧ��

    I2C_SendData(I2C3, RegAddress); // ���ͼĴ�����ַ
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTING)); // �ȴ� EV8: �����ֽڷ�����

    I2C_SendData(I2C3, Data); // ��������
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // �ȴ� EV8_2: �����ֽڷ������

    I2C_GenerateSTOP(I2C3, ENABLE); // ����ֹͣ����
}

/**
  * @brief  �� I2C3 �豸��ȡһ���ֽ�����
  * @param  SlaveAddress: �ӻ���ַ (8λ)
  * @param  RegAddress: �Ĵ�����ַ
  * @retval ��ȡ��������
  */
uint8_t I2C_ReadByte(uint8_t SlaveAddress, uint8_t RegAddress)
{
    uint8_t Data;

    while (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C3, ENABLE);
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Transmitter); // ���ʹӻ���ַ (дģʽ)
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C3, RegAddress); // ����Ҫ��ȡ�ļĴ�����ַ
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(I2C3, ENABLE); // �ٴη�����ʼ���� (�ظ���ʼ)
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Receiver); // ���ʹӻ���ַ (��ģʽ)
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    I2C_AcknowledgeConfig(I2C3, DISABLE); // �������һ���ֽ�ǰ���� NACK
    I2C_GenerateSTOP(I2C3, ENABLE);      // �������һ���ֽں���ֹͣ����

    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED)); // �ȴ� EV7: ���յ�����
    Data = I2C_ReceiveData(I2C3); // ��ȡ����

    I2C_AcknowledgeConfig(I2C3, ENABLE); // �ָ� ACK ģʽ (Ϊ�´�ͨ��׼��)

    return Data;
}

/**
  * @brief  �� I2C3 �豸��ȡ����ֽ�����
  * @param  SlaveAddress: �ӻ���ַ (8λ)
  * @param  RegAddress: �Ĵ�����ַ
  * @param  pBuffer: �洢���ݵĻ�����ָ��
  * @param  NumByteToRead: Ҫ��ȡ���ֽ���
  * @retval ��
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

    I2C_GenerateSTART(I2C3, ENABLE); // �ظ���ʼ
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C3, SlaveAddress, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    // ѭ����ȡ����
    while (NumByteToRead)
    {
        if (NumByteToRead == 1) // ���һ���ֽ�
        {
            I2C_AcknowledgeConfig(I2C3, DISABLE); // ���� NACK
            I2C_GenerateSTOP(I2C3, ENABLE);      // ����ֹͣ����
        }

        while (!I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_RECEIVED)); // �ȴ����յ�����
        *pBuffer = I2C_ReceiveData(I2C3); // ��ȡ����
        pBuffer++;
        NumByteToRead--;
    }

    I2C_AcknowledgeConfig(I2C3, ENABLE); // �ָ� ACK ģʽ
}


/**
  * @brief  ���� MPU6050 �ڲ��Ĵ�����ʹ�����ݾ����ж�
  * @param  ��
  * @retval ��
  */
void MPU6050_ConfigInterrupt(void)
{
    // ���� INT_PIN_CFG �Ĵ��� (0x37)
    // bit[7] ACTL: 1 = �ж�����͵�ƽ��Ч (Ĭ��), 0 = �ߵ�ƽ��Ч
    // bit[6] LATCH_INT_EN: 0 = �ж����� (Ĭ��), 1 = �����ж� (��Ҫ��ȡ INT_STATUS ���)
    // ����Ĭ�� 0x00�����͵�ƽ��Ч������ģʽ
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x00);

    // ���� INT_ENABLE �Ĵ��� (0x38)
    // bit[0] DATA_RDY_EN: 1 = ʹ�����ݾ����ж� (�Ƽ�ʹ��)
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_INT_ENABLE, 0x01); // ʹ�����ݾ����ж�
}


/**
  * @brief  ���� STM32 ���ⲿ�ж��� (PC1) ���� MPU6050 �� INT ����
  * @param  ��
  * @retval ��
  */
void MPU6050_EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. ʹ�� GPIOC ʱ�� (PC1) �� SYSCFG ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // ʹ�� SYSCFG ʱ��

    // 2. ���� PC1 Ϊ����ģʽ (���� MPU6050_INT)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // PC1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // ����ģʽ
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // �������� (��Ϊ MPU6050 INT Ĭ���ǵ͵�ƽ��Ч)
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 3. �� PC1 ���ӵ� EXTI Line1
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);

    // 4. ���� EXTI Line1 ����
    EXTI_InitStructure.EXTI_Line = EXTI_Line1; // EXTI Line1
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // �ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // �½��ش��� (��Ϊ MPU6050 �ж��ǵ͵�ƽ��Ч)
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; // ʹ�� EXTI Line1
    EXTI_Init(&EXTI_InitStructure);

    // 5. ���� NVIC (�жϿ�����)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; // EXTI Line1 ��Ӧ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ� (������ȼ�)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
}


/**
  * @brief  ��ʼ�� MPU6050 (���� I2C ���ú��ڲ��ж�ʹ��)
  * @param  ��
  * @retval 1: ��ʼ���ɹ�, 0: ��ʼ��ʧ��
  */
uint8_t MPU6050_Init(void)
{
    uint8_t MPU_ID;

    // 1. ��� MPU6050 �� ID (WHO_AM_I �Ĵ���)
    MPU_ID = I2C_ReadByte(MPU6050_ADDRESS, MPU6050_WHO_AM_I);
    if (MPU_ID != 0x68) // MPU6050 �� WHO_AM_I �Ĵ���ֵӦ���� 0x68
    {
        #ifdef DEBUG_PRINTF // ��������� DEBUG_PRINTF����ʹ�� printf
        printf("MPU6050 WHO_AM_I Error! ID: 0x%X\r\n", MPU_ID);
        #endif
        return 0; // ��ʼ��ʧ��
    }

    // 2. ���˯��ģʽ��ѡ��ʱ��Դ
    // PWR_MGMT_1 �Ĵ��� (0x6B): bit[7] DEVICE_RESET, bit[6] SLEEP
    // д�� 0x00��ѡ���ڲ� 8MHz ������Ϊʱ��Դ��Ĭ�ϣ��������� MPU6050
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00);

    // 3. ���ò����ʷ�Ƶ (Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV))
    // ���� Gyro Output Rate �� 1KHz (Ĭ��)������ 0x07 (����Ƶ 8) �õ� 125Hz ������
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x07);

    // 4. �������ֵ�ͨ�˲��� (DLPF)
    // CONFIG �Ĵ��� (0x1A): ���� DLPF_CFG
    // ���� DLPF Ϊ 6 (��Ӧ���ٶȼ� 5Hz ���������� 5Hz ����)
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);

    // 5. �������������� (GYRO_CONFIG �Ĵ��� 0x1B)
    // 0x00: +/- 250 deg/s (Ĭ��)
    // 0x08: +/- 500 deg/s
    // 0x10: +/- 1000 deg/s
    // 0x18: +/- 2000 deg/s
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x00); // ����Ϊ +/- 250 deg/s

    // 6. ���ü��ٶȼ����� (ACCEL_CONFIG �Ĵ��� 0x1C)
    // 0x00: +/- 2g (Ĭ��)
    // 0x08: +/- 4g
    // 0x10: +/- 8g
    // 0x18: +/- 16g
    I2C_WriteByte(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x00); // ����Ϊ +/- 2g

    // 7. ���� MPU6050 �ڲ��ж� (ʹ�����ݾ����ж�)
    MPU6050_ConfigInterrupt();

    return 1; // ��ʼ���ɹ�
}


/**
  * @brief  ��ȡ MPU6050 ��ԭʼ����
  * @param  pRawData: ָ�� MPU6050_RawData_TypeDef �ṹ���ָ�룬���ڴ洢��ȡ��������
  * @retval ��
  */
void MPU6050_ReadRawData(MPU6050_RawData_TypeDef *pRawData)
{
    uint8_t Rx_Buffer[14]; // 14���ֽڣ����ٶȼ�(6) + �¶�(2) + ������(6)

    // �� MPU6050_ACCEL_XOUT_H (0x3B) ��ʼ������ȡ 14 ���ֽ�
    I2C_ReadBytes(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, Rx_Buffer, 14);

    // ��ϸ�λ�͵�λ�ֽڣ��õ� 16 λ��ԭʼ����
    pRawData->Accel_X_RAW = (Rx_Buffer[0] << 8) | Rx_Buffer[1];
    pRawData->Accel_Y_RAW = (Rx_Buffer[2] << 8) | Rx_Buffer[3];
    pRawData->Accel_Z_RAW = (Rx_Buffer[4] << 8) | Rx_Buffer[5];
    pRawData->Temp_RAW    = (Rx_Buffer[6] << 8) | Rx_Buffer[7];
    pRawData->Gyro_X_RAW  = (Rx_Buffer[8] << 8) | Rx_Buffer[9];
    pRawData->Gyro_Y_RAW  = (Rx_Buffer[10] << 8) | Rx_Buffer[11];
    pRawData->Gyro_Z_RAW  = (Rx_Buffer[12] << 8) | Rx_Buffer[13];
}

/**
  * @brief  �� MPU6050 ���м򵥵���ƫУ׼
  * �� MPU6050 ���ã��ɼ��������ȡƽ����Ϊƫ������
  * @param  ��
  * @retval ��
  */
void MPU6050_Calibration(void)
{
    MPU6050_RawData_TypeDef RawData;
    long Accel_X_Sum = 0, Accel_Y_Sum = 0, Accel_Z_Sum = 0;
    long Gyro_X_Sum = 0, Gyro_Y_Sum = 0, Gyro_Z_Sum = 0;
    uint16_t i;
    const uint16_t num_samples = 1000; // ��������

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
        delay_ms(1); // ��ʱ 1ms��ȷ��ÿ�β�����ȡ��������
    }

    // ����ƽ��ֵ��Ϊƫ����
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