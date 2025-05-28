#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h" // ���� STM32F4xx ϵ�еĻ�������

// MPU6050 �� 8 λ I2C �ӻ���ַ (7λ��ַ 0x68 ����1λ)
#define MPU6050_ADDRESS   0xD0

// MPU6050 ���üĴ�����ַ����
#define MPU6050_WHO_AM_I         0x75 // WHO_AM_I �Ĵ�����ַ������ֵΪ 0x68
#define MPU6050_PWR_MGMT_1       0x6B // ��Դ���� 1 �Ĵ���
#define MPU6050_SMPLRT_DIV       0x19 // �����ʷ�Ƶ�Ĵ���
#define MPU6050_CONFIG           0x1A // ���üĴ��� (DLPF ���ֵ�ͨ�˲���)
#define MPU6050_GYRO_CONFIG      0x1B // ���������üĴ��� (����)
#define MPU6050_ACCEL_CONFIG     0x1C // ���ٶȼ����üĴ��� (����)

// MPU6050 �ж���ؼĴ���
#define MPU6050_INT_PIN_CFG      0x37 // �ж��������üĴ���
#define MPU6050_INT_ENABLE       0x38 // �ж�ʹ�ܼĴ���
#define MPU6050_INT_STATUS       0x3A // �ж�״̬�Ĵ���

// ��������Ĵ���
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_ACCEL_XOUT_L     0x3C
#define MPU6050_ACCEL_YOUT_H     0x3D
#define MPU6050_ACCEL_YOUT_L     0x3E
#define MPU6050_ACCEL_ZOUT_H     0x3F
#define MPU6050_ACCEL_ZOUT_L     0x40

#define MPU6050_TEMP_OUT_H       0x41
#define MPU6050_TEMP_OUT_L       0x42

#define MPU6050_GYRO_XOUT_H      0x43
#define MPU6050_GYRO_XOUT_L      0x44
#define MPU6050_GYRO_YOUT_H      0x45
#define MPU6050_GYRO_YOUT_L      0x46
#define MPU6050_GYRO_ZOUT_H      0x47
#define MPU6050_GYRO_ZOUT_L      0x48


// MPU6050 ԭʼ���ݽṹ��
typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Temp_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
} MPU6050_RawData_TypeDef;


// ��������

// I2C3 ��ʼ������
void I2C3_Init(void);

// I2C3 ��д��������
void I2C_WriteByte(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t Data);
uint8_t I2C_ReadByte(uint8_t SlaveAddress, uint8_t RegAddress);
void I2C_ReadBytes(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t* pBuffer, uint16_t NumByteToRead);

// MPU6050 ��ʼ������
uint8_t MPU6050_Init(void);

// MPU6050 �ж����ú��� (�ڲ��Ĵ�������)
void MPU6050_ConfigInterrupt(void);

// MPU6050 INT ���ŵ� STM32 �ⲿ�ж����ú��� (ʹ�� PC1)
void MPU6050_EXTI_Init(void);

// ��ȡ MPU6050 ԭʼ���ݺ���
void MPU6050_ReadRawData(MPU6050_RawData_TypeDef *pRawData);

// MPU6050 У׼����
void MPU6050_Calibration(void);

// ȫ��ƫ����
extern int16_t Accel_X_Offset, Accel_Y_Offset, Accel_Z_Offset;
extern int16_t Gyro_X_Offset, Gyro_Y_Offset, Gyro_Z_Offset;

// MPU6050 ���ݾ�����־�����жϷ��������ã�
extern volatile uint8_t MPU6050_DataReady;


#endif /* __MPU6050_H */