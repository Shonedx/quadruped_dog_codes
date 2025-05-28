#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx.h" // 包含 STM32F4xx 系列的基础定义

// MPU6050 的 8 位 I2C 从机地址 (7位地址 0x68 左移1位)
#define MPU6050_ADDRESS   0xD0

// MPU6050 常用寄存器地址定义
#define MPU6050_WHO_AM_I         0x75 // WHO_AM_I 寄存器地址，返回值为 0x68
#define MPU6050_PWR_MGMT_1       0x6B // 电源管理 1 寄存器
#define MPU6050_SMPLRT_DIV       0x19 // 采样率分频寄存器
#define MPU6050_CONFIG           0x1A // 配置寄存器 (DLPF 数字低通滤波器)
#define MPU6050_GYRO_CONFIG      0x1B // 陀螺仪配置寄存器 (量程)
#define MPU6050_ACCEL_CONFIG     0x1C // 加速度计配置寄存器 (量程)

// MPU6050 中断相关寄存器
#define MPU6050_INT_PIN_CFG      0x37 // 中断引脚配置寄存器
#define MPU6050_INT_ENABLE       0x38 // 中断使能寄存器
#define MPU6050_INT_STATUS       0x3A // 中断状态寄存器

// 数据输出寄存器
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


// MPU6050 原始数据结构体
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


// 函数声明

// I2C3 初始化函数
void I2C3_Init(void);

// I2C3 读写操作函数
void I2C_WriteByte(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t Data);
uint8_t I2C_ReadByte(uint8_t SlaveAddress, uint8_t RegAddress);
void I2C_ReadBytes(uint8_t SlaveAddress, uint8_t RegAddress, uint8_t* pBuffer, uint16_t NumByteToRead);

// MPU6050 初始化函数
uint8_t MPU6050_Init(void);

// MPU6050 中断配置函数 (内部寄存器配置)
void MPU6050_ConfigInterrupt(void);

// MPU6050 INT 引脚的 STM32 外部中断配置函数 (使用 PC1)
void MPU6050_EXTI_Init(void);

// 读取 MPU6050 原始数据函数
void MPU6050_ReadRawData(MPU6050_RawData_TypeDef *pRawData);

// MPU6050 校准函数
void MPU6050_Calibration(void);

// 全局偏移量
extern int16_t Accel_X_Offset, Accel_Y_Offset, Accel_Z_Offset;
extern int16_t Gyro_X_Offset, Gyro_Y_Offset, Gyro_Z_Offset;

// MPU6050 数据就绪标志（由中断服务函数设置）
extern volatile uint8_t MPU6050_DataReady;


#endif /* __MPU6050_H */