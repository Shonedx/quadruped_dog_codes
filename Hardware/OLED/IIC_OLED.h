#ifndef _IIC_OLED_H
#define _IIC_OLED_H
#include "stdint.h"

// OLED ģ��� I2C �ӻ���ַ��д���ַ��0x3C << 1 = 0x78����ȡ��ַΪ 0x79��
#define OLED_I2C_ADDRESS    0x78 // ��������OLEDʵ�ʵ�ַ������������0x78��0x7A

// ѡ�� I2C ����Ϊ I2C1
#define OLED_I2Cx           I2C1
#define OLED_I2Cx_CLK       RCC_APB1Periph_I2C1 // I2C1 ������ APB1 ����
#define OLED_I2Cx_PORT      GPIOB
#define OLED_I2Cx_PORT_CLK  RCC_AHB1Periph_GPIOB // GPIOB ʱ���� AHB1

// I2C1 SCL ���Ŷ���: PB6
#define OLED_I2Cx_SCL_PIN   GPIO_Pin_6
#define OLED_I2Cx_SCL_SOURCE GPIO_PinSource6
#define OLED_I2Cx_SCL_AF    GPIO_AF_I2C1 // I2C1 �ĸ��ù���

// I2C1 SDA ���Ŷ���: PB7
#define OLED_I2Cx_SDA_PIN   GPIO_Pin_7
#define OLED_I2Cx_SDA_SOURCE GPIO_PinSource7
#define OLED_I2Cx_SDA_AF    GPIO_AF_I2C1 // I2C1 �ĸ��ù���

// I2C ʱ���ٶȣ���������Ϊ 400KHz (����ģʽ)
#define I2C_SPEED           400000

// ��������
void I2C1_OLED_Init(void);
void I2C1_WriteByte(uint8_t DeviceAddress, uint8_t Command_Type, uint8_t Data);
void I2C1_WriteMultiByte(uint8_t DeviceAddress, uint8_t Command_Type, const uint8_t* Data, uint16_t Length);
void I2C1_WriteCommand(uint8_t Command);
void I2C1_WriteData(uint8_t Data);

#endif /* __I2C_OLED_H */