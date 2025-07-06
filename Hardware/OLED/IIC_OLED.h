#ifndef _IIC_OLED_H
#define _IIC_OLED_H
#include "stdint.h"

// OLED 模块的 I2C 从机地址（写入地址，0x3C << 1 = 0x78，读取地址为 0x79）
#define OLED_I2C_ADDRESS    0x78 // 请根据你的OLED实际地址调整，常见有0x78或0x7A

// 选择 I2C 外设为 I2C1
#define OLED_I2Cx           I2C1
#define OLED_I2Cx_CLK       RCC_APB1Periph_I2C1 // I2C1 挂载在 APB1 总线
#define OLED_I2Cx_PORT      GPIOB
#define OLED_I2Cx_PORT_CLK  RCC_AHB1Periph_GPIOB // GPIOB 时钟在 AHB1

// I2C1 SCL 引脚定义: PB6
#define OLED_I2Cx_SCL_PIN   GPIO_Pin_6
#define OLED_I2Cx_SCL_SOURCE GPIO_PinSource6
#define OLED_I2Cx_SCL_AF    GPIO_AF_I2C1 // I2C1 的复用功能

// I2C1 SDA 引脚定义: PB7
#define OLED_I2Cx_SDA_PIN   GPIO_Pin_7
#define OLED_I2Cx_SDA_SOURCE GPIO_PinSource7
#define OLED_I2Cx_SDA_AF    GPIO_AF_I2C1 // I2C1 的复用功能

// I2C 时钟速度，这里设置为 400KHz (快速模式)
#define I2C_SPEED           400000

// 函数声明
void I2C1_OLED_Init(void);
void I2C1_WriteByte(uint8_t DeviceAddress, uint8_t Command_Type, uint8_t Data);
void I2C1_WriteMultiByte(uint8_t DeviceAddress, uint8_t Command_Type, const uint8_t* Data, uint16_t Length);
void I2C1_WriteCommand(uint8_t Command);
void I2C1_WriteData(uint8_t Data);

#endif /* __I2C_OLED_H */