#ifndef _OLED_IIC_H
#define _OLED_IIC_H

#include "stdint.h"
#include "stm32f4xx.h"
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




void iicOledInit(void);
void iicWriteByte(u8 DeviceAddress, u8 Command_Type, u8 Data);
void iicWriteMultipleBit(u8 DeviceAddress, u8 Command_Type, const u8* Data, u16 Length);
void iicWriteCmd(uint8_t Command);
void iicWriteData(uint8_t Data);




#endif /* _OLED_IIC_H */