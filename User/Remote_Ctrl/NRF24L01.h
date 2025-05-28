#ifndef __NRF24L01_H
#define __NRF24L01_H

#define STM32F4 

#ifdef STM32F1
#include "stm32f10x.h"
#define NRF_SPI_INSTANCE    SPI2 // STM32F103通常用SPI2
#define NRF_GPIO_CLK_CMD    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)
#define NRF_SPI_CLK_CMD     RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE)
#define NRF_GPIO_AF_MODE    GPIO_Mode_AF_PP // F1复用推挽输出
#define NRF_GPIO_IN_MODE    GPIO_Mode_IPU   // F1输入上拉
#define NRF_EXTI_PORT_SRC   GPIO_PortSourceGPIOB
#define NRF_EXTI_PIN_SRC    GPIO_PinSource11
#define NRF_EXTI_LINE       EXTI_Line11
#define NRF_EXTI_IRQN       EXTI15_10_IRQn

#elif defined(STM32F4)
#include "stm32f4xx.h"
#define NRF_SPI_INSTANCE    SPI1 // STM32F407通常用SPI1
#define NRF_GPIO_CLK_CMD    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE)
#define NRF_SPI_CLK_CMD     RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE)
#define NRF_SYSCFG_CLK_CMD  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE) // F4需要SYSCFG时钟
#define NRF_GPIO_AF_MODE    GPIO_Mode_AF      // F4复用模式
#define NRF_GPIO_OUT_MODE   GPIO_Mode_OUT     // F4输出模式
#define NRF_GPIO_PP_OTYPE   GPIO_OType_PP     // F4推挽输出类型
#define NRF_GPIO_IN_MODE    GPIO_Mode_IN      // F4输入模式
#define NRF_GPIO_PU_PD      GPIO_PuPd_UP      // F4上拉
#define NRF_GPIO_NOPULL     GPIO_PuPd_NOPULL  // F4无拉
#define NRF_GPIO_AF_SCK     GPIO_AF_SPI1
#define NRF_GPIO_AF_MISO    GPIO_AF_SPI1
#define NRF_GPIO_AF_MOSI    GPIO_AF_SPI1
#define NRF_EXTI_PORT_SRC   EXTI_PortSourceGPIOC
#define NRF_EXTI_PIN_SRC    EXTI_PinSource5
#define NRF_EXTI_LINE       EXTI_Line5
#define NRF_EXTI_IRQN       EXTI9_5_IRQn
#else
#error "请定义STM32F1或STM32F4以选择正确的MCU头文件和宏定义!"
#endif


// NRF24L01 寄存器定义
#define NRF_READ_REG        0x00  // 读寄存器命令
#define NRF_WRITE_REG       0x20  // 写寄存器命令
#define RD_RX_PLOAD         0x61  // 读接收数据命令
#define WR_TX_PLOAD         0xA0  // 写发送数据命令
#define FLUSH_TX            0xE1  // 清除TX FIFO命令
#define FLUSH_RX            0xE2  // 清除RX FIFO命令
#define REUSE_TX_PL         0xE3  // 重用上一包发送数据命令
#define NOP                 0xFF  // 空操作，用于读状态寄存器

// NRF24L01 寄存器地址
#define CONFIG          0x00  // 配置寄存器
#define EN_AA           0x01  // 使能自动应答功能寄存器
#define EN_RXADDR       0x02  // 使能接收地址寄存器
#define SETUP_AW        0x03  // 地址宽度设置寄存器
#define SETUP_RETR      0x04  // 自动重发设置寄存器
#define RF_CH           0x05  // RF信道设置寄存器
#define RF_SETUP        0x06  // RF设置寄存器
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测寄存器
#define CD              0x09  // 载波检测寄存器
#define RX_ADDR_P0      0x0A  // 数据通道0接收地址寄存器
#define RX_ADDR_P1      0x0B  // 数据通道1接收地址寄存器
#define RX_ADDR_P2      0x0C  // 数据通道2接收地址寄存器
#define RX_ADDR_P3      0x0D  // 数据通道3接收地址寄存器
#define RX_ADDR_P4      0x0E  // 数据通道4接收地址寄存器
#define RX_ADDR_P5      0x0F  // 数据通道5接收地址寄存器
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收数据通道0有效数据宽度寄存器
#define RX_PW_P1        0x12  // 接收数据通道1有效数据宽度寄存器
#define RX_PW_P2        0x13  // 接收数据通道2有效数据宽度寄存器
#define RX_PW_P3        0x14  // 接收数据通道3有效数据宽度寄存器
#define RX_PW_P4        0x15  // 接收数据通道4有效数据宽度寄存器
#define RX_PW_P5        0x16  // 接收数据通道5有效数据宽度寄存器
#define FIFO_STATUS     0x17  // FIFO状态寄存器
#define DYNPD           0x1C  // 动态数据包长度使能寄存器
#define FEATURE         0x1D  // 特性寄存器

// NRF24L01 状态寄存器位
#define RX_DR   6  // 接收数据中断标志
#define TX_DS   5  // 发送完成中断标志
#define MAX_RT  4  // 最大重发次数中断标志

// NRF24L01 配置寄存器位
#define PRIM_RX         0  // 接收/发送模式选择 (0:TX, 1:RX)
#define PWR_UP          1  // 电源模式 (0:掉电, 1:上电)

// NRF24L01 RF_SETUP 寄存器位
#define RF_DR_LOW       5  // 数据速率 (0:1Mbps, 1:250kbps)
#define RF_DR_HIGH      3  // 数据速率 (0:1Mbps, 1:2Mbps)

// **重要：数据包长度，两端必须完全一致！**
#define NRF_PAYLOAD_LENGTH  20

// **重要：通信地址，两端必须完全一致！**
// 建议使用十六进制，避免字符编码问题
extern const uint8_t NRF_COMMON_ADDR[5];

// NRF24L01 引脚定义 (CE, CSN, IRQ)
// 具体引脚定义将在各自的 main.c 或项目配置中完成
#define NRF_CE_H()      GPIO_SetBits(NRF_CE_Port, NRF_CE_Pin)
#define NRF_CE_L()      GPIO_ResetBits(NRF_CE_Port, NRF_CE_Pin)
#define NRF_CSN_H()     GPIO_SetBits(NRF_CSN_Port, NRF_CSN_Pin)
#define NRF_CSN_L()     GPIO_ResetBits(NRF_CSN_Port, NRF_CSN_Pin)
#define NRF_IRQ_READ()  GPIO_ReadInputDataBit(NRF_IRQ_Port, NRF_IRQ_Pin)


// 函数声明 (通用部分)
void NRF24L01_Init_Common(void); // 通用初始化函数
uint8_t NRF24L01_Check(void);
void NRF24L01_Set_RX_Mode(void); // 设置为接收模式
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf); // 接收数据包 (中断会调用此函数)
void NRF24L01_Set_TX_Mode(void); // 设置为发送模式
uint8_t NRF24L01_TxPacket(uint8_t *txbuf); // 发送数据包
void NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
void NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);
uint8_t NRF24L01_SpiRW(uint8_t dat);

#endif /* __NRF24L01_H */