#ifndef _SPI_H
#define _SPI_H
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"

// 引脚配置（在nrf24l01.c中定义）
extern GPIO_TypeDef* NRF_CE_Port;
extern uint16_t      NRF_CE_Pin;
extern GPIO_TypeDef* NRF_CSN_Port;
extern uint16_t      NRF_CSN_Pin;
extern GPIO_TypeDef* NRF_IRQ_Port;
extern uint16_t      NRF_IRQ_Pin;
extern GPIO_TypeDef* NRF_MOSI_Port;
extern uint16_t      NRF_MOSI_Pin;
extern GPIO_TypeDef* NRF_MISO_Port;
extern uint16_t      NRF_MISO_Pin;
extern GPIO_TypeDef* NRF_SCK_Port;
extern uint16_t      NRF_SCK_Pin;

#define NRF_SPI         SPI1

// 通信参数配置
#define NRF_PAYLOAD_LENGTH  11 //传输数据长度

// 寄存器操作命令
#define NRF_READ_REG        0x00
#define NRF_WRITE_REG       0x20
#define RD_RX_PLOAD         0x61
#define WR_TX_PLOAD         0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define NOP                 0xFF

// 寄存器地址
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define RX_ADDR_P0      0x0A
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define FIFO_STATUS     0x17

// 状态寄存器位
#define RX_DR   6
#define TX_DS   5
#define MAX_RT  4

// 引脚控制宏
#define NRF_CE_H()      GPIO_SetBits(NRF_CE_Port, NRF_CE_Pin)
#define NRF_CE_L()      GPIO_ResetBits(NRF_CE_Port, NRF_CE_Pin)
#define NRF_CSN_H()     GPIO_SetBits(NRF_CSN_Port, NRF_CSN_Pin)
#define NRF_CSN_L()     GPIO_ResetBits(NRF_CSN_Port, NRF_CSN_Pin)
#define NRF_IRQ_READ()  GPIO_ReadInputDataBit(NRF_IRQ_Port, NRF_IRQ_Pin)

//定义中断
#define ENABLE_NRF24L01_IRQ 1

// 函数声明
void spiInit(void) ;
u8 SpiWriteByte(u8 byte) ;
u8 SpiWriteReg(u8 reg, u8 value) ;
u8 SpiReadReg(u8 reg) ;
void SpiWriteBuf(u8 reg, u8 *pBuf, u8 len) ;
void SpiReadBuf(u8 reg, u8 *pBuf, u8 len) ;
u8 SpiCheck(void);


#endif /* _SPI_H */
