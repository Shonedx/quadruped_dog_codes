#include "rc_spi.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_exti.h"
#include "delay.h"
#include "OLED.h"


// 引脚定义（根据您的配置）
GPIO_TypeDef* NRF_CE_Port = GPIOB;
u16      NRF_CE_Pin = GPIO_Pin_0;
GPIO_TypeDef* NRF_CSN_Port = GPIOB;
u16      NRF_CSN_Pin = GPIO_Pin_1;
GPIO_TypeDef* NRF_IRQ_Port = GPIOC;
u16      NRF_IRQ_Pin = GPIO_Pin_5;
GPIO_TypeDef* NRF_MOSI_Port = GPIOA; // SPI1
u16      NRF_MOSI_Pin = GPIO_Pin_7;
GPIO_TypeDef* NRF_MISO_Port = GPIOA; // SPI1
u16      NRF_MISO_Pin = GPIO_Pin_6;
GPIO_TypeDef* NRF_SCK_Port = GPIOA;  // SPI1
u16      NRF_SCK_Pin = GPIO_Pin_5;

void spiInit(void) { //初始化nrf模块
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_SYSCFG, ENABLE);
    
    // 2. 配置SPI引脚 (PA5-SCK, PA6-MISO, PA7-MOSI)
    GPIO_InitStructure.GPIO_Pin = NRF_SCK_Pin | NRF_MISO_Pin | NRF_MOSI_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  // SCK: PA5
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);  // MISO: PA6
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  // MOSI: PA7
    
    // 3. 配置CE、CSN、IRQ引脚
    // CE (PB0)
    GPIO_InitStructure.GPIO_Pin = NRF_CE_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(NRF_CE_Port, &GPIO_InitStructure);
    
    // CSN (PB1)
    GPIO_InitStructure.GPIO_Pin = NRF_CSN_Pin;
    GPIO_Init(NRF_CSN_Port, &GPIO_InitStructure);
    
    // IRQ (PC5)
    GPIO_InitStructure.GPIO_Pin = NRF_IRQ_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(NRF_IRQ_Port, &GPIO_InitStructure);
    
    // 4. 配置SPI
    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  // 84/8=10.5MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(NRF_SPI, &SPI_InitStructure);
    SPI_Cmd(NRF_SPI, ENABLE);
#if ENABLE_NRF24L01_IRQ
	// 5. 配置外部中断 (IRQ on PC5)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5); // 修正后的正确函数名
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
    // 6. 初始化状态
    NRF_CE_L();
    NRF_CSN_H();
	delay_ms(100);	
    
    // 7. 检查模块
    if(SpiCheck() == 0) {
        // 处理错误
        while(1);
    }
	 // 清除FIFO
    SpiWriteReg(FLUSH_TX, 0xFF);
    SpiWriteReg(FLUSH_RX, 0xFF);
}

u8 SpiWriteByte(u8 byte) { //写入一字节
    // 等待发送缓冲区空
    while(SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_TXE) == RESET);
    // 发送数据
    SPI_I2S_SendData(NRF_SPI, byte);
    // 等待接收缓冲区满
    while(SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    // 返回接收到的数据
    return SPI_I2S_ReceiveData(NRF_SPI);
}

u8 SpiWriteReg(u8 reg, u8 value) { //写入寄存器
    u8 status;
    NRF_CSN_L();
    status = SpiWriteByte(NRF_WRITE_REG | reg);
    SpiWriteByte(value);
    NRF_CSN_H();
    return status;
}

u8 SpiReadReg(u8 reg) { //读取寄存器
    u8 reg_val;
    NRF_CSN_L();
    SpiWriteByte(NRF_READ_REG | reg);
    reg_val = SpiWriteByte(NOP);
    NRF_CSN_H();
    return reg_val;
}

void SpiWriteBuf(u8 reg, u8 *pBuf, u8 len) { //写入缓冲区
    NRF_CSN_L();
    SpiWriteByte(reg);
    while(len--) {
        SpiWriteByte(*pBuf++);
    }
    NRF_CSN_H();
}

void SpiReadBuf(u8 reg, u8 *pBuf, u8 len) {
    NRF_CSN_L();
    SpiWriteByte(reg);
    while(len--) {
        *pBuf++ = SpiWriteByte(NOP);
    }
    NRF_CSN_H();
}

u8 SpiCheck(void) { //检查NRF24L01模块是否正常工作
    u8 buf[5] = {0};
    const u8 test_addr[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    
    SpiWriteBuf(NRF_WRITE_REG | TX_ADDR,(u8*)test_addr, 5);
    SpiReadBuf(NRF_READ_REG | TX_ADDR, buf, 5);
    
    for(u8 i = 0; i < 5; i++) {
        if(buf[i] != test_addr[i]) return 0;
    }
    return 1;
}
