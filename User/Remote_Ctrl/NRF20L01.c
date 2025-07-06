#include "nrf24l01.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "delay.h"
#include "OLED.h"
#include "main_params.h"
#define ENABLE_NRF24L01_IRQ 1
const uint8_t NRF_COMMON_ADDR[5] = {'e','t','h','a','n'};
// 引脚定义（根据您的配置）
GPIO_TypeDef* NRF_CE_Port = GPIOB;
uint16_t      NRF_CE_Pin = GPIO_Pin_0;
GPIO_TypeDef* NRF_CSN_Port = GPIOB;
uint16_t      NRF_CSN_Pin = GPIO_Pin_1;
GPIO_TypeDef* NRF_IRQ_Port = GPIOC;
uint16_t      NRF_IRQ_Pin = GPIO_Pin_5;
GPIO_TypeDef* NRF_MOSI_Port = GPIOA; // SPI1
uint16_t      NRF_MOSI_Pin = GPIO_Pin_7;
GPIO_TypeDef* NRF_MISO_Port = GPIOA; // SPI1
uint16_t      NRF_MISO_Pin = GPIO_Pin_6;
GPIO_TypeDef* NRF_SCK_Port = GPIOA;  // SPI1
uint16_t      NRF_SCK_Pin = GPIO_Pin_5;



void NRF24L01_Init(void) {
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
    if(NRF24L01_Check() == 0) {
        // 处理错误
        while(1);
    }
	 // 清除FIFO
    NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
    NRF24L01_Write_Reg(FLUSH_RX, 0xFF);
}

uint8_t NRF24L01_SpiRW(uint8_t dat) {
    // 等待发送缓冲区空
    while(SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_TXE) == RESET);
    // 发送数据
    SPI_I2S_SendData(NRF_SPI, dat);
    // 等待接收缓冲区满
    while(SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    // 返回接收到的数据
    return SPI_I2S_ReceiveData(NRF_SPI);
}

uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value) {
    uint8_t status;
    NRF_CSN_L();
    status = NRF24L01_SpiRW(NRF_WRITE_REG | reg);
    NRF24L01_SpiRW(value);
    NRF_CSN_H();
    return status;
}

uint8_t NRF24L01_Read_Reg(uint8_t reg) {
    uint8_t reg_val;
    NRF_CSN_L();
    NRF24L01_SpiRW(NRF_READ_REG | reg);
    reg_val = NRF24L01_SpiRW(NOP);
    NRF_CSN_H();
    return reg_val;
}

void NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    NRF_CSN_L();
    NRF24L01_SpiRW(reg);
    while(len--) {
        NRF24L01_SpiRW(*pBuf++);
    }
    NRF_CSN_H();
}

void NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len) {
    NRF_CSN_L();
    NRF24L01_SpiRW(reg);
    while(len--) {
        *pBuf++ = NRF24L01_SpiRW(NOP);
    }
    NRF_CSN_H();
}

uint8_t NRF24L01_Check(void) {
    uint8_t buf[5] = {0};
    const uint8_t test_addr[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    
    NRF24L01_Write_Buf(NRF_WRITE_REG | TX_ADDR,(uint8_t*)test_addr, 5);
    NRF24L01_Read_Buf(NRF_READ_REG | TX_ADDR, buf, 5);
    
    for(uint8_t i = 0; i < 5; i++) {
        if(buf[i] != test_addr[i]) return 0;
    }
    return 1;
}

void NRF24L01_Set_RX_Mode(void) {
	NRF_CE_L();
	
    // 设置接收地址和通道0数据长度
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF_COMMON_ADDR, 5);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, NRF_PAYLOAD_LENGTH);
    // 设置通道0自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    // 设置通道0接收使能
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    // RF设置：2Mbps, 0dBm
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);
    // 自动重发：500us, 15次
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1F);
    // 选择频道40 (2.440GHz)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);
    // 配置为接收模式，上电
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F); // PWR_UP=1, PRIM_RX=1
    // 清除状态寄存器
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, 0x70); // 清除所有中断标志
    // 清除FIFO
    NRF24L01_Write_Reg(FLUSH_RX, 0xFF);
//	NRF24L01_Write_Reg(CONFIG, 0x0B); // 0b00001011
    // 进入接收模式
    NRF_CE_H();

    delay_us(150);
}
// 设置为发送模式
void NRF24L01_Set_TX_Mode(void) {
    NRF_CE_L();
    // 设置发送地址
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t*)NRF_COMMON_ADDR, 5);
    // 设置接收地址0（用于接收ACK）
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF_COMMON_ADDR, 5);
    // 设置通道0数据长度（接收ACK包的长度）
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, NRF_PAYLOAD_LENGTH);
    // 通道0自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    // 通道0接收使能
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    // RF设置：2Mbps, 0dBm
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);
    // 自动重发：500us, 15次
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1F);
    // 选择频道40
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);
    // 配置为发送模式，上电
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0E); // PWR_UP=1, PRIM_RX=0
    // 清除状态寄存器
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, 0x70); // 清除所有中断标志
    // 清除FIFO
    NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
}
volatile uint8_t rx_flag=0;
volatile uint8_t tx_done=0;
uint8_t NRF24L01_RxPacket_IRQ(uint8_t *rxbuf)
{
	if(rx_flag==1)
	{
		rx_flag=0;
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, NRF_PAYLOAD_LENGTH);
		NRF24L01_Write_Reg(FLUSH_RX, 0xFF);  // 清除RX FIFO
		return 1;
	}
	return 0;

}
// 发送数据包（中断方式）
uint8_t NRF24L01_TxPacket_IRQ(uint8_t *txbuf) {
    tx_done = 0;
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, NRF_PAYLOAD_LENGTH);
    NRF_CE_H();
    delay_us(20);
    NRF_CE_L();
    return tx_done;
}

uint8_t NRF24L01_RxPacket_Polling(uint8_t *rxbuf) { //轮询接收数据
    uint8_t status = NRF24L01_Read_Reg(STATUS);
    
    if(status & (1 << RX_DR)) { // 检查接收完成标志
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, NRF_PAYLOAD_LENGTH);
        NRF24L01_Write_Reg(STATUS, status); // 清除中断标志
        NRF24L01_Write_Reg(FLUSH_RX, 0xFF);  // 清除RX FIFO
        return 1;
    }
    return 0;
}

// 中断处理函数
void EXTI9_5_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
#if ENABLE_NRF24L01_IRQ
		  uint8_t status = NRF24L01_Read_Reg(STATUS); // 正确读取状态寄存器
        
        // 清除中断标志（写1清除）
        NRF24L01_Write_Reg(STATUS, status); // 将当前状态写回即可清除标志
        
		if(status & (1 << RX_DR)) { // 检查接收完成标志
			rx_flag=1;
		}

#endif
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}