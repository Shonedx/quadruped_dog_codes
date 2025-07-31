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
#define ENABLE_NRF24L01_IRQ 0
const uint8_t NRF_COMMON_ADDR[5] = {'e','t','h','a','n'};
// ���Ŷ��壨�����������ã�
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
    
    // 1. ʹ��ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_SYSCFG, ENABLE);
    
    // 2. ����SPI���� (PA5-SCK, PA6-MISO, PA7-MOSI)
    GPIO_InitStructure.GPIO_Pin = NRF_SCK_Pin | NRF_MISO_Pin | NRF_MOSI_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);  // SCK: PA5
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);  // MISO: PA6
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);  // MOSI: PA7
    
    // 3. ����CE��CSN��IRQ����
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
    
    // 4. ����SPI
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
	// 5. �����ⲿ�ж� (IRQ on PC5)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5); // ���������ȷ������
    
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
    // 6. ��ʼ��״̬
    NRF_CE_L();
    NRF_CSN_H();
	delay_ms(100);	
    
    // 7. ���ģ��
    if(NRF24L01_Check() == 0) {
        // ��������
        while(1);
    }
	 // ���FIFO
    NRF24L01_Write_Reg(FLUSH_TX, 0xFF);
    NRF24L01_Write_Reg(FLUSH_RX, 0xFF);
}

uint8_t NRF24L01_SpiRW(uint8_t dat) {
    // �ȴ����ͻ�������
    while(SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_TXE) == RESET);
    // ��������
    SPI_I2S_SendData(NRF_SPI, dat);
    // �ȴ����ջ�������
    while(SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    // ���ؽ��յ�������
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
	
    // ���ý��յ�ַ��ͨ��0���ݳ���
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF_COMMON_ADDR, 5);
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, NRF_PAYLOAD_LENGTH);
    // ����ͨ��0�Զ�Ӧ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    // ����ͨ��0����ʹ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    // RF���ã�2Mbps, 0dBm
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);
    // �Զ��ط���500us, 15��
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1F);
    // ѡ��Ƶ��40 (2.440GHz)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);
    // ����Ϊ����ģʽ���ϵ�
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F); // PWR_UP=1, PRIM_RX=1
    // ���״̬�Ĵ���
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, 0x70); // ��������жϱ�־
    // ���FIFO
    NRF24L01_Write_Reg(FLUSH_RX, 0xFF);
//	NRF24L01_Write_Reg(CONFIG, 0x0B); // 0b00001011
    // �������ģʽ
    NRF_CE_H();

    delay_us(150);
}
// ����Ϊ����ģʽ
void NRF24L01_Set_TX_Mode(void) {
    NRF_CE_L();
    // ���÷��͵�ַ
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t*)NRF_COMMON_ADDR, 5);
    // ���ý��յ�ַ0�����ڽ���ACK��
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF_COMMON_ADDR, 5);
    // ����ͨ��0���ݳ��ȣ�����ACK���ĳ��ȣ�
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, NRF_PAYLOAD_LENGTH);
    // ͨ��0�Զ�Ӧ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
    // ͨ��0����ʹ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    // RF���ã�2Mbps, 0dBm
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);
    // �Զ��ط���500us, 15��
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1F);
    // ѡ��Ƶ��40
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);
    // ����Ϊ����ģʽ���ϵ�
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0E); // PWR_UP=1, PRIM_RX=0
    // ���״̬�Ĵ���
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, 0x70); // ��������жϱ�־
    // ���FIFO
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
		NRF24L01_Write_Reg(FLUSH_RX, 0xFF);  // ���RX FIFO
		return 1;
	}
	return 0;

}
// �������ݰ����жϷ�ʽ��
uint8_t NRF24L01_TxPacket_IRQ(uint8_t *txbuf) {
    tx_done = 0;
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, NRF_PAYLOAD_LENGTH);
    NRF_CE_H();
    delay_us(20);
    NRF_CE_L();
    return tx_done;
}

uint8_t NRF24L01_RxPacket_Polling(uint8_t *rxbuf) { //��ѯ��������
    uint8_t status = NRF24L01_Read_Reg(STATUS);
    
    if(status & (1 << RX_DR)) { // ��������ɱ�־
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, NRF_PAYLOAD_LENGTH);
        NRF24L01_Write_Reg(STATUS, status); // ����жϱ�־
        NRF24L01_Write_Reg(FLUSH_RX, 0xFF);  // ���RX FIFO
        return 1;
    }
    return 0;
}

// �жϴ�������
void EXTI9_5_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
#if ENABLE_NRF24L01_IRQ
		  uint8_t status = NRF24L01_Read_Reg(STATUS); // ��ȷ��ȡ״̬�Ĵ���
        
        // ����жϱ�־��д1�����
        NRF24L01_Write_Reg(STATUS, status); // ����ǰ״̬д�ؼ��������־
        
		if(status & (1 << RX_DR)) { // ��������ɱ�־
			rx_flag=1;
		}

#endif
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}