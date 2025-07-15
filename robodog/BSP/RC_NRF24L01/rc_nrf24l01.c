#include "rc_nrf24l01.h"
#include "rc_spi.h"
#include "stm32f4xx_exti.h"

const u8 NRF_COMMON_ADDR[5] = {'e','t','h','a','n'};

void nrfSetRxMode(void) {
	NRF_CE_L();
	
    // 设置接收地址和通道0数据长度
    SpiWriteBuf(NRF_WRITE_REG + RX_ADDR_P0, (u8*)NRF_COMMON_ADDR, 5);
    SpiWriteReg(NRF_WRITE_REG + RX_PW_P0, NRF_PAYLOAD_LENGTH);
    // 设置通道0自动应答
    SpiWriteReg(NRF_WRITE_REG + EN_AA, 0x01);
    // 设置通道0接收使能
    SpiWriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    // RF设置：2Mbps, 0dBm
    SpiWriteReg(NRF_WRITE_REG + RF_SETUP, 0x0F);
    // 自动重发：500us, 15次
    SpiWriteReg(NRF_WRITE_REG + SETUP_RETR, 0x1F);
    // 选择频道40 (2.440GHz)
    SpiWriteReg(NRF_WRITE_REG + RF_CH, 40);
    // 配置为接收模式，上电
    SpiWriteReg(NRF_WRITE_REG + CONFIG, 0x0F); // PWR_UP=1, PRIM_RX=1
    // 清除状态寄存器
    SpiWriteReg(NRF_WRITE_REG + STATUS, 0x70); // 清除所有中断标志
    // 清除FIFO
    SpiWriteReg(FLUSH_RX, 0xFF);
    // 进入接收模式
    NRF_CE_H();

    delay_us(150);
}
// 设置为发送模式
void nrfSetTxMode(void) {
    NRF_CE_L();
    // 设置发送地址
    SpiWriteBuf(NRF_WRITE_REG + TX_ADDR, (u8*)NRF_COMMON_ADDR, 5);
    // 设置接收地址0（用于接收ACK）
    SpiWriteBuf(NRF_WRITE_REG + RX_ADDR_P0, (u8*)NRF_COMMON_ADDR, 5);
    // 设置通道0数据长度（接收ACK包的长度）
    SpiWriteReg(NRF_WRITE_REG + RX_PW_P0, NRF_PAYLOAD_LENGTH);
    // 通道0自动应答
    SpiWriteReg(NRF_WRITE_REG + EN_AA, 0x01);
    // 通道0接收使能
    SpiWriteReg(NRF_WRITE_REG + EN_RXADDR, 0x01);
    // RF设置：2Mbps, 0dBm
    SpiWriteReg(NRF_WRITE_REG + RF_SETUP, 0x0F);
    // 自动重发：500us, 15次
    SpiWriteReg(NRF_WRITE_REG + SETUP_RETR, 0x1F);
    // 选择频道40
    SpiWriteReg(NRF_WRITE_REG + RF_CH, 40);
    // 配置为发送模式，上电
    SpiWriteReg(NRF_WRITE_REG + CONFIG, 0x0E); // PWR_UP=1, PRIM_RX=0
    // 清除状态寄存器
    SpiWriteReg(NRF_WRITE_REG + STATUS, 0x70); // 清除所有中断标志
    // 清除FIFO
    SpiWriteReg(FLUSH_TX, 0xFF);
}

volatile u8 rx_flag=0;
volatile u8 tx_done=0;
u8 nrfRxPacketIrq(u8 *rxbuf) //中断接收数据
{
	if(rx_flag==1)
	{
		rx_flag=0;
		SpiReadBuf(RD_RX_PLOAD, rxbuf, NRF_PAYLOAD_LENGTH);
		SpiWriteReg(FLUSH_RX, 0xFF);  // 清除RX FIFO
		return 1;
	}
	return 0;

}
// 发送数据包（中断方式）
u8 nrfTxPacketIrq(u8 *txbuf) {
    tx_done = 0;
    SpiWriteBuf(WR_TX_PLOAD, txbuf, NRF_PAYLOAD_LENGTH);
    NRF_CE_H();
    delay_us(20);
    NRF_CE_L();
    return tx_done;
}

u8 nrfRxPacketRolling(u8 *rxbuf) { //轮询接收数据
    u8 status = SpiReadReg(STATUS);
    
    if(status & (1 << RX_DR)) { // 检查接收完成标志
        SpiReadBuf(RD_RX_PLOAD, rxbuf, NRF_PAYLOAD_LENGTH);
        SpiWriteReg(STATUS, status); // 清除中断标志
        SpiWriteReg(FLUSH_RX, 0xFF);  // 清除RX FIFO
        return 1;
    }
    return 0;
}

// 中断处理函数
void EXTI9_5_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
#if ENABLE_NRF24L01_IRQ
		  u8 status = SpiReadReg(STATUS); // 正确读取状态寄存器
        
        // 清除中断标志（写1清除）
        SpiWriteReg(STATUS, status); // 将当前状态写回即可清除标志
        
		if(status & (1 << RX_DR)) { // 检查接收完成标志
			rx_flag=1;
		}

#endif
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}