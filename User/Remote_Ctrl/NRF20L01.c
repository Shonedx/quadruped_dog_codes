#include "nrf24l01.h"
#include "delay.h" // 确保这里引入的delay.h是你的实际延时函数库
#include "oled.h"

// **重要：通信地址，两端必须完全一致！**
// 建议使用十六进制，避免字符编码问题
const uint8_t NRF_COMMON_ADDR[5] = {'e','t','h','a','n'};

// 声明外部的GPIO端口和引脚，它们将在main.c中定义
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

/**
 * @brief  通用初始化函数：初始化NRF24L01的GPIO、SPI和外部中断
 * @param  无
 * @retval 无
 */
void NRF24L01_Init_Common(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 使能相关GPIO和SPI时钟
    NRF_GPIO_CLK_CMD; // 根据MCU宏定义使能GPIO时钟
    NRF_SPI_CLK_CMD;  // 根据MCU宏定义使能SPI时钟

    #ifdef STM32F4
    NRF_SYSCFG_CLK_CMD; // STM32F4需要使能SYSCFG时钟用于EXTI
    #endif

    // 2. 配置SPI的GPIO引脚 (SCK, MISO, MOSI)
    // SCK, MOSI 配置为复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // F1和F4都有此项

    #ifdef STM32F4
    // STM32F4的SPI GPIO配置
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_AF_MODE; // 复用模式
    GPIO_InitStructure.GPIO_OType = NRF_GPIO_PP_OTYPE; // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = NRF_GPIO_PU_PD;     // 上拉
    GPIO_InitStructure.GPIO_Pin = NRF_MOSI_Pin | NRF_SCK_Pin;
    GPIO_Init(NRF_MOSI_Port, &GPIO_InitStructure); // MOSI和SCK通常在同一个GPIO_Port

    GPIO_PinAFConfig(NRF_SCK_Port, (NRF_SCK_Pin == GPIO_Pin_5 ? GPIO_PinSource5 : (NRF_SCK_Pin == GPIO_Pin_13 ? GPIO_PinSource13 : 0)), NRF_GPIO_AF_SCK); // SCK
    GPIO_PinAFConfig(NRF_MISO_Port, (NRF_MISO_Pin == GPIO_Pin_6 ? GPIO_PinSource6 : (NRF_MISO_Pin == GPIO_Pin_14 ? GPIO_PinSource14 : 0)), NRF_GPIO_AF_MISO); // MISO
    GPIO_PinAFConfig(NRF_MOSI_Port, (NRF_MOSI_Pin == GPIO_Pin_7 ? GPIO_PinSource7 : (NRF_MOSI_Pin == GPIO_Pin_15 ? GPIO_PinSource15 : 0)), NRF_GPIO_AF_MOSI); // MOSI

    // MISO 配置为输入（复用模式下，通常也需要设置）
    GPIO_InitStructure.GPIO_Pin = NRF_MISO_Pin;
    GPIO_InitStructure.GPIO_PuPd = NRF_GPIO_PU_PD; // 上拉输入，确保稳定
    GPIO_Init(NRF_MISO_Port, &GPIO_InitStructure);

    #elif defined(STM32F1)
    // STM32F1的SPI GPIO配置
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_AF_MODE; // 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = NRF_MOSI_Pin | NRF_SCK_Pin;
    GPIO_Init(NRF_MOSI_Port, &GPIO_InitStructure); // MOSI和SCK通常在同一个GPIO_Port

    // MISO 配置为浮空输入或上拉输入
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_IN_MODE; // MISO是上拉输入
    GPIO_InitStructure.GPIO_Pin = NRF_MISO_Pin;
    GPIO_Init(NRF_MISO_Port, &GPIO_InitStructure);
    #endif


    // 3. 配置CE, CSN, IRQ引脚
    // CE - 推挽输出
    #ifdef STM32F4
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_OUT_MODE; // 通用输出模式
    GPIO_InitStructure.GPIO_OType = NRF_GPIO_PP_OTYPE; // 推挽输出
    GPIO_InitStructure.GPIO_PuPd = NRF_GPIO_NOPULL;   // 无上下拉
    #elif defined(STM32F1)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    #endif
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = NRF_CE_Pin;
    GPIO_Init(NRF_CE_Port, &GPIO_InitStructure);

    // CSN - 推挽输出
    GPIO_InitStructure.GPIO_Pin = NRF_CSN_Pin;
    GPIO_Init(NRF_CSN_Port, &GPIO_InitStructure);

    // IRQ - 浮空输入 (用于中断)
    #ifdef STM32F4
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_IN_MODE; // 通用输入模式
    GPIO_InitStructure.GPIO_PuPd = NRF_GPIO_PU_PD;   // 上拉，防止浮空
    #elif defined(STM32F1)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    #endif
    GPIO_InitStructure.GPIO_Pin = NRF_IRQ_Pin;
    GPIO_Init(NRF_IRQ_Port, &GPIO_InitStructure);

    // 4. 配置SPI
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      // 主机模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  // 8位数据帧
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         // 时钟空闲低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                       // 第一个时钟沿采样 (模式0)
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          // 软件NSS管理
    #ifdef STM32F1
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // F1: PCLK1通常36MHz，分频8是4.5MHz
    #elif defined(STM32F4)
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // F4: PCLK2通常84MHz，分频8是10.5MHz
    #endif
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 // MSB优先
    SPI_InitStructure.SPI_CRCPolynomial = 7;                           // CRC多项式，一般用默认值
    SPI_Init(NRF_SPI_INSTANCE, &SPI_InitStructure);

    SPI_Cmd(NRF_SPI_INSTANCE, ENABLE); // 使能SPI

    // 5. 配置IRQ外部中断
    #ifdef STM32F4
    SYSCFG_EXTILineConfig(NRF_EXTI_PORT_SRC, NRF_EXTI_PIN_SRC); // NRF_EXTI_PIN_SRC 在F4的GPIOC Pin5
    #elif defined(STM32F1)
    GPIO_EXTILineConfig(NRF_EXTI_PORT_SRC, NRF_EXTI_PIN_SRC); // NRF_EXTI_PIN_SRC 在F1的GPIOB Pin11
    #endif

    EXTI_InitStructure.EXTI_Line = NRF_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // NRF24L01中断是低电平触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 6. 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = NRF_EXTI_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 7. NRF24L01 上电复位，确保初始化状态
    NRF_CE_L();  // CE拉低，进入待机模式
    NRF_CSN_H(); // CSN拉高，禁止SPI操作

    delay_ms(100); // 等待NRF24L01稳定

    // 检查NRF24L01是否存在
    if(NRF24L01_Check() == 0)
    {
        
        while(1)
		{
			OLED_NewFrame();
			OLED_PrintASCIIString(0,0,"NRF24L01 Error",&afont12x6,OLED_COLOR_NORMAL);
			OLED_ShowFrame();
		} // 死循环，提示错误
    }
   

    // 清除FIFO
    NRF24L01_Write_Reg(FLUSH_TX, 0); // 清除TX FIFO
    NRF24L01_Write_Reg(FLUSH_RX, 0); // 清除RX FIFO
}

/**
 * @brief  NRF24L01 SPI读写一个字节
 * @param  dat: 要发送的数据
 * @retval 接收到的数据
 */
uint8_t NRF24L01_SpiRW(uint8_t dat)
{
    // 等待发送缓冲区空
    while(SPI_I2S_GetFlagStatus(NRF_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET);
    // 发送数据
    SPI_I2S_SendData(NRF_SPI_INSTANCE, dat);
    // 等待接收缓冲区满
    while(SPI_I2S_GetFlagStatus(NRF_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET);
    // 返回接收到的数据
    return SPI_I2S_ReceiveData(NRF_SPI_INSTANCE);
}

/**
 * @brief  NRF24L01写寄存器
 * @param  reg: 寄存器地址
 * @param  value: 要写入的值
 * @retval 状态寄存器值
 */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    NRF_CSN_L();                    // 使能SPI传输
    status = NRF24L01_SpiRW(NRF_WRITE_REG | reg); // 发送写命令和寄存器地址
    NRF24L01_SpiRW(value);          // 发送数据
    NRF_CSN_H();                    // 禁止SPI传输
    return status;
}

/**
 * @brief  NRF24L01读寄存器
 * @param  reg: 寄存器地址
 * @retval 寄存器值
 */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    NRF_CSN_L();                    // 使能SPI传输
    NRF24L01_SpiRW(NRF_READ_REG | reg); // 发送读命令和寄存器地址
    reg_val = NRF24L01_SpiRW(NOP);  // 读取数据
    NRF_CSN_H();                    // 禁止SPI传输
    return reg_val;
}

/**
 * @brief  NRF24L01写缓冲区
 * @param  reg: 寄存器地址
 * @param  pBuf: 数据缓冲区指针
 * @param  len: 写入长度
 * @retval 状态寄存器值 (这里不返回，但通常是NOP命令的返回值)
 */
void NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    NRF_CSN_L();                    // 使能SPI传输
    NRF24L01_SpiRW(reg);            // 发送命令 (WR_TX_PLOAD 或 NRF_WRITE_REG | 地址)
    for(i = 0; i < len; i++)
    {
        NRF24L01_SpiRW(pBuf[i]);    // 发送数据
    }
    NRF_CSN_H();                    // 禁止SPI传输
}

/**
 * @brief  NRF24L01读缓冲区
 * @param  reg: 寄存器地址
 * @param  pBuf: 数据缓冲区指针
 * @param  len: 读取长度
 * @retval 状态寄存器值 (这里不返回，但通常是NOP命令的返回值)
 */
void NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    NRF_CSN_L();                    // 使能SPI传输
    NRF24L01_SpiRW(reg);            // 发送命令 (RD_RX_PLOAD 或 NRF_READ_REG | 地址)
    for(i = 0; i < len; i++)
    {
        pBuf[i] = NRF24L01_SpiRW(NOP); // 读取数据
    }
    NRF_CSN_H();                    // 禁止SPI传输
}

/**
 * @brief  检查NRF24L01模块是否存在
 * @param  无
 * @retval 0: 不存在, 1: 存在
 */
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5];
    uint8_t i;
    // 使用一个与实际通信地址不同的测试地址
    const uint8_t NRF_TEST_ADDR[5] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E};

    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t*)NRF_TEST_ADDR, 5); // 写入一个测试地址
    NRF24L01_Read_Buf(NRF_READ_REG + TX_ADDR, buf, 5); // 读回这个测试地址

    for(i = 0; i < 5; i++)
    {
        if(buf[i] != NRF_TEST_ADDR[i])
        {
            return 0; // 地址不一致，模块可能不存在或有问题
        }
    }
    return 1; // 检查通过
}

/**
 * @brief  NRF24L01进入接收模式
 * @param  无
 * @retval 无
 */
void NRF24L01_Set_RX_Mode(void)
{
    NRF_CE_L(); // 进入待机模式

    // 配置接收地址 (P0通道)
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF_COMMON_ADDR, 5);
    // 配置接收数据宽度 (P0通道，必须与发送端数据包大小一致)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, NRF_PAYLOAD_LENGTH);

    // 配置RF信道 (RF_CH): 设置为2.440GHz (频道40)，与发送端一致
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);

    // 配置RF设置 (RF_SETUP): 2Mbps数据速率，0dBm发射功率
    // 0x0F: RF_DR_HIGH=1 (2Mbps), RF_PWR=0b11 (0dBm), LNA_HCURR=1 (高电流LNA增益)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);

    // 配置自动应答 (EN_AA): 开启通道0的自动应答，以便发送端能收到ACK
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01); // 0x01: 00000001B

    // 配置通用寄存器 (CONFIG):
    // EN_CRC (CRC使能), CRC_FORCED_EN (CRC校验模式，这里2字节), PWR_UP (上电), PRIM_RX (1为接收模式)
    // MASK_RX_DR (接收数据中断不屏蔽), MASK_TX_DS (发送完成中断不屏蔽), MASK_MAX_RT (最大重发中断不屏蔽)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F); // 0x0F: 00001111B, PWR_UP=1, PRIM_RX=1, CRC EN, 2-byte CRC

    // 清除所有中断标志
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    // 清除FIFO (确保接收之前FIFO是空的)
    NRF24L01_Write_Reg(FLUSH_TX, 0);
    NRF24L01_Write_Reg(FLUSH_RX, 0);

    NRF_CE_H(); // 拉高CE，NRF24L01进入接收模式
    delay_us(150); // CE拉高后需要等待130us才能进入RX模式，这里给150us更保险
  
}

/**
 * @brief  NRF24L01接收数据包
 * @param  rxbuf: 接收数据缓冲区指针
 * @retval 1=接收成功, 0=未接收到数据
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t status = NRF24L01_Read_Reg(STATUS); // 读取状态寄存器
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, status);	
    if(status & (1<<RX_DR)) // 如果接收到数据 (RX_DR 置位)
    {
        // 检查FIFO状态，确保有数据可读，且不是空包
        uint8_t fifo_status = NRF24L01_Read_Reg(FIFO_STATUS);
        if(!(fifo_status & 0x01)) // 如果RX FIFO不为空 (RX_EMPTY位为0)
        {
            NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, NRF_PAYLOAD_LENGTH); // 读取接收到的数据
			NRF24L01_Write_Reg(FLUSH_RX,0xFF);
			
			// 清除NRF24L01内部的中断标志
					

            return 1; // 接收成功
        }
    }
    return 0; // 未接收到数据
}

/**
 * @brief  NRF24L01进入发送模式
 * @param  无
 * @retval 无
 */
void NRF24L01_Set_TX_Mode(void)
{
    NRF_CE_L(); // 进入待机模式

    // 配置发送地址 (TX_ADDR)
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t*)NRF_COMMON_ADDR, 5);
    // 配置接收通道0的地址为发送地址，以接收ACK (自动应答)
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF_COMMON_ADDR, 5);

    // 配置自动应答 (EN_AA): 开启通道0的自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01); // 0x01: 00000001B

    // 配置自动重发 (SETUP_RETR): 500us重发间隔，最多重发15次
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1F); // ARD=0x01 (500us), ARC=0x0F (15次)

    // 配置RF信道 (RF_CH): 设置为2.440GHz (频道40)，与接收端一致
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);

    // 配置RF设置 (RF_SETUP): 2Mbps数据速率，0dBm发射功率
    // 0x0F: RF_DR_HIGH=1 (2Mbps), RF_PWR=0b11 (0dBm), LNA_HCURR=1 (高电流LNA增益)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);

    // 配置通用寄存器 (CONFIG):
    // EN_CRC (CRC使能), CRC_FORCED_EN (CRC校验模式，这里2字节), PWR_UP (上电), PRIM_RX (0为发送模式)
    // MASK_TX_DS (发送完成中断不屏蔽), MASK_MAX_RT (最大重发中断不屏蔽), MASK_RX_DR (接收数据中断不屏蔽)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0E); // 0x0E: 00001110B, PWR_UP=1, PRIM_RX=0, CRC EN, 2-byte CRC

    // 清除所有中断标志
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    // 清除FIFO (再次确认，确保发送之前FIFO是空的)
    NRF24L01_Write_Reg(FLUSH_TX, 0);
    NRF24L01_Write_Reg(FLUSH_RX, 0);

    // NRF_CE_H(); // 发送模式下，CE只在发送时短暂拉高，平时应保持低电平
    
}

/**
 * @brief  NRF24L01发送数据包
 * @param  txbuf: 发送数据缓冲区指针
 * @retval 发送结果: 0=发送失败(超时), 1=发送成功, 2=达到最大重发次数
 */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t status;
    uint32_t tx_timeout = 0x100000; // 增加超时计数，应对复杂环境，但会增加阻塞时间

    NRF_CE_L(); // 进入待机模式

    // 写入发送数据到TX FIFO
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, NRF_PAYLOAD_LENGTH);

    NRF_CE_H(); // 拉高CE，触发发送
    delay_us(15); // CE至少拉高10us，这里给15us更保险一点
    NRF_CE_L(); // CE拉低，NRF24L01进入待机模式I，等待TX完成或MAX_RT

    // 等待发送完成或达到最大重发次数 (通过轮询STATUS寄存器)
    do {
        status = NRF24L01_Read_Reg(STATUS); // 读取状态寄存器
        if(tx_timeout-- == 0) // 超时判断
        {
            NRF24L01_Write_Reg(FLUSH_TX, 0); // 超时清空TX FIFO
            // 同时清除所有的中断标志，避免重复触发中断
            NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));
            return 0; // 发送失败 (超时)
        }
        // delay_us(1); // 极短的延时，避免极端空转，但如果追求速度可以移除
    } while(!(status & (1<<TX_DS) || status & (1<<MAX_RT)));

    // 清除发送中断标志 (TX_DS 或 MAX_RT)
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, (1<<TX_DS)|(1<<MAX_RT));

    if(status & (1<<TX_DS)) // 发送成功 (收到ACK)
    {
        return 1;
    }
    else // status & (1<<MAX_RT) 达到最大重发次数
    {
        NRF24L01_Write_Reg(FLUSH_TX, 0); // 清除TX FIFO
        return 2; // 达到最大重发次数
    }
}