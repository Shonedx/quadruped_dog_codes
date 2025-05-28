#include "nrf24l01.h"
#include "delay.h" // ȷ�����������delay.h�����ʵ����ʱ������
#include "oled.h"

// **��Ҫ��ͨ�ŵ�ַ�����˱�����ȫһ�£�**
// ����ʹ��ʮ�����ƣ������ַ���������
const uint8_t NRF_COMMON_ADDR[5] = {'e','t','h','a','n'};

// �����ⲿ��GPIO�˿ں����ţ����ǽ���main.c�ж���
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
 * @brief  ͨ�ó�ʼ����������ʼ��NRF24L01��GPIO��SPI���ⲿ�ж�
 * @param  ��
 * @retval ��
 */
void NRF24L01_Init_Common(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. ʹ�����GPIO��SPIʱ��
    NRF_GPIO_CLK_CMD; // ����MCU�궨��ʹ��GPIOʱ��
    NRF_SPI_CLK_CMD;  // ����MCU�궨��ʹ��SPIʱ��

    #ifdef STM32F4
    NRF_SYSCFG_CLK_CMD; // STM32F4��Ҫʹ��SYSCFGʱ������EXTI
    #endif

    // 2. ����SPI��GPIO���� (SCK, MISO, MOSI)
    // SCK, MOSI ����Ϊ�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // F1��F4���д���

    #ifdef STM32F4
    // STM32F4��SPI GPIO����
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_AF_MODE; // ����ģʽ
    GPIO_InitStructure.GPIO_OType = NRF_GPIO_PP_OTYPE; // �������
    GPIO_InitStructure.GPIO_PuPd = NRF_GPIO_PU_PD;     // ����
    GPIO_InitStructure.GPIO_Pin = NRF_MOSI_Pin | NRF_SCK_Pin;
    GPIO_Init(NRF_MOSI_Port, &GPIO_InitStructure); // MOSI��SCKͨ����ͬһ��GPIO_Port

    GPIO_PinAFConfig(NRF_SCK_Port, (NRF_SCK_Pin == GPIO_Pin_5 ? GPIO_PinSource5 : (NRF_SCK_Pin == GPIO_Pin_13 ? GPIO_PinSource13 : 0)), NRF_GPIO_AF_SCK); // SCK
    GPIO_PinAFConfig(NRF_MISO_Port, (NRF_MISO_Pin == GPIO_Pin_6 ? GPIO_PinSource6 : (NRF_MISO_Pin == GPIO_Pin_14 ? GPIO_PinSource14 : 0)), NRF_GPIO_AF_MISO); // MISO
    GPIO_PinAFConfig(NRF_MOSI_Port, (NRF_MOSI_Pin == GPIO_Pin_7 ? GPIO_PinSource7 : (NRF_MOSI_Pin == GPIO_Pin_15 ? GPIO_PinSource15 : 0)), NRF_GPIO_AF_MOSI); // MOSI

    // MISO ����Ϊ���루����ģʽ�£�ͨ��Ҳ��Ҫ���ã�
    GPIO_InitStructure.GPIO_Pin = NRF_MISO_Pin;
    GPIO_InitStructure.GPIO_PuPd = NRF_GPIO_PU_PD; // �������룬ȷ���ȶ�
    GPIO_Init(NRF_MISO_Port, &GPIO_InitStructure);

    #elif defined(STM32F1)
    // STM32F1��SPI GPIO����
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_AF_MODE; // �����������
    GPIO_InitStructure.GPIO_Pin = NRF_MOSI_Pin | NRF_SCK_Pin;
    GPIO_Init(NRF_MOSI_Port, &GPIO_InitStructure); // MOSI��SCKͨ����ͬһ��GPIO_Port

    // MISO ����Ϊ�����������������
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_IN_MODE; // MISO����������
    GPIO_InitStructure.GPIO_Pin = NRF_MISO_Pin;
    GPIO_Init(NRF_MISO_Port, &GPIO_InitStructure);
    #endif


    // 3. ����CE, CSN, IRQ����
    // CE - �������
    #ifdef STM32F4
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_OUT_MODE; // ͨ�����ģʽ
    GPIO_InitStructure.GPIO_OType = NRF_GPIO_PP_OTYPE; // �������
    GPIO_InitStructure.GPIO_PuPd = NRF_GPIO_NOPULL;   // ��������
    #elif defined(STM32F1)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // �������
    #endif
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = NRF_CE_Pin;
    GPIO_Init(NRF_CE_Port, &GPIO_InitStructure);

    // CSN - �������
    GPIO_InitStructure.GPIO_Pin = NRF_CSN_Pin;
    GPIO_Init(NRF_CSN_Port, &GPIO_InitStructure);

    // IRQ - �������� (�����ж�)
    #ifdef STM32F4
    GPIO_InitStructure.GPIO_Mode = NRF_GPIO_IN_MODE; // ͨ������ģʽ
    GPIO_InitStructure.GPIO_PuPd = NRF_GPIO_PU_PD;   // ��������ֹ����
    #elif defined(STM32F1)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // ��������
    #endif
    GPIO_InitStructure.GPIO_Pin = NRF_IRQ_Pin;
    GPIO_Init(NRF_IRQ_Port, &GPIO_InitStructure);

    // 4. ����SPI
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // ȫ˫��
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      // ����ģʽ
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  // 8λ����֡
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                         // ʱ�ӿ��е͵�ƽ
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                       // ��һ��ʱ���ز��� (ģʽ0)
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                          // ���NSS����
    #ifdef STM32F1
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // F1: PCLK1ͨ��36MHz����Ƶ8��4.5MHz
    #elif defined(STM32F4)
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; // F4: PCLK2ͨ��84MHz����Ƶ8��10.5MHz
    #endif
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                 // MSB����
    SPI_InitStructure.SPI_CRCPolynomial = 7;                           // CRC����ʽ��һ����Ĭ��ֵ
    SPI_Init(NRF_SPI_INSTANCE, &SPI_InitStructure);

    SPI_Cmd(NRF_SPI_INSTANCE, ENABLE); // ʹ��SPI

    // 5. ����IRQ�ⲿ�ж�
    #ifdef STM32F4
    SYSCFG_EXTILineConfig(NRF_EXTI_PORT_SRC, NRF_EXTI_PIN_SRC); // NRF_EXTI_PIN_SRC ��F4��GPIOC Pin5
    #elif defined(STM32F1)
    GPIO_EXTILineConfig(NRF_EXTI_PORT_SRC, NRF_EXTI_PIN_SRC); // NRF_EXTI_PIN_SRC ��F1��GPIOB Pin11
    #endif

    EXTI_InitStructure.EXTI_Line = NRF_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // NRF24L01�ж��ǵ͵�ƽ����
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 6. ����NVIC
    NVIC_InitStructure.NVIC_IRQChannel = NRF_EXTI_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // ��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;      // �����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 7. NRF24L01 �ϵ縴λ��ȷ����ʼ��״̬
    NRF_CE_L();  // CE���ͣ��������ģʽ
    NRF_CSN_H(); // CSN���ߣ���ֹSPI����

    delay_ms(100); // �ȴ�NRF24L01�ȶ�

    // ���NRF24L01�Ƿ����
    if(NRF24L01_Check() == 0)
    {
        
        while(1)
		{
			OLED_NewFrame();
			OLED_PrintASCIIString(0,0,"NRF24L01 Error",&afont12x6,OLED_COLOR_NORMAL);
			OLED_ShowFrame();
		} // ��ѭ������ʾ����
    }
   

    // ���FIFO
    NRF24L01_Write_Reg(FLUSH_TX, 0); // ���TX FIFO
    NRF24L01_Write_Reg(FLUSH_RX, 0); // ���RX FIFO
}

/**
 * @brief  NRF24L01 SPI��дһ���ֽ�
 * @param  dat: Ҫ���͵�����
 * @retval ���յ�������
 */
uint8_t NRF24L01_SpiRW(uint8_t dat)
{
    // �ȴ����ͻ�������
    while(SPI_I2S_GetFlagStatus(NRF_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET);
    // ��������
    SPI_I2S_SendData(NRF_SPI_INSTANCE, dat);
    // �ȴ����ջ�������
    while(SPI_I2S_GetFlagStatus(NRF_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET);
    // ���ؽ��յ�������
    return SPI_I2S_ReceiveData(NRF_SPI_INSTANCE);
}

/**
 * @brief  NRF24L01д�Ĵ���
 * @param  reg: �Ĵ�����ַ
 * @param  value: Ҫд���ֵ
 * @retval ״̬�Ĵ���ֵ
 */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    NRF_CSN_L();                    // ʹ��SPI����
    status = NRF24L01_SpiRW(NRF_WRITE_REG | reg); // ����д����ͼĴ�����ַ
    NRF24L01_SpiRW(value);          // ��������
    NRF_CSN_H();                    // ��ֹSPI����
    return status;
}

/**
 * @brief  NRF24L01���Ĵ���
 * @param  reg: �Ĵ�����ַ
 * @retval �Ĵ���ֵ
 */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    NRF_CSN_L();                    // ʹ��SPI����
    NRF24L01_SpiRW(NRF_READ_REG | reg); // ���Ͷ�����ͼĴ�����ַ
    reg_val = NRF24L01_SpiRW(NOP);  // ��ȡ����
    NRF_CSN_H();                    // ��ֹSPI����
    return reg_val;
}

/**
 * @brief  NRF24L01д������
 * @param  reg: �Ĵ�����ַ
 * @param  pBuf: ���ݻ�����ָ��
 * @param  len: д�볤��
 * @retval ״̬�Ĵ���ֵ (���ﲻ���أ���ͨ����NOP����ķ���ֵ)
 */
void NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    NRF_CSN_L();                    // ʹ��SPI����
    NRF24L01_SpiRW(reg);            // �������� (WR_TX_PLOAD �� NRF_WRITE_REG | ��ַ)
    for(i = 0; i < len; i++)
    {
        NRF24L01_SpiRW(pBuf[i]);    // ��������
    }
    NRF_CSN_H();                    // ��ֹSPI����
}

/**
 * @brief  NRF24L01��������
 * @param  reg: �Ĵ�����ַ
 * @param  pBuf: ���ݻ�����ָ��
 * @param  len: ��ȡ����
 * @retval ״̬�Ĵ���ֵ (���ﲻ���أ���ͨ����NOP����ķ���ֵ)
 */
void NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    NRF_CSN_L();                    // ʹ��SPI����
    NRF24L01_SpiRW(reg);            // �������� (RD_RX_PLOAD �� NRF_READ_REG | ��ַ)
    for(i = 0; i < len; i++)
    {
        pBuf[i] = NRF24L01_SpiRW(NOP); // ��ȡ����
    }
    NRF_CSN_H();                    // ��ֹSPI����
}

/**
 * @brief  ���NRF24L01ģ���Ƿ����
 * @param  ��
 * @retval 0: ������, 1: ����
 */
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5];
    uint8_t i;
    // ʹ��һ����ʵ��ͨ�ŵ�ַ��ͬ�Ĳ��Ե�ַ
    const uint8_t NRF_TEST_ADDR[5] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E};

    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t*)NRF_TEST_ADDR, 5); // д��һ�����Ե�ַ
    NRF24L01_Read_Buf(NRF_READ_REG + TX_ADDR, buf, 5); // ����������Ե�ַ

    for(i = 0; i < 5; i++)
    {
        if(buf[i] != NRF_TEST_ADDR[i])
        {
            return 0; // ��ַ��һ�£�ģ����ܲ����ڻ�������
        }
    }
    return 1; // ���ͨ��
}

/**
 * @brief  NRF24L01�������ģʽ
 * @param  ��
 * @retval ��
 */
void NRF24L01_Set_RX_Mode(void)
{
    NRF_CE_L(); // �������ģʽ

    // ���ý��յ�ַ (P0ͨ��)
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF_COMMON_ADDR, 5);
    // ���ý������ݿ�� (P0ͨ���������뷢�Ͷ����ݰ���Сһ��)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, NRF_PAYLOAD_LENGTH);

    // ����RF�ŵ� (RF_CH): ����Ϊ2.440GHz (Ƶ��40)���뷢�Ͷ�һ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);

    // ����RF���� (RF_SETUP): 2Mbps�������ʣ�0dBm���书��
    // 0x0F: RF_DR_HIGH=1 (2Mbps), RF_PWR=0b11 (0dBm), LNA_HCURR=1 (�ߵ���LNA����)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);

    // �����Զ�Ӧ�� (EN_AA): ����ͨ��0���Զ�Ӧ���Ա㷢�Ͷ����յ�ACK
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01); // 0x01: 00000001B

    // ����ͨ�üĴ��� (CONFIG):
    // EN_CRC (CRCʹ��), CRC_FORCED_EN (CRCУ��ģʽ������2�ֽ�), PWR_UP (�ϵ�), PRIM_RX (1Ϊ����ģʽ)
    // MASK_RX_DR (���������жϲ�����), MASK_TX_DS (��������жϲ�����), MASK_MAX_RT (����ط��жϲ�����)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F); // 0x0F: 00001111B, PWR_UP=1, PRIM_RX=1, CRC EN, 2-byte CRC

    // ��������жϱ�־
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    // ���FIFO (ȷ������֮ǰFIFO�ǿյ�)
    NRF24L01_Write_Reg(FLUSH_TX, 0);
    NRF24L01_Write_Reg(FLUSH_RX, 0);

    NRF_CE_H(); // ����CE��NRF24L01�������ģʽ
    delay_us(150); // CE���ߺ���Ҫ�ȴ�130us���ܽ���RXģʽ�������150us������
  
}

/**
 * @brief  NRF24L01�������ݰ�
 * @param  rxbuf: �������ݻ�����ָ��
 * @retval 1=���ճɹ�, 0=δ���յ�����
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t status = NRF24L01_Read_Reg(STATUS); // ��ȡ״̬�Ĵ���
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, status);	
    if(status & (1<<RX_DR)) // ������յ����� (RX_DR ��λ)
    {
        // ���FIFO״̬��ȷ�������ݿɶ����Ҳ��ǿհ�
        uint8_t fifo_status = NRF24L01_Read_Reg(FIFO_STATUS);
        if(!(fifo_status & 0x01)) // ���RX FIFO��Ϊ�� (RX_EMPTYλΪ0)
        {
            NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, NRF_PAYLOAD_LENGTH); // ��ȡ���յ�������
			NRF24L01_Write_Reg(FLUSH_RX,0xFF);
			
			// ���NRF24L01�ڲ����жϱ�־
					

            return 1; // ���ճɹ�
        }
    }
    return 0; // δ���յ�����
}

/**
 * @brief  NRF24L01���뷢��ģʽ
 * @param  ��
 * @retval ��
 */
void NRF24L01_Set_TX_Mode(void)
{
    NRF_CE_L(); // �������ģʽ

    // ���÷��͵�ַ (TX_ADDR)
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t*)NRF_COMMON_ADDR, 5);
    // ���ý���ͨ��0�ĵ�ַΪ���͵�ַ���Խ���ACK (�Զ�Ӧ��)
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF_COMMON_ADDR, 5);

    // �����Զ�Ӧ�� (EN_AA): ����ͨ��0���Զ�Ӧ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01); // 0x01: 00000001B

    // �����Զ��ط� (SETUP_RETR): 500us�ط����������ط�15��
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1F); // ARD=0x01 (500us), ARC=0x0F (15��)

    // ����RF�ŵ� (RF_CH): ����Ϊ2.440GHz (Ƶ��40)������ն�һ��
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);

    // ����RF���� (RF_SETUP): 2Mbps�������ʣ�0dBm���书��
    // 0x0F: RF_DR_HIGH=1 (2Mbps), RF_PWR=0b11 (0dBm), LNA_HCURR=1 (�ߵ���LNA����)
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0F);

    // ����ͨ�üĴ��� (CONFIG):
    // EN_CRC (CRCʹ��), CRC_FORCED_EN (CRCУ��ģʽ������2�ֽ�), PWR_UP (�ϵ�), PRIM_RX (0Ϊ����ģʽ)
    // MASK_TX_DS (��������жϲ�����), MASK_MAX_RT (����ط��жϲ�����), MASK_RX_DR (���������жϲ�����)
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0E); // 0x0E: 00001110B, PWR_UP=1, PRIM_RX=0, CRC EN, 2-byte CRC

    // ��������жϱ�־
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    // ���FIFO (�ٴ�ȷ�ϣ�ȷ������֮ǰFIFO�ǿյ�)
    NRF24L01_Write_Reg(FLUSH_TX, 0);
    NRF24L01_Write_Reg(FLUSH_RX, 0);

    // NRF_CE_H(); // ����ģʽ�£�CEֻ�ڷ���ʱ�������ߣ�ƽʱӦ���ֵ͵�ƽ
    
}

/**
 * @brief  NRF24L01�������ݰ�
 * @param  txbuf: �������ݻ�����ָ��
 * @retval ���ͽ��: 0=����ʧ��(��ʱ), 1=���ͳɹ�, 2=�ﵽ����ط�����
 */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t status;
    uint32_t tx_timeout = 0x100000; // ���ӳ�ʱ������Ӧ�Ը��ӻ�����������������ʱ��

    NRF_CE_L(); // �������ģʽ

    // д�뷢�����ݵ�TX FIFO
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, NRF_PAYLOAD_LENGTH);

    NRF_CE_H(); // ����CE����������
    delay_us(15); // CE��������10us�������15us������һ��
    NRF_CE_L(); // CE���ͣ�NRF24L01�������ģʽI���ȴ�TX��ɻ�MAX_RT

    // �ȴ�������ɻ�ﵽ����ط����� (ͨ����ѯSTATUS�Ĵ���)
    do {
        status = NRF24L01_Read_Reg(STATUS); // ��ȡ״̬�Ĵ���
        if(tx_timeout-- == 0) // ��ʱ�ж�
        {
            NRF24L01_Write_Reg(FLUSH_TX, 0); // ��ʱ���TX FIFO
            // ͬʱ������е��жϱ�־�������ظ������ж�
            NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));
            return 0; // ����ʧ�� (��ʱ)
        }
        // delay_us(1); // ���̵���ʱ�����⼫�˿�ת�������׷���ٶȿ����Ƴ�
    } while(!(status & (1<<TX_DS) || status & (1<<MAX_RT)));

    // ��������жϱ�־ (TX_DS �� MAX_RT)
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, (1<<TX_DS)|(1<<MAX_RT));

    if(status & (1<<TX_DS)) // ���ͳɹ� (�յ�ACK)
    {
        return 1;
    }
    else // status & (1<<MAX_RT) �ﵽ����ط�����
    {
        NRF24L01_Write_Reg(FLUSH_TX, 0); // ���TX FIFO
        return 2; // �ﵽ����ط�����
    }
}