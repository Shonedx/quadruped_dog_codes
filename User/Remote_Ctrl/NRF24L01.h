#ifndef __NRF24L01_H
#define __NRF24L01_H

#define STM32F4 

#ifdef STM32F1
#include "stm32f10x.h"
#define NRF_SPI_INSTANCE    SPI2 // STM32F103ͨ����SPI2
#define NRF_GPIO_CLK_CMD    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE)
#define NRF_SPI_CLK_CMD     RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE)
#define NRF_GPIO_AF_MODE    GPIO_Mode_AF_PP // F1�����������
#define NRF_GPIO_IN_MODE    GPIO_Mode_IPU   // F1��������
#define NRF_EXTI_PORT_SRC   GPIO_PortSourceGPIOB
#define NRF_EXTI_PIN_SRC    GPIO_PinSource11
#define NRF_EXTI_LINE       EXTI_Line11
#define NRF_EXTI_IRQN       EXTI15_10_IRQn

#elif defined(STM32F4)
#include "stm32f4xx.h"
#define NRF_SPI_INSTANCE    SPI1 // STM32F407ͨ����SPI1
#define NRF_GPIO_CLK_CMD    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE)
#define NRF_SPI_CLK_CMD     RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE)
#define NRF_SYSCFG_CLK_CMD  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE) // F4��ҪSYSCFGʱ��
#define NRF_GPIO_AF_MODE    GPIO_Mode_AF      // F4����ģʽ
#define NRF_GPIO_OUT_MODE   GPIO_Mode_OUT     // F4���ģʽ
#define NRF_GPIO_PP_OTYPE   GPIO_OType_PP     // F4�����������
#define NRF_GPIO_IN_MODE    GPIO_Mode_IN      // F4����ģʽ
#define NRF_GPIO_PU_PD      GPIO_PuPd_UP      // F4����
#define NRF_GPIO_NOPULL     GPIO_PuPd_NOPULL  // F4����
#define NRF_GPIO_AF_SCK     GPIO_AF_SPI1
#define NRF_GPIO_AF_MISO    GPIO_AF_SPI1
#define NRF_GPIO_AF_MOSI    GPIO_AF_SPI1
#define NRF_EXTI_PORT_SRC   EXTI_PortSourceGPIOC
#define NRF_EXTI_PIN_SRC    EXTI_PinSource5
#define NRF_EXTI_LINE       EXTI_Line5
#define NRF_EXTI_IRQN       EXTI9_5_IRQn
#else
#error "�붨��STM32F1��STM32F4��ѡ����ȷ��MCUͷ�ļ��ͺ궨��!"
#endif


// NRF24L01 �Ĵ�������
#define NRF_READ_REG        0x00  // ���Ĵ�������
#define NRF_WRITE_REG       0x20  // д�Ĵ�������
#define RD_RX_PLOAD         0x61  // ��������������
#define WR_TX_PLOAD         0xA0  // д������������
#define FLUSH_TX            0xE1  // ���TX FIFO����
#define FLUSH_RX            0xE2  // ���RX FIFO����
#define REUSE_TX_PL         0xE3  // ������һ��������������
#define NOP                 0xFF  // �ղ��������ڶ�״̬�Ĵ���

// NRF24L01 �Ĵ�����ַ
#define CONFIG          0x00  // ���üĴ���
#define EN_AA           0x01  // ʹ���Զ�Ӧ���ܼĴ���
#define EN_RXADDR       0x02  // ʹ�ܽ��յ�ַ�Ĵ���
#define SETUP_AW        0x03  // ��ַ������üĴ���
#define SETUP_RETR      0x04  // �Զ��ط����üĴ���
#define RF_CH           0x05  // RF�ŵ����üĴ���
#define RF_SETUP        0x06  // RF���üĴ���
#define STATUS          0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ��Ĵ���
#define CD              0x09  // �ز����Ĵ���
#define RX_ADDR_P0      0x0A  // ����ͨ��0���յ�ַ�Ĵ���
#define RX_ADDR_P1      0x0B  // ����ͨ��1���յ�ַ�Ĵ���
#define RX_ADDR_P2      0x0C  // ����ͨ��2���յ�ַ�Ĵ���
#define RX_ADDR_P3      0x0D  // ����ͨ��3���յ�ַ�Ĵ���
#define RX_ADDR_P4      0x0E  // ����ͨ��4���յ�ַ�Ĵ���
#define RX_ADDR_P5      0x0F  // ����ͨ��5���յ�ַ�Ĵ���
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ��������ͨ��0��Ч���ݿ�ȼĴ���
#define RX_PW_P1        0x12  // ��������ͨ��1��Ч���ݿ�ȼĴ���
#define RX_PW_P2        0x13  // ��������ͨ��2��Ч���ݿ�ȼĴ���
#define RX_PW_P3        0x14  // ��������ͨ��3��Ч���ݿ�ȼĴ���
#define RX_PW_P4        0x15  // ��������ͨ��4��Ч���ݿ�ȼĴ���
#define RX_PW_P5        0x16  // ��������ͨ��5��Ч���ݿ�ȼĴ���
#define FIFO_STATUS     0x17  // FIFO״̬�Ĵ���
#define DYNPD           0x1C  // ��̬���ݰ�����ʹ�ܼĴ���
#define FEATURE         0x1D  // ���ԼĴ���

// NRF24L01 ״̬�Ĵ���λ
#define RX_DR   6  // ���������жϱ�־
#define TX_DS   5  // ��������жϱ�־
#define MAX_RT  4  // ����ط������жϱ�־

// NRF24L01 ���üĴ���λ
#define PRIM_RX         0  // ����/����ģʽѡ�� (0:TX, 1:RX)
#define PWR_UP          1  // ��Դģʽ (0:����, 1:�ϵ�)

// NRF24L01 RF_SETUP �Ĵ���λ
#define RF_DR_LOW       5  // �������� (0:1Mbps, 1:250kbps)
#define RF_DR_HIGH      3  // �������� (0:1Mbps, 1:2Mbps)

// **��Ҫ�����ݰ����ȣ����˱�����ȫһ�£�**
#define NRF_PAYLOAD_LENGTH  20

// **��Ҫ��ͨ�ŵ�ַ�����˱�����ȫһ�£�**
// ����ʹ��ʮ�����ƣ������ַ���������
extern const uint8_t NRF_COMMON_ADDR[5];

// NRF24L01 ���Ŷ��� (CE, CSN, IRQ)
// �������Ŷ��彫�ڸ��Ե� main.c ����Ŀ���������
#define NRF_CE_H()      GPIO_SetBits(NRF_CE_Port, NRF_CE_Pin)
#define NRF_CE_L()      GPIO_ResetBits(NRF_CE_Port, NRF_CE_Pin)
#define NRF_CSN_H()     GPIO_SetBits(NRF_CSN_Port, NRF_CSN_Pin)
#define NRF_CSN_L()     GPIO_ResetBits(NRF_CSN_Port, NRF_CSN_Pin)
#define NRF_IRQ_READ()  GPIO_ReadInputDataBit(NRF_IRQ_Port, NRF_IRQ_Pin)


// �������� (ͨ�ò���)
void NRF24L01_Init_Common(void); // ͨ�ó�ʼ������
uint8_t NRF24L01_Check(void);
void NRF24L01_Set_RX_Mode(void); // ����Ϊ����ģʽ
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf); // �������ݰ� (�жϻ���ô˺���)
void NRF24L01_Set_TX_Mode(void); // ����Ϊ����ģʽ
uint8_t NRF24L01_TxPacket(uint8_t *txbuf); // �������ݰ�
void NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
void NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_Read_Reg(uint8_t reg);
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);
uint8_t NRF24L01_SpiRW(uint8_t dat);

#endif /* __NRF24L01_H */