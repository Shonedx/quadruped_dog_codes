#include "stm32f4xx.h"  // ����STM32F4ϵ�е�Ӳ��������ͷ�ļ�
#include "Allheaderfile.h"  // ���������Զ���ͷ�ļ����������һЩ�Զ���ĺ궨�塢�ṹ��ȣ�

// ����RCͨ���Ĵ�����ֵ�������ж��ź��Ƿ��쳣
#define RC_CHANNAL_ERROR_VALUE 700

// ����һ����̬���������ڼ������ֵ
static int16_t RC_abs(int16_t value);

// ����һ����̬���������ڽ�SBUSЭ������ת��Ϊң������������
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// ����һ��ȫ�ֱ��������ڴ洢ң������������
RC_ctrl_t rc_ctrl;

// ����һ��˫������ջ�����������DMA����SBUS����
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

// ������������ڴ洢ң������ǰ��������Ϣ�����������ں����Ŀ����߼���
int left_x, left_y, right_x, right_y;

/**
 * ��ʼ��ң����ģ�飬����USART2��DMA���ڽ���SBUS���ݡ�
 * @param rx1_buf DMA���ջ�����1
 * @param rx2_buf DMA���ջ�����2
 * @param dma_buf_num DMA��������С
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    /* �������ģ���ʱ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // ����GPIOAʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);   // ����DMA1ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // ����USART2ʱ��

    // ��λUSART2
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);

    // ����GPIOA3ΪUSART2�ĸ��ù���
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* ����GPIOA3ΪUSART2�Ľ������� */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  // ѡ��GPIOA3
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  // ����Ϊ���ù���
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // �������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // ����GPIO�ٶ�Ϊ100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // �����ڲ���������������
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // ��ʼ��USART2
        USART_DeInit(USART2);  // ����USART2

        USART_InitStructure.USART_BaudRate = 100000;  // ���ò�����Ϊ100000
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // ���ݳ���Ϊ8λ
        USART_InitStructure.USART_StopBits = USART_StopBits_1;  // 1��ֹͣλ
        USART_InitStructure.USART_Parity = USART_Parity_Even;  // żУ��
        USART_InitStructure.USART_Mode = USART_Mode_Rx;  // ������ģʽ
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ��Ӳ��������
        USART_Init(USART2, &USART_InitStructure);

        // ����USART2��DMA��������
        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

        // ���USART2�ı�־λ
        USART_ClearFlag(USART2, USART_FLAG_IDLE);
        USART_ClearFlag(USART2, USART_FLAG_RXNE);

        // ����USART2�Ŀ������ж�
        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

        // ����USART2
        USART_Cmd(USART2, ENABLE);
    }

    /* ����NVIC�ж����ȼ� */
    {
        NVIC_InitTypeDef NVIC_InitStructure;

        // ����USART2�ж����ȼ�
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // ��ռ���ȼ�Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  // �����ȼ�Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // ʹ���ж�
        NVIC_Init(&NVIC_InitStructure);

        // ����DMA1_Stream5�ж����ȼ�
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // ��ռ���ȼ�Ϊ1
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  // �����ȼ�Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // ʹ���ж�
        NVIC_Init(&NVIC_InitStructure);
    }

    /* ����DMA1_Stream5���ڽ���SBUS���� */
    {
        DMA_InitTypeDef DMA_InitStructure;

        // ��ʼ��DMA1_Stream5
        DMA_DeInit(DMA1_Stream5);

        DMA_InitStructure.DMA_Channel = DMA_Channel_4;  // ѡ��ͨ��4
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);  // �����ַΪUSART2��DR�Ĵ���
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;  // �ڴ��ַΪrx1_buf
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  // ���ݴ��䷽��Ϊ���赽�ڴ�
        DMA_InitStructure.DMA_BufferSize = dma_buf_num;  // ��������С
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // ���������ַ����
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // �����ڴ��ַ����
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  // �������ݴ�СΪ�ֽ�
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  // �ڴ����ݴ�СΪ�ֽ�
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  // ѭ��ģʽ
        DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  // �е����ȼ�
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // ����FIFO
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;  // FIFO��ֵ
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;  // �ڴ�ͻ������Ϊ����
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  // ����ͻ������Ϊ����
        DMA_Init(DMA1_Stream5, &DMA_InitStructure);

        // ����˫����ģʽ
        DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)rx2_buf, DMA_Memory_0);
        DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);

        // ����DMA1_Stream5
        DMA_Cmd(DMA1_Stream5, DISABLE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
}

/**
 * ����ң����ģ�飨�ر�USART2��
 */
void RC_unable(void)
{
    USART_Cmd(USART2, DISABLE);  // ����USART2
}

/**
 * ����ң����ģ�飨���³�ʼ��USART2��DMA��
 * @param dma_buf_num DMA��������С
 */
void RC_restart(uint16_t dma_buf_num)
{
    USART_Cmd(USART2, DISABLE);  // ����USART2
    DMA_Cmd(DMA1_Stream5, DISABLE);  // ����DMA1_Stream5

    // ��������DMA�ĵ�ǰ���ݼ�����
    DMA_SetCurrDataCounter(DMA1_Stream5, dma_buf_num);

    // ���USART2�Ŀ����߱�־
    USART_ClearFlag(USART2, USART_FLAG_IDLE);

    // ���DMA�Ĵ�����ɱ�־
    DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2);
    DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF2);

    // ��������DMA1_Stream5��USART2
    DMA_Cmd(DMA1_Stream5, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

/**
 * ��ʼ��ң����ģ�飬����RC_Init����
 */
void remote_control_init(void)
{
    RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], RC_FRAME_LENGTH);  // ��ʼ��ң����ģ��
}

/**
 * ��ȡң�����������ݵ�ָ��
 * @return ң�����������ݵ�ָ��
 */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;  // ����ң�����������ݵĵ�ַ
}

/**
 * ���ң���������Ƿ��쳣
 * @return 0��ʾ����������1��ʾ�����쳣
 */
uint8_t RC_data_is_error(void)
{
    // ���ÿ��ͨ����ֵ�Ƿ񳬳���ֵ
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;  // ���������ֵ����ת��������
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;  // ��������

error:
    // ��������쳣��������ͨ���Ϳ���ֵ����ΪĬ��ֵ
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;  // ���������쳣
}

/**
 * ���ң������ʧ���⣨����ң����ģ�飩
 */
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);  // ����ң����ģ��
}

/**
 * ������ݴ������⣨����ң����ģ�飩
 */
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);  // ����ң����ģ��
}

// ����һ�����������ڼ�¼���ν��յ������ݳ���
static uint16_t this_time_rx_len = 0;

/**
 * USART2�жϷ����������ڴ�����յ���SBUS����
 */
void USART2_IRQHandler(void)
{
    // ����Ƿ���յ�����
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        (void)USART2->SR;  // ���״̬�Ĵ���
        (void)USART2->DR;  // ������ݼĴ���
    }
    // ����Ƿ��⵽�����ߣ���ʾ���ݽ�����ɣ�
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        (void)USART2->SR;  // ���״̬�Ĵ���
        (void)USART2->DR;  // ������ݼĴ���

        // �жϵ�ǰʹ�õ����ĸ�DMA������
        if (DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
        {
            // �����ǰʹ�õ��ǻ�����0
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);  // ������յ������ݳ���
            DMA_Cmd(DMA1_Stream5, DISABLE);  // ����DMA
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);  // ��������DMA��������С
            DMA1_Stream5->CR |= DMA_SxCR_CT;  // �л���������1
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);  // ���DMA��־
            DMA_Cmd(DMA1_Stream5, ENABLE);  // ��������DMA

            // �����յ�������ת��Ϊң������������
            SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // ������յ������ݳ��ȵ���֡���ȣ��ٴδ������ݣ����������ദ��
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            // �����ǰʹ�õ��ǻ�����1
            DMA_Cmd(DMA1_Stream5, DISABLE);  // ����DMA
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);  // ������յ������ݳ���
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);  // ��������DMA��������С
            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);  // �л���������0
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);  // ���DMA��־
            DMA_Cmd(DMA1_Stream5, ENABLE);  // ��������DMA

            // �����յ�������ת��Ϊң������������
            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

/**
 * �������ֵ
 * @param value ����ֵ
 * @return ����ֵ
 */
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;  // ���ֵΪ����ֱ�ӷ���
    }
    else
    {
        return -value;  // ���ֵΪ�����������෴��
    }
}

/**
 * ��SBUSЭ������ת��Ϊң������������
 * @param sbus_buf SBUS���ݻ�����
 * @param rc_ctrl ң�����������ݽṹ��
 */
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;  // ����������Ϊ�գ�ֱ�ӷ���
    }

    // ����SBUS���ݣ���ȡ����ͨ���Ϳ��ص�ֵ
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;  // ͨ��0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;  // ͨ��1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;  // ͨ��2
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;  // ͨ��3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);  // ����0
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;  // ����1

    // �������ͼ������ݣ��������Զ���Э����չ��
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);  // ���X��
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);  // ���Y��
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);  // ���Z��
    rc_ctrl->mouse.press_l = sbus_buf[12];  // ������
    rc_ctrl->mouse.press_r = sbus_buf[13];  // ����Ҽ�
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);  // ����ֵ
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);  // ͨ��4������δʹ�ã�

    // ��ͨ��ֵ����ƫ��������
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    // ���ݿ���״̬��������ҡ�˵�ֵ
    if (rc_ctrl->rc.s[1] == 3)
    {
        left_x = rc_ctrl->rc.ch[2];
        left_y = rc_ctrl->rc.ch[3];
    }
    if (rc_ctrl->rc.s[1] == 1)
    {
        left_x = rc_ctrl->rc.ch[2];
        left_y = rc_ctrl->rc.ch[3];
    }
    if (rc_ctrl->rc.s[0] == 3)
    {
        right_x = rc_ctrl->rc.ch[0];
        right_y = rc_ctrl->rc.ch[1];
    }
    if (rc_ctrl->rc.s[0] == 1)
    {
        right_x = rc_ctrl->rc.ch[0];
        right_y = rc_ctrl->rc.ch[1];
    }
}
