#include "stm32f4xx.h"  // 包含STM32F4系列的硬件抽象层库头文件
#include "Allheaderfile.h"  // 包含其他自定义头文件（假设包含一些自定义的宏定义、结构体等）

// 定义RC通道的错误阈值，用于判断信号是否异常
#define RC_CHANNAL_ERROR_VALUE 700

// 声明一个静态函数，用于计算绝对值
static int16_t RC_abs(int16_t value);

// 声明一个静态函数，用于将SBUS协议数据转换为遥控器控制数据
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// 定义一个全局变量，用于存储遥控器控制数据
RC_ctrl_t rc_ctrl;

// 定义一个双缓冲接收缓冲区，用于DMA接收SBUS数据
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

// 定义变量，用于存储遥控器的前后左右信息（可能是用于后续的控制逻辑）
int left_x, left_y, right_x, right_y;

/**
 * 初始化遥控器模块，配置USART2和DMA用于接收SBUS数据。
 * @param rx1_buf DMA接收缓冲区1
 * @param rx2_buf DMA接收缓冲区2
 * @param dma_buf_num DMA缓冲区大小
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    /* 启用相关模块的时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // 启用GPIOA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);   // 启用DMA1时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 启用USART2时钟

    // 复位USART2
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);

    // 配置GPIOA3为USART2的复用功能
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    /* 配置GPIOA3为USART2的接收引脚 */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  // 选择GPIOA3
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  // 设置为复用功能
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  // 推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 设置GPIO速度为100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // 禁用内部上拉或下拉电阻
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // 初始化USART2
        USART_DeInit(USART2);  // 重置USART2

        USART_InitStructure.USART_BaudRate = 100000;  // 设置波特率为100000
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 数据长度为8位
        USART_InitStructure.USART_StopBits = USART_StopBits_1;  // 1个停止位
        USART_InitStructure.USART_Parity = USART_Parity_Even;  // 偶校验
        USART_InitStructure.USART_Mode = USART_Mode_Rx;  // 仅接收模式
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // 无硬件流控制
        USART_Init(USART2, &USART_InitStructure);

        // 启用USART2的DMA接收请求
        USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

        // 清除USART2的标志位
        USART_ClearFlag(USART2, USART_FLAG_IDLE);
        USART_ClearFlag(USART2, USART_FLAG_RXNE);

        // 启用USART2的空闲线中断
        USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

        // 启用USART2
        USART_Cmd(USART2, ENABLE);
    }

    /* 配置NVIC中断优先级 */
    {
        NVIC_InitTypeDef NVIC_InitStructure;

        // 配置USART2中断优先级
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 抢占优先级为0
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  // 子优先级为0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // 使能中断
        NVIC_Init(&NVIC_InitStructure);

        // 配置DMA1_Stream5中断优先级
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 抢占优先级为1
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  // 子优先级为0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  // 使能中断
        NVIC_Init(&NVIC_InitStructure);
    }

    /* 配置DMA1_Stream5用于接收SBUS数据 */
    {
        DMA_InitTypeDef DMA_InitStructure;

        // 初始化DMA1_Stream5
        DMA_DeInit(DMA1_Stream5);

        DMA_InitStructure.DMA_Channel = DMA_Channel_4;  // 选择通道4
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);  // 外设地址为USART2的DR寄存器
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;  // 内存地址为rx1_buf
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  // 数据传输方向为外设到内存
        DMA_InitStructure.DMA_BufferSize = dma_buf_num;  // 缓冲区大小
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // 禁用外设地址自增
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // 启用内存地址自增
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  // 外设数据大小为字节
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  // 内存数据大小为字节
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  // 循环模式
        DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  // 中等优先级
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // 禁用FIFO
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;  // FIFO阈值
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;  // 内存突发传输为单次
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  // 外设突发传输为单次
        DMA_Init(DMA1_Stream5, &DMA_InitStructure);

        // 配置双缓冲模式
        DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)rx2_buf, DMA_Memory_0);
        DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);

        // 启用DMA1_Stream5
        DMA_Cmd(DMA1_Stream5, DISABLE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
}

/**
 * 禁用遥控器模块（关闭USART2）
 */
void RC_unable(void)
{
    USART_Cmd(USART2, DISABLE);  // 禁用USART2
}

/**
 * 重启遥控器模块（重新初始化USART2和DMA）
 * @param dma_buf_num DMA缓冲区大小
 */
void RC_restart(uint16_t dma_buf_num)
{
    USART_Cmd(USART2, DISABLE);  // 禁用USART2
    DMA_Cmd(DMA1_Stream5, DISABLE);  // 禁用DMA1_Stream5

    // 重新设置DMA的当前数据计数器
    DMA_SetCurrDataCounter(DMA1_Stream5, dma_buf_num);

    // 清除USART2的空闲线标志
    USART_ClearFlag(USART2, USART_FLAG_IDLE);

    // 清除DMA的传输完成标志
    DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2);
    DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF2);

    // 重新启用DMA1_Stream5和USART2
    DMA_Cmd(DMA1_Stream5, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

/**
 * 初始化遥控器模块，调用RC_Init函数
 */
void remote_control_init(void)
{
    RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], RC_FRAME_LENGTH);  // 初始化遥控器模块
}

/**
 * 获取遥控器控制数据的指针
 * @return 遥控器控制数据的指针
 */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;  // 返回遥控器控制数据的地址
}

/**
 * 检查遥控器数据是否异常
 * @return 0表示数据正常，1表示数据异常
 */
uint8_t RC_data_is_error(void)
{
    // 检查每个通道的值是否超出阈值
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;  // 如果超出阈值，跳转到错误处理
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
    return 0;  // 数据正常

error:
    // 如果数据异常，将所有通道和开关值重置为默认值
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
    return 1;  // 返回数据异常
}

/**
 * 解决遥控器丢失问题（重启遥控器模块）
 */
void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);  // 重启遥控器模块
}

/**
 * 解决数据错误问题（重启遥控器模块）
 */
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);  // 重启遥控器模块
}

// 定义一个变量，用于记录本次接收到的数据长度
static uint16_t this_time_rx_len = 0;

/**
 * USART2中断服务函数，用于处理接收到的SBUS数据
 */
void USART2_IRQHandler(void)
{
    // 检查是否接收到数据
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        (void)USART2->SR;  // 清除状态寄存器
        (void)USART2->DR;  // 清除数据寄存器
    }
    // 检查是否检测到空闲线（表示数据接收完成）
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        (void)USART2->SR;  // 清除状态寄存器
        (void)USART2->DR;  // 清除数据寄存器

        // 判断当前使用的是哪个DMA缓冲区
        if (DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
        {
            // 如果当前使用的是缓冲区0
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);  // 计算接收到的数据长度
            DMA_Cmd(DMA1_Stream5, DISABLE);  // 禁用DMA
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);  // 重新设置DMA缓冲区大小
            DMA1_Stream5->CR |= DMA_SxCR_CT;  // 切换到缓冲区1
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);  // 清除DMA标志
            DMA_Cmd(DMA1_Stream5, ENABLE);  // 重新启用DMA

            // 将接收到的数据转换为遥控器控制数据
            SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                // 如果接收到的数据长度等于帧长度，再次处理数据（可能是冗余处理）
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            // 如果当前使用的是缓冲区1
            DMA_Cmd(DMA1_Stream5, DISABLE);  // 禁用DMA
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);  // 计算接收到的数据长度
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);  // 重新设置DMA缓冲区大小
            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);  // 切换到缓冲区0
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);  // 清除DMA标志
            DMA_Cmd(DMA1_Stream5, ENABLE);  // 重新启用DMA

            // 将接收到的数据转换为遥控器控制数据
            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

/**
 * 计算绝对值
 * @param value 输入值
 * @return 绝对值
 */
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;  // 如果值为正，直接返回
    }
    else
    {
        return -value;  // 如果值为负，返回其相反数
    }
}

/**
 * 将SBUS协议数据转换为遥控器控制数据
 * @param sbus_buf SBUS数据缓冲区
 * @param rc_ctrl 遥控器控制数据结构体
 */
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;  // 如果输入参数为空，直接返回
    }

    // 解析SBUS数据，提取各个通道和开关的值
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;  // 通道0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;  // 通道1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;  // 通道2
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;  // 通道3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);  // 开关0
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;  // 开关1

    // 解析鼠标和键盘数据（可能是自定义协议扩展）
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);  // 鼠标X轴
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);  // 鼠标Y轴
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);  // 鼠标Z轴
    rc_ctrl->mouse.press_l = sbus_buf[12];  // 鼠标左键
    rc_ctrl->mouse.press_r = sbus_buf[13];  // 鼠标右键
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);  // 键盘值
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);  // 通道4（可能未使用）

    // 对通道值进行偏移量调整
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    // 根据开关状态更新左右摇杆的值
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
