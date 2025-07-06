#include "stm32f4xx_rcc.h"
#include "Allheaderfile.h"
#include "RC_Command.h"
#define RC_CHANNAL_ERROR_VALUE 700  // 通道值错误阈值，超过此值视为异常数据

// 取绝对值函数（用于通道值校验）
static int16_t RC_abs(int16_t value);

// SBUS协议解析函数（核心）
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

// 遥控器控制结构体（存储解析后的数据）
RC_ctrl_t rc_ctrl;  // 全局变量，存储通道值/开关状态等

// 双缓冲DMA接收数组（防止数据溢出）
static uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];  // SBUS帧长25字节，双缓冲设计

// 摇杆原始数据缓存（调试用）
int left_x, left_y, right_x, right_y;  // 范围：-660~660

/**
 * 初始化DMA和串口1（USART2）
 * @param rx1_buf 缓冲区1指针
 * @param rx2_buf 缓冲区2指针
 * @param dma_buf_num DMA缓冲区大小
 * 注：SBUS协议要求 100000波特率/8数据位/偶校验/2停止位(8E2)[10,11](@ref)
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    /* ------------ 时钟使能 ------------ */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  

    /* ------------ GPIO配置 ------------ */
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    // PA3复用为USART2_RX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 高速模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // 无上拉下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ------------ 串口配置 ------------ */
    USART_DeInit(USART2);
    USART_InitStructure.USART_BaudRate = 100000;        // SBUS标准波特率[10](@ref)
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;       // 2停止位(实际配置见下)
    USART_InitStructure.USART_Parity = USART_Parity_Even;        // 偶校验
    USART_InitStructure.USART_Mode = USART_Mode_Rx;              // 仅接收
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无流控
    USART_Init(USART2, &USART_InitStructure);
    
    // 手动配置CR2寄存器实现2停止位
    USART2->CR2 |= USART_StopBits_2;  // 设置2位停止位[10](@ref)
    
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  // 使能DMA接收
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // 使能空闲中断
    USART_Cmd(USART2, ENABLE);                      // 启动串口

    /* ------------ NVIC中断配置 ------------ */
    NVIC_InitTypeDef NVIC_InitStructure;
    // USART2中断（空闲中断）
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 高优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // DMA1_Stream5中断（传输完成中断）
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // 次优先级
    NVIC_Init(&NVIC_InitStructure);

    /* ------------ DMA双缓冲配置 ------------ */
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;            // 通道4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR); // 外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf; // 内存缓冲区1
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;    // 外设到内存
    DMA_InitStructure.DMA_BufferSize = dma_buf_num;            // 缓冲区大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;    // 内存地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;            // 循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      // 中优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;     // 禁用FIFO（实时性要求高）
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    
    // 双缓冲配置：rx2_buf作为第二缓冲区
    DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)rx2_buf, DMA_Memory_0);
    DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
    DMA_Cmd(DMA1_Stream5, ENABLE);  // 启动DMA
}

/** 补充说明：
 * 双缓冲模式：当DMA正在写入缓冲区1时，用户可读取缓冲区2，避免数据竞争
 * 禁用FIFO：SBUS要求实时处理，数据到达后立即搬运[11](@ref)
 */

// 失能串口（紧急停止）
void RC_unable(void) {
    USART_Cmd(USART2, DISABLE);
}

// 重启RC接收（错误恢复）
void RC_restart(uint16_t dma_buf_num) {
    USART_Cmd(USART2, DISABLE);
    DMA_Cmd(DMA1_Stream5, DISABLE);
    
    DMA_SetCurrDataCounter(DMA1_Stream5, dma_buf_num);  // 重置数据计数器
    USART_ClearFlag(USART2, USART_FLAG_IDLE);           // 清除空闲标志
    DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2);        // 清除传输完成标志
    
    DMA_Cmd(DMA1_Stream5, ENABLE);
    USART_Cmd(USART2, ENABLE);  // 重新启动
}

// 遥控器初始化入口
void remote_control_init(void) {
    RC_Init(SBUS_rx_buf[0], SBUS_rx_buf[1], RC_FRAME_LENGTH);
}

// 获取遥控器数据结构体指针
const RC_ctrl_t *get_remote_control_point(void) {
    return &rc_ctrl;
}

/**
 * 校验RC数据是否异常
 * @return 0=正常, 1=异常
 * 规则：通道值绝对值不超过700，开关状态不为0[10](@ref)
 */
uint8_t RC_data_is_error(void) {
    // 检查四个主通道
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE) goto error;
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE) goto error;
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE) goto error;
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE) goto error;
    
    // 检查开关状态
    if (rc_ctrl.rc.s[0] == 0) goto error;
    if (rc_ctrl.rc.s[1] == 0) goto error;
    
    return 0;  // 数据正常

error:  // 异常处理
    rc_ctrl.rc.ch[0] = 0;  // 通道值清零
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;  // 开关置为"向下"状态
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;           // 鼠标数据清零
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;             // 键盘值清零
    return 1;  // 返回异常标志
}

// 处理RC信号丢失
void slove_RC_lost(void) {
    RC_restart(SBUS_RX_BUF_NUM);  // 重启接收
}

// 处理数据错误
void slove_data_error(void) {
    RC_restart(SBUS_RX_BUF_NUM);  // 重启接收
}

static uint16_t this_time_rx_len = 0;  // 当前接收数据长度

/**
 * USART2中断服务函数（核心）
 * 处理空闲中断(IDLE)：当一帧数据接收完成时触发
 */
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
        (void)USART2->SR;  // 读SR寄存器清除标志
        (void)USART2->DR;  // 读DR寄存器清除标志
        
        // 判断当前DMA缓冲区
        if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0) {
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
            
            // 双缓冲切换：当前使用缓冲区0，切换到缓冲区1
            DMA_Cmd(DMA1_Stream5, DISABLE);
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
            DMA1_Stream5->CR |= DMA_SxCR_CT;  // 切换内存目标
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA1_Stream5, ENABLE);
            
            // 校验长度并解析（SBUS标准帧25字节）
            if(this_time_rx_len == RC_FRAME_LENGTH) {
                SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);  // 解析缓冲区0数据
            }
        } 
        else {  // 当前使用缓冲区1
            DMA_Cmd(DMA1_Stream5, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);  // 切换回内存目标0
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA1_Stream5, ENABLE);
            
            if(this_time_rx_len == RC_FRAME_LENGTH) {
                SBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);  // 解析缓冲区1数据
            }
        }
    }
}

// 绝对值计算（用于通道值校验）
static int16_t RC_abs(int16_t value) {
    return (value > 0) ? value : -value;
}

/**
 * SBUS协议解析核心函数
 * @param sbus_buf 原始SBUS数据帧（25字节）
 * @param rc_ctrl 解析结果存储结构体
 * SBUS帧结构：[头][CH1-CH16][标志位][尾] = 1+22+1+1=25字节[10,11](@ref)
 * 每个通道11bit，共16通道（22字节存储16 * 11=176bit）
 */
static void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl) {
    if (!sbus_buf || !rc_ctrl) return;

    /* ------------ 通道数据解析（11bit/通道） ------------ */
    // 通道0: 字节0的低8位 + 字节1的低3位
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
    // 通道1: 字节1的高5位 + 字节2的低6位
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;
    // 通道2: 字节2的高2位 + 字节3的8位 + 字节4的低1位
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;
    // 通道3: 字节4的高7位 + 字节5的低4位
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;
    
    /* ------------ 开关状态解析 ------------ */
    // s[0]：左开关（字节5的bit4-5）
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);
    // s[1]：右开关（字节5的bit6-7）
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 6) & 0x0003);
    
    /* ------------ 鼠标数据解析 ------------ */
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);      // X轴（2字节）
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);      // Y轴
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);    // Z轴（滚轮）
    rc_ctrl->mouse.press_l = sbus_buf[12];                    // 左键按下
    rc_ctrl->mouse.press_r = sbus_buf[13];                    // 右键按下
    
    /* ------------ 键盘数据 ------------ */
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);      // 键盘值（2字节）
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);   // 扩展通道（通常未使用）

    /* ------------ 数据归一化（移除偏移量）------------ */
    for(int i=0; i<5; i++) {
        rc_ctrl->rc.ch[i] -= RC_CH_VALUE_OFFSET;  // 典型偏移量1024
    }

    /* ------------ 摇杆数据映射（根据开关状态） ------------ */
    // 右摇杆：通道0/1（X/Y）
    if(rc_ctrl->rc.s[0] == 3 || rc_ctrl->rc.s[0] == 1) {
        right_x = rc_ctrl->rc.ch[0];
        right_y = rc_ctrl->rc.ch[1];
    }
    
    // 左摇杆：通道2/3（X/Y）
    if(rc_ctrl->rc.s[1] == 3 || rc_ctrl->rc.s[1] == 1) {
        left_x = rc_ctrl->rc.ch[2];
        left_y = rc_ctrl->rc.ch[3];
    }
}