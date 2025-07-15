#include "stm32f4xx_can.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "can.h"

u8 can1_tx_buffer[8]={0};
u8 can2_tx_buffer[8]={0};
u8 can1_rx_buffer[8]={0};
u8 can2_rx_buffer[8]={0};

MotorData_t can1_motors[4]={0};
MotorData_t can2_motors[4]={0};

uint8_t canInit(void)
{
    CAN_InitTypeDef CAN_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
//初始化can
    // 初始化can1和can2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

    // 配置收发引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    CAN_StructInit(&CAN_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    CAN_StructInit(&CAN_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    // Initialize CAN1
    CAN_DeInit(CAN1); //设置默认值
    CAN_DeInit(CAN2); //设置默认值

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
    
    CAN_InitStructure.CAN_Prescaler = 6; // Assuming 84 MHz clock

    CAN_Init(CAN1, &CAN_InitStructure);
    CAN_Init(CAN2, &CAN_InitStructure);

//配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber=0;          //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //屏蔽模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;   

    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);//CAN1  

    CAN_FilterInitStructure.CAN_FilterNumber=14; //过滤器14
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //屏蔽模式
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;

    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0)
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);  //CAN2

//配置中断
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.                    
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1,
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.             
    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    if (CAN_Init(CAN1, &CAN_InitStructure) != SUCCESS || CAN_Init(CAN2, &CAN_InitStructure) != SUCCESS){ //初始化错误检测
        return 0; // Initialization failed
    }
    return 1; // Initialization successful
}
void calDataFromCAN(MotorData_t *motor_data,u8 *data) //解析can发来的数据
{
    motor_data->current = (data[4] << 8) | data[5];
	motor_data->speed=(data[2]<<8)| data[3];	//获得转子转速
    motor_data->mechanical_angle=((data[0]<<8)|data[1])/8192*360;
    motor_data->temperature=data[6]; //获得温度

    if(motor_data->speed>(u16)0xFFFF/2)
    {
        motor_data->speed = motor_data->speed - 0xFFFF; // 处理负值
    }
}
u8 rxDataFromCAN(u8 *rx_msg,u8 can_id)
{
    u32 i;
	CanRxMsg RxMessage;
    if(can_id==0)
    {
        if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)
            return -1;		//没有接收到数据,直接退出
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    }
    else if(can_id==1)
    {
        if(CAN_MessagePending(CAN2,CAN_FIFO0)==0)
            return -1;		//没有接收到数据,直接退出
        CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//读取数据	
    }
	for(i=0;i<RxMessage.DLC;i++)
		rx_msg[i]=RxMessage.Data[i]; 
	return RxMessage.StdId-CAN_ALL_ID;	
}
u8 txDataToCAN(u8* tx_msg,u8 len,u8 can_id)
{	
    u8 mbox;
    u16 i=0;
    CanTxMsg TxMessage;
    TxMessage.StdId=CAN_ALL_ID;	 // 标准标识符为0
    TxMessage.ExtId=CAN_ALL_ID;	 // 设置扩展标示符（29位）
    TxMessage.IDE=CAN_ID_STD;		  // 使用扩展标识符
    TxMessage.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
    TxMessage.DLC=len;							 // 发送两帧信息
    for(i=0;i<len;i++)
            TxMessage.Data[i]=tx_msg[i];
    if(can_id==0)
    {
        mbox= CAN_Transmit(CAN1, &TxMessage);   
        while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
    }				          
    else if(can_id==1)
    {
        mbox= CAN_Transmit(CAN2, &TxMessage);   
        while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
    }
    i=0;
    if(i>=0xFFFF)return 0; //发送失败
    return 1;	//发送成功
}

void CAN1_RX0_IRQHandler(void)
{
	// CanRxMsg RxMessage;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0) != RESET)
	{
		switch (rxDataFromCAN(can1_rx_buffer,0)) // 0 表示can1
		{
			case 1:
				calDataFromCAN(&can1_motors[0],can1_rx_buffer);
				break;
			case 2:
				calDataFromCAN(&can1_motors[1],can1_rx_buffer);
				break;
			case 3:
				calDataFromCAN(&can1_motors[2],can1_rx_buffer);
				break;
			case 4:
				calDataFromCAN(&can1_motors[3],can1_rx_buffer);
				break;	
			default:
				break;
		}			
		memset(can1_rx_buffer,0,sizeof(can1_rx_buffer));	
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}
void CAN2_RX0_IRQHandler(void)
{
	// CanRxMsg RxMessage;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0) != RESET)
	{
		switch (rxDataFromCAN(can2_rx_buffer,1)) //1 表示can2 
		{
			case 1:
				calDataFromCAN(&can2_motors[0],can2_rx_buffer);
				break;
			case 2:
				calDataFromCAN(&can2_motors[1],can2_rx_buffer);
				break;
			case 3:
				calDataFromCAN(&can2_motors[2],can2_rx_buffer);
				break;
			case 4:
				calDataFromCAN(&can2_motors[3],can2_rx_buffer);
				break;
			default:
				break;
		}			
		memset(can2_rx_buffer,0,sizeof(can2_rx_buffer));	
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	}
}