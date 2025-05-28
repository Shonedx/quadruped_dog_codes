#include "Allheaderfile.h"
//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败; 

u8 canbuf[8];//前四个3508电机输出电流值
u8 canbuf_[8];//储存数据帧
u8 canbuf2[8];//后四个3508电机输出电流值
u8 canbuf2_[8];//储存数据帧

u8 Can_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        CAN_InitTypeDef        CAN_InitStructure;
        CAN_FilterInitTypeDef  CAN_FilterInitStructure;
        #if CAN1_RX0_INT_ENABLE 
        NVIC_InitTypeDef  NVIC_InitStructure;
        #endif
        //使能相关时钟
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟                                                                                                                    
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟         

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN1时钟        

        //初始化GPIO
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉0 
        GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz- D) H/ t. l- r  a6 A; E' j+ h3 n
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
        GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA12,PA13

        //引脚复用映射配置
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN12 
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN14 

        GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11复用为CAN2
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12复用为CAN2
        //CAN单元设置
        CAN_InitStructure.CAN_TTCM=DISABLE;        //非时间触发通信模式   
        CAN_InitStructure.CAN_ABOM=DISABLE;        //软件自动离线管理        
        CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
        CAN_InitStructure.CAN_NART=ENABLE;        //禁止报文自动传送
        CAN_InitStructure.CAN_RFLM=DISABLE;        //报文不锁定,新的覆盖旧的  
        CAN_InitStructure.CAN_TXFP=DISABLE;        //优先级由报文标识符决定 
        CAN_InitStructure.CAN_Mode= mode;         //模式设置 
        CAN_InitStructure.CAN_SJW=tsjw;        //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
        CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
        CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~        
        CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1      
        CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1        
        CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2.
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
        CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化/ 
        /*---------------------------------------------------------------------------------------------------------------------------------*/               

        CAN_FilterInitStructure.CAN_FilterNumber=14; //过滤器14
        CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //屏蔽模式
        CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
        CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//32位ID
        CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;

        CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
        CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0) K! n- M$ m8 c- \8 h% J
        CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
        CAN_FilterInit(&CAN_FilterInitStructure);  //CAN2

        #if CAN1_RX0_INT_ENABLE

        CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.                    

        NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1,
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        #endif

        #if CAN2_RX0_INT_ENABLE

        CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.             

        NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        #endif
        return 0;
}
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器

  	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	
//	// CAN1滤波器配置（过滤器组0）
//	CAN_FilterInitStructure.CAN_FilterIdHigh = (0x201 << 5) & 0xFFFF;  // 标准ID左移5位对齐
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x7FC << 5;        // 允许低3位变化（0x201~0x207）
	
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}
/******can2设置*******/
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 
//  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, DISABLE);

    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PB11,PB12
	
	  //引脚复用映射配置
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11复用为CAN1
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
	
  	CAN_FilterInitStructure.CAN_FilterNumber=20;	  //过滤器14，can2从14开始
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 

	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	
//	// CAN2滤波器配置（过滤器组14）
//	CAN_FilterInitStructure.CAN_FilterIdHigh = (0x201 << 5) & 0xFFFF;  
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x7FC << 5;  
	
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN2_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}

void Set_Datas_From_Motor(Motor_Property * motors,u8 * buf)
{
	motors->received_current = (buf[1] << 8) | buf[0];
	motors->current_speed=(buf[2]<<8 | buf[3]);	//获得转子转速
	if(motors->current_speed>65536/2)//得到反转的速度
	motors->current_speed=-(0XFFFF-motors->current_speed);
	motors->ecd =(buf[0] << 8 |buf[1]);//获得转子机械角度范围是0~8191，对应360度
	motors->current_angle = motors->ecd/8192.0f*360.0f;//得到转子角度
	motors->last_ecd = motors->ecd;
}
 
#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
	// CanRxMsg RxMessage;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0) != RESET)
	{
		switch (CAN1_Receive_Msg(canbuf_))
		{
			case 1:
				Set_Datas_From_Motor(&motors.ID[0],canbuf_);
				break;
			case 2:
				Set_Datas_From_Motor(&motors.ID[1],canbuf_);
				break;
			case 3:
				Set_Datas_From_Motor(&motors.ID[2],canbuf_);
				break;
			case 4:
				Set_Datas_From_Motor(&motors.ID[3],canbuf_);
				break;	
			default:
				break;
		}			
		memset(canbuf_,0,sizeof(canbuf_));	
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}
#endif
/****************/
//can2中断//
/*************/
#if CAN2_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN2_RX0_IRQHandler(void)
{
	// CanRxMsg RxMessage;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0) != RESET)
	{
		switch (CAN2_Receive_Msg(canbuf2_))
		{
			case 1:
				Set_Datas_From_Motor(&motors.ID[4],canbuf2_);
				break;
			case 2:
				Set_Datas_From_Motor(&motors.ID[5],canbuf2_);
				break;
			case 3:
				Set_Datas_From_Motor(&motors.ID[6],canbuf2_);
				break;
			case 4:
				Set_Datas_From_Motor(&motors.ID[7],canbuf2_);
				break;
			default:
				break;
		}			
		memset(canbuf2_,0,sizeof(canbuf2_));	
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	}
}
#endif
//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=CAN_CHASSIS_ALL_ID;	 // 标准标识符为0
  TxMessage.ExtId=CAN_CHASSIS_ALL_ID;	 // 设置扩展标示符（29位）
  TxMessage.IDE=CAN_ID_STD;		  // 使用扩展标识符
  TxMessage.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
		TxMessage.Data[i]=msg[i];				          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
}
u8 CAN2_Send_Msg(u8* msg,u8 len)    //发送信息给电调ID6
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage2;
  TxMessage2.StdId=CAN_CHASSIS_ALL_ID;	 // 标准标识符为0
  TxMessage2.ExtId=CAN_CHASSIS_ALL_ID;	 // 设置扩展标示符（29位）
  TxMessage2.IDE=CAN_ID_STD;		  // 使用扩展标识符
  TxMessage2.RTR=CAN_RTR_DATA;		  // 消息类型为数据帧，一帧8位
  TxMessage2.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
		TxMessage2.Data[i]=msg[i];				          
  mbox= CAN_Transmit(CAN2, &TxMessage2);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;

u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
	if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)
	return -1;		//没有接收到数据,直接退出
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	for(i=0;i<RxMessage.DLC;i++)
		buf[i]=RxMessage.Data[i]; 
	return RxMessage.StdId-CAN_CHASSIS_ALL_ID;	
}
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage2;
	if(CAN_MessagePending(CAN2,CAN_FIFO0)==0)return -1;		//没有接收到数据,直接退出
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage2);//读取数据	
	for(i=0;i<RxMessage2.DLC;i++)
		buf[i]=RxMessage2.Data[i]; 
	return RxMessage2.StdId-CAN_CHASSIS_ALL_ID;	
}

void M3508_ALL_ZERO_SET(void)
{
	for(int i=0;i<8;i++)
	{
		motors.ID[i].absolute_angle=0;
	}
}











