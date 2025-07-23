#include "Allheaderfile.h"
#include "can.h"
#include "main_params.h"
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 

u8 canbuf[8];//ǰ�ĸ�3508����������ֵ
u8 canbuf_[8];//��������֡
u8 canbuf2[8];//���ĸ�3508����������ֵ
u8 canbuf2_[8];//��������֡

u8 Can_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
        GPIO_InitTypeDef GPIO_InitStructure;
        CAN_InitTypeDef        CAN_InitStructure;
        CAN_FilterInitTypeDef  CAN_FilterInitStructure;
        #if CAN1_RX0_INT_ENABLE 
        NVIC_InitTypeDef  NVIC_InitStructure;
        #endif
        //ʹ�����ʱ��
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��                                                                                                                    
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��         

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��        
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN1ʱ��        

        //��ʼ��GPIO
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����0 
        GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz- D) H/ t. l- r  a6 A; E' j+ h3 n
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
        GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA12,PA13

        //���Ÿ���ӳ������
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN12 
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN14 

        GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11����ΪCAN2
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12����ΪCAN2
        //CAN��Ԫ����
        CAN_InitStructure.CAN_TTCM=DISABLE;        //��ʱ�䴥��ͨ��ģʽ   
        CAN_InitStructure.CAN_ABOM=DISABLE;        //�����Զ����߹���        
        CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
        CAN_InitStructure.CAN_NART=ENABLE;        //��ֹ�����Զ�����
        CAN_InitStructure.CAN_RFLM=DISABLE;        //���Ĳ�����,�µĸ��Ǿɵ�  
        CAN_InitStructure.CAN_TXFP=DISABLE;        //���ȼ��ɱ��ı�ʶ������ 
        CAN_InitStructure.CAN_Mode= mode;         //ģʽ���� 
        CAN_InitStructure.CAN_SJW=tsjw;        //����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
        CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
        CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~        
        CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1      
        CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1        
        CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN2.
        //���ù�����
        CAN_FilterInitStructure.CAN_FilterNumber=0;          //������0
        CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //����ģʽ
        CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ
        CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//32λID
        CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;   

        CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
        CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
        CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
        CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��/ 
        /*---------------------------------------------------------------------------------------------------------------------------------*/               

        CAN_FilterInitStructure.CAN_FilterNumber=14; //������14
        CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //����ģʽ
        CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
        CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//32λID
        CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;

        CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
        CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0) K! n- M$ m8 c- \8 h% J
        CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
        CAN_FilterInit(&CAN_FilterInitStructure);  //CAN2

        #if CAN1_RX0_INT_ENABLE

        CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.                    

        NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1,
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        #endif

        #if CAN2_RX0_INT_ENABLE

        CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.             

        NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
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
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//�����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����

  	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
	
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	
//	// CAN1�˲������ã���������0��
//	CAN_FilterInitStructure.CAN_FilterIdHigh = (0x201 << 5) & 0xFFFF;  // ��׼ID����5λ����
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x7FC << 5;        // ������3λ�仯��0x201~0x207��
	
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}
/******can2����*******/
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 
//  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN2ʱ��
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, DISABLE);

    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PB11,PB12
	
	  //���Ÿ���ӳ������
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11����ΪCAN1
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//�����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ����������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
	
  	CAN_FilterInitStructure.CAN_FilterNumber=20;	  //������14��can2��14��ʼ
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 

	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	
//	// CAN2�˲������ã���������14��
//	CAN_FilterInitStructure.CAN_FilterIdHigh = (0x201 << 5) & 0xFFFF;  
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x7FC << 5;  
	
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN2_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}

void Set_Datas_From_Motor(Motor_Property * motors,u8 * buf)
{
	motors->received_current = (buf[1] << 8) | buf[0];
	motors->current_speed=(buf[2]<<8 | buf[3]);	//���ת��ת��
	if(motors->current_speed>65536/2)//�õ���ת���ٶ�
	motors->current_speed=-(0XFFFF-motors->current_speed);
	motors->ecd =(buf[0] << 8 |buf[1]);//���ת�ӻ�е�Ƕȷ�Χ��0~8191����Ӧ360��
	motors->current_angle = motors->ecd/8192.0f*360.0f;//�õ�ת�ӽǶ�
	motors->last_ecd = motors->ecd;
}
 
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
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
//can2�ж�//
/*************/
#if CAN2_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
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
//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=CAN_CHASSIS_ALL_ID;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=CAN_CHASSIS_ALL_ID;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=CAN_ID_STD;		  // ʹ����չ��ʶ��
  TxMessage.RTR=CAN_RTR_DATA;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
		TxMessage.Data[i]=msg[i];				          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		
}
u8 CAN2_Send_Msg(u8* msg,u8 len)    //������Ϣ�����ID6
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage2;
  TxMessage2.StdId=CAN_CHASSIS_ALL_ID;	 // ��׼��ʶ��Ϊ0
  TxMessage2.ExtId=CAN_CHASSIS_ALL_ID;	 // ������չ��ʾ����29λ��
  TxMessage2.IDE=CAN_ID_STD;		  // ʹ����չ��ʶ��
  TxMessage2.RTR=CAN_RTR_DATA;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage2.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
		TxMessage2.Data[i]=msg[i];				          
  mbox= CAN_Transmit(CAN2, &TxMessage2);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		
}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;

u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
	if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)
	return -1;		//û�н��յ�����,ֱ���˳�
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
	for(i=0;i<RxMessage.DLC;i++)
		buf[i]=RxMessage.Data[i]; 
	return RxMessage.StdId-CAN_CHASSIS_ALL_ID;	
}
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage2;
	if(CAN_MessagePending(CAN2,CAN_FIFO0)==0)return -1;		//û�н��յ�����,ֱ���˳�
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage2);//��ȡ����	
	for(i=0;i<RxMessage2.DLC;i++)
		buf[i]=RxMessage2.Data[i]; 
	return RxMessage2.StdId-CAN_CHASSIS_ALL_ID;	
}
extern int start;
void M3508_ALL_ZERO_SET(void)
{
	for(int i=0;i<8;i++)
	{
		motors.ID[i].absolute_angle=0;
	}
}











