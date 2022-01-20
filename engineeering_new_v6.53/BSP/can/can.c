#include "can.h"

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+tbs2+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);
//则波特率为:42M/((6+7+1)*3)=1Mbps
//返回值:0,初始化OK;
//    其他,初始化失败; 


/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN1_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	

  /* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(CAN1_GPIO_CLK , ENABLE);


	  /* Configure CAN TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure CAN RX  pins */
  GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);

	/* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
  GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT); 
}

static void CAN2_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	

  /* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(CAN2_GPIO_CLK,ENABLE);

	  /* Configure CAN TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);
	
	/* Configure CAN RX  pins */
  GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
  GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT); 

}
/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN1_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*中断设置*/
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX_IRQ2;	   //CAN RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void CAN2_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*中断设置*/
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX_IRQ;	   //CAN RX1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			   //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN1_Mode_Config(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN通信参数设置**********************************/
	/* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);

	/*CAN寄存器初始化*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元
	 
	/* ss=1 bs1=4 bs2=2 位时间宽度为(1+4+2) 波特率即为时钟周期tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;		   //BTR-TS1 时间段1 占用了10个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;		   //BTR-TS1 时间段2 占用了3个时间单元	
	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 45 MHz) */
	CAN_InitStructure.CAN_Prescaler =5;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 45/(1+5+3)/5=1 Mbps
	CAN_Init(CAN1, &CAN_InitStructure);
}


static void CAN2_Mode_Config(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN通信参数设置**********************************/
	/* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);

	/*CAN寄存器初始化*/
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元
	 
	/* ss=1 bs1=4 bs2=2 位时间宽度为(1+4+2) 波特率即为时钟周期tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;		   //BTR-TS1 时间段1 占用了10个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;		   //BTR-TS1 时间段2 占用了3个时间单元	
	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 45 MHz) */
	CAN_InitStructure.CAN_Prescaler =5;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 45/(1+5+3)/5=1 Mbps
	CAN_Init(CAN2, &CAN_InitStructure);
}
/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN1_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//筛选器组0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;//((((u32)0x200<<3 )|CAN_ID_STD|CAN_RTR_DATA) &0xFFFF0000)>>16;		//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;//(((u32)0x200<<3 )|CAN_ID_STD|CAN_RTR_DATA) &0xFFFF; //要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000; //((((u32)0x3F0)<<3 | 0x07)&0xFFFF0000)>>16;			//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;//((((u32)0x3F0)<<3 | 0x07))&0xFFFF;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

static void CAN2_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=14;						//筛选器组14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.CAN_FilterIdHigh= 0x0000;//((((u32)0x200<<3 )|CAN_ID_STD|CAN_RTR_DATA) &0xFFFF0000)>>16;		//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;//(((u32)0x200<<3 )|CAN_ID_STD|CAN_RTR_DATA) &0xFFFF; //要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000; //((((u32)0x3F0)<<3 | 0x07)&0xFFFF0000)>>16;			//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;//((((u32)0x3F0)<<3 | 0x07))&0xFFFF;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}

/*
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN1_Config(void)
{
  CAN1_GPIO_Config();
  CAN1_Mode_Config();
  CAN1_Filter_Config(); 
  CAN1_NVIC_Config();  
}
void CAN2_Config(void)
{
  CAN2_GPIO_Config();
  CAN2_Mode_Config();
  CAN2_Filter_Config();   
	CAN2_NVIC_Config();
}

/**
  * @brief  初始化 Rx Message数据结构体
  * @param  RxMessage: 指向要初始化的数据结构体
  * @retval None
  */
void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t ubCounter = 0;

	/*把接收结构体清零*/
  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    RxMessage->Data[ubCounter] = 0x00;
  }
}


/*****CAN1******/
/**
   * @function函数：CAN1_Send_CHASSIS_Msg
   * @brief描述：CAN1发送底盘电机邮件
   * @param输入：u8* msg
   * @retval返回值：0
   */
u8 CAN1_Send_CHASSIS_Msg(u8* msg)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 // 标准标识符为0
  TxMessage.ExtId=0;	 // 设置扩展标示符（29位） 
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
  for(i=0;i<8;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息
  mbox= CAN_Transmit(CAN1, &TxMessage);
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;

}

/**
   * @function函数：CAN1_Send_handel_Msg
   * @brief描述：CAN1发送机械臂电机邮件
   * @param输入：u8* msg
   * @retval返回值：0
   */
u8 CAN1_Send_handle_Msg(u8* msg)
{
u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x1FF;	 
  TxMessage.ExtId=0;	 // 设置扩展标示符（29位） 
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
  for(i=0;i<8;i++)
  TxMessage.Data[i]=msg[i];			// 第一帧信息
	
		for (int i=0;i<8;i++)
		{
			 int can_temp=TxMessage.Data[i];
			Usart_SendByte(USART2,can_temp);
		}

	
  mbox= CAN_Transmit(CAN1, &TxMessage);
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;
}

/**
   * @function函数：CAN1_Send_flip_Msg
   * @brief描述：CAN1发送机械臂翻转结构电机邮件
   * @param输入：u8* msg
   * @retval返回值：0
   */
u8 CAN1_Send_flip_Msg(u8* msg)//********************新增
{
u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x1FF;	 
  TxMessage.ExtId=0;	 // 设置扩展标示符（29位） 
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
  for(i=0;i<8;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息
  mbox= CAN_Transmit(CAN1, &TxMessage);
	
	
	for (int i=0;i<8;i++)
		{
			 int can_temp=TxMessage.Data[i];
			Usart_SendByte(USART2,can_temp);
		}
	
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束	
  if(i>=0XFFF)return 1;
  return 0;
}

/**********CAN2**********/
/**
   * @function函数：CAN2_Send_resucecard_Msg
   * @brief描述：CAN2发送救援卡电机邮件
   * @param输入：u8* msg
   * @retval返回值：0
   */
u8 CAN2_Send_resucecard_Msg(u8* msg)
{
u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x1FF;	 
  TxMessage.ExtId=0;	 // 设置扩展标示符（29位） 
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
  for(i=0;i<8;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息
  mbox= CAN_Transmit(CAN2, &TxMessage);
	
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束	
  if(i>=0XFFF)return 1;
  return 0;
}
/**
   * @function函数：CAN2_Send_belt_Msg
   * @brief描述：CAN2发送同步带电机邮件
   * @param输入：u8* msg
   * @retval返回值：0
   */
u8 CAN2_Send_belt_Msg(u8* msg)
{
u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 
  TxMessage.ExtId=0;	 // 设置扩展标示符（29位） 
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
  for(i=0;i<8;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息
  mbox= CAN_Transmit(CAN2, &TxMessage);
	
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束	
  if(i>=0XFFF)return 1;
  return 0;
}
/**
   * @function函数：CAN2_Send_resuce_Msg
   * @brief描述：CAN2发送救援抓电机邮件
   * @param输入：u8* msg
   * @retval返回值：0
   */
u8 CAN2_Send_resuce_Msg(u8* msg)
{
u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x200;	 
  TxMessage.ExtId=0;	 // 设置扩展标示符（29位） 
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
  for(i=0;i<8;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息
  mbox= CAN_Transmit(CAN2, &TxMessage);
	
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束	
  if(i>=0XFFF)return 1;
  return 0;
}



