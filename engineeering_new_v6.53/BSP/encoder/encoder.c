#include "encoder.h"
#include "stm32f4xx.h"

/******************************************************编码器初始化****************************************************/
int Encoder4_Timer_Overflow=0,Encoder2_Timer_Overflow=0;                                      //编码器溢出次数（每266*4溢出一次）
u16 Previous_Count_2=0,Previous_Count_4=0;          //上次TIM3->CNT的值

/**
   * @function函数：Encoder_Light_TIM2
   * @brief描述：定时器2编码器模式
   * @param输入：重装载值、预分频系数
   * @retval返回值：无
   */
void Encoder_Light_TIM2(u16 arr,u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1 ;          //GPIOA6和GPIOA7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    //复用模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //速度100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //浮空	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                  //推挽复用输出
  GPIO_Init(GPIOA,&GPIO_InitStructure);                          //初始化PA6和PA7

  GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_TIM2);           //GPIOD12复用为定时器4通道1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_TIM2);           //GPIOD13复用为定时器4通道2
	
    TIM_TimeBaseStructure.TIM_Period = arr; 	                      //(编码器线数-1)*4	四倍频原理
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                        //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           //时钟分频因子，不分频
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);                  //初始化TIM3
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  //选择输入端IC1映射到TI1上
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI1上
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
    TIM_ICInitStructure.TIM_ICFilter =10;   	                         //配置输入滤波器
    TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
    TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //选择输入端IC2映射到TI2上
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI2上
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
    TIM_ICInitStructure.TIM_ICFilter=10;                             //配置输入滤波器
    TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );//编码器配置（定时器、编码模式、上升沿、上升沿）

  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;                   //定时器4中断分组配置
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;            //响应优先级2
	NVIC_Init(&NVIC_InitStructure);                                 //配置定时器3
		
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  
	TIM_SetCounter(TIM2,0);	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                        //允许定时器3更新中断
	TIM_Cmd(TIM2,ENABLE);   
}

/**
   * @function函数：Encoder_Light_TIM9（未正常工作）
   * @brief描述：定时器9编码器模式
   * @param输入：重装载值、预分频系数
   * @retval返回值：无
   */
void Encoder_Light_TIM9(u16 arr,u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;          //GPIOE5和GPIOE6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    //复用模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //速度100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //浮空	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                  //推挽复用输出
  GPIO_Init(GPIOE, &GPIO_InitStructure);                          //初始化PA6和PA7

  GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);           //GPIOA6复用为定时器3通道1
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9);           //GPIOA7复用为定时器3通道2
	
  TIM_TimeBaseStructure.TIM_Period = arr; 	                      //(编码器线数-1)*4	四倍频原理
  TIM_TimeBaseStructure.TIM_Prescaler=psc;                        //定时器分频
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       //向上计数模式
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           //时钟分频因子，不分频
  TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);                  //初始化TIM3
	
  TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  //选择输入端IC1映射到TI1上
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_BothEdge;	      //上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI1上
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter =10;                            //配置输入滤波器
  TIM_ICInit(TIM9,&TIM_ICInitStructure);
	
  TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //选择输入端IC2映射到TI2上
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_BothEdge;	      //上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //映射到TI2上
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter=10;                             //配置输入滤波器
  TIM_ICInit(TIM9,&TIM_ICInitStructure);
	
  TIM_EncoderInterfaceConfig(TIM9,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge );//编码器配置（定时器、编码模式、上升沿、上升沿）
		
  NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn;                   //定时器9中断分组配置
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //使能
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;            //响应优先级2
  NVIC_Init(&NVIC_InitStructure);                                 //配置定时器3
  
  TIM_ClearITPendingBit(TIM9,TIM_IT_Update);  
  TIM_SetCounter(TIM9,0);	
	/* 使能捕获/比较2中断请求 */
  TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	  /* 使能高级控制定时器 */
  TIM_Cmd(TIM9,ENABLE);   
}

/**
   * @function函数：encoder_hall_Init
   * @brief描述：编码器初始化
   * @param输入：无
   * @retval返回值：无
   */
void encoder_hall_Init(void)
{
	Encoder_Light_TIM2(ENCODER_TIM_PERIOD,ENCODER_TIM_PRESCALER);
  Encoder_Light_TIM9(ENCODER_TIM_PERIOD,ENCODER_TIM_PRESCALER);

}

/******************************************************编码器中断服务函数*******************************************************/

int EncoderOverflowCountA=0,EncoderOverflowCountB=0,EncoderOverflowCountC=0,Encoder_Direction1,Encoder_Direction2;
/**
   * @function函数：TIM2_IRQHandler
   * @brief描述：定时器2（编码器）中断服务函数
   * @param输入：无
   * @retval返回值：无
   */
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)                    //溢出中断
	{   
		if((TIM2->CR1&0x0010) == 0x0010) //判断计数方向
		{
		EncoderOverflowCountC--;

		}
		else
		{
			EncoderOverflowCountC++;
			
		}
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  
}


/**
   * @function函数：TIM9_IRQHandler
   * @brief描述：定时器9（编码器）中断服务函数
   * @param输入：无
   * @retval返回值：无
   */
void TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update)==SET)                    //溢出中断
	{   
		if((TIM9->CR1&0x0010) == 0x0010) //判断计数方向
		{
		EncoderOverflowCountB--;

		Encoder_Direction2=1;
		}
		else
		{
			EncoderOverflowCountB++;
			
			Encoder_Direction2=-1;		
		}
	}
	TIM_ClearITPendingBit(TIM9,TIM_IT_Update);  

}

/*****************************************************编码器计数值读取******************************************************/

   u16 Current_Count1,Current_Count2,Current_Count3;                                              //当前TIM3->CNT的值
	u16 Enc_Timer_Overflow1,Enc_Timer_Overflow2,Enc_Timer_Overflow3;	
/**
   * @function函数：Read_Encoder
   * @brief描述：读取计数值
   * @param输入：无
   * @retval返回值：计数值
   */

float Read_Encoder1()//读取编码器得出脉冲
{
    u32 Count1;    	//一段时间内转过的脉冲数

		Enc_Timer_Overflow1=EncoderOverflowCountA;

    Current_Count1 = TIM_GetCounter(TIM4);      //获取TIM4_>cnt的值
	TIM4->CNT=0;
	EncoderOverflowCountA=0;    	//如果正转
	
	Count1 = (u32)(Current_Count1 + (Enc_Timer_Overflow1*ENCODER_TIM_PERIOD));

//	Count/=4;
	return Count1;
}
float Read_Encoder2()//读取编码器得出脉冲
{
    u32 Count2;    	//一段时间内转过的脉冲数

		Enc_Timer_Overflow2=EncoderOverflowCountB;

    Current_Count2 = TIM_GetCounter(TIM9);      //获取TIM4_>cnt的值
//	TIM9->CNT=0;
//	EncoderOverflowCountB=0;    	//如果正转
	
		if(Enc_Timer_Overflow2>5000)
	Enc_Timer_Overflow2-=65536;
else
	Enc_Timer_Overflow2+=0;

	Count2 = (u32)(Current_Count2 + (Enc_Timer_Overflow2*ENCODER_TIM_PERIOD));

//	Count/=4;
	return Count2;
}

float Read_Encoder3()//读取编码器得出脉冲
{
    u32 Count3;    	//一段时间内转过的脉冲数

		Enc_Timer_Overflow3=EncoderOverflowCountC;

    Current_Count3 = TIM_GetCounter(TIM2);      //获取TIM4_>cnt的值
	TIM2->CNT=0;
	EncoderOverflowCountB=0;    	//如果正转
	
		if(Enc_Timer_Overflow3>5000)
	Enc_Timer_Overflow3-=65536;
else
	Enc_Timer_Overflow3+=0;

	Count3 = (u32)(Current_Count2 + (Enc_Timer_Overflow2*ENCODER_TIM_PERIOD));

//	Count/=4;
	return Count3;
}

/**
   * @function函数：Read_Encoder_Derection
   * @brief描述：读取方向
   * @param输入：无
   * @retval返回值：encoder_derection
   */
int Read_Encoder_Derection()
{
	int encoder_derection=Encoder_Direction1;
		Encoder_Direction1=0;
	
	return encoder_derection;//1是上升，0是下降
}


