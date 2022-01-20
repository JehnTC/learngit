#include "encoder.h"
#include "stm32f4xx.h"

/******************************************************��������ʼ��****************************************************/
int Encoder4_Timer_Overflow=0,Encoder2_Timer_Overflow=0;                                      //���������������ÿ266*4���һ�Σ�
u16 Previous_Count_2=0,Previous_Count_4=0;          //�ϴ�TIM3->CNT��ֵ

/**
   * @function������Encoder_Light_TIM2
   * @brief��������ʱ��2������ģʽ
   * @param���룺��װ��ֵ��Ԥ��Ƶϵ��
   * @retval����ֵ����
   */
void Encoder_Light_TIM2(u16 arr,u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1 ;          //GPIOA6��GPIOA7
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    //����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //�ٶ�100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //����	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                  //���츴�����
  GPIO_Init(GPIOA,&GPIO_InitStructure);                          //��ʼ��PA6��PA7

  GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_TIM2);           //GPIOD12����Ϊ��ʱ��4ͨ��1
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_TIM2);           //GPIOD13����Ϊ��ʱ��4ͨ��2
	
    TIM_TimeBaseStructure.TIM_Period = arr; 	                      //(����������-1)*4	�ı�Ƶԭ��
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                        //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           //ʱ�ӷ�Ƶ���ӣ�����Ƶ
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);                  //��ʼ��TIM3
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  //ѡ�������IC1ӳ�䵽TI1��
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //�����ز���
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI1��
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
    TIM_ICInitStructure.TIM_ICFilter =10;   	                         //���������˲���
    TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
    TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //ѡ�������IC2ӳ�䵽TI2��
    TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      //�����ز���
    TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI2��
    TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
    TIM_ICInitStructure.TIM_ICFilter=10;                             //���������˲���
    TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );//���������ã���ʱ��������ģʽ�������ء������أ�

  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;                   //��ʱ��4�жϷ�������
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;            //��Ӧ���ȼ�2
	NVIC_Init(&NVIC_InitStructure);                                 //���ö�ʱ��3
		
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  
	TIM_SetCounter(TIM2,0);	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);                        //����ʱ��3�����ж�
	TIM_Cmd(TIM2,ENABLE);   
}

/**
   * @function������Encoder_Light_TIM9��δ����������
   * @brief��������ʱ��9������ģʽ
   * @param���룺��װ��ֵ��Ԥ��Ƶϵ��
   * @retval����ֵ����
   */
void Encoder_Light_TIM9(u16 arr,u16 psc)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);  
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;          //GPIOE5��GPIOE6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    //����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //�ٶ�100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //����	
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                  //���츴�����
  GPIO_Init(GPIOE, &GPIO_InitStructure);                          //��ʼ��PA6��PA7

  GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);           //GPIOA6����Ϊ��ʱ��3ͨ��1
  GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9);           //GPIOA7����Ϊ��ʱ��3ͨ��2
	
  TIM_TimeBaseStructure.TIM_Period = arr; 	                      //(����������-1)*4	�ı�Ƶԭ��
  TIM_TimeBaseStructure.TIM_Prescaler=psc;                        //��ʱ����Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           //ʱ�ӷ�Ƶ���ӣ�����Ƶ
  TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);                  //��ʼ��TIM3
	
  TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  //ѡ�������IC1ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_BothEdge;	      //�����ز���
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI1��
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter =10;                            //���������˲���
  TIM_ICInit(TIM9,&TIM_ICInitStructure);
	
  TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  //ѡ�������IC2ӳ�䵽TI2��
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_BothEdge;	      //�����ز���
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   //ӳ�䵽TI2��
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter=10;                             //���������˲���
  TIM_ICInit(TIM9,&TIM_ICInitStructure);
	
  TIM_EncoderInterfaceConfig(TIM9,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge );//���������ã���ʱ��������ģʽ�������ء������أ�
		
  NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn;                   //��ʱ��9�жϷ�������
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;                   //ʹ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01;      //��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0x02;            //��Ӧ���ȼ�2
  NVIC_Init(&NVIC_InitStructure);                                 //���ö�ʱ��3
  
  TIM_ClearITPendingBit(TIM9,TIM_IT_Update);  
  TIM_SetCounter(TIM9,0);	
	/* ʹ�ܲ���/�Ƚ�2�ж����� */
  TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE);
	  /* ʹ�ܸ߼����ƶ�ʱ�� */
  TIM_Cmd(TIM9,ENABLE);   
}

/**
   * @function������encoder_hall_Init
   * @brief��������������ʼ��
   * @param���룺��
   * @retval����ֵ����
   */
void encoder_hall_Init(void)
{
	Encoder_Light_TIM2(ENCODER_TIM_PERIOD,ENCODER_TIM_PRESCALER);
  Encoder_Light_TIM9(ENCODER_TIM_PERIOD,ENCODER_TIM_PRESCALER);

}

/******************************************************�������жϷ�����*******************************************************/

int EncoderOverflowCountA=0,EncoderOverflowCountB=0,EncoderOverflowCountC=0,Encoder_Direction1,Encoder_Direction2;
/**
   * @function������TIM2_IRQHandler
   * @brief��������ʱ��2�����������жϷ�����
   * @param���룺��
   * @retval����ֵ����
   */
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)                    //����ж�
	{   
		if((TIM2->CR1&0x0010) == 0x0010) //�жϼ�������
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
   * @function������TIM9_IRQHandler
   * @brief��������ʱ��9�����������жϷ�����
   * @param���룺��
   * @retval����ֵ����
   */
void TIM9_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM9,TIM_IT_Update)==SET)                    //����ж�
	{   
		if((TIM9->CR1&0x0010) == 0x0010) //�жϼ�������
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

/*****************************************************����������ֵ��ȡ******************************************************/

   u16 Current_Count1,Current_Count2,Current_Count3;                                              //��ǰTIM3->CNT��ֵ
	u16 Enc_Timer_Overflow1,Enc_Timer_Overflow2,Enc_Timer_Overflow3;	
/**
   * @function������Read_Encoder
   * @brief��������ȡ����ֵ
   * @param���룺��
   * @retval����ֵ������ֵ
   */

float Read_Encoder1()//��ȡ�������ó�����
{
    u32 Count1;    	//һ��ʱ����ת����������

		Enc_Timer_Overflow1=EncoderOverflowCountA;

    Current_Count1 = TIM_GetCounter(TIM4);      //��ȡTIM4_>cnt��ֵ
	TIM4->CNT=0;
	EncoderOverflowCountA=0;    	//�����ת
	
	Count1 = (u32)(Current_Count1 + (Enc_Timer_Overflow1*ENCODER_TIM_PERIOD));

//	Count/=4;
	return Count1;
}
float Read_Encoder2()//��ȡ�������ó�����
{
    u32 Count2;    	//һ��ʱ����ת����������

		Enc_Timer_Overflow2=EncoderOverflowCountB;

    Current_Count2 = TIM_GetCounter(TIM9);      //��ȡTIM4_>cnt��ֵ
//	TIM9->CNT=0;
//	EncoderOverflowCountB=0;    	//�����ת
	
		if(Enc_Timer_Overflow2>5000)
	Enc_Timer_Overflow2-=65536;
else
	Enc_Timer_Overflow2+=0;

	Count2 = (u32)(Current_Count2 + (Enc_Timer_Overflow2*ENCODER_TIM_PERIOD));

//	Count/=4;
	return Count2;
}

float Read_Encoder3()//��ȡ�������ó�����
{
    u32 Count3;    	//һ��ʱ����ת����������

		Enc_Timer_Overflow3=EncoderOverflowCountC;

    Current_Count3 = TIM_GetCounter(TIM2);      //��ȡTIM4_>cnt��ֵ
	TIM2->CNT=0;
	EncoderOverflowCountB=0;    	//�����ת
	
		if(Enc_Timer_Overflow3>5000)
	Enc_Timer_Overflow3-=65536;
else
	Enc_Timer_Overflow3+=0;

	Count3 = (u32)(Current_Count2 + (Enc_Timer_Overflow2*ENCODER_TIM_PERIOD));

//	Count/=4;
	return Count3;
}

/**
   * @function������Read_Encoder_Derection
   * @brief��������ȡ����
   * @param���룺��
   * @retval����ֵ��encoder_derection
   */
int Read_Encoder_Derection()
{
	int encoder_derection=Encoder_Direction1;
		Encoder_Direction1=0;
	
	return encoder_derection;//1��������0���½�
}


