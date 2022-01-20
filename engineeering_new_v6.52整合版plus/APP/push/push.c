#include "key.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "push.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"

	//��ʱ��5ͨ��1��Ӧ���Ƹ�1��Ƶ��1kHz GPIOH, GPIO_PinSource10
	//��ʱ��5ͨ��2��Ӧ���Ƹ�2��Ƶ��1kHz GPIOH, GPIO_PinSource11
	//��ʱ��5ͨ��3��Ӧ���Ƹ�3��Ƶ��1kHz GPIOH, GPIO_PinSource12
	//��ʱ��5ͨ��4��Ӧ���Ƹ�4��Ƶ��1kHz GPIOI, GPIO_PinSource0
 void TIM_Push_PWM_Init51()  
{									
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);		//APB1tim90mhz
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);      
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5);//��ʱ��2ͨ��1
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5);//��ʱ��2ͨ��2
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource12, GPIO_AF_TIM5);//��ʱ��2ͨ��3

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* ���ø��� */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* ����������� */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* ������������ */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
		GPIO_Init(GPIOH, &GPIO_InitStructure);  
  
		
    TIM_TimeBaseStructure.TIM_Period =1000-1; /* �ۼ� TIM_Period �������һ�����»����ж� */
		//����ʱ���� 0 ������ 1000-1����Ϊ 1000 �Σ�Ϊһ����ʱ���� 
		
		// ͨ�ÿ��ƶ�ʱ��ʱ��Դ TIMxCLK = HCLK/2=180MHz
		// �趨��ʱ��Ƶ��Ϊ =TIMxCLK/180=1MHz,������Ϊ1us
    TIM_TimeBaseStructure.TIM_Prescaler =180-1; 
		
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//���Ƶ
		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
		
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ����Ϊ PWM ģʽ 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 0; //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ��50%��arr/2
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* ����ʱ������ֵС�� CCR1_Val ʱΪ�ߵ�ƽ */
  
  /* ��ʼ����ʱ�� */
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);       

	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);   

	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);       
           
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5,ENABLE);
    TIM_Cmd(TIM5, ENABLE);		//ʹ��
} 

 void TIM_Push_PWM_Init52()  
{									
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);		//APB2tim180mhz
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);   

	  GPIO_PinAFConfig(GPIOI, GPIO_PinSource0, GPIO_AF_TIM5);//��ʱ��2ͨ��4
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* ���ø��� */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* ����������� */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* ������������ */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOI, &GPIO_InitStructure);  
      
      
    TIM_TimeBaseStructure.TIM_Period =1000-1; /* �ۼ� TIM_Period �������һ�����»����ж� */
		//����ʱ���� 0 ������ 1000-1����Ϊ 1000 �Σ�Ϊһ����ʱ���� 
		
		// ͨ�ÿ��ƶ�ʱ��ʱ��Դ TIMxCLK = HCLK/2=180MHz
		// �趨��ʱ��Ƶ��Ϊ =TIMxCLK/180=1MHz,������Ϊ1us
    TIM_TimeBaseStructure.TIM_Prescaler =180-1; 
		
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//���Ƶ
		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
		
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ����Ϊ PWM ģʽ 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 0; //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ��50%��arr/2
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* ����ʱ������ֵС�� CCR1_Val ʱΪ�ߵ�ƽ */
  
  /* ��ʼ����ʱ�� */

	TIM_OC4Init(TIM5, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5,ENABLE);
    TIM_Cmd(TIM5, ENABLE);		//ʹ��
} 



 void TIM_Push_PWM_Init4()  //��ʱ��4����ΪPWM���ģʽ
{									
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);		//APB1tim90mhz
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);   

	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);//��ʱ��4ͨ��3
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);//��ʱ��4ͨ��4
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* ���ø��� */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* ����������� */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* ������������ */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);  
      
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* ���ø��� */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* ����������� */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* ������������ */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);  

    TIM_TimeBaseStructure.TIM_Period =1000-1; /* �ۼ� TIM_Period �������һ�����»����ж� */
		//����ʱ���� 0 ������ 1000-1����Ϊ 1000 �Σ�Ϊһ����ʱ���� 
		
		// ͨ�ÿ��ƶ�ʱ��ʱ��Դ TIMxCLK = HCLK/2=90MHz
		// �趨��ʱ��Ƶ��Ϊ =TIMxCLK/90=1MHz,������Ϊ1us
    TIM_TimeBaseStructure.TIM_Prescaler =90-1; 
		
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//���Ƶ
		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
		
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ����Ϊ PWM ģʽ 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 1000; //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ��50%��arr/2
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* ����ʱ������ֵС�� CCR1_Val ʱΪ�ߵ�ƽ */
  
  /* ��ʼ����ʱ�� */
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM4,ENABLE);
    TIM_Cmd(TIM4, ENABLE);		//ʹ��
} 
 void TIM_Push_PWM_Init8()  //��ʱ��8����ΪPWM���ģʽ
{									
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);		//APB2tim180mhz
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);   

	  GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_TIM8);//��ʱ��8ͨ��3
	  GPIO_PinAFConfig(GPIOI, GPIO_PinSource5, GPIO_AF_TIM8);//��ʱ��8ͨ��4
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* ���ø��� */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* ����������� */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* ������������ */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOI, &GPIO_InitStructure);  
     
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* ���ø��� */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* ����������� */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* ������������ */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOI, &GPIO_InitStructure);  

      
    TIM_TimeBaseStructure.TIM_Period =1000-1; /* �ۼ� TIM_Period �������һ�����»����ж� */
		//����ʱ���� 0 ������ 1000-1����Ϊ 1000 �Σ�Ϊһ����ʱ���� 
		
		// ͨ�ÿ��ƶ�ʱ��ʱ��Դ TIMxCLK = HCLK/2=180MHz
		// �趨��ʱ��Ƶ��Ϊ =TIMxCLK/180=1MHz,������Ϊ1us
    TIM_TimeBaseStructure.TIM_Prescaler =180-1; 
		
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//���Ƶ
		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
		
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // ����Ϊ PWM ģʽ 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 1000; //���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ��50%��arr/2
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* ����ʱ������ֵС�� CCR1_Val ʱΪ�ߵ�ƽ */
  
  /* ��ʼ����ʱ�� */
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8,ENABLE);
    TIM_Cmd(TIM8, ENABLE);		//ʹ��
} 

 void TIM_Push_PWM_Init()  
 {
	 TIM_Push_PWM_Init51();  
	 TIM_Push_PWM_Init52();  
	 TIM_Push_PWM_Init4();  
	 TIM_Push_PWM_Init8();
 }

 //����: push_init()
 //����: ����GPIO��ʼ��
// ����: ��
void Push_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOA����
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOC����

	  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
		GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.4
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);						 //Pa.4 �����
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		//LED0-->PB.5 �˿�����
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		 //�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
		GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);						 //Pa.5 �����
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
		GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.4
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);						 //Pa.4 �����
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		//LED0-->PB.5 �˿�����
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		 //�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
		GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
		GPIO_ResetBits(GPIOC,GPIO_Pin_1);						 //Pa.5 �����

}



