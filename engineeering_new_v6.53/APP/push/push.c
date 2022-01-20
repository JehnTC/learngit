#include "key.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "push.h"
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"

	//定时器5通道1对应电推杆1，频率1kHz GPIOH, GPIO_PinSource10
	//定时器5通道2对应电推杆2，频率1kHz GPIOH, GPIO_PinSource11
	//定时器5通道3对应电推杆3，频率1kHz GPIOH, GPIO_PinSource12
	//定时器5通道4对应电推杆4，频率1kHz GPIOI, GPIO_PinSource0
 void TIM_Push_PWM_Init51()  
{									
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);		//APB1tim90mhz
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);      
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5);//定时器2通道1
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5);//定时器2通道2
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource12, GPIO_AF_TIM5);//定时器2通道3

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* 设置复用 */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* 设置输出类型 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* 设置引脚速率 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
		GPIO_Init(GPIOH, &GPIO_InitStructure);  
  
		
    TIM_TimeBaseStructure.TIM_Period =1000-1; /* 累计 TIM_Period 个后产生一个更新或者中断 */
		//当定时器从 0 计数到 1000-1，即为 1000 次，为一个定时周期 
		
		// 通用控制定时器时钟源 TIMxCLK = HCLK/2=180MHz
		// 设定定时器频率为 =TIMxCLK/180=1MHz,即周期为1us
    TIM_TimeBaseStructure.TIM_Prescaler =180-1; 
		
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//零分频
		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
		
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 配置为 PWM 模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 0; //设置比较值,此值用来确定占空比，默认50%是arr/2
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* 当定时器计数值小于 CCR1_Val 时为高电平 */
  
  /* 初始化定时器 */
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);       

	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);   

	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);       
           
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5,ENABLE);
    TIM_Cmd(TIM5, ENABLE);		//使能
} 

 void TIM_Push_PWM_Init52()  
{									
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);		//APB2tim180mhz
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);   

	  GPIO_PinAFConfig(GPIOI, GPIO_PinSource0, GPIO_AF_TIM5);//定时器2通道4
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* 设置复用 */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* 设置输出类型 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* 设置引脚速率 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOI, &GPIO_InitStructure);  
      
      
    TIM_TimeBaseStructure.TIM_Period =1000-1; /* 累计 TIM_Period 个后产生一个更新或者中断 */
		//当定时器从 0 计数到 1000-1，即为 1000 次，为一个定时周期 
		
		// 通用控制定时器时钟源 TIMxCLK = HCLK/2=180MHz
		// 设定定时器频率为 =TIMxCLK/180=1MHz,即周期为1us
    TIM_TimeBaseStructure.TIM_Prescaler =180-1; 
		
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//零分频
		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
		
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 配置为 PWM 模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 0; //设置比较值,此值用来确定占空比，默认50%是arr/2
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* 当定时器计数值小于 CCR1_Val 时为高电平 */
  
  /* 初始化定时器 */

	TIM_OC4Init(TIM5, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5,ENABLE);
    TIM_Cmd(TIM5, ENABLE);		//使能
} 



 void TIM_Push_PWM_Init4()  //定时器4复用为PWM输出模式
{									
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);		//APB1tim90mhz
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);   

	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);//定时器4通道3
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);//定时器4通道4
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* 设置复用 */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* 设置输出类型 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* 设置引脚速率 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);  
      
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* 设置复用 */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* 设置输出类型 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* 设置引脚速率 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);  

    TIM_TimeBaseStructure.TIM_Period =1000-1; /* 累计 TIM_Period 个后产生一个更新或者中断 */
		//当定时器从 0 计数到 1000-1，即为 1000 次，为一个定时周期 
		
		// 通用控制定时器时钟源 TIMxCLK = HCLK/2=90MHz
		// 设定定时器频率为 =TIMxCLK/90=1MHz,即周期为1us
    TIM_TimeBaseStructure.TIM_Prescaler =90-1; 
		
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//零分频
		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
		
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 配置为 PWM 模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 1000; //设置比较值,此值用来确定占空比，默认50%是arr/2
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* 当定时器计数值小于 CCR1_Val 时为高电平 */
  
  /* 初始化定时器 */
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM4,ENABLE);
    TIM_Cmd(TIM4, ENABLE);		//使能
} 
 void TIM_Push_PWM_Init8()  //定时器8复用为PWM输出模式
{									
	GPIO_InitTypeDef GPIO_InitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef  TIM_OCInitStructure;                         
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);		//APB2tim180mhz
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);   

	  GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_TIM8);//定时器8通道3
	  GPIO_PinAFConfig(GPIOI, GPIO_PinSource5, GPIO_AF_TIM8);//定时器8通道4
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* 设置复用 */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* 设置输出类型 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* 设置引脚速率 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOI, &GPIO_InitStructure);  
     
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;/* 设置复用 */
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;/* 设置输出类型 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;/* 设置引脚速率 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOI, &GPIO_InitStructure);  

      
    TIM_TimeBaseStructure.TIM_Period =1000-1; /* 累计 TIM_Period 个后产生一个更新或者中断 */
		//当定时器从 0 计数到 1000-1，即为 1000 次，为一个定时周期 
		
		// 通用控制定时器时钟源 TIMxCLK = HCLK/2=180MHz
		// 设定定时器频率为 =TIMxCLK/180=1MHz,即周期为1us
    TIM_TimeBaseStructure.TIM_Prescaler =180-1; 
		
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 	//零分频
		
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
      
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
		
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 配置为 PWM 模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_Pulse = 1000; //设置比较值,此值用来确定占空比，默认50%是arr/2
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   /* 当定时器计数值小于 CCR1_Val 时为高电平 */
  
  /* 初始化定时器 */
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8,ENABLE);
    TIM_Cmd(TIM8, ENABLE);		//使能
} 

 void TIM_Push_PWM_Init()  
 {
	 TIM_Push_PWM_Init51();  
	 TIM_Push_PWM_Init52();  
	 TIM_Push_PWM_Init4();  
	 TIM_Push_PWM_Init8();
 }

 //函数: push_init()
 //描述: 按键GPIO初始化
// 参数: 无
void Push_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA外设
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC外设

	  
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
		GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.4
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);						 //Pa.4 输出低
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		//LED0-->PB.5 端口配置
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		 //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
		GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);						 //Pa.5 输出低
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
		GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.4
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);						 //Pa.4 输出低
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		//LED0-->PB.5 端口配置
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		 //推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
		GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
		GPIO_ResetBits(GPIOC,GPIO_Pin_1);						 //Pa.5 输出低

}



