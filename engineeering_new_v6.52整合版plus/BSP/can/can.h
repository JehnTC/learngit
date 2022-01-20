#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	     
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "bsp_debug_usart.h"

//#define CAN1                       	
#define CAN1_CLK                    	RCC_APB1Periph_CAN1 |RCC_APB1Periph_CAN2
#define CAN1_RX_IRQ2									CAN1_RX0_IRQn
#define CAN1_RX_IRQHandler2						CAN1_RX0_IRQHandler	//重复定义了

#define CAN1_GPIO_CLK 								RCC_AHB1Periph_GPIOD
#define CAN1_RX_PIN                 	GPIO_Pin_0
#define CAN1_TX_PIN                 	GPIO_Pin_1
#define CAN1_RX_GPIO_PORT          		GPIOD
#define CAN1_TX_GPIO_PORT          		GPIOD
#define CAN1_RX_GPIO_CLK           		RCC_AHB1Periph_GPIOD
#define CAN1_TX_GPIO_CLK           		RCC_AHB1Periph_GPIOD
#define CAN1_AF_PORT                	GPIO_AF_CAN1
#define CAN1_RX_SOURCE              	GPIO_PinSource0
#define CAN1_TX_SOURCE              	GPIO_PinSource1 

//#define CAN1  
#define CAN2_CLK                    	RCC_APB1Periph_CAN1 //|RCC_APB1Periph_CAN2
#define CAN2_RX_IRQ										CAN2_RX0_IRQn
#define CAN2_RX_IRQHandler						CAN2_RX0_IRQHandler

#define CAN2_GPIO_CLK 								RCC_AHB1Periph_GPIOB
#define CAN2_RX_PIN                 	GPIO_Pin_12
#define CAN2_TX_PIN                 	GPIO_Pin_13
#define CAN2_RX_GPIO_PORT          		GPIOB
#define CAN2_TX_GPIO_PORT          		GPIOB
#define CAN2_RX_GPIO_CLK           		RCC_AHB1Periph_GPIOB
#define CAN2_TX_GPIO_CLK           		RCC_AHB1Periph_GPIOB
#define CAN2_AF_PORT                	GPIO_AF_CAN2
#define CAN2_RX_SOURCE              	GPIO_PinSource12
#define CAN2_TX_SOURCE              	GPIO_PinSource13 


static void CAN1_GPIO_Config(void);
static void CAN1_NVIC_Config(void);
static void CAN1_Mode_Config(void);
static void CAN1_Filter_Config(void);
void CAN1_Config(void);

static void CAN2_GPIO_Config(void);
static void CAN2_NVIC_Config(void);
static void CAN2_Mode_Config(void);
static void CAN2_Filter_Config(void);
void CAN2_Config(void);					 
void Init_RxMes(CanRxMsg *RxMessage);
u8 CAN1_Send_flip_Msg(u8* msg);

//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
		
#define CAN2_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    


u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化

u8 CAN1_Send_CHASSIS_Msg(u8* msg);						//发送数据
u8 CAN1_Send_handle_Msg(u8* msg);
u8 CAN1_Send_GIMBAL_Msg(u8* msg);
u8 CAN2_Send_flip_Msg(u8* msg);//********************新增
u8 CAN2_Send_resucecard_Msg(u8* msg);//********************新增
u8 CAN2_Send_belt_Msg(u8* msg);//********************新增
u8 CAN2_Send_resuce_Msg(u8* msg);//********************新增

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
u8 CAN2_Receive_Msg(u8 *buf);							//接收数据

#endif

















