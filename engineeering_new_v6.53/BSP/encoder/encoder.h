#ifndef __ENCOUDER_H__
#define __ENCOUDER_H__
#include "sys.h"	     
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "bsp_debug_usart.h"
#include "stm32f4xx_tim.h"
#include <stdio.h>


#define ENCODER_TIM_PERIOD 					160
#define ENCODER_TIM_PRESCALER 			0


void encoder_hall_Init(void);
float Read_Encoder1(void);
float Read_Encoder2(void);
float Read_Encoder3(void);
int Read_Encoder_Derection(void);

#endif
