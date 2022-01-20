#ifndef _PUSH_H
#define _PUSH_H

#include "sys.h"

void Push_Init(void);
void TIM_Push_PWM_Init2(void);  
void TIM_Push_PWM_Init(void);  //定时器2通道1对应舵机1，频率50Hz PA0

#endif
