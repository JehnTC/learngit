#include "My_Init.h"
#include "delay.h"
#include "key.h"
#include "LED.h"
#include "timer.h"
#include "push.h"
#include "can.h"
#include "power.h"
#include "motor.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "remote_code.h"
#include "bsp_debug_usart.h"
#include "bsp_uart7.h"
#include "bsp_dbus.h"
#include "bsp_imu_usart.h"
#include "steering_engine.h"
#include "stepper_motor.h"
#include "limit_switch.h"

/*****     ���ļ�ר�������Ÿ�����ʼ������    *****/

RCC_ClocksTypeDef get_rcc_clock;		//ϵͳʱ�ӽṹ��

// ����: All_Init()
// ����: ���������в�����ʼ��
// ��������
// �������

void All_Init()
{
	Stm32_Clock_Init(360,12,2,8);				//����ʱ��,180Mhz = 12M / 25 * 350 / 2
	pid_init();                     //��ʼ��pid���������ֵ

	Push_Init();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
	delay_init(180);									//��ʱ��ʼ��
	
	power_init();										  //�����Դ��ʼ����Ĭ��Ϊ��
	power_close_all();									//�������е����Դ
	Dji_Remote_Init();									//��ң������ʼ��
	led_init();											//led��ʼ��
	key_init();											//������ʼ��
	CAN1_Config();
  CAN2_Config();
	Debug_USART_Config();           //ͨ�Ŵ��ڳ�ʼ����USART2��
	motor_init();		//���������ʼ��
	VPID_Init_All();								//�ٶ�pid������ʼ��
	APID_Init_All();								//�Ƕ�pid������ʼ��
	TIM_Steering_Engine_PWM_Init();
	limit_init_test();
	RCC_GetClocksFreq(&get_rcc_clock); 			//�鿴ϵͳʱ��(watch��)
	TIM3_Int_Init(10-1,9000-1);		  //��ʱ��ʱ��90M��9000������90M/9000=10Khz�ļ���Ƶ�ʣ�����10��Ϊ1ms  ��1khz
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	TIM_Push_PWM_Init();
}
		

