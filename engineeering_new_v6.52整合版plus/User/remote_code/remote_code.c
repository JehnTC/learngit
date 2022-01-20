/**
  ******************************************************************************
  * @file    Project/HARDWARE/remote_code.c 
  * @author  Siyuan Qiao & Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention ң�������Ƴ���
  ******************************************************************************
      ..................NEUQ_SUDO..................
*/

#include "kinematic.h" 
#include "remote_code.h"
#include "motor.h"
#include "stm32f4xx_tim.h"
#include <math.h>
#include "delay.h"
#include "flag.h"
#include "stepper_motor.h"
#include "bsp_dbus.h"
#include "speed_pid.h"
#include "steering_engine.h"

float x_speed;
float y_speed;
float z_speed;
float x_spee_d;
int belt_speed;
int card_angle;
int resuce_speed1;
int resuce_speed2;
int a;
uint16_t pulse_cnt = 0;   //��������������
uint16_t i = 0;           //ץȡ��ʯ���̲����

int steering_angle=0;//��צ��ʼ�Ƕ�(��ʼ�Ƕ�ˮƽ0�ȣ���ת����Χ-180--180)
int steering_angle_claw = 96;

//�ڲ���������
/**
  * @brief  ң�ش��룬��ң���������Ӧ�������˾��嶯���ϣ����ڶ�ʱ���ﲻ�ϵ�ˢ
  */
void Remote_Control()    //���������Ͳ��ϵ��ж�ÿ��ͨ����ֵ�������������������Ӧ����
{	
	/*****����ȫ�ֿ��ƴ���*****/
	x_speed = caculate_linear_speed(y_CH_width, y_initial_value, y_min_value, y_max_value); //ǰ������
	y_speed = caculate_linear_speed(x_CH_width, x_initial_value, x_min_value, x_max_value);
	z_speed = caculate_rotational_speed(r_CH_width*2, r_initial_value*2, r_min_value*2, r_max_value*2); //��ת
	dji_remote_assignment();

// �ж�: �󲦸�����
// ����: LEFT_LEVER==1
if(LEFT_LEVER == 1)
{
		/****CH4���ƻ�е�۷�ת****/
		
		if(i_CH_width>1200)
			Flip_Angular=2000;
		
		else if(i_CH_width<800)
			Flip_Angular=-1500;
				
		else Flip_Angular=0;
		
	/*****���ֿ��Ƽ�צ��ת*****/
	if(DJI_Motion_Yaw < 1024)
	{
		if(steering_angle < 102)
		{
			steering_angle++;
		}
		else
		{
			steering_angle = 102;
		}
		set_steering_gear_angle(steering_angle);
	}
	else if(DJI_Motion_Yaw > 1024)
	{
		if(steering_angle > 0)
		{
			steering_angle--;
		}
		else
		{
			steering_angle = 0;
		}
		set_steering_gear_angle(steering_angle);
	}
	else if(DJI_Motion_Yaw == 1024)//ֹͣ�ڵ�ǰ�Ƕ�
	{
		steering_angle = steering_angle;
		set_steering_gear_angle(steering_angle);
	}
		
	/****�Ҳ��˿��Ƽ�צ����****/
		if(RIGHT_LEVER == 2)//�Ҳ�����
		{
		if(steering_angle_claw<96)
			steering_angle_claw = steering_angle_claw + 3;
		else
			steering_angle_claw=96;
		}
		if(RIGHT_LEVER == 1)//�Ҳ�����
		{
			if(steering_angle_claw>33)
				steering_angle_claw = steering_angle_claw - 3;
			else
				steering_angle_claw=33;
		}
		else if(RIGHT_LEVER == 3)//ֹͣ�ڵ�ǰ�Ƕ�
		{
			steering_angle_claw=steering_angle_claw;
		}
			set_steering_gear_angle_repeat(steering_angle_claw);
	
}

// �ж�: �󲦸�����
// ����: LEFT_LEVER==2
if(LEFT_LEVER == 2)
{
		/*****�Ҳ��˿��ƺ���Ƹ�*****/
		if(RIGHT_LEVER==3)//�Ҳ�����		
			{   
			TIM_SetCompare1(TIM5, 0);//GPIOH, GPIO_PinSource10 D  ���Ƹ�1
			TIM_SetCompare2(TIM5, 0);//GPIOH, GPIO_PinSource11 C  ���Ƹ�1
			TIM_SetCompare3(TIM5, 0);//GPIOH, GPIO_PinSource12 B  ���Ƹ�2
			TIM_SetCompare4(TIM5, 0);//GPIOI, GPIO_PinSource0  A  ���Ƹ�2
			}	
		if(RIGHT_LEVER==1)//�Ҳ�����-->ǰ�������Ƹ�̧��
		{
			TIM_SetCompare1(TIM5, 1000);//pc6  ���Ƹ�1��
			TIM_SetCompare2(TIM5, 10);//pc7  ���Ƹ�1��
			TIM_SetCompare3(TIM5, 1000);//  ���Ƹ�2��
			TIM_SetCompare4(TIM5, 0);//  ���Ƹ�2��
		}
			if(RIGHT_LEVER==2)//�Ҳ�����-->ǰ�������Ƹ�̧��
		{
			TIM_SetCompare1(TIM5, 10);//pc6  ���Ƹ�1��
			TIM_SetCompare2(TIM5, 1000);//pc7  ���Ƹ�1��
			TIM_SetCompare3(TIM5, 0);//���Ƹ�2��
			TIM_SetCompare4(TIM5, 1000);//���Ƹ�2��
		}
			/*****���ֿ���ǰ���Ƹ�*****/
	 if(DJI_Motion_Yaw==1024)//����ԭλ
		{
			TIM_SetCompare3(TIM4, 0);//GPIOD, GPIO_PinSource14-->F  ���Ƹ�3
			TIM_SetCompare4(TIM4, 0);//GPIOD, GPIO_PinSource15-->E
			GPIO_SetBits(GPIOA,GPIO_Pin_4);//p2  ���Ƹ�4
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//p1
		}
	 if(DJI_Motion_Yaw<1024)//������-->���������Ƹ���
		{
			TIM_SetCompare3(TIM4, 1000);//GPIOD, GPIO_PinSource14-->F  ���Ƹ�3��
			TIM_SetCompare4(TIM4, 0);//GPIOD, GPIO_PinSource15-->E
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);//p2  ���Ƹ�4��
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//p1
		}
	 if(DJI_Motion_Yaw>1024)//������-->���������Ƹ���
		{
			TIM_SetCompare3(TIM4, 0);//GPIOD, GPIO_PinSource14-->F  ���Ƹ�3��
			TIM_SetCompare4(TIM4, 1000);//GPIOD, GPIO_PinSource15-->E
			GPIO_SetBits(GPIOA,GPIO_Pin_4);//p2  ���Ƹ�4��
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);//p1
		}
		/******CH4���ƾ�Ԯץ******/
		if(i_CH_width>1048)
		{	resuce_speed1=1000;
		  resuce_speed2=-200;
		}
		else if(i_CH_width<1000)	
		{	resuce_speed1=-1000 ;
		  resuce_speed2=200;
		}
     else
		 {
			 resuce_speed1=0;
		  resuce_speed2=0;
		 }
	  set_resuce_speed(resuce_speed1,resuce_speed2);
		
	}

// �ж�: �󲦸�����
// ����: LEFT_LEVER==3
if(LEFT_LEVER == 3)
{
		/****�Ҳ��˿��ƻ�е��������ջ�*****/
		if(RIGHT_LEVER==3)//�Ҳ�����		
		{   
			Kinematics.handle_L.target_angular=0;
		}	
		if(RIGHT_LEVER==1)//�Ҳ���	��
		{
			Kinematics.handle_L.target_angular=GIMBAL_vPID_max;
		}
			if(RIGHT_LEVER==2)//�Ҳ�����
		{
			Kinematics.handle_L.target_angular=-GIMBAL_vPID_max;
		}

		/*****���ֿ���ͬ����*****/

	 if(DJI_Motion_Yaw < 1024)
	 {
			 belt_speed = 7000;
			 set_belt_speed(belt_speed,-belt_speed);
	 }
		 
		else if(DJI_Motion_Yaw > 1024)
		{
			belt_speed = -7000;
			set_belt_speed(belt_speed,-belt_speed);
		}
     else 
		{
			belt_speed = 0;
			set_belt_speed(belt_speed,-belt_speed);
		}
		/*****CH4����ͬ����ͬ��ת��*****/
     if(i_CH_width > 1048)
		 {
			 belt_speed = -7000;
			 set_belt_speed(belt_speed,belt_speed);
		 }

		else if(i_CH_width < 1000)
		{
			 belt_speed =  7000;
			set_belt_speed(belt_speed,belt_speed);
		}
	}
}

// ����: caculate_speed()
// ����: ��ң����ҡ�����ӳ�䵽�����������ٶ���
// ������width��ͨ��ֵ 
//			 mid��ͨ���м�ֵ 
//			 min��ͨ�������Сֵ
//       max��ͨ��������ֵ
// �������Ӧ���ٶ�ֵ
//�ڲ��������û��������
static float caculate_linear_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//�м���������
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_linear_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_linear_speed);
  else
    speed=0;
  return speed;                
}

static float caculate_rotational_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//�м���������
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_rotational_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_rotational_speed);
  else
    speed=0;
	x_spee_d=speed;
  return speed;                
}

