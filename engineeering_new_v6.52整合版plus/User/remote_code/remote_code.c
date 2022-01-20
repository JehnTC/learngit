/**
  ******************************************************************************
  * @file    Project/HARDWARE/remote_code.c 
  * @author  Siyuan Qiao & Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention 遥控器控制程序
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
uint16_t pulse_cnt = 0;   //步进电机脉冲计数
uint16_t i = 0;           //抓取矿石流程步骤层

int steering_angle=0;//夹爪初始角度(初始角度水平0度，可转动范围-180--180)
int steering_angle_claw = 96;

//内部函数声明
/**
  * @brief  遥控代码，将遥控器输出对应到机器人具体动作上，放在定时器里不断地刷
  */
void Remote_Control()    //这个函数里就不断地判断每个通道的值，如果满足条件就做相应动作
{	
	/*****底盘全局控制代码*****/
	x_speed = caculate_linear_speed(y_CH_width, y_initial_value, y_min_value, y_max_value); //前后左右
	y_speed = caculate_linear_speed(x_CH_width, x_initial_value, x_min_value, x_max_value);
	z_speed = caculate_rotational_speed(r_CH_width*2, r_initial_value*2, r_min_value*2, r_max_value*2); //旋转
	dji_remote_assignment();

// 判断: 左拨杆置上
// 描述: LEFT_LEVER==1
if(LEFT_LEVER == 1)
{
		/****CH4控制机械臂翻转****/
		
		if(i_CH_width>1200)
			Flip_Angular=2000;
		
		else if(i_CH_width<800)
			Flip_Angular=-1500;
				
		else Flip_Angular=0;
		
	/*****滚轮控制夹爪旋转*****/
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
	else if(DJI_Motion_Yaw == 1024)//停止在当前角度
	{
		steering_angle = steering_angle;
		set_steering_gear_angle(steering_angle);
	}
		
	/****右拨杆控制夹爪动作****/
		if(RIGHT_LEVER == 2)//右拨杆下
		{
		if(steering_angle_claw<96)
			steering_angle_claw = steering_angle_claw + 3;
		else
			steering_angle_claw=96;
		}
		if(RIGHT_LEVER == 1)//右拨杆下
		{
			if(steering_angle_claw>33)
				steering_angle_claw = steering_angle_claw - 3;
			else
				steering_angle_claw=33;
		}
		else if(RIGHT_LEVER == 3)//停止在当前角度
		{
			steering_angle_claw=steering_angle_claw;
		}
			set_steering_gear_angle_repeat(steering_angle_claw);
	
}

// 判断: 左拨杆置下
// 描述: LEFT_LEVER==2
if(LEFT_LEVER == 2)
{
		/*****右拨杆控制后电推杆*****/
		if(RIGHT_LEVER==3)//右拨杆中		
			{   
			TIM_SetCompare1(TIM5, 0);//GPIOH, GPIO_PinSource10 D  电推杆1
			TIM_SetCompare2(TIM5, 0);//GPIOH, GPIO_PinSource11 C  电推杆1
			TIM_SetCompare3(TIM5, 0);//GPIOH, GPIO_PinSource12 B  电推杆2
			TIM_SetCompare4(TIM5, 0);//GPIOI, GPIO_PinSource0  A  电推杆2
			}	
		if(RIGHT_LEVER==1)//右拨杆上-->前两个电推杆抬升
		{
			TIM_SetCompare1(TIM5, 1000);//pc6  电推杆1上
			TIM_SetCompare2(TIM5, 10);//pc7  电推杆1上
			TIM_SetCompare3(TIM5, 1000);//  电推杆2上
			TIM_SetCompare4(TIM5, 0);//  电推杆2上
		}
			if(RIGHT_LEVER==2)//右拨杆下-->前两个电推杆抬升
		{
			TIM_SetCompare1(TIM5, 10);//pc6  电推杆1下
			TIM_SetCompare2(TIM5, 1000);//pc7  电推杆1下
			TIM_SetCompare3(TIM5, 0);//电推杆2下
			TIM_SetCompare4(TIM5, 1000);//电推杆2下
		}
			/*****滚轮控制前电推杆*****/
	 if(DJI_Motion_Yaw==1024)//滚轮原位
		{
			TIM_SetCompare3(TIM4, 0);//GPIOD, GPIO_PinSource14-->F  电推杆3
			TIM_SetCompare4(TIM4, 0);//GPIOD, GPIO_PinSource15-->E
			GPIO_SetBits(GPIOA,GPIO_Pin_4);//p2  电推杆4
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//p1
		}
	 if(DJI_Motion_Yaw<1024)//滚轮上-->后两个电推杆上
		{
			TIM_SetCompare3(TIM4, 1000);//GPIOD, GPIO_PinSource14-->F  电推杆3上
			TIM_SetCompare4(TIM4, 0);//GPIOD, GPIO_PinSource15-->E
			GPIO_ResetBits(GPIOA,GPIO_Pin_4);//p2  电推杆4上
			GPIO_SetBits(GPIOA,GPIO_Pin_5);//p1
		}
	 if(DJI_Motion_Yaw>1024)//滚轮下-->后两个电推杆下
		{
			TIM_SetCompare3(TIM4, 0);//GPIOD, GPIO_PinSource14-->F  电推杆3下
			TIM_SetCompare4(TIM4, 1000);//GPIOD, GPIO_PinSource15-->E
			GPIO_SetBits(GPIOA,GPIO_Pin_4);//p2  电推杆4下
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);//p1
		}
		/******CH4控制救援抓******/
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

// 判断: 左拨杆置中
// 描述: LEFT_LEVER==3
if(LEFT_LEVER == 3)
{
		/****右拨杆控制机械臂伸出与收回*****/
		if(RIGHT_LEVER==3)//右拨杆中		
		{   
			Kinematics.handle_L.target_angular=0;
		}	
		if(RIGHT_LEVER==1)//右拨杆	上
		{
			Kinematics.handle_L.target_angular=GIMBAL_vPID_max;
		}
			if(RIGHT_LEVER==2)//右拨杆下
		{
			Kinematics.handle_L.target_angular=-GIMBAL_vPID_max;
		}

		/*****滚轮控制同步带*****/

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
		/*****CH4控制同步带同向转动*****/
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

// 函数: caculate_speed()
// 描述: 将遥控器摇杆输出映射到机器人三轴速度上
// 参数：width：通道值 
//			 mid：通道中间值 
//			 min：通道输出最小值
//       max：通道输出最大值
// 输出：对应的速度值
//内部函数，用户无需调用
static float caculate_linear_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//中间消除波动
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
  if(width>=(mid+2))		//中间消除波动
    speed=(1.0*(width-(mid+2))/(max-(mid+2))*max_base_rotational_speed);
  else if(width<=(mid-2))
    speed=(1.0*(width-(mid-2))/((mid-2)-min)*max_base_rotational_speed);
  else
    speed=0;
	x_spee_d=speed;
  return speed;                
}

