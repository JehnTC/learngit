/**
  ******************************************************************************
  * @file    Project/APP/kinematic.c 
  * @author  Siyuan Qiao&Junyu Luo 
  * @version V1.0.0
  * @date    2.2020
  * @brief   底盘正逆运动学演算
  *          线速度单位： cm/s
  *          角速度单位： rad/s
	*          转速单位：   rpm
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#include "stm32f4xx_gpio.h"
#include "remote_code.h"
#include "kinematic.h"
#include "motor.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "algorithm.h"
#include "flag.h"
#include "remote_code.h"
#include "delay.h"
#include "push.h"
#include "key.h"

Kinematics_t Kinematics;
uint16_t stop_flag_1=0;
float resucecard_angle,synbelt_angle;
/**
  * @brief  逆运动学公式,把想要得到的底盘速度转换为轮子的线速度
  */
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.wheel1.target_speed.linear_vel = linear_x + linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel2.target_speed.linear_vel = linear_x - linear_y - angular_z*(half_width+half_length);
	Kinematics.wheel3.target_speed.linear_vel = linear_x - linear_y + angular_z*(half_width+half_length);
	Kinematics.wheel4.target_speed.linear_vel = linear_x +  linear_y - angular_z*(half_width+half_length);
	//线速度 cm/s  转转度  RPM 
	
	Kinematics.wheel1.target_speed.rpm = Kinematics.wheel1.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel2.target_speed.rpm = Kinematics.wheel2.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel3.target_speed.rpm = Kinematics.wheel3.target_speed.linear_vel * VEL2RPM;
	Kinematics.wheel4.target_speed.rpm = Kinematics.wheel4.target_speed.linear_vel * VEL2RPM;
	
	motor1.target_speed =   (int)(Kinematics.wheel1.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor2.target_speed = - (int)(Kinematics.wheel2.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor3.target_speed =   (int)(Kinematics.wheel3.target_speed.rpm * M3508_REDUCTION_RATIO);
	motor4.target_speed = - (int)(Kinematics.wheel4.target_speed.rpm * M3508_REDUCTION_RATIO);
}

/**
  * @brief  正运动学公式,通过轮胎的实际转速计算底盘几何中心的三轴速度
  */
void Get_Base_Velocities(void)
{
	//根据电机转速测算轮子转速
	Kinematics.wheel1.actual_speed.rpm = - motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel2.actual_speed.rpm =   motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel3.actual_speed.rpm =   motor3.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.wheel4.actual_speed.rpm = - motor4.actual_speed / M3508_REDUCTION_RATIO;
	//轮子转速转换为轮子线速度
	Kinematics.wheel1.actual_speed.linear_vel = Kinematics.wheel1.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel2.actual_speed.linear_vel = Kinematics.wheel2.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel3.actual_speed.linear_vel = Kinematics.wheel3.actual_speed.rpm * RPM2VEL;
	Kinematics.wheel4.actual_speed.linear_vel = Kinematics.wheel4.actual_speed.rpm * RPM2VEL;
	//轮子线速度转换为底盘中心三轴的速度
	Kinematics.actual_velocities.angular_z = ( Kinematics.wheel1.actual_speed.linear_vel - Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f*(half_width + half_length));
	Kinematics.actual_velocities.linear_x  = (-Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				- Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
	Kinematics.actual_velocities.linear_y  = ( Kinematics.wheel1.actual_speed.linear_vel + Kinematics.wheel2.actual_speed.linear_vel\
				+ Kinematics.wheel3.actual_speed.linear_vel + Kinematics.wheel4.actual_speed.linear_vel)/(4.0f);
}

/**
  * @brief  底盘速度控制，将宏观速度指令解算，赋值到具体电机
  */


void chassic_speed_control(float speed_x, float speed_y, float speed_r)
{
		int max;
		stop_flag_1 = 0;
		//速度换算
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
 
		max=find_max(motor1.target_speed,motor2.target_speed,motor3.target_speed,motor4.target_speed);
		if(max>max_motor_speed)
		{
			motor1.target_speed=(int)(motor1.target_speed*max_motor_speed*1.0/max);
			motor2.target_speed=(int)(motor2.target_speed*max_motor_speed*1.0/max);
			motor3.target_speed=(int)(motor3.target_speed*max_motor_speed*1.0/max);
			motor4.target_speed=(int)(motor4.target_speed*max_motor_speed*1.0/max);
		}
			//改变速度pid目标速度
			set_chassis_speed(motor1.target_speed, motor2.target_speed, motor3.target_speed, motor4.target_speed);
}	

/**
  * @brief  抓手速度控制
  */	
void handle_speed_control(float motor5_speed)
{
//	int motor5_round_cnt_flag=0;
	set_handle_speed(motor5_speed);
	
//	if(motor5.round_cnt>40&&motor5_speed>=0&&motor5_round_cnt_flag!=1)	//到达最大圈数停止旋转
//		{
//			motor5_round_cnt_flag = 1;			//停止   此标志为了避免多次进入
//			set_handle_speed(0);			//停下来
//		}
//	else if(motor5.round_cnt<2&&motor5_speed<=0&&motor5_round_cnt_flag!=2)
//		{
//			motor5_round_cnt_flag = 2;
//			set_handle_speed(0);			//停下来
//		}
//	else
//		set_handle_speed(motor5_speed);
//		motor5_round_cnt_flag=0;
		
}


/**
  * @brief  翻转装置控制
  */	
void flip_speed_control(float motor6_speed,float motor7_speed)    //
{	
	set_flip_speed(motor6_speed,motor7_speed);
}

void flip_angle_control(float flip_angle)
{
		flip_angle=flip_angle/360*8191*19;
	
		set_flip_angle(flip_angle);

}


void card_speed_control(float card_angle)    //
{
	resucecard_angle=card_angle/360.0f*8191*36;
  set_card_angle(resucecard_angle);

}

void belt_angle_control(float belt_angle)
{
  synbelt_angle=belt_angle/360.0f*8191*36;
	set_belt_angle(synbelt_angle);

}

int motor5_flag=0;
int angle_judge_flag1=0;
int angle_judge_flag2=0;
void handle_angle_control(float yaw_angle,float pitch_angle)
{
	
	if(motor5_flag==0&&yaw_angle==0)
		{
			motor5_flag=1;
			
			stop_handel_motor();
		}
	else
		{
		motor5_flag=0;
			
		yaw_angle = yaw_angle/360*8191*19;
		if(angle_judge_flag1==0)
		{
			motor5.apid.trigger_first_total_angle_storage=motor5.total_angle;
			angle_judge_flag1++;
		}
		pitch_angle = pitch_angle/360*8191*19;
		if(angle_judge_flag2==0)
		{
			motor6.apid.trigger_first_total_angle_storage=motor6.total_angle;
			angle_judge_flag2++;
		}
		set_handle_angle(yaw_angle,pitch_angle);
	}
}



