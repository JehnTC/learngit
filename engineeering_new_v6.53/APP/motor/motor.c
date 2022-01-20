/**
  ******************************************************************************
  * @file    Project/HARDWARE/motor.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   
  ******************************************************************************
  * @attention
  ******************************************************************************
	// * 范围  ：3508  数据范围                       金属头转速为0.22rps,即13.3rpm
//						给定电流 -16384 ~ 0 ~ +16384
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=450	RPM
//						实际转矩电流
//						电机温度	
//			   ：GM6020数据范围 
//						给定电压 -30000 ~ 0 ~ +30000
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=350	RPM
//						实际转矩电流
//						电机温度
//			   ：P2006 数据范围 
//						给定电流 -10000 ~ 0 ~ +10000
//						转子角度	0~8191 等价	0-360°
//						转子转速	max=450	RPM
//						实际输出转矩
      ..................NEUQ_SUDO..................

*/  
#include "motor.h"
#include "can.h"
#include "delay.h"
#include "angle_pid.h"
#include "kinematic.h"
#include "bsp_debug_usart.h"


MOTOR_t motor1,motor2,motor3,motor4,motor5,motor6,motor7,stepper_motor_left,stepper_motor_right,limit_switch, motor_can2_1,motor10,motor11,motor12,motor13;
LOOPBACK loopback;

int max_motor_speed=MAX_MOTOR_SPEED;		//电机最大线速度
float max_base_linear_speed=MAX_BASE_LINEAR_SPEED;  //底盘中心最大线速度 
float max_base_rotational_speed=MAX_BASE_ROTATIONAL_SPEED;  //地盘中心最大角速度
int callback_flag=1;

/**
  * @breif 电机can回调赋值函数
	*/
void record_motor_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = speed;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
		motor->start_angle = angle;
		motor->start_angle_flag++;	//只在启动时记录一次初始角度
	}
	
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->round_cnt --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->round_cnt ++;
	motor->total_angle = motor->round_cnt * 8192 + motor->actual_angle;	// - motor->start_angle;
}
/**********************三值滤波****************************/
/*  @function name:record_gimbal_callback()
    @author:junyu luo
    @date:2021.1.5
    @instruction:采样三次can总线的数据，去掉三个中的最大和最小值，剩下 入数据，避免电机数据错误引起的控制量变化
*/     
void record_gimbal_callback(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	static int temp_speed1,temp_speed2,temp_speed3,temp_angle1,temp_angle2,temp_angle3;
	static int maxangle,minangle,maxspeed,minspeed;
	motor->last_angle = motor->actual_angle;
	motor->actual_current = current;
	//motor->actual_speed = speed;
	//motor->actual_current = current;

	switch(callback_flag)
	{
		case(1):
		{
		temp_angle1=angle;
		temp_speed1=speed;
		callback_flag=2;
	  }
	break;
		case(2):
	{
		temp_angle2=angle;
		temp_speed2=speed;
		callback_flag=3;
	}
	break;
		case(3):
	{
		temp_angle3=angle;
		temp_speed3=speed;
		maxspeed=(temp_speed1>temp_speed2?temp_speed1:temp_speed2);
		maxspeed=(maxspeed>temp_speed3?maxspeed:temp_speed3);
		minspeed=(temp_speed1<temp_speed2?temp_speed1:temp_speed2);
		minspeed=(maxspeed<temp_speed3?minspeed:temp_speed3);

		maxangle=(temp_angle1>temp_angle2?temp_angle1:temp_angle2);
		maxangle=(maxangle>temp_angle3?maxspeed:temp_angle3);
		minangle=(temp_angle1<temp_angle2?temp_angle1:temp_angle2);
		minangle=(maxangle<temp_angle3?minangle:temp_angle3);

		motor->actual_angle =(temp_angle1+temp_angle2+temp_angle3)-maxangle-minangle;
	  motor->actual_speed =(temp_speed1+temp_speed2+temp_speed3)-maxspeed-minspeed;
		
		callback_flag=1;
	}
	break;
	default:break;
}
}
	

/**
  * @breif 电机参数初始化
  */
void motor_init()
{
	//1号电机.
	motor1.start_angle = 0;
	motor1.actual_angle = 0;
	motor1.actual_speed = 0;
	motor1.start_angle_flag = 0;
	motor1.actual_current = 0;
	motor1.target_current = 0;
	//motor1.temp = 0;

	//2号电机
	motor2.start_angle = 0;
	motor2.actual_angle = 0;
	motor2.start_angle_flag = 0;
	motor2.actual_speed = 0;
	motor2.actual_current = 0;
	motor2.target_current = 0;
	//motor2.temp = 0;

	//3号电机
	motor3.start_angle = 0;
	motor3.actual_angle = 0;
	motor3.start_angle_flag = 0;
	motor3.actual_speed = 0;
	motor3.actual_current = 0;
	motor3.target_current = 0;
	//motor3.temp = 0;

	//4号电机
  motor4.start_angle = 0;
	motor4.actual_angle = 0;
	motor4.start_angle_flag = 0;
	motor4.actual_speed = 0;
	motor4.actual_current = 0;
	motor4.target_current = 0;
	//motor4.temp = 0;
	
	motor_can2_1.start_angle = 0;
	motor_can2_1.actual_angle = 0;
	motor_can2_1.start_angle_flag = 0;
	motor_can2_1.actual_speed = 0;
	motor_can2_1.actual_current = 0;
	motor_can2_1.target_current = 0;
	
	Kinematics.stepper_motor_left.pulse_num=0;
	Kinematics.stepper_motor_left.pwm_pulse=0;
	Kinematics.stepper_motor_right.pulse_num=0;
	Kinematics.stepper_motor_right.pwm_pulse=0;	
 
}


/**
  * @breif 电机电流赋值函数
  */
void set_chassis_current()
{
	u8 current_msg[8];
	
	//电机目标电流为速度pid输出
	motor1.target_current = motor1.vpid.PID_OUT;
	motor2.target_current = motor2.vpid.PID_OUT;
	motor3.target_current = motor3.vpid.PID_OUT;
	motor4.target_current = motor4.vpid.PID_OUT;

	
	//can总线通信协议，参照电调说明书
	current_msg[0] = motor1.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = motor1.target_current & 0xff;		//1号电机电流低8位
	current_msg[2] = motor2.target_current >> 8;			//2号电机电流高8位
	current_msg[3] = motor2.target_current & 0xff;		//2号电机电流低8位
	current_msg[4] = motor3.target_current >> 8;			//3号电机电流高8位
	current_msg[5] = motor3.target_current & 0xff;		//3号电机电流低8位
	current_msg[6] = motor4.target_current >> 8;			//4号电机电流高8位
	current_msg[7] = motor4.target_current & 0xff;		//4号电机电流低8位
	
	//can发送数据帧
	CAN1_Send_CHASSIS_Msg(current_msg);
}

int handle_temp=20;

void set_handle_current()
{
	u8 current_msg[8];
	
	//电机目标电流为速度pid输出
	motor5.target_current = motor5.vpid.PID_OUT;//
	
	//can总线通信协议，参照电调说明书
	current_msg[0] = motor5.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = motor5.target_current & 0xff;		//1号电机电流低8位

	//can发送数据帧
	CAN1_Send_handle_Msg(current_msg);
}

void set_flip_current()
{
	u8 current_msg[8];
	
	//电机目标电流为速度pid输出
	motor6.target_current = motor6.vpid.PID_OUT;  
	motor7.target_current = motor7.vpid.PID_OUT;
	
	//can总线通信协议，参照电调说明书
	current_msg[2] = motor6.target_current >> 8;			//1号电机电流高8位
	current_msg[3] = motor6.target_current & 0xff;		//1号电机电流低8位
	current_msg[4] = motor7.target_current >> 8;			//1号电机电流高8位
	current_msg[5] = motor7.target_current & 0xff;		//1号电机电流低8位
		
	

	//使用can1发送数据帧
	CAN1_Send_flip_Msg(current_msg);
}

void set_belt_current()
{
	u8 current_msg[8];

	motor10.target_current = motor10.vpid.PID_OUT;
	motor11.target_current = motor11.vpid.PID_OUT;
	
	current_msg[0] = motor10.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = motor10.target_current & 0xff;		//1号电机电流低8位
	current_msg[2] = motor11.target_current >> 8;			//1号电机电流高8位
	current_msg[3] = motor11.target_current & 0xff;		//1号电机电流低8位		
	
	//使用can2发送数据帧
	CAN2_Send_belt_Msg(current_msg);
}

void set_resuce_current()
{
  u8 current_msg[8];

	motor12.target_current = motor12.vpid.PID_OUT;
	motor13.target_current = motor13.vpid.PID_OUT;
	
	current_msg[4] = motor12.target_current >> 8;			//1号电机电流高8位
	current_msg[5] = motor12.target_current & 0xff;		//1号电机电流低8位
	current_msg[6] = motor13.target_current >> 8;			//1号电机电流高8位
	current_msg[7] = motor13.target_current & 0xff;		//1号电机电流低8位		
	
	//使用can2发送数据帧
	CAN2_Send_resuce_Msg(current_msg);

}

int temp_resuce;
void set_resucecard_current()
{
	u8 current_msg[8];

	motor_can2_1.target_current = motor_can2_1.vpid.PID_OUT;
	
	current_msg[0] = motor_can2_1.target_current >> 8;			//1号电机电流高8位
	current_msg[1] = motor_can2_1.target_current & 0xff;		//1号电机电流低8位
		
	

	//使用can2发送数据帧
	CAN2_Send_resucecard_Msg(current_msg);
}


/**
  * @breif 将底盘电机停止到当前角度
  */
void stop_chassis_motor()
{
	//读取当前角度值
	motor1.stop_angle = motor1.actual_angle;
	motor2.stop_angle = motor2.actual_angle;
	motor3.stop_angle = motor3.actual_angle;
	motor4.stop_angle = motor4.actual_angle;
	
	//改变角度pid目标角度值
	set_chassis_motor_angle(motor1.stop_angle,motor2.stop_angle,motor3.stop_angle,motor4.stop_angle);
	
}

/**
  * @breif 将拨弹轮电机停止到当前角度
  */
void stop_handel_motor()
{
	motor5.stop_angle = motor5.apid.actual_angle;

	set_handle_angle(motor5.stop_angle,0);
}


void handle_90()
{
	Kinematics.handle_L.target_angle=-80;
	Kinematics.handle_R.target_angle=-80;
}

void handle_180()
{
	Kinematics.handle_L.target_angle=-200;
	Kinematics.handle_R.target_angle=-200;
}


