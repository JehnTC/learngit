/**
  ******************************************************************************
  * @file    Project/USER/Tim3_Events.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   定时器3相关函数
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#include "Tim3_Events.h"
#include "speed_pid.h"
#include "key.h"
/**
  * @breif 运动控制函数
	* @param  各pid参数
	*/
	
int temp_flip,temp_handle,temp_chassic1,temp_chassic2,temp_chassic3,temp_chassic4,temp_speed;
void Robo_Move()
{
		
	/*------ 运动控制函数 ------*/
	
	
	chassic_speed_control(Liner_X, Liner_Y, Angular_Z);	//底盘
	
	handle_speed_control(Handle_Left_Angular);//机械臂伸出
	
	card_speed_control(card_angle);    
  apid_card_realize(a_card_p,a_card_i,a_card_d);
	vpid_card_realize(v_card_p,v_card_i,v_card_d);		
		
	
	vpid_resuce_realize(v_resuce_p,v_resuce_i,v_resuce_d);		

	vpid_belt_realize(v_card_p,v_card_i,v_card_d);		
		
	flip_speed_control(Flip_Angular,-Flip_Angular);//机械臂翻转装置

	
	/*------ pid计算 ------*/	
	vpid_chassic_realize(v_chassic_p,v_chassic_i,v_chassic_d);		
	vpid_handle_realize(v_handle_p,v_handle_i,v_handle_d);
	vpid_flip_realize(v_flip_p,v_flip_i,v_flip_d);
	
  /*------ 电流赋值 ------*/	
	set_chassis_current();	
	set_handle_current();
	set_flip_current();
	set_resucecard_current();
	set_belt_current();
	set_resuce_current();

}

/**
  * @breif 调试用的按键，即单片机上的白色按键
	* @param key_flag
	*/
void Debug_Key()
{
	static int key_flag = unpressed;		//用于控制下面判断在按下的过程中只进入一次
	u8 get_key;
	get_key = key_press();
	if(get_key && key_flag == unpressed)		//如果按键被按下
	{
		LED4=!LED4;												//LED4反转，用于状态指示
		key_flag = pressed;			          //按键被按下
		
	}
	else if(!key_press())
		key_flag=unpressed;		//按键未被按下
}

