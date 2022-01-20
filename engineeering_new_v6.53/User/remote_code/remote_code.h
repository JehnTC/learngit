#ifndef _REMOTE_CODE_H_
#define _REMOTE_CODE_H_
#include "sys.h"


extern int belt_speed;
extern int card_angle;
extern int resuce_speed1;
extern int resuce_speed2;

/*
  //大疆遥控器参数范围
  ch0:364-1024-1684
	ch1:364-1024-1684
	ch2:364-1024-1684
  ch3:364-1024-1684
  s1: 1-3-2
  s2: 1-3-2
*/
	
//大疆遥控器

  #define x_CH_width            rc.ch0         //x方向通道脉宽   右摇杆左右
	#define y_CH_width            rc.ch1         //y方向通道脉宽   右摇杆上下
	#define r_CH_width            rc.ch2         //r方向通道脉宽   左摇杆左右
	#define i_CH_width            rc.ch3         //云台俯仰通道脉宽 左摇杆上下
	#define	DJI_Motion_Yaw		  	rc.sw					//使用左手滑轮控制云台yaw
	#define RIGHT_LEVER           rc.s2          //右拨杆
  #define LEFT_LEVER            rc.s1          //左拨杆
	#define Lever_down        	  2   					 //拨到最下
	#define Lever_mid      				3  				     //拨到中间 
	#define Lever_up					    1              //拨到上 
	
	/*以下定义中的数值均通过watch中观察各通道值所得*/
	
  //各方向初始值
	#define x_initial_value       1024            
	#define y_initial_value       1024
	#define r_initial_value       1024
	#define i_initial_value       1024
	//各方向最大最小值
	#define x_max_value           1684             
	#define x_min_value           364
	#define y_max_value           1684
	#define y_min_value           364
	#define r_max_value           1684
	#define r_min_value           364
	#define i_max_value           1684
	#define i_min_value           364
	//阈值上下限
	#define stop_max_value				2.5
	#define stop_min_value				1.5
	#define remote_max_value      4
	#define remote_min_value     0.5		//remote范围应包含stop
	
	#define mouse_x               RC_Ctl.mouse.x     //遥控器额外通道控制：鼠标键盘
	#define mouse_y               RC_Ctl.mouse.y
	#define mouse_z               RC_Ctl.mouse.z
    #define mouse_pre_left        RC_Ctl.mouse.press_l
	#define mouse_pre_right       RC_Ctl.mouse.press_r
	#define key_board             RC_Ctl.key.v

	#define W_key                0x000001
	#define S_key                0x000002
	#define ws_key               0x000003
	
	#define A_key                0x000004
	#define D_key                0x000008
	#define ad_key               0x00000C
	
	#define Q_key                0x000040
	#define E_key                0x000080
	#define qe_key               0x0000C0
	
	#define SHIFT_key            0x000010
	#define CTRL_key             0x000020
	
	#define F_key                0x000200
	#define Null_key             0x000000
	
	#define R_key                0x100000
  
	#define C_key                0x200000


	#define remote_ch_init() \
do{ \
		rc.ch0=1024;\
		rc.ch1=1024;\
		rc.ch2=1024;\
		rc.ch3=1024;\
}while(0)\



  #define Liner_X  Kinematics.target_velocities.linear_x
  #define Liner_Y  Kinematics.target_velocities.linear_y
  #define Angular_Z Kinematics.target_velocities.angular_z

#define dji_remote_assignment() \
	do{ \
	Kinematics.target_velocities.linear_x=x_speed; \
  Kinematics.target_velocities.linear_y=y_speed; \
	Kinematics.target_velocities.angular_z=z_speed; \
	}while(0) \

	#define Handle_Left_Angel    Kinematics.handle_L.target_angle
	#define Handle_Left_Angular  Kinematics.handle_L.target_angular

	#define Flip_Angular				 Kinematics.flip.target_angular	//翻转结构
	#define Flip_Angle					 Kinematics.flip.target_angle


	#define	Steering_Angle 			 Kinematics.clamping_claw.target_angle		//舵机输入角度


  
#define Rescue_Claw_Pulse_Right    Kinematics.rescue_claw_pulse_right              //救援爪舵机脉宽
#define Rescue_Claw_Pulse_Left    Kinematics.rescue_claw_pulse_left              //救援爪舵机脉宽

void Remote_Control(void);
float x_max_speed_caculator(float x);
float y_max_speed_caculator(float y);
float z_max_speed_caculator(float z);
float caculate_linear_speed(int width,int mid,int min,int max);
float caculate_rotational_speed(int width,int mid,int min,int max);
float caculate_gimbal_pitch_angle(int width,int mid,int min,int max);
float caculate_gimbal_yaw_angle(int width,int mid,int min,int max);
float caculate_handel_speed(int width,int mid,int min,int max);
float steering_moving_control(void);
void left_act1(void);
void left_act2(void);
void left_act3(void);


#define MOTOR5_ROUND_MAX 60			//motor5转轮最大圈数
#define MOTOR5_ROUND_MIN 2			//motor5转轮最小圈数

#endif 
