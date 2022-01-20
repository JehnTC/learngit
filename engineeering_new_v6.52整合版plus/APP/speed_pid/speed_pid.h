#ifndef _SPEED_PID_H
#define _SPEED_PID_H
#include "motor.h"
#include "Kinematic.h"

/*3508������������Χ��     -16384 ~ 16384
                            ��Ӧ�������  -20A ~ 20A
	2006������������Χ��     -10000 ~ +10000
                            ��Ӧ�������  -10A ~ 10A	
	6020�����ѹ��Χ						-30000 ~ +30000
*/

#define CHASSIC_Integral_max         5000            //�����ֱ���
#define CHASSIC_IntegralSeparation   500             //���ַ���
#define CHASSIC_vPID_max             8000            //����޷�

#define GIMBAL_Integral_max         500
#define GIMBAL_IntegralSeparation   20
#define GIMBAL_vPID_max             3000

#define FLIP_Integral_max         500
#define FLIP_IntegralSeparation   20
#define FLIP_vPID_max             5000

#define BELT_vPID_max             5000



//�궨�壬���Ŀ���ٶȸ���ֵ
#define set_chassis_speed(motor1_speed,motor2_speed,motor3_speed,motor4_speed) \
        do{                                                                    \
					motor1.vpid.target_speed = motor1_speed;		                         \
	        motor2.vpid.target_speed = motor2_speed;                             \
	        motor3.vpid.target_speed = motor3_speed;                             \
	        motor4.vpid.target_speed = motor4_speed;                             \
	        motor1.target_speed = motor1_speed;		                               \
	        motor2.target_speed = motor2_speed;                                  \
	        motor3.target_speed = motor3_speed;                                  \
	        motor4.target_speed = motor4_speed;                                  \
				}while(0)                                                              \

#define set_handle_speed(motor5_speed) \
        do{                                                                    \
	        motor5.vpid.target_speed = motor5_speed;                             \
	        motor5.target_speed = motor5_speed;                                  \
				}while(0)                                                              \

#define set_flip_speed(motor6_speed,motor7_speed) \
        do{   					\
					motor6.vpid.target_speed = motor6_speed;                             \
	        motor6.target_speed = motor6_speed;                                  \
	        motor7.vpid.target_speed = motor7_speed;                             \
	        motor7.target_speed = motor7_speed;                                  \
				}while(0)                                                              \

#define set_card_speed(motor_can2_1_speed) \
        do{                                                                    \
	        motor_can2_1.vpid.target_speed = motor_can2_1_speed;                             \
	        motor_can2_1.target_speed = motor_can2_1_speed;                                  \
				}while(0)                                                              \
				
#define set_belt_speed(motor10_speed,motor11_speed) \
        do{                                                                    \
	        motor10.vpid.target_speed = motor10_speed;                             \
					motor10.target_speed = motor10_speed;                             \
					motor11.vpid.target_speed = motor11_speed;                             \
					motor11.target_speed = motor11_speed;                             \
				}while(0)                                                              \
				
#define set_resuce_speed(motor12_speed,motor13_speed) \
        do{                                                                    \
	        motor12.vpid.target_speed = motor12_speed;                             \
					motor12.target_speed = motor12_speed;                             \
	        motor13.vpid.target_speed = motor13_speed;                             \
					motor13.target_speed = motor13_speed;                             \
				}while(0)                                                              \
				
#define pid_init() \
				do{                 \
					/*�����ٶȻ�*/                        \
					pid_t.chassic_pid.speed_loop.kp = 3.05 ; \
					pid_t.chassic_pid.speed_loop.ki = 0.27 ; \
					pid_t.chassic_pid.speed_loop.kd = 0; \
					                                     \
					pid_t.trigger_pid.speed_loop.kp = 2.5; \
					pid_t.trigger_pid.speed_loop.ki = 0.05; \
					pid_t.trigger_pid.speed_loop.kd = 0; \
                                               \
					pid_t.handle.position_loop.kp = 0.22; \
					pid_t.handle.position_loop.ki = 0; \
					pid_t.handle.position_loop.kd = 0.15; \
					                                    \
					pid_t.handle.speed_loop.kp = 4; \
					pid_t.handle.speed_loop.ki = 0.8; \
					pid_t.handle.speed_loop.kd = 1.6; \
					                                    \
					pid_t.flip.speed_loop.kp = 4; \
					pid_t.flip.speed_loop.ki = 0.8; \
					pid_t.flip.speed_loop.kd = 1.6; \
																							\
					pid_t.flip.position_loop.kp = 0.22; \
					pid_t.flip.position_loop.ki = 0; \
					pid_t.flip.position_loop.kd = 0.15; \
					\
			    pid_t.card.speed_loop.kp = 8; \
					pid_t.card.speed_loop.ki = 1.2; \
					pid_t.card.speed_loop.kd = 0.5; \
					\
					pid_t.card.position_loop.kp = 0.065; \
					pid_t.card.position_loop.ki = 0; \
					pid_t.card.position_loop.kd = 0.05; \
					\
   				pid_t.resuce.speed_loop.kp = 2.3; \
					pid_t.resuce.speed_loop.ki = 0.54; \
					pid_t.resuce.speed_loop.kd = 0.25; \
				}while(0)                       \
					
	    
				
typedef enum switch_flag_t
{
	CHASSIC = 1,
	TRIGGER = 2,
	HANDLE  = 3,
	FLIP    =	4,
	CARD    = 5,
	BELT    = 6,
	RESUCE  = 7,
	NUL     =	0,
}switch_flag_t;

typedef struct
{
	float kp;			
	float ki;	
  float kd;	
}Parameter_t;

typedef struct
{
  Parameter_t position_loop;
	Parameter_t speed_loop;
}PID_Loop_t;
	
typedef struct
{
  PID_Loop_t chassic_pid;
	PID_Loop_t trigger_pid;
	PID_Loop_t handle;
	PID_Loop_t flip;
	PID_Loop_t card;
	PID_Loop_t resuce;
}PID_t;

extern PID_t pid_t;
		
extern int pid_target_speed;
extern int pid_target_angle;
extern switch_flag_t switch_flag;

void VPID_Init_All(void);			//���ת��PID������ʼ��

void vpid_chassic_realize(float kp,float ki,float kd);				//���ת��PIDʵ��
void vpid_handle_realize(float kp,float ki,float kd); //��е������ٶȻ�
void vpid_flip_realize(float kp,float ki,float kd);//��תװ���ٶȻ�PID
void vpid_card_realize(float kp,float ki,float kd);
void vpid_belt_realize(float kp,float ki,float kd);
void vpid_resuce_realize(float kp,float ki,float kd);

int abs(int input);				//�����ֵ����
int pid_auto(void);
int pid_pc(void);
#endif
