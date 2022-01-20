#ifndef _ANGLE_PID_H
#define _ANGLE_PID_H



#define set_handel_motor_angle(motor5_angle) \
        do{ \
						motor5.apid.target_angle = motor5_angle; \
						motor5.target_angle = motor5_angle; \
				}while(0)                                    \

//设置电机电机目标角度				
#define set_chassis_motor_angle(motor1_angle,motor2_angle,motor3_angle,motor4_angle) \
				do{  \
						motor1.apid.target_angle = motor1_angle; \
	          motor2.apid.target_angle = motor2_angle; \
	          motor3.apid.target_angle = motor3_angle; \
	          motor4.apid.target_angle = motor4_angle; \
        }while(0)                                    \
				
#define set_flip_angle(motor7_angle) \
        do{ \
						motor7.apid.target_angle = motor7_angle; \
						motor7.target_angle = motor7_angle; \
				}while(0)                                   \

#define set_card_angle(motor_can2_1_angle) \
        do{ \
						motor_can2_1.apid.target_angle = motor_can2_1_angle; \
						motor_can2_1.target_angle = motor_can2_1_angle; \
				}while(0)                                   \
				
#define set_belt_angle(motor10_angle) \
        do{ \
						motor10.apid.target_angle = motor10_angle; \
						motor10.target_angle = motor10_angle; \
				}while(0)                                   \
				
void APID_Init_All(void);			//电机机械角度PID参数初始化
void apid_chassic_realize(float kp,float ki,float kd);			//电机机械角度pid实现
void apid_handle_realize(float kp,float ki,float kd);
void apid_flip_realize(float kp,float ki,float kd);
void apid_card_realize(float kp,float ki,float kd);
void apid_belt_realize(float kp,float ki,float kd);

#endif
