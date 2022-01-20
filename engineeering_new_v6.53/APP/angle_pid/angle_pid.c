/**
  ******************************************************************************
  * @file    Project/APP/angle_pid.c 
  * @author  Siyuan Qiao&Junyu Luo
  * @version V1.0.0
  * @date    1.2021
  * @brief   pid �����ļ�
  ******************************************************************************
  * @attention
  ******************************************************************************
*/

#include "angle_pid.h"
#include "motor.h"
#include <math.h>
#include "speed_pid.h"

#define aPID_OUT_MAX          30000		//������ٶ�

// ����: APID_Init()
// ����: �����е�Ƕ�pid������ʼ��
// �����������е�Ƕ�pid�����ṹ��
// �������
// �ڲ��������û��������
void APID_Init(APID_t *apid)
{
	apid->actual_angle=0;
	apid->target_angle=0;
	apid->total_angle=0;
	apid->err=0;
	apid->err_integration=0;
	apid->P_OUT=0;
	apid->I_OUT=0;
	apid->D_OUT=0;
	apid->PID_OUT=0;
}

// ����: APID_Init_All()
// ����: ���е����е�Ƕ�pid������ʼ��
// ��������
// �������
void APID_Init_All()
{
	APID_Init(&motor1.apid);
	APID_Init(&motor2.apid);
	APID_Init(&motor3.apid);
	APID_Init(&motor4.apid);
  APID_Init(&motor5.apid);
	APID_Init(&motor6.apid);
	APID_Init(&motor7.apid);
	APID_Init(&motor_can2_1.apid);
	
}
void apid_realize(APID_t *apid,float kp,float ki,float kd)
{
	
	apid->err = apid->target_angle - apid->total_angle;
	switch(switch_flag)
	{  
		case(HANDLE):
		{
			
    	apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > 3000) 			
	   	apid->PID_OUT = 3000;
	    else if((apid->P_OUT + apid->D_OUT) < -3000) 
		  apid->PID_OUT = -3000;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
    }
		break;
	  case(TRIGGER):
		{
			apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > aPID_OUT_MAX) 			
	   	apid->PID_OUT = aPID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT) < -aPID_OUT_MAX) 
		  apid->PID_OUT = -aPID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
		}
		break;
		case(CHASSIC):
		{
			apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > aPID_OUT_MAX) 			
	   	apid->PID_OUT = aPID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT) < -aPID_OUT_MAX) 
		  apid->PID_OUT = -aPID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
			}
		break;
		case(FLIP):
		{
			apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > aPID_OUT_MAX) 			
	   	apid->PID_OUT = aPID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT) < -aPID_OUT_MAX) 
		  apid->PID_OUT = -aPID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
			}
		break;
		case(CARD):
		{
			apid->P_OUT = kp * apid->err;
	    apid->D_OUT = kd * (apid->err-apid->last_err);
	    apid->last_err = apid->err;
	
    	if((apid->P_OUT + apid->D_OUT) > aPID_OUT_MAX) 			
	   	apid->PID_OUT = aPID_OUT_MAX;
	    else if((apid->P_OUT + apid->D_OUT) < -aPID_OUT_MAX) 
		  apid->PID_OUT = -aPID_OUT_MAX;
	    else
		  apid->PID_OUT = apid->P_OUT + apid->D_OUT;
			}
		break;
		default:break;
  }	
}

// ����: apid_chassic_realize()
// ����: �����е�Ƕ�pidʵ��
// �����������е�Ƕ�pidϵ��
// �������
void apid_chassic_realize(float kp,float ki,float kd)
{
	//��ȡ��ǰ�Ƕ�ֵ
	motor1.apid.actual_angle = motor1.actual_angle;
	motor2.apid.actual_angle = motor2.actual_angle;
	motor3.apid.actual_angle = motor3.actual_angle;
	motor4.apid.actual_angle = motor4.actual_angle;
	
	switch_flag = CHASSIC;
	//��������е�Ƕ�pid
	apid_realize(&motor1.apid,kp,ki,kd);
	apid_realize(&motor2.apid,kp,ki,kd);
	apid_realize(&motor3.apid,kp,ki,kd);
	apid_realize(&motor4.apid,kp,ki,kd);
	switch_flag = NUL;

	//���õ��Ŀ���ٶ�
	set_chassis_speed(motor1.apid.PID_OUT,motor2.apid.PID_OUT,motor3.apid.PID_OUT,motor4.apid.PID_OUT);
}



void apid_flip_realize(float kp,float ki,float kd)
{
	//��ȡ��ǰ�Ƕ�ֵ
	motor6.apid.actual_angle = motor6.total_angle-motor6.apid.trigger_first_total_angle_storage;
	motor7.apid.actual_angle = motor7.total_angle-motor7.apid.trigger_first_total_angle_storage;

	switch_flag = FLIP;
	//��������е�Ƕ�pid
	apid_realize(&motor6.apid,kp,ki,kd);
	apid_realize(&motor7.apid,kp,ki,kd);
	switch_flag = NUL;
	
	set_flip_speed(motor6.apid.PID_OUT,motor7.apid.PID_OUT);
}


void apid_card_realize(float kp,float ki,float kd)
{
	motor_can2_1.apid.actual_angle = motor_can2_1.actual_angle;
	motor_can2_1.apid.total_angle = motor_can2_1.total_angle;
	switch_flag = CARD;
	//��������е�Ƕ�pid
	apid_realize(&motor_can2_1.apid,kp,ki,kd);
	switch_flag = NUL;
	
	set_card_speed(motor_can2_1.apid.PID_OUT);
}

//void apid_belt_realize(float kp,float ki,float kd)
//{
//	motor10.apid.actual_angle = motor10.actual_angle;
//	motor10.apid.total_angle  = motor10.total_angle;
//	
//	switch_flag = CARD;
//	//��������е�Ƕ�pid
//	apid_realize(&motor10.apid,kp,ki,kd);
//	
//	switch_flag = NUL;
//	
//	set_belt_speed(motor10.apid.PID_OUT,motor11.apid.PID_OUT);

//}

