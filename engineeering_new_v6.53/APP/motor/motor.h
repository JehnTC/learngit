#ifndef MOTOR_H
#define MOTOR_H


//can stdid
#define CAN_LoopBack_ID				 0x200		//���ڻ���ģʽ�Լ����
#define	CAN_3508Motor1_ID  	   0x201		//0x20+���ID
#define	CAN_3508Motor2_ID      0x202
#define	CAN_3508Motor3_ID      0x203
#define	CAN_3508Motor4_ID      0x204
#define CAN_3508Motor5_ID      0x205
#define CAN_3508Motor6_ID      0x206
#define CAN_3508Motor7_ID      0x207
#define CAN_2006Motor10_ID     0x201
#define CAN_2006Motor11_ID     0x202
#define CAN_3508Motor12_ID     0x203  	//ͬ����
#define CAN_3508Motor13_ID     0x204		//ͬ����
#define CAN_2006Motor9_ID      0x205  	//��Ԯ��

//���ת��pid�����ṹ��
typedef struct{
	int err;
	int last_err;
	int err_integration;
	int target_speed;
	int actual_speed;
	
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
}VPID_t;

//�����е�ǶȲ���
typedef struct{
	
	int err;
	int last_err;
	int err_integration;
	int actual_angle;
	int target_angle;
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
  int total_angle;	
	int trigger_first_total_angle_storage;
}APID_t;
//��������ṹ��
typedef struct{
	
	int start_angle;			//�����ʼ�Ƕ�ֵ
	int start_angle_flag;	//��¼�����ʼ�Ƕ�ֵ��flag
	int stop_angle;				//����ֹͣ����ʱ��ĽǶ�ֵ
	int target_angle;
	
	float actual_angle;			//��ǰ��ʵ�Ƕ�ֵ
	int last_angle;				//��һ�η��صĽǶ�ֵ
	int round_cnt;				//��Կ���ʱת����Ȧ��
	int total_angle;			//�ܹ�ת���ļ���
	
	float actual_speed;			//�����ʵ�ٶ�,rpm
	int target_speed;			//���Ŀ���ٶ�,rpm  ת/min
	
	int actual_current;		//�����ʵ����
	int target_current;		//���Ŀ�����
	//int temp;							//����¶ȣ�2006�����֧�֣�3508֧�֣�
	VPID_t vpid;
	APID_t apid;
	int stop_flag;
	int count;
	int pwm_num;
}MOTOR_t;

extern MOTOR_t motor1,motor2,motor3,motor4,motor5,motor6,motor7,stepper_motor_left,stepper_motor_right,limit_switch, motor_can2_1,motor10,motor11,motor12,motor13;

extern int max_motor_speed;
extern float max_base_linear_speed;
extern float max_base_rotational_speed;

//������������ṹ��
typedef struct{			
	
	int motor1_current;
	int motor2_current;
	int motor3_current;
	int motor4_current;
	int motor5_current;
	int motor6_current;
	int motor7_current;
	
}LOOPBACK;

extern LOOPBACK loopback;
	
void record_gimbal_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
void record_motor_callback(MOTOR_t *motor, unsigned short angle, short speed, short current);
void motor_init(void);			//�����ʼ��
void set_chassis_current(void);	//���õ������
void set_handle_current(void);
void stop_chassis_motor(void);	//������Ƕȹ̶��ڵ�ǰֵ
void stop_handel_motor(void);		//����е��λ�ù̶��ڵ�ǰֵ
void set_gimbal_current(void); //������̨���� 2020.7.24
void set_flip_current(void);	//���÷�תװ�õ������
void set_resucecard_current(void);
void set_belt_current(void);
void set_resuce_current(void);

void handle_90(void);
void handle_180(void);

#endif



