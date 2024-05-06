#ifndef __MOVEBASE_H
#define __MOVEBASE_H
/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"
#include "can.h"
#include "HareWare.h"
#include "MoveBase.h"
#include "moto.h"
#include "calculation.h"
#include "PID.h"
#include "stm32f4xx.h"
#include "FSM.h"
typedef struct
{
	float X;
	float Y;
	float Yaw;
	float V_x;
	float V_y;
	float W;
}PATH_TYPEDEF;

typedef struct ROBOT_CHASSIS
{
	float World_V[3]; // Y , X , W
	float Robot_V[2];
	//float Position[2];
	//float Motor_RPM[4];
	float expect_angle ;
	float Angle;
	int flag;//�͵���ͨѶ��
	
} ROBOT_CHASSIS;

typedef struct
{
	float Distance;
	float Pstart;        //��ʼλ��
	float Pend;          //����λ��
	float Vstart;        //��ʼ���ٶ�           
	float Vmax;          //�����ٶ�
	float Vend;          //ĩβ���ٶ�
	float Rac;           //����·�̵ı���
	float Rde;           //����·�̵ı���
	int flag;            //��ɱ�־λ��ͣ������ʱ����1
}TrapezoidPlaning_TYPEDEF;

void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos);
int PDControllertest(float x,float y,float w);
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw);
int YawAdjust(float Target_angle);
void AngleLimit(float *angle);
//void World_3wheels(float Vx_RPM, float Vy_RPM, float W_RPM, float theta);
int moving_point_track(float POS_X, float POS_Y, float POS_YAW,float V_max);
void Move_Init(void);
int near_pillar(float x,float y,float POS_YAW,float V_max);
int TrapezoidPlaning(float real_time,float T,float POS_X,float POS_Y,float POS_YAW,float V_start,float V_end,float V_max,float R_ac, float R_de,int* first_time_flag);

int chassis_TrapezoidPlaning(float POS_X_start,
	                    float POS_Y_start,
											float POS_X_end,
											float POS_Y_end,
											float POS_YAW,
											float V_start,
											float V_end,
											float V_max,
											float R_ac,
											float R_de);
int laser_speed_control(float Xstart,float Ystart,//��ʼʱ�������ֵ
												float Xend,  float Yend,  //����ʱĿ��ļ�����ֵ
												int Xnow,  int Ynow,	//ʵʱ�ļ�����ֵ
												float Vmax,  float Vstart,float Vend,float POS_YAW);//����ٶȣ���ʼ�ٶȣ������ٶ�			

int chassis_TrapezoidPlaning_laser(float POS_X_start,//��ʼʱ�������ֵ
	                    float POS_Y_start,//��ʼʱ�������ֵ
											float POS_X_end,//����ʱĿ��ļ�����ֵ
											float POS_Y_end,//����ʱĿ��ļ�����ֵ
											float POS_YAW,//�Ƕ�ֵ
											float V_start,//��ʼ�ٶ�
											float V_end,//�����ٶ�
											float V_max,//����ٶ�
											float R_ac,//
											float R_de,//
												int POS_X_now,//
                        int POS_Y_now)	;//											
											
extern PID point_pid;//��Ե�׷��PID
extern PID yaw_pid;//�Ƕ�PID
extern ROBOT_CHASSIS Robot_Chassis;

//extern TrapezoidPlaning_TYPEDEF TPlaning_DATA;
/* define ------------------------------------------------------------------*/
//��ȫ���ֵ��̵Ĳ���
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)

// Chassis Config
#define WHEEL_R            0.076f	                  //���Ӱ뾶(��λ��m)
#define Robot_R            0.406f                  	//���ֵ����ľ���(��λ��m)
#define M3508_RM_To_MS     (PI*WHEEL_R)/570.0f      //ת�����ٶȵ�ת�� (��λ��m/s) 
#define M3508_MS_To_RM     1.0f/(PI*WHEEL_R)      //�ٶ���ת�ٵ�ת�� (��λ��m/s)  
#define MS_TO_RPM          21*60/(PI*WHEEL_R*2)     //����ֱ��152mm��������ٱ�1:21������һȦpi*152mm
#define RM_transition_MS (PI * WHEEL_R) / 570.0f //ת�����ٶȵ�ת��
#define MS_transition_RM 1.0f / (PI * WHEEL_R) //�ٶ���ת�ٵ�ת��
																										// ���㹫ʽ��1/��pi*����ֱ����*���ٱ�*60

//�����תһ�ܵ�������
#define COUNTS_PER_ROUND (32768)
//������ת��
#define MAX_MOTOR_SPEED (COUNTS_PER_ROUND*100)
//����ֱ������λ��mm��
#define WHEEL_DIAMETER (70.0f)
//��λϵͳX�᷽�����ľ���
#define DISX_OPS2CENTER (0.0f)
//��λϵͳY�᷽�����ľ���
#define DISY_OPS2CENTER (-307.0f)
//�����ּ��ٱ�
#define WHEEL_REDUCTION_RATIO (2.0f/1.0f)
//M2006���ٱ�
#define M2006_REDUCTION_RATIO (36.0f/1.0f)
//ת����ּ��ٱ�
#define TURNING_REDUCTION_RATIO (4.0f/1.0f)
//����ת����ٱ�
#define WHEEL_TURNING_REDUCTION_RATIO (M2006_REDUCTION_RATIO*TURNING_REDUCTION_RATIO)
//������ת�뾶
#define MOVEBASE_RADIUS (362.039f)
//�Ƕ���ת��Ϊ������
#define ANGLE2RAD(x) (x/180.0f*PI)
//������ת��Ϊ�Ƕ���
#define RAD2ANGLE(x) (x/PI*180.0f)
																				
//��ǰ��ID��
#define LEFT_FRONT_ID (1)
//��ǰ��ID��
#define RIGHT_FRONT_ID (2)
//�����ID��
#define LEFT_REAR_ID (3)
//�Һ���ID��
#define RIGHT_REAR_ID (4)
//��ǰ��ת��ID��
#define LEFT_FRONT_TURNING_ID (5)
//��ǰ��ת��ID��
#define RIGHT_FRONT_TURNING_ID (6)
//�����ת��ID��
#define LEFT_REAR_TURNING_ID (7)
//�Һ���ת��ID��
#define RIGHT_REAR_TURNING_ID (8)

#endif
