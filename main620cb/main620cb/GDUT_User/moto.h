/**
  ******************************************************************************
  * @file    
  * @author  ������
  * @version 1.1.0
  * @date    2023/3/29
  * @brief   ������ƿ�
  ******************************************************************************
  * @attention
  *�ٶȿ��ƺ�λ�ÿ��ƿ����ã�����ģʽ��ת��ģʽҲ�����ã�����ע��ת��ģʽ�µ�����ܻ���ֶ���
	*���߹滮�����ι滮������ֲ�У��𼱣���������԰�����ֲ��
	*�Ұ�����¡���˶�����ɾ���ˣ�����������˭�пհ��Ұ����ƻ���
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTO_H
#define __MOTO_H
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
//#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "PID.h"
#include "moto.h"
#include "math.h"
#include "can.h"
/* define --------------------------------------------------------------------*/
// M3508������
#define M3508_CHASSIS_MOTOR_ID_1      0x201
#define M3508_CHASSIS_MOTOR_ID_2      0x202
#define M3508_CHASSIS_MOTOR_ID_3      0x203
#define M3508_CHASSIS_MOTOR_ID_4      0x204
#define M3508_CHASSIS_MOTOR_ID_5      0x205
#define M3508_CHASSIS_MOTOR_ID_6      0x206
#define M2006_CHASSIS_MOTOR_ID_0			0x207
#define M2006_CHASSIS_MOTOR_ID_1			0x208
//����������ģʽ
#define  SPEED_CONTROL_MODE				2
#define  VELOCITY_PLANNING_MODE   3
#define  CURRENT_MODE             4
#define  POSITION_CONTROL_MODE		5
#define  SPEED_TARQUE_CONTROL_MODE 6
#define  POSITION_TORQUE_MODE		  7
#define  HOMEINGMODE		          9
#define  MOTO_OFF		              0



/*******************************��������������************************************/

/** 
  * @brief  �������  3508 or 2006
  */   
typedef enum
{ 
  RM_3508   = 1, 
  M_2006    = 2,
	NONE      = 3  //none��ʾû�нӵ��

}MotorType_TypeDef;

/** 
  * @brief  currentType structure definition  
  * @note     
  */
typedef struct
{
	float current;
	
	int32_t cnt;
	
	int flag;
}currentType;

/** 
  * @brief  HomingMode type structure definition  
  * @note     
  */
typedef struct
{
	float vel;
	
	float current;
	
	float initPos;//���ó�ʼλ�ã���ʱ����
	
	int32_t cnt;
	
	int flag;
}HomingModeType;

/** 
  * @brief  Pos_TorqueMode type structure definition  
  * @note     
  */
typedef struct
{
	float current;
	
	float Pos;//Ŀ��λ��
	
	float output;
	
	int16_t  TARGET_TORQUE;//Ŀ��ת�أ��õ�����ʾ
	
	int flag;
	
	int32_t cnt;
}Pos_TorqueModeType;

/** 
  * @brief  Speed_TorqueMode type structure definition  
  * @note     
  */
typedef struct
{
	float current;
	
	float Vel;//Ŀ���ٶ�
	
	float output;
	
	int16_t  TARGET_TORQUE;//Ŀ��ת�أ��õ�����ʾ
	
	int flag;
	
	int32_t cnt;
}Vel_TorqueModeType;

/** 
  * @brief   VELOCITY_PLANNING type structure definition  
  * @note     
  */
typedef struct VELOCITY_PLANNING //�ٶȹ滮
{
	float Distance;
	float Pstart;        //��ʼλ��
	float Pend;          //����λ��
	float Vstart;        //��ʼ���ٶ�           // ��λ��RPM ����ֵ
	float Vmax;          //�����ٶ�
	float Vend;          //ĩβ���ٶ�
	float Rac;           //����·�̵ı���
	float Rde;           //����·�̵ı���
	int flag;            //��ɱ�־λ�����ͣ������ʱ����1
}VELOCITY_PLANNING;

/** 
  * @brief  �����Ϣ  
  * @note     
  */
typedef struct MOTO_REAL_INFO
{
	// ���ģʽ
	uint32_t unitMode;//���ģʽ
		//POSITION_CONTROL_MODEλ��ģʽ
		//POSITION_TARQUE_CONTROL_MODEλ��_����ģʽ
	  //SPEED_TARQUE_CONTROL_MODEλ��_����ģʽ
		//SPEED_CONTROL_MODE�ٶ�ģʽ
		//MOTO_OFF����ر�-->����������
	  //VELOCITY_PLANNING_MODE���ι滮ģʽ
	//
	MotorType_TypeDef type;//������ͣ�m3508��m2006
	uint16_t ANGLE;   		//�����Ƕ�						
	int16_t  RPM;					//�ٶ�ֵ			
	int16_t  CURRENT;     //����ֵ
	int16_t  TARGET_CURRENT;//Ŀ�����ֵ
	int16_t  TARGET_POS;//Ŀ��Ƕ�
	float    TARGET_RPM;//Ŀ��ת��
	int      Velflag;//����Ϊ��ʱ����1 
	//�ṹ��
	HomingModeType homingMode;//�������ģʽ
	Pos_TorqueModeType pos_torquemode;//λ��ת��ģʽ
	Vel_TorqueModeType vel_torquemode;//�ٶ�ת��ģʽ
	VELOCITY_PLANNING velocity_planning;//�ٶȹ滮

	
	// �ǶȻ���ʱ�õ��������
	float		 REAL_ANGLE;         //���������ʵ�Ƕȣ�������float��
	uint8_t	 FIRST_ANGLE_INTEGRAL_FLAG;  //?
	uint16_t LAST_ANGLE;   //?
	int16_t filter_RPM;
}MOTO_REAL_INFO;



//�����Ŀ���ٶ�
typedef struct MOTOR_RPM
{
	float MOTOR1_RPM;
	float MOTOR2_RPM;
	float MOTOR3_RPM;
	float MOTOR4_RPM;
	float MOTOR5_RPM;
	float MOTOR6_RPM;
	float MOTOR7_RPM;
	float MOTOR8_RPM;
}MOTOR_RPM;

//�����Ŀ��λ��
typedef struct MOTOR_POS
{
	float MOTOR1_POS;
	float MOTOR2_POS;
	float MOTOR3_POS;
	float MOTOR4_POS;
	float MOTOR5_POS;
	float MOTOR6_POS;
	float MOTOR7_POS;
	float MOTOR8_POS;
}MOTOR_POS;


//�������߹滮�Ľṹ��
//�ò���
/* �������ٶ����߶��� */
typedef struct CurveObject {
  float startSpeed;    //��ʼ����ʱ�ĳ�ʼ�ٶ�
  float currentSpeed;   //��ǰ�ٶ�
  float targetSpeed;    //Ŀ���ٶ�
  float stepSpeed;     //���ٶ�
  float speedMax;     //����ٶ�
  float speedMin;     //��С�ٶ�
  uint32_t aTimes;     //����ʱ��
  uint32_t maxTimes;    //���ٿ��
	float  p_add;    //���ٵ�ռ��
	float  p_decrease; //���ٵ�ռ��
  
}CurveObjectType;




/* Exported types ------------------------------------------------------------*/
extern struct PID MOTOR_PID_POS[8];				// λ�û�
extern struct PID MOTOR_PID_RPM[8];	// 1-4���̵�� 5-6������
extern struct PID laser_X_pid;//����pid
extern struct PID laser_Y_pid;
extern struct PID laser_K_pid;
extern struct PID laser_B_pid;
extern struct MOTO_REAL_INFO MOTOR_REAL_INFO[8]; 
extern struct MOTOR_RPM MOTOR_TARGET_RPM;
extern struct MOTOR_POS MOTOR_TARGET_POS;

/* Exported functions ------------------------------------------------------- */
float MaxMinLimit(float val,float limit);
void M3508_Motor_Init(void);
void m3508_update_m3508_info(CAN_RxHeaderTypeDef *msg,uint8_t	can1_RxData[8]);
void chassis_m3508_send_motor_currents(void);//���͵���
void M3508AngleIntegral(MOTO_REAL_INFO *M3508_MOTOR);//�ǶȻ���
void MotorCtrl(void);//���ݼ���
float CurrentCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_current);
float VelCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_vel);
float PosCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_pos);
void HomingMode(MOTO_REAL_INFO *MOTOR_REAL_INFO);
void Pos_TorqueCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,int16_t target_torque,float target_pos);
void Vel_TorqueCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,int16_t target_torque,float target_Vel);
float MaxMinLimit(float val,float limit);
void VelocityPlanningMODE(MOTO_REAL_INFO *M3508_MOTOR);
float position_control_an_delay(PID *pp_po,PID *pp_rpm,MOTO_REAL_INFO *info,float v_xian,float tar_position,float current_key,int dalay_time,int next_flag,int first_flag);
void Velocity_Planning_setpos(MOTO_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde);
double filter(double input, double prev_output, double prev_input, double cutoff_freq, double sample_rate) ;
#endif
/***********************END OF FILE*************/

