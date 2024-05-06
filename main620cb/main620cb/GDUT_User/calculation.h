/**
  ******************************************************************************
  * @file    
  * @author  LLY
  * @version 1.0
  * @date    2023/3/16
  * @brief   ��ֲ������ѧrm3508_can_opensource-master 
  ******************************************************************************
  * @attention
  *
	*
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __calculation_h
#define __calculation_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"
#include "moto.h"
#include "HareWare.h"
#include "MoveBase.h"


/* 9����---------------------------------------------------------------------- */
//#define control_nineblock_0 0
//#define control_nineblock_1 1
//#define control_nineblock_2 2
//#define control_nineblock_3 3
//#define control_nineblock_4 4
//#define control_nineblock_5 5
//#define control_nineblock_6 6
//#define control_nineblock_7 7
//#define control_nineblock_8 8
//#define control_nineblock_9 9
/* define --------------------------------------------------------------------*/
#define  push 1 
#define  back -1
#define	 hold  0
/* Exported types ------------------------------------------------------------*/

//typedef enum//arm����
//{ 
//	start,
//	arm_raise,
//	arm_dowm,
//	clamp_open,
//	clamp_close,
//	arm_middle,
//	homing,//ȡ��λ��У׼����
//	arm_home,//�м�λ��
//	arm_groud,//ȡ��λ��
//	arm_raise_to_home,//��ȡ��λ��̧�����м䶯��
//	arm_down_to_home,//�ӷŻ�λ���½����м䶯��
//	catch_ok
//}ARM_STATE_ITEMS;

//// ���������ٶ�
//typedef struct ROBOT_TARGET_VELOCITY
//{
//	float Vx;
//	float Vy;
//	float W;
//	float Vx_RPM;
//	float Vy_RPM;
//	float W_RPM;
//}ROBOT_TARGET_VELOCITY;

typedef enum//�������/��̨����
{ 
	shout1,
	shout2,
	shout3,
	load,//�ϻ�
	home//����
}SHOOT_ITEMS;

typedef struct averageFilte_TPYE
{
	float data[5];
	float indata;
	float outdata;
} averageFilter_TPYE;
//extern ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;

/**����������״̬��**/
extern SHOOT_ITEMS SHOOT_STATE;
//extern ARM_STATE_ITEMS ARM1_STATE;
extern int pwm_key_flag;
extern int armstate;


/* functions ------------------------------------------------------- */
void moto_calculation(void);
void mit_calculation(void);
void m2006_init(void);
void shoot_calculation(void);
void arm_calculation(void);
void Robot_Wheels_RPM_calculate(void);
void averageFilter(averageFilter_TPYE *ff);
void get_9block(void);
void Update_Action(void);
#endif

/****************** (C) COPYRIGHT 2016 ACTION *****END OF FILE*************/

