#ifndef __ROBOT__H_
#define __ROBOT__H_
#include "stdint.h"
#include "stm32f4xx.h"
#include "arm_math.h"
#include "can.h"
#include "HareWare.h"
#include "MoveBase.h"
#include "moto.h"
#include "calculation.h"
#include "PID.h"
#include "path.h"
#include "MIT.h"


/** 
  * @brief  ��Ź������������˵ı�־λ
  * @note     
  */
typedef struct ROBOT_FLAG_Type
{
	int move_ok; //0�������ƶ���1�����ƶ�
	int shoot_ok;//0�����������1�������
	
//	int location;
	int Chassis_send;
	int Chassis_receive;
	int Gun_send;	//0װ����1MOVE_1��2MOVE_2
	int Gun_receive;
}ROBOT_FLAG_Type;
/** 
  * @brief  ���ҷ������
  * @note     
  */
typedef struct SHOOT_Type//��������
{
	float pitch;
	float yaw;
	float tiny_pitch;
	float tiny_pitch_data[9];
	float tiny_yaw;
	float tiny_yaw_data[9];
	float shoot_yaw;
	float shoot_pitch;
	float shout_speed_left;
	float shout_speed_right;
	int shout_ok;
	int push_state;
	int gun_state;//�������������״̬0����ʼλ�ã�װ���� 1���ֶ�����  2���Զ�����
}SHOOT_Type;

/** 
  * @brief  KEY
  * @note     
  */
typedef struct KEY_Type//��������
{
  int KEY_armtop;//armtop
	int KEY_armbottom;//armbottom
	int KEY_push;//push
}KEY_Type;


extern KEY_Type KEY_DATA;
extern ROBOT_FLAG_Type ROBOT_FLAG; 
extern SHOOT_Type SHOOT_DATA;//��ߵķ������
/* Exported functions ------------------------------------------------------- */
int CatchRing(void);
int Larm_down(void);
int Larm_down_first(void);
int Larm_wait(void);
int Lclamp_close(void);
int Larm_raise(void);
int Lclamp_open(void);
int Larm_down_to_home(void);
int Larm_home(void);
void shoot_push(void);
void shoot_back(void);

	





#endif
