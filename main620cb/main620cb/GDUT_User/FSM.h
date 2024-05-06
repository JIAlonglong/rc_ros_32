#ifndef __FSM__H_
#define __FSM__H_
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



typedef enum MOVE_STATE_ITEMS
{
	MOVE_OK,
	
	MOVE_FREE,
	
	MOVE_STOP,
	
	//ȡ����
	MOVE_1_LOAD_POINT,
	//ȡ����
	MOVE_2_LOAD_POINT,
	//�价1
	MOVE_1_SHOOT,
	
	// �价2
	MOVE_2_SHOOT,
	//����ط����价
	MOVE_SHOOT,
	//������
	MOVE_1_RESTART,
	
	MOVE_LASER_NEAR,
	
	MOVE_LASER,
	//�ֶ�У׼
	MOVE_MANUAL,
	//�ɵ��ƶ���ȡ����
		MOVE_LOAD,
	
}MOVE_STATE_ITEMS;


typedef enum SHOOTING_STATE_ITEMS
{

control_nineblock_0,//��ʼ״̬
	

control_nineblock_1,//�������1����

control_nineblock_2,//�������2����

control_nineblock_3,//�������3����
	
control_nineblock_4,//�������4����

control_nineblock_5,//ֹͣ���

control_nineblock_6,//�������6����
	
control_nineblock_7,//Զ������	

control_nineblock_8,//���8��ߺ���	

control_nineblock_9,//Զ������
	
}SHOOTING_STATE_ITEMS;




extern MOVE_STATE_ITEMS MOVE_STATE;
extern SHOOTING_STATE_ITEMS SHOOTING_STATE;
extern int direction;

void FSM_Init(void);
void move_FSM(void);
void shoot_FSM(void);
void move(void);
void shoot(void);
void test(void);


#endif
