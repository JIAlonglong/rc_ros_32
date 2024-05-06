/**
  ******************************************************************************
  * @file    moto.c
  * @author  ������
  * @version V1.1.0
  * @date    2023/3/29
  * @brief   
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "moto.h"

/* Private  variables ---------------------------------------------------------*/
MOTO_REAL_INFO MOTOR_REAL_INFO[8] = {0}; // 1-4�ֱ��Ӧ˳ʱ�뷽��ĵ��̵��
PID MOTOR_PID_RPM[8];	//�ٶ�pid��Ϣ 1-4���̵�� 5-6������
PID MOTOR_PID_POS[8];	//λ��pid��Ϣ

PID laser_X_pid;//����pid
PID laser_Y_pid;
PID laser_K_pid;
PID laser_B_pid;

MOTOR_RPM MOTOR_TARGET_RPM;
MOTOR_POS MOTOR_TARGET_POS;


/**
  * @brief  M3508��ʼ��
	* @param  None
	* @retval None
  * @attention
  *�Ƚ����е����ʼ����3508������
  */
void M3508_Motor_Init(void)
{
	//���Ȼ�pid
	//PID *pp, float Kp, float Ki, float Kd, float outputmax, float Integralmax, float deadzone
	
	PID_parameter_init(&MOTOR_PID_RPM[0], 12.0f, 0.4f, 0.1f, 20000, 20000, -0.5);
	PID_parameter_init(&MOTOR_PID_RPM[1], 12.0f, 0.4f, 0.1f, 20000, 20000, -0.5);
	PID_parameter_init(&MOTOR_PID_RPM[2], 12.0f, 0.4f, 0.1f, 20000, 20000, -0.5);
	// ������PID��ʼ��
	PID_parameter_init(&MOTOR_PID_RPM[3], 12.0f, 1.0f, 0.1f, 20000, 20000, -0.5);
	PID_parameter_init(&MOTOR_PID_RPM[4], 12.0f, 1.0f, 0.1f, 20000, 20000, -0.5);

	
	// arm���
	PID_parameter_init(&MOTOR_PID_RPM[5] , 12.0f, 1.0f, 0.1f, 20000, 20000, 1);
	//2006
	//��צ���
	PID_parameter_init(&MOTOR_PID_RPM[6] , 12.0f, 0.4f, 0.0f, 7000, 16384, 1);
	//������
	PID_parameter_init(&MOTOR_PID_RPM[7] , 12.0f, 1.0f, 0.1f, 16384, 16384, 1);
	
	//λ�û�pid
	PID_parameter_init(&MOTOR_PID_POS[0] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[1] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[2] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[3] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[4] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[5] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[6] , 100, 0, 1, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[7] , 100, 0, 1, 7000, 7000, 0.05);
	
	//�������
	MOTOR_REAL_INFO[0].type = RM_3508;
	MOTOR_REAL_INFO[1].type = RM_3508;
	MOTOR_REAL_INFO[2].type = RM_3508;
	MOTOR_REAL_INFO[3].type = RM_3508;
	MOTOR_REAL_INFO[4].type = RM_3508;
	MOTOR_REAL_INFO[5].type = RM_3508;
	MOTOR_REAL_INFO[6].type = RM_3508;
	MOTOR_REAL_INFO[7].type = RM_3508;
	
	//���ģʽ
	MOTOR_REAL_INFO[0].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[1].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[2].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[3].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[4].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[5].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[6].unitMode = MOTO_OFF;
	MOTOR_REAL_INFO[7].unitMode = MOTO_OFF;
	//�������ģʽ�����޷�
	MOTOR_REAL_INFO[0].homingMode.current = 3000.0f;
	MOTOR_REAL_INFO[1].homingMode.current = 3000.0f;
	MOTOR_REAL_INFO[2].homingMode.current = 3000.0f;
	MOTOR_REAL_INFO[3].homingMode.current = 3000.0f;
	MOTOR_REAL_INFO[4].homingMode.current = 3000.0f;
	MOTOR_REAL_INFO[5].homingMode.current = 3000.0f;
	MOTOR_REAL_INFO[6].homingMode.current = 2000.0f;
	MOTOR_REAL_INFO[7].homingMode.current = 3000.0f;
	//�������ģʽ�ٶȣ�ע��������
	MOTOR_REAL_INFO[0].homingMode.vel = -160.0f;
	MOTOR_REAL_INFO[1].homingMode.vel = -160.0f;
	MOTOR_REAL_INFO[2].homingMode.vel = -160.0f;
	MOTOR_REAL_INFO[3].homingMode.vel = -160.0f;
	MOTOR_REAL_INFO[4].homingMode.vel = -160.0f;
	MOTOR_REAL_INFO[5].homingMode.vel = -160.0f;
	MOTOR_REAL_INFO[6].homingMode.vel = -1000.0f;
	MOTOR_REAL_INFO[7].homingMode.vel = -160.0f;
}


// ���õ��ͨ��CAN���������ݸ���m3508��״̬��Ϣ
// ����Ƶ�ʣ�1kHz
//2023/3/15LLy���Ը���
void m3508_update_m3508_info(CAN_RxHeaderTypeDef *msg,uint8_t	can1_RxData[8])
{
	switch(msg -> StdId)  // ����׼ID
	{
    case M3508_CHASSIS_MOTOR_ID_1:
		{ 
			MOTOR_REAL_INFO[0].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_2:
		{ 
			MOTOR_REAL_INFO[1].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_3:
		{ 
			MOTOR_REAL_INFO[2].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			MOTOR_REAL_INFO[2].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			MOTOR_REAL_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;	
		
		case M3508_CHASSIS_MOTOR_ID_4:
		{ 
			MOTOR_REAL_INFO[3].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			MOTOR_REAL_INFO[3].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			MOTOR_REAL_INFO[3].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		

		case M3508_CHASSIS_MOTOR_ID_5:
		{ 
			MOTOR_REAL_INFO[4].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			MOTOR_REAL_INFO[4].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			MOTOR_REAL_INFO[4].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		case M3508_CHASSIS_MOTOR_ID_6:
		{ 
		  MOTOR_REAL_INFO[5].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			MOTOR_REAL_INFO[5].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			MOTOR_REAL_INFO[5].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		
		case M2006_CHASSIS_MOTOR_ID_0://2006������ұߣ�
		{ 
			MOTOR_REAL_INFO[6].ANGLE= (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			MOTOR_REAL_INFO[6].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			MOTOR_REAL_INFO[6].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���

		}; break;

		case M2006_CHASSIS_MOTOR_ID_1://2006�������ߣ�
		{ 
			MOTOR_REAL_INFO[7].ANGLE= (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			MOTOR_REAL_INFO[7].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			MOTOR_REAL_INFO[7].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
	
		}; break;

		default: break;
	}
}



//���͵���
void chassis_m3508_send_motor_currents(void)
{
	
	/***********************************����IDΪ 1 2 3 4 �ĵ��*********************************/
	CAN_TxHeaderTypeDef tx_message_1;
  uint8_t send_buf1[8] = {0};
	uint32_t msg_box1;
	// ���ÿ��ƶ�
	tx_message_1.IDE = CAN_ID_STD;//���ĵ�11λ��׼��ʶ��CAN_ID_STD��ʾ�������Ǳ�׼֡
	tx_message_1.RTR = CAN_RTR_DATA;//�������ͱ�־RTRλCAN_ID_STD��ʾ�����ĵ�����֡
	tx_message_1.DLC = 0x08;//���ݶγ���
	tx_message_1.TransmitGlobalTime = DISABLE;
	// �����ٲöκ����ݶ�	
	tx_message_1.StdId = 0x200;  // ����IDΪ 1 2 3 4 �ĵ��
	
	send_buf1[0] = (uint8_t)(MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	send_buf1[1] = (uint8_t) MOTOR_REAL_INFO[0].TARGET_CURRENT;
	send_buf1[2] = (uint8_t)(MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	send_buf1[3] = (uint8_t) MOTOR_REAL_INFO[1].TARGET_CURRENT;
	send_buf1[4] = (uint8_t)(MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	send_buf1[5] = (uint8_t) MOTOR_REAL_INFO[2].TARGET_CURRENT;
	send_buf1[6] = (uint8_t)(MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	send_buf1[7] = (uint8_t) MOTOR_REAL_INFO[3].TARGET_CURRENT;

	                                                                                                                                                                                          
	    if (HAL_CAN_AddTxMessage(&hcan1,&tx_message_1,send_buf1,&msg_box1)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }		
	/***********************************����IDΪ 5 6 7 8 �ĵ��*********************************/
	CAN_TxHeaderTypeDef tx_message_2;
	uint8_t send_buf2[8] = {0};
	uint32_t msg_box2;
	
	// ���ÿ��ƶ�
	tx_message_2.IDE = CAN_ID_STD;//���ĵ�11λ��׼��ʶ��CAN_ID_STD��ʾ�������Ǳ�׼֡
	tx_message_2.RTR = CAN_RTR_DATA;//�������ͱ�־RTRλCAN_ID_STD��ʾ�����ĵ�����֡
	tx_message_2.DLC = 0x08;//���ݶγ���
	tx_message_2.TransmitGlobalTime = DISABLE;
	
	// �����ٲöκ����ݶ�
	tx_message_2.StdId = 0x1FF;  // ����IDΪ 5 6 7 8 �ĵ��(������3508����2006)
	send_buf2[0] = (uint8_t)(MOTOR_REAL_INFO[4].TARGET_CURRENT >> 8);
	send_buf2[1] = (uint8_t) MOTOR_REAL_INFO[4].TARGET_CURRENT;	      //ID 5 
	send_buf2[2] = (uint8_t)(MOTOR_REAL_INFO[5].TARGET_CURRENT >> 8);
	send_buf2[3] = (uint8_t) MOTOR_REAL_INFO[5].TARGET_CURRENT;	      //ID 6 cast
	send_buf2[4] = (uint8_t)( MOTOR_REAL_INFO[6].TARGET_CURRENT >> 8 ) ;
  send_buf2[5] = (uint8_t) MOTOR_REAL_INFO[6].TARGET_CURRENT; //ID 7
  send_buf2[6] = (uint8_t)( MOTOR_REAL_INFO[7].TARGET_CURRENT >> 8 ) ;
  send_buf2[7] = (uint8_t) MOTOR_REAL_INFO[7].TARGET_CURRENT; //ID 8
	
	
	if (HAL_CAN_AddTxMessage(&hcan1,&tx_message_2,send_buf2,&msg_box2)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }
}


/**
  * @brief  MotorCtrl������ݼ���
	* @param  None
	* @retval None
  */

void MotorCtrl(void)
{

	for(int i = 0; i < 8; i++)
	{
		if(MOTOR_REAL_INFO[i].type == NONE)
			break;
		
		
	//�ٶ�λ�ü���
			
		switch(MOTOR_REAL_INFO[i].unitMode)
		{
			case POSITION_CONTROL_MODE://λ��ģʽ
				PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_PID_POS[i].output);//�ٶȻ�
				PID_position_PID_calculation(&MOTOR_PID_POS[i], MOTOR_REAL_INFO[i].REAL_ANGLE, MOTOR_REAL_INFO[i].TARGET_POS);//λ�û�
				
				//MOTOR_REAL_INFO[i].TARGET_CURRENT =MOTOR_PID_RPM[i].output;//������ֵ
			
				break;
			case SPEED_CONTROL_MODE://�ٶ�ģʽ
        PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM);//�ٶȻ�
				break;
			case HOMEINGMODE://У׼ģʽ
	      HomingMode(&MOTOR_REAL_INFO[i]);
			  PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM);//�ٶȻ�
			  MOTOR_PID_RPM[i].output = MaxMinLimit(MOTOR_PID_RPM[i].output,MOTOR_REAL_INFO[i].homingMode.current);//����homeģʽʱ����ֵ
				break;
			case VELOCITY_PLANNING_MODE://����ģʽ
				VelocityPlanningMODE(&MOTOR_REAL_INFO[i]);
				PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM);//�ٶȻ�
				break;
			case POSITION_TORQUE_MODE://λ��ת��ģʽ
	     
			  PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_PID_POS[i].output);//�ٶȻ�
				PID_position_PID_calculation(&MOTOR_PID_POS[i], MOTOR_REAL_INFO[i].REAL_ANGLE, MOTOR_REAL_INFO[i].pos_torquemode.Pos);//λ�û�
			  MOTOR_PID_RPM[i].output = MaxMinLimit(MOTOR_PID_RPM[i].output,MOTOR_REAL_INFO[i].pos_torquemode.TARGET_TORQUE);//����ת��ģʽʱ����ֵ
			  //�ж��Ƿ��ȡ
			  //flag = 1��ȡ�ɹ�
			  if(fabsf(MOTOR_REAL_INFO[i].RPM) <=10){		//2
		      MOTOR_REAL_INFO[i].pos_torquemode.cnt++;
	        }
				else
				 {
		     MOTOR_REAL_INFO[i].pos_torquemode.cnt = 0;
	       }
		
				if(MOTOR_REAL_INFO[i].pos_torquemode.cnt>=100)//100ms
				{
					MOTOR_REAL_INFO[i].pos_torquemode.cnt = 0;
					MOTOR_REAL_INFO[i].pos_torquemode.flag = 1;
					
				}
				break;
			case SPEED_TARQUE_CONTROL_MODE://�ٶ�ת��ģʽ
				PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].vel_torquemode.Vel);//�ٶȻ�
				MOTOR_PID_RPM[i].output = MaxMinLimit(MOTOR_PID_RPM[i].output,MOTOR_REAL_INFO[i].vel_torquemode.TARGET_TORQUE);//����ת��ģʽʱ����ֵ
			  //�ж��Ƿ��ȡ
			  //flag = 1��ȡ�ɹ�
			  if(fabsf(MOTOR_REAL_INFO[i].RPM) <=10){		//2
		      MOTOR_REAL_INFO[i].vel_torquemode.cnt++;
	        }
				else
				 {
		     MOTOR_REAL_INFO[i].vel_torquemode.cnt = 0;
	       }
		
				if(MOTOR_REAL_INFO[i].vel_torquemode.cnt>=50)//500ms
				{
					MOTOR_REAL_INFO[i].vel_torquemode.cnt = 0;
					MOTOR_REAL_INFO[i].vel_torquemode.flag = 1;
//					MOTOR_REAL_INFO[i].unitMode= SPEED_CONTROL_MODE;
//					MOTOR_REAL_INFO[i].TARGET_RPM = 0;
				}
					break;
				
			case CURRENT_MODE://����ģʽ
				
				//ʲô����ִ�У�ֱ�ӵ�����ֵ
				break;
				
			case MOTO_OFF://����ر�
				MOTOR_REAL_INFO[i].TARGET_CURRENT = 0.0f;//������ֵ
				break;
			default:break;
		}

	}
	
	for(int i = 0; i < 8; i++)
			{
				if(MOTOR_REAL_INFO[i].type == NONE)
			break;
			}
			
	for(int i = 0; i < 8; i++)
			{
				if(MOTOR_REAL_INFO[i].unitMode == CURRENT_MODE){//����ģʽ�µ��������
			}
			else{
				if(MOTOR_REAL_INFO[i].type == RM_3508)
				MOTOR_REAL_INFO[i].TARGET_CURRENT =  MOTOR_PID_RPM[i].output*16384.0f/20000.0f;//M3508��λ����
		
				else if(MOTOR_REAL_INFO[i].type == M_2006)
				MOTOR_REAL_INFO[i].TARGET_CURRENT =  MOTOR_PID_RPM[i].output*10000.0f/10000.0f;  //M2006��λ����
				else 
				MOTOR_REAL_INFO[i].TARGET_CURRENT = 0.0f;
			}
			}
	chassis_m3508_send_motor_currents();
	
}
/**
  * @brief  �ٶȿ���
  * @param   target_velĿ���ٶ�
  * @retval 
  */
float CurrentCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_current)
{
	MOTOR_REAL_INFO->unitMode = CURRENT_MODE;
  MOTOR_REAL_INFO->TARGET_CURRENT = target_current;
	
	return 0;
}

/**
  * @brief  �ٶȿ���
  * @param   target_velĿ���ٶ�
  * @retval 
  */
float VelCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_vel)
{
	MOTOR_REAL_INFO->unitMode = SPEED_CONTROL_MODE;
  MOTOR_REAL_INFO->TARGET_RPM = target_vel;
	
	return 0;
}

/**
  * @brief  λ�ÿ���(��λ�û�����)
  * @param  target_posĿ��λ��
  * @retval 
  */
float PosCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_pos)
{
	MOTOR_REAL_INFO->unitMode = POSITION_CONTROL_MODE;
	MOTOR_REAL_INFO->TARGET_POS = target_pos;
	if(ABS(MOTOR_REAL_INFO->TARGET_POS-target_pos)<1)return 1;	
	else return 0;

}

/**
  * @brief  Homing mode ����ģʽ
  * @param  None
  * @retval None
  */

void HomingMode(MOTO_REAL_INFO *MOTOR_REAL_INFO)
{
	int signvel=1.0f;
	float output;
	MOTOR_REAL_INFO->homingMode.flag=0;	
  if(MOTOR_REAL_INFO[0].homingMode.vel>=0)signvel=-1.0f;
		
	MOTOR_REAL_INFO->TARGET_RPM = MOTOR_REAL_INFO->homingMode.vel;

	
	if(fabsf(MOTOR_REAL_INFO->RPM) <=30){		//2
		MOTOR_REAL_INFO->homingMode.cnt++;
	}else{
		MOTOR_REAL_INFO->homingMode.cnt = 0;
	}
	
	if(MOTOR_REAL_INFO->homingMode.cnt >= 50)//500ms
	{														
		//MOTOR_REAL_INFO->TARGET_POS = MOTOR_REAL_INFO->REAL_ANGLE + signvel*18.0f;//����һȦ
		//������
		MOTOR_REAL_INFO->homingMode.cnt = 0;
		MOTOR_REAL_INFO->REAL_ANGLE=0.0f;	
		MOTOR_REAL_INFO->homingMode.flag=1;
		MOTOR_REAL_INFO->unitMode = SPEED_CONTROL_MODE;
    MOTOR_REAL_INFO->TARGET_RPM = 0;
	}
}


/**
  * @brief  
	* @param  target_torqueĿ��ת�أ��õ�����ʾ
	          target_posĿ��λ��
	* @retval none
  */
void Pos_TorqueCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,int16_t target_torque,float target_pos)
{
	
  MOTOR_REAL_INFO->unitMode = POSITION_TORQUE_MODE;
	MOTOR_REAL_INFO->pos_torquemode.Pos = target_pos;
	MOTOR_REAL_INFO->pos_torquemode.TARGET_TORQUE = target_torque;
}

/**
  * @brief  
	* @param  target_torqueĿ��ת�أ��õ�����ʾ
	          target_velĿ��λ��
	* @retval none
  */
void Vel_TorqueCtrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,int16_t target_torque,float target_Vel)
{	
  MOTOR_REAL_INFO->unitMode = SPEED_TARQUE_CONTROL_MODE;
	MOTOR_REAL_INFO->vel_torquemode.Vel = target_Vel;
	MOTOR_REAL_INFO->vel_torquemode.TARGET_TORQUE = target_torque;
}

/**
  * @brief  max min limit
	* @param  inDat:
	* @retval outDat
  */
float MaxMinLimit(float val,float limit)
{
	if(val > limit) val =  limit;
	if(val <-limit) val = -limit;
	
	return val;
}
/**
  * @brief  α�ݶȹ滮λ�ÿ��ƣ����������λ��������λ������ӳ�
	* @param  //����ṹ��  pp_po  pp2  3508_REAL_INFO
//�ٶ�΢���޷�  v_xian
//Ŀ��λ�� tar_position
//������λֵ  current_key
//��ʱ  dalay_time
//������һ��״̬�ı�־λ  next_flag
//��һ�ν���ı�־λ  first_flag,ÿ��ʹ�õ�ʱ�����ñ�־λ ��1
	* @retval none
Ҷ����д�ģ�����������
*/

float position_control_an_delay(PID *pp_po,PID *pp_rpm,MOTO_REAL_INFO *info,float v_xian,float tar_position,float current_key,int dalay_time,int next_flag,int first_flag)
{ 
 static int PCAD_tim,PCAD_tim2,PCAD_STOP;
 if(first_flag==1){PCAD_tim = 0;PCAD_STOP = 0;PCAD_STOP = 0;}//?*?�����ڲ�pid��û������
 else
	 {
  
  if(PCAD_STOP==1){tar_position = info->REAL_ANGLE;}
  PID_position_PID_calculation(pp_po, info->REAL_ANGLE ,tar_position);
  if(pp_po->output >= v_xian)pp_po->output = v_xian;
  if(pp_po->output <= -v_xian)pp_po->output = -v_xian;
  PID_incremental_PID_calculation(pp_rpm, info->RPM ,pp_po->output);
   /* ��������޷�  ����ѭ�� */
   if(info->CURRENT >= current_key   &&   info->CURRENT <= -current_key  )
   {
    if( PCAD_tim<=4){PCAD_tim++;}
    else {PCAD_tim =0;PCAD_STOP = 1;}
   }
   else {PCAD_tim =0;}
   /* ��ʱ */
   if(PCAD_STOP==1)
   {if(PCAD_tim2 <=dalay_time){PCAD_tim2++;}
    else {PCAD_tim2 =0;next_flag = 1;}
   }
   
  }
 return 0 ;
}

//M3508����ǶȻ���
void M3508AngleIntegral(MOTO_REAL_INFO *M3508_MOTOR)
{
	float delta_pos = 0;
	
	// ��¼��һ�ν���ʱ������
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// ����仯�ĽǶ�
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 19;	//���ٱ�
			}
		}
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//���ٱ�
		}
		
		// �˲�
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����	
	}
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos /19;	//���ٱ�
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//���ٱ�
		}
		
		// �˲�
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����
	}

	// �洢�Ƕ�ֵ 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}

/**
  * @brief  �ٶȹ滮
	* @param  
	* @retval none
  */
//                                                   ��ʼλ��   ����λ��    ��ʼ���ٶ�(RPM ����ֵ)  �����ٶ�	ĩβ���ٶ�   ����·�̵ı���   ����·�̵ı���
void VelocityPlanningMODE(MOTO_REAL_INFO *M3508_MOTOR)	
{
	static int cnt;//��ʱ��
	float Ssu;   //��·��
	float Sac;   //����·��
	float Sde;   //����·��
	float Sco;   //����·��
	float Aac;   //���ټ��ٶ�
	float Ade;   //���ټ��ٶ�
	float S;     //��ǰ·��
	// �����������������ִ���ٶȹ滮		
	if((M3508_MOTOR->velocity_planning.Rac > 1) || (M3508_MOTOR->velocity_planning.Rac < 0) ||		//����·�̵ı���
		 (M3508_MOTOR->velocity_planning.Rde > 1) || (M3508_MOTOR->velocity_planning.Rde < 0) ||	//����·�̵ı���
		 (M3508_MOTOR->velocity_planning.Vmax < M3508_MOTOR->velocity_planning.Vstart) )			//�����ٶ�<��ʼ���ٶ� 
	{
		M3508_MOTOR->TARGET_RPM = 0;  // ���צ���˶�
		return;
	}
	// ����ģʽ
	if(M3508_MOTOR->velocity_planning.Pstart == M3508_MOTOR->velocity_planning.Pend)	//��ʼλ��=����λ��
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vmax;	//��ʼ���ٶ�*�����ٶ�
		return;
	}
	
	// ����һЩ����
	Ssu = ABS(M3508_MOTOR->velocity_planning.Pend - M3508_MOTOR->velocity_planning.Pstart); 	//��·��   
	Sac = Ssu * M3508_MOTOR->velocity_planning.Rac;		//����·�� =	��·�� * ����·�̵ı���
	Sde = Ssu * M3508_MOTOR->velocity_planning.Rde;		//����·�� =	��·�� * ����·�̵ı���
	Sco = Ssu - Sac - Sde;		//����·�� = ��·�� - ����·�� - ����·��
	Aac = (M3508_MOTOR->velocity_planning.Vmax * M3508_MOTOR->velocity_planning.Vmax - M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart) / (2.0f * Sac);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
	Ade = (M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend -   M3508_MOTOR->velocity_planning.Vmax *   M3508_MOTOR->velocity_planning.Vmax) / (2.0f * Sde);	  
	
	// �����쳣���
	if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pstart)) ||		//[(����λ�� > ��ʼλ��) && (���������ʵ�Ƕ�pos <��ʼλ��)]	||
		 ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pstart)))		//	[(����λ�� < ��ʼλ��) && (���������ʵ�Ƕ�pos >��ʼλ��)]
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart;	//TARGET_RPM = ��ʼ���ٶ�
	}
	else if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pend)) ||
		      ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pend)))
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vend;	//TARGET_RPM = ĩβ���ٶ�
	}
	else
	{
		S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pstart);      //��ʼλ��
		
		// �滮RPM
		if     (S < Sac)       M3508_MOTOR->TARGET_RPM = sqrt(2.0f * Aac * S + M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart);               // ���ٽ׶�
		else if(S < (Sac+Sco)) M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vmax;                                                        // ���ٽ׶�
		else                   M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend - 2.0f * Ade * ABS(Ssu - S));  // ���ٽ׶�
	}
	 
	// ������ʵ�������
	if(M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
	//�ж��Ƿ����
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) < 3)
		M3508_MOTOR->velocity_planning.flag = 1;//���ñ�־λ			
						
   if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) > 3)
			M3508_MOTOR->velocity_planning.flag = 0;
}

/**
  * @brief  �����ٶȹ滮�Ĳ����������ٶȹ滮����
	* @param  
	* float Pstart;        //��ʼλ��
	* float Pend;          //����λ��
	* float Vstart;        //��ʼ���ٶ�           // ��λ��RPM ����ֵ
	* float Vmax;          //�����ٶ�
	* float Vend;          //ĩβ���ٶ�
	* float Rac;           //����·�̵ı���
	* float Rde;           //����·�̵ı���
	* @retval none
  */
void Velocity_Planning_setpos(MOTO_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde)
{
	M3508_MOTOR->unitMode = VELOCITY_PLANNING_MODE;//����ģʽ
	M3508_MOTOR->velocity_planning.Pstart = Pstart;
	M3508_MOTOR->velocity_planning.Pend = Pend;
	M3508_MOTOR->velocity_planning.Vstart = Vstart;
	M3508_MOTOR->velocity_planning.Vmax = Vmax;
	M3508_MOTOR->velocity_planning.Vend = Vend;
	M3508_MOTOR->velocity_planning.Rac = Rac;
	M3508_MOTOR->velocity_planning.Rde = Rde;
	
}
/**
  * @brief  ��ͨ�˲���
	* @param  input��ǰ�����ź�
  *         prev_output��һ������ź�
  *         prev_input��һ�������ź�
  *         cutoff_freq��ֹƵ��
  *         sample_rate������
	* @retval none
  */

double filter(double input, double prev_output, double prev_input, double cutoff_freq, double sample_rate) {
    double RC = 1.0 / (2.0 * PI * cutoff_freq);
    double alpha = 1.0 / (1.0 + RC * sample_rate);
    double output = alpha * (input + prev_input) + (1 - alpha) * prev_output;
    return output;
}



