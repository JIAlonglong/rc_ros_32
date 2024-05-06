/**
  ******************************************************************************
  * @file    robot.c
  * @author  ������
  * @version V1.0.0
  * @date    2023/5/17
  * @brief   
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "moto.h"
#include "robot.h"
#include "cmsis_os.h"
#include "comunication.h"
#include "tim.h"
#include "calculation.h"
/* Private  variables ---------------------------------------------------------*/



//��צ��� ID:6
// arm��� ID:5
/**********************
����Ϊ0
����Ϊ
homeΪ150
*********************/

//1
//���м��½�
int Larm_down(void)
{
//	SHOOT_DATA.gun_state = 0;//�����Է���
//	if(MOTOR_REAL_INFO[5].CURRENT<10000&&KEY_DATA.KEY_armbottom==1){//�������
//	Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],60,250,100,1000,100,0.3,0.4);//�½��滮��ĩ�ٶȲ�Ϊ0
//	return 0;
//	}
//	
//	else{
//	VelCrl(&MOTOR_REAL_INFO[5],0);	
//	MOTOR_REAL_INFO[5].REAL_ANGLE =265;
//	
//	return 1;
	
SHOOT_DATA.gun_state = 0;//�����Է���
	if((MOTOR_REAL_INFO[5].CURRENT>2000&&KEY_DATA.KEY_armbottom==0)||MOTOR_REAL_INFO[5].CURRENT>7000){//�������
	VelCrl(&MOTOR_REAL_INFO[5],0);	
	MOTOR_REAL_INFO[5].REAL_ANGLE =265;
	return 1;	
	}
		else{
  Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],60,250,100,2000,500,0.3,0.4);//�½��滮��ĩ�ٶȲ�Ϊ0
	return 0;
	}
}

//��һ���½�
int Larm_down_first(void)
{

	
	
	SHOOT_DATA.gun_state = 0;//�����Է���
	if((MOTOR_REAL_INFO[5].CURRENT>2000&&KEY_DATA.KEY_armbottom==0)||MOTOR_REAL_INFO[5].CURRENT>7000){//�������
	VelCrl(&MOTOR_REAL_INFO[5],0);	
	MOTOR_REAL_INFO[5].REAL_ANGLE =265;
	return 1;
	}
	
	else{
	
  Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],0,250,100,2000,500,0.3,0.4);//�½��滮��ĩ�ٶȲ�Ϊ0
	return 0;
	}
}
//2
//�ȴ�У׼
int Larm_wait(void)
{
VelCrl(&MOTOR_REAL_INFO[5],0);
	return 0;
}
//2
int Lclamp_close(void)
{
	Vel_TorqueCtrl(&MOTOR_REAL_INFO[6],5000,-3000);
	if(MOTOR_REAL_INFO[6].CURRENT<-3500)
		{						
			VelCrl(&MOTOR_REAL_INFO[6],0);
			MOTOR_REAL_INFO[6].vel_torquemode.flag = 0;
			return 1;							
		}
			else return 0;	
}

//3
int Larm_raise(void)
{

	static int cast_cnt4;
	if(MOTOR_REAL_INFO[5].CURRENT<-2000&&KEY_DATA.KEY_armtop==0){//�������
	VelCrl(&MOTOR_REAL_INFO[5],0);	
	MOTOR_REAL_INFO[5].REAL_ANGLE =0;
	return 1;	
	}
	
	
	 if(MOTOR_REAL_INFO[5].CURRENT<-5000)
	 {
	 cast_cnt4++;
		 if(cast_cnt4>50)
		 {
			 VelCrl(&MOTOR_REAL_INFO[5],0);	
	     MOTOR_REAL_INFO[5].REAL_ANGLE =0;
	     return 1;	
		 }
	 }
		
	  if(ABS(MOTOR_REAL_INFO[5].CURRENT)<10000&&KEY_DATA.KEY_armtop==1)
			{
			cast_cnt4=0;
       Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],265,0,100,1000,100,0.3,0.4);//�½��滮��ĩ�ٶȲ�Ϊ0	
		return 0;
	}
}     

//4
int Lclamp_open(void)
{
	
	Vel_TorqueCtrl(&MOTOR_REAL_INFO[6],5000,3000);
	if(MOTOR_REAL_INFO[6].CURRENT>4000)
	{
				CurrentCrl(&MOTOR_REAL_INFO[6],0);
				MOTOR_REAL_INFO[6].vel_torquemode.flag = 0;
		    MOTOR_REAL_INFO[5].velocity_planning.flag=0;
				MOTOR_REAL_INFO[6].REAL_ANGLE = 0;			
				return 1;
				}
	else return 0;
}


//5
int Larm_down_to_home(void)
{
	Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],0,60,100,500,0,0.4,0.3);//�½��滮��ĩ�ٶȲ�Ϊ0
	if(MOTOR_REAL_INFO[5].velocity_planning.flag == 1) 
	{	
	VelCrl(&MOTOR_REAL_INFO[5],0);
	SHOOT_DATA.gun_state = 1;//���Է���
	return 1;
	}
	else return 0;
} 


//0
int Larm_home(void)
{
	
	SHOOT_DATA.gun_state = 1;//���Է���
	return 0;
}


/**
  * @brief  �ƻ���������
	* @param  None
	* @retval None
  * @attention
  */
void shoot_push(void)
{			
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 2000);
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
    osDelay(500);
		SHOOT_DATA.push_state = push;
}
void shoot_back(void)
{
		SHOOT_DATA.push_state = back;
		osDelay(500);
		__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 500);
		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 2500);
}
	

//·������

