#ifndef __HAREWARE_H
#define __HAREWARE_H

#include "elmo.h"
#include "stm32f4xx.h"

/* define --------------------------------------------------------------------*/
#define   robot_start 0
#define   robot_load_left 1
#define	  robot_shoot 2
#define   robot_load_right 3

//Functions
extern int location_x;
extern int location_y;
extern int location_k;
extern int location_b;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART_SendString(USART_TypeDef* USARTx, char *DataString);

//struct
typedef struct
{
	struct  //遥控原始数据，8通道
	{
	 uint16_t roll;			//右摇杆
	 uint16_t pitch;		//
	 uint16_t thr;
	 uint16_t yaw;
	 uint16_t AUX1;
	 uint16_t AUX2;
	 uint16_t AUX3;
	 uint16_t AUX4; 
	 uint16_t	BUX1;
	 uint16_t	BUX2;		
	}Remote; 

}Air_Contorl;


typedef struct ACTION_GL_POS//action数据
{
	float ANGLE_Z;
	float ANGLE_X;
	float ANGLE_Y;
	float POS_X;
	float POS_Y;
	float W_Z;

	float LAST_POS_X;
	float LAST_POS_Y;

	float DELTA_POS_X;
	float DELTA_POS_Y;	
	float REAL_X;
	float REAL_Y;
	float OFFSET_YAW;
	
	
} ACTION_GL_POS;

// 机器人的真实位置
typedef struct ROBOT_REAL_POS
{
  float POS_X;
  float POS_Y;     
  float POS_YAW;
	int robot_location;
}ROBOT_REAL_POS;

extern Air_Contorl  Device;
extern ACTION_GL_POS ACTION_GL_POS_DATA;
extern float OFFSET_YAW;
extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;


//define
#define SWA		PPM_Databuf[4]				//AUX4 1000~2000//没用
#define SWB		PPM_Databuf[5]				//AUX2 1000-1500-2000
#define SWD		PPM_Databuf[7]			//AUX1 1000~2000
#define SWC		PPM_Databuf[6]				//AUX3 1000-1500-2000
#define ROCK_R_X			PPM_Databuf[3]					//YAW  1000-1500-2000
#define ROCK_R_Y			PPM_Databuf[2]					//THR  1000-1500-2000
#define ROCK_L_Y			PPM_Databuf[1]				//ROLL 1000-1500-2000//未知bug
#define	ROCK_L_X		  PPM_Databuf[0]				//PITCH 1000-1500-2000P
#define LEFT_BUTTON		Device.Remote.BUX1
#define RIGHT_BUTTON	Device.Remote.BUX2				//注意！！这里修改了两个按键的宏定义
//action数据校准
#define INSTALL_ERROR_Y		0//190.3f
#define INSTALL_ERROR_X		0
#define USART_REC_LEN     200   //定义最大接收字节数 200
#define RXBUFFERSIZE   1 //缓存大小

//微调标志位


//extern
extern Air_Contorl  Device;
extern ACTION_GL_POS ACTION_GL_POS_DATA;
extern float OFFSET_YAW;
extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;
extern unsigned char aRxBuffer[RXBUFFERSIZE];//HAL库USART接收Buffer
extern uint16_t PPM_Databuf[10];//所有通道的数组

#endif
