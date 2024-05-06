/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "moto.h"
#include "MIT.h"
#include "calculation.h"
#include "HareWare.h"
#include "MoveBase.h"
#include "DataScope_DP.h"
#include "FSM.h"
#include "tim.h"
#include "comunication.h"
#include "Communication_STM32.h"
#include "robot.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
short v;
short th;
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
static float a;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for move_base_task */
osThreadId_t move_base_taskHandle;
const osThreadAttr_t move_base_task_attributes = {
  .name = "move_base_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for Robot_state_tas */
osThreadId_t Robot_state_tasHandle;
const osThreadAttr_t Robot_state_tas_attributes = {
  .name = "Robot_state_tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for Auto_task */
osThreadId_t Auto_taskHandle;
const osThreadAttr_t Auto_task_attributes = {
  .name = "Auto_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for data_update_tas */
osThreadId_t data_update_tasHandle;
const osThreadAttr_t data_update_tas_attributes = {
  .name = "data_update_tas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for motor_control_t */
osThreadId_t motor_control_tHandle;
const osThreadAttr_t motor_control_t_attributes = {
  .name = "motor_control_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for SHOWTask07 */
osThreadId_t SHOWTask07Handle;
const osThreadAttr_t SHOWTask07_attributes = {
  .name = "SHOWTask07",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void move_base(void *argument);
void Robot_state(void *argument);
void Auto(void *argument);
void data_update(void *argument);
void motor_control(void *argument);
void StartTask07(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of move_base_task */
  move_base_taskHandle = osThreadNew(move_base, NULL, &move_base_task_attributes);

  /* creation of Robot_state_tas */
  Robot_state_tasHandle = osThreadNew(Robot_state, NULL, &Robot_state_tas_attributes);

  /* creation of Auto_task */
  Auto_taskHandle = osThreadNew(Auto, NULL, &Auto_task_attributes);

  /* creation of data_update_tas */
  data_update_tasHandle = osThreadNew(data_update, NULL, &data_update_tas_attributes);

  /* creation of motor_control_t */
  motor_control_tHandle = osThreadNew(motor_control, NULL, &motor_control_t_attributes);

  /* creation of SHOWTask07 */
  SHOWTask07Handle = osThreadNew(StartTask07, NULL, &SHOWTask07_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	static portTickType move_xLastWakeTime;
	const portTickType move_xFrequency = pdMS_TO_TICKS(10); // 延时10ms
	move_xLastWakeTime = xTaskGetTickCount(); // 获取当前计数值
  for(;;)
  {

		shoot_calculation();
	
		move();

    vTaskDelayUntil(&move_xLastWakeTime, move_xFrequency); // 绝对延时

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_move_base */
/**
* @brief Function implementing the move_base_task thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_move_base */
void move_base(void *argument)
{
  /* USER CODE BEGIN move_base */
	
  /* Infinite loop */
  for(;;)
  {
		VelCrl(&MOTOR_REAL_INFO[0],4000);
				VelCrl(&MOTOR_REAL_INFO[1],-4000);
			VelCrl(&MOTOR_REAL_INFO[2],5000);
				VelCrl(&MOTOR_REAL_INFO[3],-5000);
		
		mit_calculation();
		moto_calculation();//底盘电机计算&&遥控器计算
		osDelay(5);
}
  /* USER CODE END move_base */
}

/* USER CODE BEGIN Header_Robot_state */
/**
* @brief Function implementing the Robot_state_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robot_state */
void Robot_state(void *argument)
{
  /* USER CODE BEGIN Robot_state */
  /* Infinite loop */
  for(;;)
  {

	  	move_FSM();
		shoot_FSM();
			
		shoot();
		//STM32_WRITE_TO_ROS(1, 1,1,0, 0, 0,0,0,0,0);

//	test();
    osDelay(5);
  }
  /* USER CODE END Robot_state */
}

/* USER CODE BEGIN Header_Auto */
/**
* @brief Function implementing the Auto_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Auto */
void Auto(void *argument)
{
  /* USER CODE BEGIN Auto */
//	 static portTickType move_xLastWakeTime;
//	const portTickType move_xFrequency = pdMS_TO_TICKS(10); // 延时10ms
//	move_xLastWakeTime = xTaskGetTickCount(); // 获取当前计数值
  /* Infinite loop */
  for(;;)
  {

		//自动路径和夹取信号
		arm_calculation();
//   vTaskDelayUntil(&move_xLastWakeTime, move_xFrequency); // 绝对延时
		
		 osDelay(1);
  }
  /* USER CODE END Auto */
}

/* USER CODE BEGIN Header_data_update */
/**
* @brief Function implementing the data_update_tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_data_update */
void data_update(void *argument)
{
  /* USER CODE BEGIN data_update */
  /* Infinite loop */
  for(;;)
  {
		//发送数据给底盘
	Usart4_SendData(Robot_Chassis.World_V[1],Robot_Chassis.World_V[0],Robot_Chassis.World_V[2],ROBOT_FLAG.Chassis_send);//xyw



		

	
    osDelay(10);
  }
  /* USER CODE END data_update */
}

/* USER CODE BEGIN Header_motor_control */
/**
* @brief Function implementing the motor_control_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_control */
void motor_control(void *argument)
{
  /* USER CODE BEGIN motor_control */
  /* Infinite loop */
  for(;;)
  {
	 
//	VelCrl(&MOTOR_REAL_INFO[0],7000);
		
		MotorCtrl();//所有的电机的控制
    osDelay(1);
  }
  /* USER CODE END motor_control */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the SHOWTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
		
		
		
		
		
//					a+=0.1;
//			if(a>3.14)  a=-3.14; 
//			DataScope_Get_Channel_Data(MOTOR_REAL_INFO[0].TARGET_POS, 1 );
//			DataScope_Get_Channel_Data(MOTOR_REAL_INFO[0].REAL_ANGLE, 2 );
//			DataScope_Get_Channel_Data(MOTOR_REAL_INFO[0].CURRENT, 3 ); 
//			DataScope_Get_Channel_Data(0 , 4 );   
//			DataScope_Get_Channel_Data(0, 5 );
//			DataScope_Get_Channel_Data(0 , 6 );
//			DataScope_Get_Channel_Data(0, 7 );
//			DataScop0e_Get_Channel_Data( 0, 8 ); 
//			DataScope_Get_Channel_Data(0, 9 );  
//			DataScope_Get_Channel_Data( 0 , 10);
//			Send_Count = DataScope_Data_Generate(10);
//			for( i = 0 ; i < Send_Count; i++) 
//			{
//			while((USART1->SR&0X40)==0);  
//			USART1->DR = DataScope_OutPut_Buffer[i]; 
//			}
    osDelay(10);
  }
  /* USER CODE END StartTask07 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
