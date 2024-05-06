//
//JIAlonglong 2023.2.17
//
#include "HareWare.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "comunication.h"
#include "robot.h"
int location_x=0;
int location_y=0;
int location_k=0;
int location_b=0;
//uint8_t * kRxBuffer;
//  uint8_t ch;

KEY_Type KEY_DATA;




/***********************************************��ģң��<PPM>************************************/
//��ģ�ṹ��ʵ��
Air_Contorl  Device;
//�������

uint16_t Time_Sys[4]={0};
uint16_t Microsecond_Cnt=0;

static uint16_t PPM_buf[10]={0};
uint16_t PPM_Databuf[10]={0};
uint8_t ppm_update_flag=0;
uint32_t now_ppm_time_send=0;
uint32_t TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;

#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0

/**
  * ��������: �����ⲿ�жϻص�����
  * �������: GPIO_Pin���ж�����
  * �� �� ֵ: ��
  * ˵    ��: ��
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;//�õ����������½��ص�ʱ��
	if(GPIO_Pin==GPIO_PIN_7)//�ж��Ƿ�Ϊ�������������жϣ���������ΪPIN8
	{
		//ϵͳ����ʱ���ȡ����λus
		last_ppm_time=now_ppm_time;//��ȡ��һ�εĵ�ǰʱ����Ϊ�ϴ�ʱ��
		now_ppm_time_send=now_ppm_time=10000*TIME_ISR_CNT+TIM2->CNT;//us
		ppm_time_delta=now_ppm_time-last_ppm_time;//����õ�һ������ʱ��
		//PPM������ʼ
		if(ppm_ready==1)//�ж�֡����ʱ����ʼ�����µ�һ��PPM
		{
			if(ppm_time_delta>=2200)//֡������ƽ����2ms=2000us�����ڲ����ϰ汾ң������//���ջ����PPM�źŲ���׼�������ֽ����쳣ʱ�����Ը�С��ֵ�������������һ����ʹ����ط��ϰ汾ң����
			{
				//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_ready = 1;
				ppm_sample_cnt=0;//��Ӧ��ͨ��ֵ
				ppm_update_flag=1;
			} 
			else if(ppm_time_delta>=950&&ppm_time_delta<=2050)//����PWM������1000-2000us�������趨900-2100��Ӧ����Ϊ�������ݴ�
			{         
				PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//��Ӧͨ��д�뻺���� 
				if(ppm_sample_cnt>=8)//���ν�������0-7��ʾ8��ͨ���������������ʾ10��ͨ���������ֵӦ��Ϊ0-9�������޸�
				{
					memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));
					//ppm_ready=0;
					ppm_sample_cnt=0;
				}
			}
			else  ppm_ready=0;
		}
		else if(ppm_time_delta>=2200)//֡������ƽ����2ms=2000us
		{
			ppm_ready=1;
			ppm_sample_cnt=0;
			ppm_update_flag=0;
		}
	}
	if(GPIO_Pin==GPIO_PIN_11)
	{
		KEY_DATA.KEY_armtop=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11);
	}
	if(GPIO_Pin==GPIO_PIN_12)
	{
		KEY_DATA.KEY_armbottom=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
	}
	if(GPIO_Pin==GPIO_PIN_13)
	{
		KEY_DATA.KEY_push=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13);
	}
	}


	


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)  // ����Ƿ���TIM2��ʱ���ж�
  {
    LAST_TIME_ISR_CNT = TIME_ISR_CNT;
    TIME_ISR_CNT++;
    Microsecond_Cnt++;
    if (Microsecond_Cnt >= 100)
    {
      Microsecond_Cnt = 0;
      Time_Sys[Second]++;
      if (Time_Sys[Second] >= 60)
      {
        Time_Sys[Second] = 0;
        Time_Sys[Minute]++;
        if (Time_Sys[Minute] >= 60)
        {
          Time_Sys[Minute] = 0;
          Time_Sys[Hour]++;
        }
      }
    }
    Time_Sys[MicroSecond] = Microsecond_Cnt;
  }
}


/***********************************************action************************************/
ACTION_GL_POS ACTION_GL_POS_DATA;
ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0, 0, 0};



//û��action��
//action���ݽṹ��
//float OFFSET_YAW = 0;
////����actionȫ����λ��ֵ
////��ֱ�Ӹ���
//void Update_Action(float value[6])
//{
//	float error;
////������һ�ε�ֵ
//	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
//	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
//// ��¼��ε�ֵ
//	ACTION_GL_POS_DATA.ANGLE_Z = value[0];  // ����
//	ACTION_GL_POS_DATA.ANGLE_X = value[1];
//	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
//	ACTION_GL_POS_DATA.POS_X   = value[3];  // ����
//	ACTION_GL_POS_DATA.POS_Y   = value[4];  // ����
//	ACTION_GL_POS_DATA.W_Z     = value[5];
//	
//	// �������
//	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
//	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
//	
//	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
//	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
//	
//	// ƫ����ֱ�Ӹ�ֵ����ʱ��Ϊ����˳ʱ��Ϊ����
//  ROBOT_REAL_POS_DATA.POS_YAW = ACTION_GL_POS_DATA.ANGLE_Z - OFFSET_YAW;
//	
//	//������е���,��ֵX��Y
//	ROBOT_REAL_POS_DATA.POS_
// ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
//	ROBOT_REAL_POS_DATA.POS_Y = ACTION_GL_POS_DATA.REAL_Y - INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
//	 
//}






//void UART_SendString(USART_TypeDef* USARTx, char *DataString)
//{
//	int i = 0;
//	while(DataString[i] != '\0')												//�ַ���������
//	{
//		HAL_UART_Transmit(&huart4, (uint8_t*)&DataString[i], 1, HAL_MAX_DELAY);//ÿ�η����ַ�����һ���ַ�
//		while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET);					//�ȴ����ݷ��ͳɹ�
//		i++;
//	}
//}






