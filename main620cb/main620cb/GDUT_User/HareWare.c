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




/***********************************************航模遥控<PPM>************************************/
//航模结构体实例
Air_Contorl  Device;
//定义变量

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
  * 函数功能: 按键外部中断回调函数
  * 输入参数: GPIO_Pin：中断引脚
  * 返 回 值: 无
  * 说    明: 无
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;//得到上升沿与下降沿的时间
	if(GPIO_Pin==GPIO_PIN_7)//判断是否为接收器产生的中断，例程设置为PIN8
	{
		//系统运行时间获取，单位us
		last_ppm_time=now_ppm_time;//获取上一次的当前时间作为上次时间
		now_ppm_time_send=now_ppm_time=10000*TIME_ISR_CNT+TIM2->CNT;//us
		ppm_time_delta=now_ppm_time-last_ppm_time;//相减得到一个周期时间
		//PPM解析开始
		if(ppm_ready==1)//判断帧结束时，开始解析新的一轮PPM
		{
			if(ppm_time_delta>=2200)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、//接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
			{
				//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_ready = 1;
				ppm_sample_cnt=0;//对应的通道值
				ppm_update_flag=1;
			} 
			else if(ppm_time_delta>=950&&ppm_time_delta<=2050)//单个PWM脉宽在1000-2000us，这里设定900-2100，应该是为了提升容错
			{         
				PPM_buf[ppm_sample_cnt++]=ppm_time_delta;//对应通道写入缓冲区 
				if(ppm_sample_cnt>=8)//单次解析结束0-7表示8个通道。我这里可以显示10个通道，故这个值应该为0-9！！待修改
				{
					memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));
					//ppm_ready=0;
					ppm_sample_cnt=0;
				}
			}
			else  ppm_ready=0;
		}
		else if(ppm_time_delta>=2200)//帧结束电平至少2ms=2000us
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
  if (htim->Instance == TIM2)  // 检查是否是TIM2定时器中断
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



//没有action啦
//action数据结构体
//float OFFSET_YAW = 0;
////更新action全场定位的值
////不直接跟新
//void Update_Action(float value[6])
//{
//	float error;
////储存上一次的值
//	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
//	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
//// 记录这次的值
//	ACTION_GL_POS_DATA.ANGLE_Z = value[0];  // 有用
//	ACTION_GL_POS_DATA.ANGLE_X = value[1];
//	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
//	ACTION_GL_POS_DATA.POS_X   = value[3];  // 有用
//	ACTION_GL_POS_DATA.POS_Y   = value[4];  // 有用
//	ACTION_GL_POS_DATA.W_Z     = value[5];
//	
//	// 差分运算
//	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
//	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
//	
//	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
//	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
//	
//	// 偏航角直接赋值（逆时针为正，顺时针为负）
//  ROBOT_REAL_POS_DATA.POS_YAW = ACTION_GL_POS_DATA.ANGLE_Z - OFFSET_YAW;
//	
//	//消除机械误差,赋值X、Y
//	ROBOT_REAL_POS_DATA.POS_
// ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
//	ROBOT_REAL_POS_DATA.POS_Y = ACTION_GL_POS_DATA.REAL_Y - INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
//	 
//}






//void UART_SendString(USART_TypeDef* USARTx, char *DataString)
//{
//	int i = 0;
//	while(DataString[i] != '\0')												//字符串结束符
//	{
//		HAL_UART_Transmit(&huart4, (uint8_t*)&DataString[i], 1, HAL_MAX_DELAY);//每次发送字符串的一个字符
//		while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET);					//等待数据发送成功
//		i++;
//	}
//}






