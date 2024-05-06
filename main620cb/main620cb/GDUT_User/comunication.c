/**
  ******************************************************************************
  * @file    comunication.c
  * @author  ������ lly
  * @version V1.2.0
  * @date    2023/4/5
  * @brief   
  ******************************************************************************
  */ 

#include "comunication.h"
#include "HareWare.h" 
#include "moto.h"
#include "MoveBase.h"
#include "calculation.h"
#include "FSM.h"
#include "robot.h"
#include "Communication_STM32.h"


float data1=0;
float data2=0;
float data3=0;
float data4=0;
float data5=0;
float data6=0;
float data7=0;
float data8=0;
float data9=0;
float data10=0;


//���ݽ����ݴ���
//unsigned char  receiveBuff[50] = {0};
//unsigned char  buf[50] = {0};//������
unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};
//��������
unsigned char USART1_Receiver             = 0;          
//unsigned char USART2_Receiver             = 0;          
//unsigned char USART3_Receiver             = 0;          
unsigned char UART4_Receiver             = 0;          
unsigned char UART5_Receiver             = 0;          
unsigned char USART6_Receiver             = 0;          
union receiveData1
{
	int d;
	unsigned char data[4];
}x_position,y_position,k_position,b_position;


/**
  * @brief  �����λѭ������У�飬��usartSendData��usartReceiveOneData��������
  * @param   �����ַ�������С
  * @retval 
  */
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}

/**
  * @brief  �ĸ�������usart1
  * @param   �����ĸ��������Ϣ
  * @retval 
  */
//�����޸ģ��޸������ĸ�������
unsigned char  receiveBuff_u1[22] = {0};
int VisionReceiveData_1(int *theta_angle,int *theta_pitch,int *theta_yaw,int *theta_ras)
{

	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
	static short dataLength                   = 0;
  HAL_UART_Receive_IT(&huart1, &USART1_Receiver, 1); // ��������
	
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(USART1_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				//printf("header ok\n");
				receiveBuff_u1[0]=header[0];         //buf[0]
				receiveBuff_u1[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
			}
		}
		else 
		{
			USARTReceiverFront = USART1_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://�������ݵĳ���
				receiveBuff_u1[2] = USART1_Receiver;
				dataLength     = receiveBuff_u1[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff_u1[j + 3] = USART1_Receiver;        //buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]\	
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
			case 2://����У��ֵ��Ϣ(�趨Ϊ0x07)
				receiveBuff_u1[2 + dataLength] = USART1_Receiver;
				checkSum = getCrc8(receiveBuff_u1, 3 + dataLength);
//				checkSum = 0x07;
				  // �����ϢУ��ֵ
//				if (checkSum != receiveBuff[3 + dataLength]) //buf[11]
//				{
////					printf("Received data check sum error!");
//					return 0;
//				}
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					//����0d     buf[11]  �����ж�
					k++;
				}
				else if (k==1)
				{
					//����0a     buf[12] �����ж�

					//�����ٶȸ�ֵ����					
					 for(k = 0; k < 4; k++)
					{
						x_position.data[k]  = receiveBuff_u1[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						y_position.data[k] = receiveBuff_u1[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
						k_position.data[k]  = receiveBuff_u1[k + 11]; //buf[3]  buf[4] buf[5]  buf[6]
						b_position.data[k] = receiveBuff_u1[k + 15]; //buf[7]  buf[8] buf[9]  buf[10]
						
						
					}				
//					if(x_position.d==12)
//					{
//						printf("OK!");
//					}
//					else
//					{
//						printf("error!");
//					}
					//��ֵ����
					*theta_angle = x_position.d;//ƫ����
					*theta_pitch = y_position.d;//������
					*theta_yaw = k_position.d;
					*theta_ras = b_position.d;
//					*theta =(int)angle.d;
//					
//					//ctrlFlag
//					*flag = receiveBuff[9];                //buf[9]
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}

/**
  * @brief  ʹ��uart4�͵���ͨѶ�����ܣ�
  * @param   ��action���ݴ���ROBOT_REAL_POS_DATA
  * @retval 
  */
//����������
unsigned char  receiveBuff_u4[22] = {0};
union uart4_ReceiveData
{
	float d;
	unsigned char data[4];
}real_x,real_y,real_w;

union uart4_ReceiveData_int
{
	int d;
	unsigned char data[4];
}chassic_flag;
int uart4_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag)
{

	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
	static short dataLength                   = 0;

	HAL_UART_Receive_IT(&huart4, &UART4_Receiver, 1); // ��������
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(UART4_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				receiveBuff_u4[0]=header[0];         //buf[0]
				receiveBuff_u4[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
			}
		}
		else 
		{
			USARTReceiverFront = UART4_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://�������ݵĳ���
				receiveBuff_u4[2] = UART4_Receiver;
				dataLength     = receiveBuff_u4[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff_u4[j + 3] = UART4_Receiver;        //buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]\	
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
			case 2://����У��ֵ��Ϣ(�趨Ϊ0x07)
				receiveBuff_u4[2 + dataLength] = UART4_Receiver;
				checkSum = getCrc8(receiveBuff_u4, 3 + dataLength);
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					k++;
				}
				else if (k==1)
				{				
					 for(k = 0; k < 4; k++)
					{
						real_x.data[k]  = receiveBuff_u4[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						real_y.data [k] = receiveBuff_u4[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
						real_w.data [k]  = receiveBuff_u4[k + 11]; //buf[3]  buf[4] buf[5]  buf[6]
						chassic_flag.data[k] = receiveBuff_u4[k + 15]; //buf[7]  buf[8] buf[9]  buf[10]
					}				

					//��ֵ����
					*action_x = -real_x.d;
					*action_y = -real_y.d;
					*action_w = real_w.d;
					*flag = chassic_flag.d;
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**
  * @brief  ʹ��uart4�͵���ͨѶ�����ͣ�
* @param   x,y,w��Ϊ��������ϵ�µ��ٶȣ�flag����������������
  * @retval 
  */
//unsigned char buf[22]={0};//���ݻ�����

union Uart4_SendData//�������ݵĹ�����
{
	float d;
	unsigned char data[4];
}uart4_vx,uart4_vy,uart4_vw;

union Uart4_SendData_int//�������ݵĹ�����
{
	int d;
	unsigned char data[4];
}uart4_flag;



void Usart4_SendData(float X,float Y,float W,int flag)
{
	
	int i,length = 0;
	unsigned char  buf_u4[22] = {0};
	 //memset(buf,0,50);//�������
	uart4_vx.d = X;
	uart4_vy.d = Y;
	uart4_vw.d = W;
	uart4_flag.d = flag;
	for(i=0;i<2;i++)
	{
		buf_u4[i]=header[i];//Э������ͷ
	}
	length = 17;
	buf_u4[2] =length;//sizeof
	for(i=0;i<4;i++)
	{
		buf_u4[i+3]=uart4_vx.data[i];
		buf_u4[i+7]=uart4_vy.data[i];
		buf_u4[i+11]=uart4_vw.data[i];
		buf_u4[i+15]=uart4_flag.data[i];
		
		
	}
	buf_u4[3+length-1]=getCrc8(buf_u4,3+length);
	buf_u4[3+length]=ender[0];
	buf_u4[3+length+1]=ender[1];
	
	UART4_Send_String(buf_u4,sizeof(buf_u4));//�����ַ������ͺ�����������
}




/**
  * @brief  ʹ��uart4���ҷ������ͨѶ�����ܣ�
  * @param   ��action���ݴ���ROBOT_REAL_POS_DATA
  * @retval 
  */
//����������
unsigned char  receiveBuff_u5[22] = {0};
union uart5_ReceiveData
{
	int d;
	unsigned char data[4];
}gun1,gun2,gun3,gun_flag;

int uart5_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag)
{

	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
	static short dataLength                   = 0;

	HAL_UART_Receive_IT(&huart5, &UART5_Receiver, 1); // ��������
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(UART5_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				receiveBuff_u5[0]=header[0];         //buf[0]
				receiveBuff_u5[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
			}
		}
		else 
		{
			USARTReceiverFront = UART4_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://�������ݵĳ���
				receiveBuff_u5[2] = UART5_Receiver;
				dataLength     = receiveBuff_u5[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff_u5[j + 3] = UART5_Receiver;        //buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]\	
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
			case 2://����У��ֵ��Ϣ(�趨Ϊ0x07)
				receiveBuff_u5[2 + dataLength] = UART5_Receiver;
				checkSum = getCrc8(receiveBuff_u5, 3 + dataLength);
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					k++;
				}
				else if (k==1)
				{				
					 for(k = 0; k < 4; k++)
					{
						x_position.data[k]  = receiveBuff_u5[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						y_position.data[k] = receiveBuff_u5[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
						k_position.data[k]  = receiveBuff_u5[k + 11]; //buf[3]  buf[4] buf[5]  buf[6]
						b_position.data[k] = receiveBuff_u5[k + 15]; //buf[7]  buf[8] buf[9]  buf[10]
					}				

					//��ֵ����
					*action_x = real_x.d;
					*action_y = real_y.d;
					*action_w = real_w.d;
					*flag = chassic_flag.d;
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**
  * @brief  ʹ��uart5�͵���ͨѶ�����ͣ�
* @param   x,y,w��Ϊ��������ϵ�µ��ٶȣ�flag����������������
  * @retval 
  */
//unsigned char buf[22]={0};//���ݻ�����

union Uart5_SendData//�������ݵĹ�����
{
	float d;
	unsigned char data[4];
}uart5_vx,uart5_vy,uart5_vw,uart5_flag;
void Usart5_SendData(float X,float Y,float W,int flag)
{
	unsigned char  buf_u5[22] = {0};
	int i,length = 0;
	uart4_vx.d = X;
	uart4_vy.d = Y;
	uart4_vw.d = W;
	uart4_flag.d = flag;
	for(i=0;i<2;i++)
	{
		buf_u5[i]=header[i];//Э������ͷ
	}
	length = 17;
	buf_u5[2] =length;//sizeof
	for(i=0;i<4;i++)
	{
		buf_u5[i+3]=uart5_vx.data[i];
		buf_u5[i+7]=uart5_vy.data[i];
		buf_u5[i+11]=uart5_vw.data[i];
		buf_u5[i+15]=uart5_flag.data[i];
		
		
	}
	buf_u5[3+length-1]=getCrc8(buf_u5,3+length);
	buf_u5[3+length]=ender[0];
	buf_u5[3+length+1]=ender[1];
	
	UART5_Send_String(buf_u5,sizeof(buf_u5));//�����ַ������ͺ�����������
}


/*
�ַ������ͺ���1-6
����ָ����С���ַ�����
��ڲ����������ַ�������С
*/

void USART1_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length=0;//��̬������ֹ���ݶ�ʧ
	while(length<sendSize)
	{
		while(!(USART1->SR&(0x01<<7)));//���ͻ�����Ϊ��(�������ݻ���λӦ��Ϊ�ڰ�λ)
		USART1->DR=*p;
		p++;
		length++;
	}
	length=0;
}

void UART4_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length=0;//��̬������ֹ���ݶ�ʧ
	while(length<sendSize)
	{
		while(!(UART4->SR&(0x01<<7)));//���ͻ�����Ϊ��(�������ݻ���λӦ��Ϊ�ڰ�λ)
		UART4->DR=*p;
		p++;
		length++;
	}
	length=0;
}

void UART5_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length=0;//��̬������ֹ���ݶ�ʧ
	while(length<sendSize)
	{
		while(!(UART5->SR&(0x01<<7)));//���ͻ�����Ϊ��(�������ݻ���λӦ��Ϊ�ڰ�λ)
		UART5->DR=*p;
		p++;
		length++;
	}
	length=0;
}



/**
  * @brief  �����жϻص�����
	* @param  
	* @retval None
  * @attention
  */
//��������̬��������������
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;
	static uint8_t count = 0;
	static uint8_t i = 0;
	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
//usart1
//����
	if(huart->Instance==USART1)
	{
	 VisionReceiveData_1(&location_y,&location_x,&location_k,&location_b);
	 HAL_UART_Receive_IT(&huart1, &USART1_Receiver , 1); // ������
	}
//usart4
//����
	if(huart->Instance==UART4)//����Ǵ���4.
	{
	//��action���ݴ���ROBOT_REAL_POS_DATA
	
	Update_Action();
	HAL_UART_Receive_IT(&huart4, &UART4_Receiver, 1);
	}
		if(huart->Instance==UART5)//����Ǵ���5
	{
	//��action���ݴ���ROBOT_REAL_POS_DATA
	//uart4_ReceiveData(&ROBOT_REAL_POS_DATA.POS_X,&ROBOT_REAL_POS_DATA.POS_Y,&ROBOT_REAL_POS_DATA.POS_YAW,&ROBOT_FLAG.Chassis_receive);
	HAL_UART_Receive_IT(&huart5, &UART5_Receiver, 1);
	}
 
	if(huart->Instance==USART2)
	{
		STM32_READ_FROM_ROS(&data1, &data2, &data3, &data4, &data5, &data6, &data7, &data8, &data9, &data10);

	HAL_UART_Receive_IT(&huart2, &USART_Receiver2 , 1); // ������
	}
}



/*----------------------------------------����------------------------------------------------------*/
//�޸����ڷ��䶨λ
int Laser_calibration(float x, float y,float yaw,float v_max)
{

		float Laser_error_x,Laser_error_y,ERROR_SHOOTING;

	YawAdjust(yaw);
	//ת��ָ���Ƕ�
	Laser_error_x =  location_x - x;
	Laser_error_y =  location_y - y;
	ERROR_SHOOTING=sqrt(Laser_error_x*Laser_error_x+Laser_error_y*Laser_error_y);
//�жϾ����Ƿ����
	if(Laser_error_x<500&&Laser_error_y<10000)
	{
		if(ABS(Laser_error_x)>3||ABS(Laser_error_y)>3)
		{
//pidֱ����� 
			laser_K_pid.outputmax = ABS(v_max);
			PID_position_PID_calculation_by_error(&laser_B_pid, ERROR_SHOOTING);

			Robot_Chassis.World_V[0] = laser_B_pid.output*0.1f*Laser_error_x/sqrt(ERROR_SHOOTING);
			Robot_Chassis.World_V[1]= -laser_B_pid.output*0.1f*Laser_error_y/sqrt(ERROR_SHOOTING);
			
		Laser_error_x =  location_x - x;
	Laser_error_y =  location_y - y;
		}
		else
		{
			ACTION_GL_POS_DATA.REAL_X=218.0f;
			ACTION_GL_POS_DATA.REAL_Y=905.0f;
			return 1;
		}
	}
   
	return 0;
	
}
//�޸�����ȡ����λ
int Laser_calibration_1(float k, float b,float yaw,float v_max,int direction_1)
{
	float error_K,error_B,ERROR;

	YawAdjust(yaw);
	//ת��ָ���Ƕ�
	error_K =  location_k - k;
	error_B =  location_b - b;
	ERROR=sqrt(error_K*error_K+error_B*error_B);
//�жϾ����Ƿ����
	if(error_K<1500&&error_B<1500)
	{
		if(ABS(error_K)>8||ABS(error_B)>8)
		{
//pidֱ����� 
			laser_K_pid.outputmax = ABS(v_max);
			PID_position_PID_calculation_by_error(&laser_K_pid, ERROR);
			if(direction_1==1)
			{
			Robot_Chassis.World_V[0] = -laser_K_pid.output*1.0f*error_K/ERROR;
			Robot_Chassis.World_V[1]= -laser_K_pid.output*1.0f*error_B/ERROR;
			}
			if(direction_1==2)
			{
			Robot_Chassis.World_V[1] = laser_K_pid.output*1.0f*error_K/ERROR;
			Robot_Chassis.World_V[0]= -laser_K_pid.output*1.0f*error_B/ERROR;
			}
			error_K =  location_k - k;
			error_B =  location_b - k;
		}
		else
			return 1;
	}
   
	return 0;
}
	



/******************************************************************************************************/

/**************************************************/
///*
////��������
//*/
//unsigned char buf[14]={0};//���ݻ�����

//union sendData//�������ݵĹ�����
//{
//	int d;
//	unsigned char data[4];
//}dr_x,dr_y,dr_yaw;
//void VisionSendData(int X,int Y)
//{
//	
//	int i,length = 0;
//	
//	//�����������ٶȣ�δ֪
//	dr_x.d = X;
//	dr_y.d = Y;

//	
//	for(i=0;i<2;i++)
//	{
//		buf[i]=header[i];//Э������ͷ
//	}
//	length = 9;
//	buf[2] =length;//sizeof
//	for(i=0;i<4;i++)
//	{
//		buf[i+3]=dr_x.data[i];
//		buf[i+7]=dr_y.data[i];
//		
//	}
//	buf[3+length-1]=getCrc8(buf,3+length);
//	buf[3+length]=ender[0];
//	buf[3+length+1]=ender[1];
//	
//	USART_Send_String(buf,sizeof(buf));//�����ַ������ͺ�����������
//}
///*
//�ַ������ͺ���
//����ָ����С���ַ�����
//��ڲ����������ַ�������С
//*/
//void USART_Send_String(uint8_t *p,uint16_t sendSize)
//{
//	static int length=0;//��̬������ֹ���ݶ�ʧ
//	while(length<sendSize)
//	{
//		while(!(USART1->SR&(0x01<<7)));//���ͻ�����Ϊ��(�������ݻ���λӦ��Ϊ�ڰ�λ)
//		USART1->DR=*p;
//		p++;
//		length++;
//	}
//	length=0;
//}
