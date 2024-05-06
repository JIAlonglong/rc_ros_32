#include "calculation.h"
#include "math.h"
#include "HareWare.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "MIT.h"
#include "cmsis_os.h"
#include "tim.h"
#include "robot.h"
#include "comunication.h"
//ROBOT_TARGET_VELOCITY ROBOT_TARGET_VELOCITY_DATA;
SHOOT_ITEMS SHOOT_STATE;
SHOOT_Type SHOOT_DATA;

averageFilter_TPYE fitlerx;
averageFilter_TPYE fitlery;
averageFilter_TPYE fitlerw;
 float yuntai_controller;
 float pitch_controller;
 int move_ok;

 /**
  * @brief  M3508��ʼ��
	* @param  None
	* @retval None
  * @attention
  *�Ƚ����е����ʼ����3508������
  */

int ROCK_L_X_Processed=1500;
int ROCK_L_Y_Processed=1500;
int ROCK_R_X_Processed=1500;
void moto_calculation(void)
{
		if(move_ok==1)
		{		
//		{
//		if(ROCK_L_X>1460&&ROCK_L_X<1540)
//			ROCK_L_X=1500;
//			Robot_Chassis.World_V[1]=-(ROCK_L_X-1500)*2;
//		 if(ROCK_L_Y>1460&&ROCK_L_Y<1540)
//			ROCK_L_Y=1500; 
//		 Robot_Chassis.World_V[0]=-(ROCK_L_Y-1500)*2;
//		 if(ROCK_R_X>1460&&ROCK_R_X<1540)
//			ROCK_R_X=1500; 
//		 Robot_Chassis.World_V[2]=-(ROCK_R_X-1500)*2;
//		}
		
		if(ROCK_L_X>1460&&ROCK_L_X<1540)
			ROCK_L_X=1500;
			fitlerx.indata=-(ROCK_L_X-1500)*4;
		 if(ROCK_L_Y>1460&&ROCK_L_Y<1540)
			ROCK_L_Y=1500; 
		 fitlery.indata=-(ROCK_L_Y-1500)*4;
		 if(ROCK_R_X>1460&&ROCK_R_X<1540)
			ROCK_R_X=1500; 
		  fitlerw.indata=-(ROCK_R_X-1500)*4;
//		if(ROCK_L_X<=1550&&ROCK_L_X>=1450){ROCK_L_X = 1500;}	
//		if(ROCK_L_Y<=1550&&ROCK_L_Y>=1450){ROCK_L_Y = 1500;}
//		if(ROCK_R_X<=1550&&ROCK_R_X>=1450){ROCK_R_X = 1500;}
//		if(ROCK_R_X_Processed<ROCK_R_X){ROCK_R_X_Processed+=20;}
//		else if(ROCK_R_X_Processed>ROCK_R_X){ROCK_R_X_Processed-=20;}
//		if(ROCK_L_X_Processed<ROCK_L_X){ROCK_L_X_Processed+=20;}
//		else if(ROCK_L_X_Processed>ROCK_L_X){ROCK_L_X_Processed-=20;}
//		if(ROCK_L_Y_Processed<ROCK_L_Y) {ROCK_L_Y_Processed+=20;}
//		else if(ROCK_L_Y_Processed>ROCK_L_Y){ROCK_L_Y_Processed-=20;}
		
		Robot_Chassis.Angle=ROBOT_REAL_POS_DATA.POS_YAW;
		
		//�˲�
//		fitlerx.indata=-(ROCK_L_X_Processed-1500)*4;
//		fitlery.indata=-(ROCK_L_Y_Processed-1500)*4;
//		fitlerw.indata=-(ROCK_R_X_Processed-1500)*4;

		averageFilter(&fitlerx);
		averageFilter(&fitlery);
		averageFilter(&fitlerw);
		Robot_Chassis.World_V[1]=fitlerx.outdata;
		Robot_Chassis.World_V[0]=fitlery.outdata;
		Robot_Chassis.World_V[2]=fitlerw.outdata;
		}
		
}  

void mit_calculation(void)//Ҫ��������һ��task����
{
	
	{
		SHOOT_DATA.shoot_yaw=SHOOT_DATA.yaw+SHOOT_DATA.tiny_yaw;
	SHOOT_DATA.shoot_pitch=SHOOT_DATA.pitch+SHOOT_DATA.tiny_pitch;
	}
	/**�޷�**/
	if(SHOOT_DATA.shoot_yaw>2)SHOOT_DATA.shoot_yaw=2;//��ƫ
	if(SHOOT_DATA.shoot_yaw<-5)SHOOT_DATA.shoot_yaw=-5;
	if(SHOOT_DATA.shoot_pitch>3.5)SHOOT_DATA.shoot_pitch=3.5;
	if(SHOOT_DATA.shoot_pitch<-1)SHOOT_DATA.shoot_pitch=-1;//����
	/**�趨λ��**/
	
	ctrl_motor2(DM43_ID1,SHOOT_DATA.shoot_yaw,5);   //yaw
	osDelay(1);
	ctrl_motor2(DM43_ID2,SHOOT_DATA.shoot_pitch,5);   //pitch
	osDelay(1);
}

/**
  * @brief  2006����Ļ���У׼
	* @param  None
	* @retval none
  */
void m2006_init(void)
{
	MOTOR_REAL_INFO[6].type=M_2006;
	MOTOR_REAL_INFO[7].type=M_2006;
		//OTOR_REAL_INFO[0].type=M_2006;

}

/**
  * @brief  �价���ƣ��ƻ���������̨��
	* @param  none
	* @retval none
  * @attention
  *
  */

void shoot_calculation(void)
{
 static int stop_flag1;
 static int stop_flag2;
 static int pwm_stop_flag;
 static int cnt1;
 static int cnt2;
 static int cnt3;

 /**�������**/
if(1)
{
 
  if(SHOOT_DATA.push_state == back)
	{
   if(stop_flag2 ==0)
   {
       pwm_stop_flag = 1;
//  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 500);//�ɵ�
//  __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 2500);
    //PosCtrl(&MOTOR_REAL_INFO[7],0);
    Vel_TorqueCtrl(&MOTOR_REAL_INFO[7],2000,7000);
    if(ABS(MOTOR_REAL_INFO[7].CURRENT)>1500)
     {  
      cnt2++;
         }
    else
     {
     cnt2 = 0;
        }
    if(cnt2>=50)//50ms
    {
     VelCrl(&MOTOR_REAL_INFO[7],0);
     stop_flag2 = 1; 
     MOTOR_REAL_INFO[7].REAL_ANGLE = 830;//У׼
		 SHOOT_DATA.push_state = push; //�ĳ�������
     
    }   
   }
  cnt1 = 0;//����ƻ��ļ���
  stop_flag1 = 0; 
  }
}

  if(SHOOT_DATA.push_state == push) //��ǰ
     {
    if(stop_flag1 ==0)
    {
//     __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 2000);//�ɵ�
//        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
    
     Vel_TorqueCtrl(&MOTOR_REAL_INFO[7],8000,-10000);
    if(ABS(MOTOR_REAL_INFO[7].CURRENT)>3500)
     {  
        cnt1++;
      }
    else
     {
     cnt1 = 0;
     }
    if(cnt1>=50)//100ms
     { 
     VelCrl(&MOTOR_REAL_INFO[7],0);
     stop_flag1 = 1; 
     MOTOR_REAL_INFO[7].REAL_ANGLE = 0;//У׼
//     pwm_stop_flag = 1;
			 SHOOT_DATA.push_state = hold; 
      }

   } 
   cnt2 = 0;//����ƻ��ļ���
   stop_flag2 = 0;
   } 
  
 
  if(SHOOT_DATA.push_state == hold)//ͣ��
   VelCrl(&MOTOR_REAL_INFO[7],0);
  
  

  if(pwm_key_flag==0)//��ʹ�ü�צ������ͷ���
	{
		if(pwm_stop_flag ==1&&cnt3<=100)
		{
			if(cnt3<=50)
			{
//				__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 1500);					 

//				__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 2000);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//�Ƹ˺���
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
			}
		 else
			{   
			 __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 500);
			 __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 2500);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);//�Ƹ��ƽ�
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

			}
			cnt3++;
		}
		else
		{
			cnt3 = 0;
			pwm_stop_flag=0;
			__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 2000);
			__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
				
				
		}
	}
	else//�����צ״̬�ſ����
	{
		 __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 500);
		 __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 2500);
	}

}




/**
  * @brief  ��צ����(̧�������ϼ�צ)
	* @param  none
	* @retval none
  * @attention
  *����->clamp_close->arm_raise->clamp_open->arm_dowm->arm_home
  */
int armstate=0;
int arm_first_time_flag=0;
int pwm_key_flag=0;
int CatchRing(void)
{
  armstate=1;

}
void arm_calculation(void)
{
	static int cast_cnt2;
	
	if(SWB>1700&&SWC>1200)//�Զ�ģʽ�²ſ��Կ���
		armstate=1;
	

		switch(armstate){		 	
			case 0:
				Larm_home();
				break;
			
		  case 1:	
				if(arm_first_time_flag==0)//��һ��
				{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//��ȡ��ʼ���Ƹ˺���
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				
				if(Larm_down_first())
				{
					arm_first_time_flag=1;
					VelCrl(&MOTOR_REAL_INFO[5],0);//�ȴ�У׼
					armstate=2;
				}					
				}
				else
				{
				if(Larm_down())
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);//��ȡ��ʼ���Ƹ˺���
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
					arm_first_time_flag=1;
					VelCrl(&MOTOR_REAL_INFO[5],0);//�ȴ�У׼
					armstate=2;
					pwm_key_flag=1;
				}			
				}
				break;
				
			case 2:
				Larm_wait();
				if(SWC<1200)armstate=3;
			
				break;
			
//				default: break;		
//			}
//		}
	 
	
			
			case 3:
				if(Lclamp_close())				
				{
					armstate=4;	
					SHOOT_DATA.pitch = 0;			
				}
				break;
				
			case 4:				
				if(Larm_raise())
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);//��ȡ��ɣ��Ƹ��ƽ�
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
					VelCrl(&MOTOR_REAL_INFO[5],0); 
					cast_cnt2++;
    
					if(cast_cnt2>50)
					{
					SHOOT_DATA.pitch = 0.6;
					armstate=5;
					cast_cnt2=0;
					}		
				}
				break;
				
		case 5:		
			if(Lclamp_open())
			{	
			 armstate=6;
       SHOOT_DATA.pitch = 0.6;			
	  	}
			break;
			
		case 6:
			if(Larm_down_to_home())
			{
				armstate=0;
				SHOOT_DATA.pitch = 0.6;
			}	
			break;
			

		
		default: 
			break;
	 }
 }



/**
  * @brief  �������������
	* @param  None
	* @retval None
  * @attention
  */

void gun_calculation(void)
{
	
	//�������
	
	//�ҷ������
}
/**
  * @brief  ƽ��ֵ�˲�
	* @param  None
	* @retval None
  * @attention
  */
void averageFilter(averageFilter_TPYE *ff)
{
	float sum = 0;
	for(int i=0; i<4; i++)
	{
		ff->data[i]=ff->data[i+1];
		sum = sum + ff->data[i];
	}
	ff->data[4] = ff->indata;
	sum = sum + ff->data[4];
	
	ff->outdata=sum/5;
 
}

/**
  * @brief  �Ź���
	* @param  None
	* @retval None
  * @attention
  */

void get_9block(void)
{
	if(SWD >= 1600)//��ʼ��¼�Ź���
	{
		if(ROCK_L_Y>=1800)//123
		{
			if(ROCK_L_X<=1200)SHOOTING_STATE=control_nineblock_3;//3
			if(ROCK_L_X<1800&&ROCK_L_X>1200)SHOOTING_STATE=control_nineblock_2;//2
			if(ROCK_L_X>=1800)SHOOTING_STATE=control_nineblock_1;//1
		}
		else if(ROCK_L_Y<1800&&ROCK_L_Y>1200)//456
		{
			if(ROCK_L_X<=1200)SHOOTING_STATE=control_nineblock_6;//6
			if(ROCK_L_X<1800&&ROCK_L_X>1200)SHOOTING_STATE=control_nineblock_5;//5
			if(ROCK_L_X>=1800)SHOOTING_STATE=control_nineblock_4;//4
			
		}
		else if(ROCK_L_Y<=1200)//789
		{
			if(ROCK_L_X<=1200)SHOOTING_STATE=control_nineblock_9;//9
			if(ROCK_L_X<1800&&ROCK_L_X>1200)SHOOTING_STATE=control_nineblock_8;//8
			if(ROCK_L_X>=1800)SHOOTING_STATE=control_nineblock_7;//7
			
		}
	}
}
/**
  * @brief  ����actionȫ����λ��ֵ
	* @param  None
	* @retval None
  * @attention��ֱ�Ӹ���
  */
void Update_Action(void)
{
//	float error;
//������һ�ε�ֵ
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
	
	uart4_ReceiveData(&ACTION_GL_POS_DATA.POS_X,&ACTION_GL_POS_DATA.POS_Y,&ROBOT_REAL_POS_DATA.POS_YAW,&ROBOT_FLAG.Chassis_receive);
	// �������
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
	
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
	

	// ƫ����ֱ�Ӹ�ֵ����ʱ��Ϊ����˳ʱ��Ϊ����
  //ROBOT_REAL_POS_DATA.POS_YAW = ACTION_GL_POS_DATA.ANGLE_Z - ACTION_GL_POS_DATA.OFFSET_YAW;
	
	//������е���,��ֵX��Y
	ROBOT_REAL_POS_DATA.POS_X = ACTION_GL_POS_DATA.REAL_X; //+ INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
	ROBOT_REAL_POS_DATA.POS_Y = ACTION_GL_POS_DATA.REAL_Y; //- INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
	 
}
