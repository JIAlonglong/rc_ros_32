#include "FSM.h"
#include "cmsis_os.h"
#include "comunication.h"
#include "robot.h"
#include "HareWare.h"
MOVE_STATE_ITEMS MOVE_STATE;
SHOOTING_STATE_ITEMS SHOOTING_STATE; 
ROBOT_FLAG_Type ROBOT_FLAG; 

 float V_robot_manual[3];
 float Yaw_robot_manual = 0;
 float Yaw_robot_manual_error = 0;

extern SHOOT_ITEMS SHOOT_STATE;
extern int move_ok;
//ARM_STATE_ITEMS ARM1_STATE;
	float move_time_counter = 0;
	
	int move_ok1=0;
	int move_1=0;//�����жϵ綯�Ƹ˵��ƶ�״̬
	int direction=0;
	int laser_point_flag=0;//�����жϼ���ƽ�Զ�㣬���ǽ���
	float POSX_TrapezoidPlaning=0;
	float POSY_TrapezoidPlaning=0;
	int test1flag=0;
	
void FSM_Init(void)
{
  MOVE_STATE=MOVE_STOP;
	SHOOTING_STATE=control_nineblock_0;
	KEY_DATA.KEY_armtop=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11);
	KEY_DATA.KEY_armbottom=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
	KEY_DATA.KEY_push=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13);
	//SHOOT_DATA.push_state = hold;
	//ARM1_STATE = arm_home;
}
/**************************************************�ƶ�״̬��*********************************************/
void move_FSM(void)
{
	
//	POSX_TrapezoidPlaning=(float)location_b;
//			 POSY_TrapezoidPlaning=(float)location_k;
	if(SWA<1500) 
	{		
		//ֹͣ
		if(SWB<1200)
		{
			move_ok=0;
		MOVE_STATE=MOVE_STOP;
		}
		
		//�ֶ�
		if(SWB>1200&&SWB<1700)
		{
		MOVE_STATE=MOVE_FREE;
			
			move_ok=1;
		}
		
		//�Զ�
		if(SWB>1700)
		{
			move_ok=0;
			if(SWC<1200)
			{
				//�����װ��
				if(ROCK_R_X>1950)
				{
					//�������ι滮��ʼֵ
					POSX_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_X;
					POSY_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_Y;
					MOVE_STATE=MOVE_1_LOAD_POINT;			
				}
				//���ұ�װ��
				else if (ROCK_R_X<1050)
				{
					//�������ι滮��ʼֵ
					POSX_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_X;
					POSY_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_Y;					
					MOVE_STATE=MOVE_2_LOAD_POINT	;
				}
				//����
				else if (ROCK_R_Y>1950)
				{
					//�������ι滮��ʼֵ
					POSX_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_X;
					POSY_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_Y;
					MOVE_STATE=MOVE_1_RESTART;
				}
				else if(ROCK_R_Y<1050)//����
				{
					//�������ι滮��ʼֵ
					POSX_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_X;
					POSY_TrapezoidPlaning=ROBOT_REAL_POS_DATA.POS_Y;
					MOVE_STATE=MOVE_SHOOT;
				}
			}
		if (SWC>1700)//C����
			 {
				 /************************�ֶ�����*******************************/
				 MOVE_STATE=MOVE_LOAD;
				 /*******************************************************/
				 
			
			 if(direction==1)
			 {
			  POSX_TrapezoidPlaning=(float)location_b;
			 POSY_TrapezoidPlaning=(float)location_k;
			 }
			 else if(direction==2)
			 {
			  POSX_TrapezoidPlaning=(float)location_k;
			 POSY_TrapezoidPlaning=(float)location_b;
			 }
			 else
			 {
			 POSX_TrapezoidPlaning=(float)location_y;
			 POSY_TrapezoidPlaning=(float)location_x;
			 }
	
			 }
		
		}
	}

}

/*********************************�ƶ�״̬����Ϊ*********************************************/
void move(void)
{
	
	/************************�ֶ���������*****************************/
	//���Ƕ�С��-20�ȣ�direction=1��
	if(ROBOT_REAL_POS_DATA.POS_YAW<-20)
	{
		direction=1;
	}
	else if(ROBOT_REAL_POS_DATA.POS_YAW>20)
	{
		direction=2;
	}
	else
		direction=3;
	
	
	/****************************************************************/
	
	if(SWA<1400)
	{
	//	GUN_DATA.SHOOT_DATA_L.push_state = hold;//�ƻ�λ�ò���			
   switch(MOVE_STATE)
    {
			
			case MOVE_STOP:
				move_ok=0;//ֹͣ�����ܶ�
				Robot_Chassis.World_V[0]=0;//y��
				Robot_Chassis.World_V[1]=0;//x��
				Robot_Chassis.World_V[2]=0;//w��
			  break;
			
			////ȡ����
			case MOVE_1_LOAD_POINT:
//			 move_time_counter += 0.01f;
//                                                                             POS_end  YAW
			 if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,-4650,470,-45,100,300,2900,0.2,0.68))
//     if(PathPlan(move_time_counter,2.8,7, X0 , Y0, Yaw0))
			{
			//���¼����ʼֵ
       POSX_TrapezoidPlaning=(float)location_b;
			 POSY_TrapezoidPlaning=(float)location_k;
				move_time_counter = 0;
				if(location_b<500&&location_k<700)
				
				MOVE_STATE = MOVE_LASER_NEAR;
				else
					MOVE_STATE = MOVE_LASER;
//				armstate=1;
        direction=1;//��ʾ������ȡ��										
			}
				break;
			case MOVE_2_LOAD_POINT:
//			 move_time_counter += 0.01f;
//                                                                             POS_end  YAW
			 if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,5000,450,45,100,100,2800,0.2,0.68))
//     if(PathPlan(move_time_counter,2.8,7, X0 , Y0, Yaw0))
			{
       POSX_TrapezoidPlaning=(float)location_k;
			 POSY_TrapezoidPlaning=(float)location_b;
				move_time_counter = 0;
			if(location_b<500&&location_k<500)
				
				MOVE_STATE = MOVE_LASER_NEAR;
				else
					MOVE_STATE = MOVE_LASER;
        laser_point_flag=0;				
				
					
			}
				break;	

			case MOVE_SHOOT://����ط����价
				if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,250,600,0,100,100,1000,0.2,0.68)
					||(location_x>4000&&location_y<700))
				//if(chassis_TrapezoidPlaning_laser(0,0,8280,80,0,100,0,200,0.3,0.6,location_y,location_x))
				{	
					POSX_TrapezoidPlaning=(float)location_x;
			    POSY_TrapezoidPlaning=(float)location_y;
					MOVE_STATE = MOVE_STOP;	
					direction=3;//�м�								
					
				}
				break;
				
			case MOVE_1_RESTART:						 
				if(chassis_TrapezoidPlaning(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,0,0,0,100,100,1000,0.2,0.68))
				{	
					MOVE_STATE = MOVE_STOP;
					ROBOT_REAL_POS_DATA.robot_location=robot_start;	
				}
				break;
				
			case MOVE_FREE:

		 
				break;
				
			case MOVE_LASER:
			
			if(1)//�Զ�
				{	
				if(direction==1)//-45,laser_point_flag����1��ʱ�򣬽��бƽ�����
					{		
             				
					if(Laser_calibration_1(567, 487,-45,200,direction))//��ʾ�������ȡ��
				
						//if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,567,487,-45,200,50,600,0.3,0.6,location_b,location_k))
						{
						test1flag=1;
							//�ȴ�����
//						ACTION_GL_POS_DATA.REAL_X=-4900.0f;
//						ACTION_GL_POS_DATA.REAL_Y=100.0f;
						MOVE_STATE = MOVE_LASER_NEAR;
							ROBOT_REAL_POS_DATA.robot_location=robot_load_left;
						}					
					}	
    
				else if(direction==2)//45,laser_point_flag����1��ʱ�򣬽��бƽ�����
					{
						//if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,231,250,45,50,0,200,0.3,0.6,location_k,location_b))
						if(Laser_calibration_1(560, 500,45,200,direction))//��ʾ�����ұ�ȡ��
						{
						MOVE_STATE = MOVE_LASER_NEAR;
							//�ȴ�����
//						ACTION_GL_POS_DATA.REAL_X=5200.0f;
//						ACTION_GL_POS_DATA.REAL_Y=100.0f;
						ROBOT_REAL_POS_DATA.robot_location=robot_load_right;	
						}				
					}
					
					
				else if(direction==3)//0
					{
						//����PD������
					//if(Laser_calibration(5600,164,0,200 ));
						if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,5680,164,0,50,0,200,0.3,0.6,location_x,location_y))//ȥ����
						{						
						MOVE_STATE = MOVE_LASER_NEAR;
						ROBOT_REAL_POS_DATA.robot_location=robot_shoot;	
						}		
					}
				else MOVE_STATE = MOVE_STOP;


				}						


			break;

			case MOVE_LASER_NEAR:
				
			if(direction==1)//-45,laser_point_flag����1��ʱ�򣬽��бƽ�����
					{		
             				
					if(Laser_calibration_1(180, 185,-45,200,direction))//��ʾ�������ȡ��
				
					//	if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,358,499,-45,100,0,500,0.3,0.6,location_b,location_k))
						{
						test1flag=1;
						ACTION_GL_POS_DATA.REAL_X=-4900.0f;
						ACTION_GL_POS_DATA.REAL_Y=100.0f;
						MOVE_STATE = MOVE_STOP;
							ROBOT_REAL_POS_DATA.robot_location=robot_load_left;
						}					
					}	
    
				else if(direction==2)//45,laser_point_flag����1��ʱ�򣬽��бƽ�����
					{
				//		if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,231,250,45,50,0,200,0.3,0.6,location_k,location_b))
						if(Laser_calibration_1(200, 170,45,200,direction))//��ʾ�����ұ�ȡ��
						{
						MOVE_STATE = MOVE_STOP;
						ACTION_GL_POS_DATA.REAL_X=5300.0f;
						ACTION_GL_POS_DATA.REAL_Y=100.0f;
						ROBOT_REAL_POS_DATA.robot_location=robot_load_right;	
						}				
					}
					
					
				else if(direction==3)//0
					{
					
						if(chassis_TrapezoidPlaning_laser(POSX_TrapezoidPlaning,POSY_TrapezoidPlaning,8280,80,0,50,0,200,0.3,0.6,location_y,location_x))//ȥ����
						{						
						MOVE_STATE = MOVE_STOP;
						ROBOT_REAL_POS_DATA.robot_location=robot_shoot;	
						}		
					}
				else MOVE_STATE = MOVE_STOP;


			
				break;

				case MOVE_LOAD: 
					if(ROCK_L_X>1460&&ROCK_L_X<1540)ROCK_L_X=1500;				
					V_robot_manual[1]=-(ROCK_L_X-1500)*0.1f;//vx
					V_robot_manual[0]=-50;//vyһֱ���
					if(ROCK_R_X>1460&&ROCK_R_X<1540)ROCK_R_X=1500; 			
					if(1490>ROCK_R_X)Yaw_robot_manual_error-=0.02f;//vw			
					if(1510<ROCK_R_X)Yaw_robot_manual_error+=0.02f;//vw
					//ȷ������
					if(direction==1)
					YawAdjust(-45.0f+Yaw_robot_manual_error);
					if(direction==2)
					YawAdjust(45.0f+Yaw_robot_manual_error); 
					// 1. ����һ���任��������ת������
					float cos_theta = cos(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
					float sin_theta = sin(ROBOT_REAL_POS_DATA.POS_YAW*PI/180);
					// 2. ת��Ϊ����������
					Robot_Chassis.World_V[1] = V_robot_manual[1] * cos_theta - V_robot_manual[0] * sin_theta;
					Robot_Chassis.World_V[0] = V_robot_manual[1] * sin_theta + V_robot_manual[0] * cos_theta;	
					//����
					if(SWC<1700&&SWC>1200) //C���м�
						{
						Robot_Chassis.World_V[1]=0;
						Robot_Chassis.World_V[0]=0;
						Robot_Chassis.World_V[2]=0;
						Yaw_robot_manual_error=0;	
						MOVE_STATE = MOVE_STOP;
						}
				
				break;
			
			
			default:break;
				
   }
 }
}

/*********************************************���״̬��******************************************/
//��ߵķ������
void shoot_FSM(void)
{	
if(SWA>1600)
	{	
//		if(move_1==0)//��ʼ��Ϊ0���綯�Ƹ����ƽ�
//		{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//			osDelay(100);//�綯�Ƹ��ƶ��ľ���ȡ�����ӳٵ�ʱ�䡣
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
//			move_1=1;//��1�󣬵綯�Ƹ� �������ƶ�
//		}
		get_9block();
//		SHOOTING_STATE = control_nineblock;
		move_ok = 0;
		
//	if(SWD>1600){SHOOT_DATA.tiny_yaw=SHOOT_DATA.tiny_pitch=0;}
//		if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw += 0.01f;}
//		if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw -= 0.01f;}
//		if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch += 0.01f;}
//		if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch -= 0.01f;}	
	}
			
}




/***********************************************���״̬��ÿ������******************************************/
void shoot(void)
{
	if(SWA>1600)//��SWA�����������ܽ������״̬
	{	pwm_key_flag=0;
		if(SWD<1300)
		{
//		__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 2000);
//		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1000);
//    osDelay(500);
			SHOOT_DATA.push_state = back;//���ƻ�������
		
		}
		
		switch(SHOOTING_STATE)
		{
			case control_nineblock_5://ֹͣ���//�����������
				SHOOT_DATA.yaw = 0;
				SHOOT_DATA.pitch = 0;
				SHOOT_DATA.tiny_yaw = 0;
				SHOOT_DATA.tiny_pitch =0;
				VelCrl(&MOTOR_REAL_INFO[4],0);
				VelCrl(&MOTOR_REAL_INFO[3],0);
			  break;
		
			case control_nineblock_1://�������1����
				//û�����״̬
				break;
				
			case control_nineblock_2://�������2����				
				SHOOT_DATA.yaw = 2.0;
				SHOOT_DATA.pitch = -0.418;		
				VelCrl(&MOTOR_REAL_INFO[4],1900);
				VelCrl(&MOTOR_REAL_INFO[3],-1500);
			
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[1] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[1] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[1] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[1] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[1];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[1];				
				break;
			
			case control_nineblock_3://�������3����
				SHOOT_DATA.yaw = -4.647;
				SHOOT_DATA.pitch = 1.094;	
				VelCrl(&MOTOR_REAL_INFO[4],2980);
				VelCrl(&MOTOR_REAL_INFO[3],-3500);
				
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[2] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[2] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[2] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[2] -= 0.001f;}	
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[2];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[2];					
				break;
				
			case control_nineblock_4://�������4����
				//û�����״̬		
				break;

			case control_nineblock_6://�������6����
				SHOOT_DATA.yaw = -0.3809;
				SHOOT_DATA.pitch = 0.685;	
				VelCrl(&MOTOR_REAL_INFO[4],3000);
				VelCrl(&MOTOR_REAL_INFO[3],-3500);
		
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[5] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[5] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[5] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[5] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[5];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[5];				
				break;

			case control_nineblock_7://Զ������				
//				SHOOT_DATA.yaw = 2;
//				SHOOT_DATA.pitch = 0.1;		
//				VelCrl(&MOTOR_REAL_INFO[4],3400);
//				VelCrl(&MOTOR_REAL_INFO[3],-4200);
//			
//				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[6] += 0.01f;}
//				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[6] -= 0.01f;}
//				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_yaw_data[6] += 0.01f;}
//				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_yaw_data[6] -= 0.01f;}
//				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[6];
//				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[6];				
				break;

			case control_nineblock_8://���8��ߺ���				
				SHOOT_DATA.yaw = 1.413;
				SHOOT_DATA.pitch = -0.201;	
				VelCrl(&MOTOR_REAL_INFO[4],3400);
				VelCrl(&MOTOR_REAL_INFO[3],-4400);
			
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[7] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[7] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[7] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[7] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[7];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[7];				
				break;

			case control_nineblock_9://Զ������
				SHOOT_DATA.yaw = 0.24689;
				SHOOT_DATA.pitch = 0.4089;		
				VelCrl(&MOTOR_REAL_INFO[4],3800);
				VelCrl(&MOTOR_REAL_INFO[3],-4600);
			
				if(ROCK_R_X>1800){SHOOT_DATA.tiny_yaw_data[8] += 0.001f;}
				if(ROCK_R_X<1200){SHOOT_DATA.tiny_yaw_data[8] -= 0.001f;}
				if(ROCK_R_Y>1800){SHOOT_DATA.tiny_pitch_data[8] += 0.001f;}
				if(ROCK_R_Y<1200){SHOOT_DATA.tiny_pitch_data[8] -= 0.001f;}
				SHOOT_DATA.tiny_yaw = SHOOT_DATA.tiny_yaw_data[8];
				SHOOT_DATA.tiny_pitch = SHOOT_DATA.tiny_pitch_data[8];				
				break;
				
			case control_nineblock_0:
				//��ʼ״̬��ʲô����ִ��
				break;
			default:break;
		}
 }

}
