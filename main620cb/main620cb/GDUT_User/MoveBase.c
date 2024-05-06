/**
  ******************************************************************************
  * @file    moto.c
  * @author  �±�
  * @version V1.1.0
  * @date    2023/3/29
  * @brief   
  ******************************************************************************
  */ 
#include "MoveBase.h"



/********************************************·���滮***************************************/
int testflag=0;//���ڲ��Դ���Ľ���


float kp_x = 6;
float kd_x = 0;	//0.00011
float kp_y = 6;
float kd_y = 0;	//0.00011
float kp_yaw = 1;
float kd_yaw = 0;
float error_X;float error_Y;	// ����X��Yƫ��
float error_x;float error_y;	// ����x��yƫ��
float error_Yaw;							// ƫ����ƫ��
float now_yaw;								// ��ǰ������ƫ����
float u_output;								// ��������x�����ٶ����
float v_output;								// ��������y�����ٶ����
float w_ouput;								// ���ٶ����
float Error=0;
ROBOT_CHASSIS Robot_Chassis;
/*pid��ؽṹ��--------------------------------------*/
PID point_traker_x_pid;
PID point_traker_y_pid;
PID point_traker_yaw_pid;
//�״�׷��
PID point_traker_ladar_y_pid;
PID point_traker_ladar_x_pid;
PID yaw_pid_ladar;
/**
* @brief  �ƶ�������ݳ�ʼ��
* @note		
* @param  
* @retval 
*/
void Move_Init(void)
{
		//�״����׷��
	PID_parameter_init(&point_traker_ladar_y_pid, 0.5,0.1, 0.1, 500, 0, 1);
	PID_parameter_init(&point_traker_ladar_x_pid, 0.5,0.1, 0.1, 500, 0, 1);
	PID_parameter_init(&yaw_pid_ladar, 30,0, 0.1, 800, 0, -1);

	//PD������
	PID_parameter_init(&point_traker_x_pid, 3,0, 0.5, 2000, 0, 10);
	PID_parameter_init(&point_traker_y_pid, 3,0, 0.5, 2000, 0, 10); 
	PID_parameter_init(&point_traker_yaw_pid, 30,0, 0.1,1000, 0, 1);
	
	// ����PID
	PID_parameter_init(&laser_X_pid ,  1,0, 0.5, 500, 0, 10);
	PID_parameter_init(&laser_Y_pid ,  1,0, 0.5, 100, 0, 10);
	PID_parameter_init(&laser_K_pid ,  3,0, 0.3, 300, 0, 5);
	PID_parameter_init(&laser_B_pid ,  2,0, 0.5, 300, 0, 5);
	
	// �Զ�·��PID
	PID_parameter_init(&point_pid ,  1.2f,0, 0.5f, 500, 0, 5);
	PID_parameter_init(&yaw_pid ,  30.0f,0.0f, 1.0f, 500.0f, 0.0f, 0.1f);
}
/**
* @brief  PDController������
* @note		���ٹ滮�õ�·��
* @param  target_point:��λʱ��Ҫ���ٵĵ㣨���ȹ滮���ٶȣ���robot_now_pos:�����˵�ǰ���������µ�λ��
* @retval 
*/
void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos)
{
	YawAdjust(target_point.Yaw);
	// �������
	error_X = target_point.X - robot_now_pos.POS_X;
	error_Y = target_point.Y - robot_now_pos.POS_Y;
	//error_Yaw = target_point.Yaw - robot_now_pos.POS_YAW;
//	//�Ƕ���ת��Ϊ������
//	now_yaw = robot_now_pos.POS_YAW * PI / 180.0f;
//	// ���㵽��������
//	error_x =  cos(now_yaw) * error_X + sin(now_yaw) * error_Y;
//	error_y = -sin(now_yaw) * error_X + cos(now_yaw) * error_Y;
//	
//	// �����ٶ�
//	w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
//	u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
//																		target_point.V_y  * sin(now_yaw) + \
//																		w_ouput * error_y * cos(now_yaw) - \
//																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
//	v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
//																		target_point.V_y  * cos(now_yaw) - \
//																		w_ouput * error_y * sin(now_yaw) - \
//																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);
//																		 
//	// ����Ϊ��������ϵ�µ��ٶ�
//	Robot_Chassis.World_V[1] = -(u_output * cos(now_yaw) - v_output * sin(now_yaw));
//	Robot_Chassis.World_V[0] = -(u_output * sin(now_yaw) + v_output * cos(now_yaw));
//	Robot_Chassis.World_V[2]  = -w_ouput;
	PID_position_PID_calculation_by_error(&point_traker_x_pid, error_X);
	PID_position_PID_calculation_by_error(&point_traker_y_pid, error_Y);
	//PID_position_PID_calculation_by_error(&point_traker_yaw_pid, error_Yaw);
	
	//��Ӹ���
	Robot_Chassis.World_V[1] = point_traker_x_pid.output;
	Robot_Chassis.World_V[0] = point_traker_y_pid.output;
	//Robot_Chassis.World_V[2]  = -point_traker_yaw_pid.output;
}

int PDControllertest(float x,float y,float w)
{
	int yawflag;
	yawflag=YawAdjust(w);
	// �������
	error_X = x- ROBOT_REAL_POS_DATA.POS_X;
	error_Y = y - ROBOT_REAL_POS_DATA.POS_Y;
//	error_Yaw = w - ROBOT_REAL_POS_DATA.POS_YAW;

	PID_position_PID_calculation_by_error(&point_traker_x_pid, error_X);
	PID_position_PID_calculation_by_error(&point_traker_y_pid, error_Y);
//	PID_position_PID_calculation_by_error(&point_traker_yaw_pid, error_Yaw);
	
	//��Ӹ���
	Robot_Chassis.World_V[1] = point_traker_x_pid.output;
	Robot_Chassis.World_V[0] = point_traker_y_pid.output;
	//Robot_Chassis.World_V[2]  = -point_traker_yaw_pid.output;
	
		if((ABS(ROBOT_REAL_POS_DATA.POS_X - x)<10.0f&&ABS(ROBOT_REAL_POS_DATA.POS_Y - y)<10.0f)&&yawflag==1)
	{
		return 1;
	}
		return 0;
}


int k;
float t;
float f1s;float f2s;float f3s;float f4s;
float last_X;float last_Y;float last_Yaw;
float Sx_error;float Sy_error;
float Hz;
int first_time_flag = 1;
PATH_TYPEDEF now_path_point;

/**
* @brief  PathPlan�滮+����
* @note		����B�����滮�����ֱ�Ӹ�ֵ�������յ㷵��1�����򷵻�0
* @param  t_real:��ʵ������ʱ�䣬t_target:Ŀ����ʱ�䣬num:���Ƶ���Ŀ+1��X��Y:���Ƶ�����
* @retval 
*/
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
{ 
	float PathPlanerror_X;
	float PathPlanerror_Y;
	k = (int)(t_real * num / t_target);	// ��k��
	t = t_real - k * t_target / num;		// ��k��ʱ��
  t = t * num / t_target;							// ��һ��

	// λ����������
	f1s = (1 - t) * (1 - t) * (1 - t) / 6;
	f2s = (3 * t * t * t - 6 * t * t + 4) / 6;
	f3s = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
	f4s = (t * t * t) / 6;
	
	// ����Ŀ����ٵ�
	now_path_point.X = X[k] * f1s + X[k+1] * f2s + X[k+2] * f3s + X[k+3] * f4s;
	now_path_point.Y = Y[k] * f1s + Y[k+1] * f2s + Y[k+2] * f3s + Y[k+3] * f4s;
	now_path_point.Yaw = Yaw[k] * f1s + Yaw[k+1] * f2s + Yaw[k+2] * f3s + Yaw[k+3] * f4s;
	if(first_time_flag)
	{
		now_path_point.V_x = 0;
		now_path_point.V_y = 0;
		now_path_point.W = 0;
		first_time_flag = 0;
		Hz = 1 / t_real;
	}
	else
	{
		now_path_point.V_x = (now_path_point.X - last_X) * Hz;
		now_path_point.V_y = (now_path_point.Y - last_Y) * Hz;
		now_path_point.W = (now_path_point.Yaw - last_Yaw) * Hz;
	}
	if(t_real < (t_target))
	{
	// PD������
	PDController(now_path_point, ROBOT_REAL_POS_DATA);
//	PathPlanerror_X=ABS(ROBOT_REAL_POS_DATA.POS_X-now_path_point.X);
//	PathPlanerror_Y=ABS(ROBOT_REAL_POS_DATA.POS_Y-now_path_point.Y);
	}	
	// ��������ֵ
	last_X = now_path_point.X;
	last_Y = now_path_point.Y;
	last_Yaw = now_path_point.Yaw;
	
	// �����յ�
	if(t_real > (t_target))
	{
		
//		if(moving_point_track(X[num+3], Y[num+3], Yaw[num+3],200))
//		{
		first_time_flag = 1;
		Robot_Chassis.World_V[1] = 0;//x��
		Robot_Chassis.World_V[0] = 0;//y��
		
			return 1;
//		}
	
	} 
	else	return 0;
}

PID point_pid;//��Ե�׷��PID
PID yaw_pid;//�Ƕ�PID

//�����
int moving_point_track(float POS_X, float POS_Y, float POS_YAW,float V_max)
{
		YawAdjust(POS_YAW);
	 float error;
	
	  //�������
	error = sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));  // �������
	point_pid.outputmax = ABS(V_max);
  PID_position_PID_calculation_by_error(&point_pid, error);
	Robot_Chassis.World_V[1] =-(point_pid.output * 1.0f*(ROBOT_REAL_POS_DATA.POS_X - POS_X) /error);//x��
	Robot_Chassis.World_V[0] = -(point_pid.output * 1.0f*(ROBOT_REAL_POS_DATA.POS_Y - POS_Y) /error);//y��
	
	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X)<10&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y)<10)
	{
		return 1;
	}
		return 0;
}

/**
* @brief  YawAdjustƫ���ǿ���
* @note		��ƫ���ǿ�����Ŀ��Ƕ�
* @param  Target_angle:Ҫ���Ƶ�ֵ
* @retval 
*/
int YawAdjust(float Target_angle)
{
   float YawAdjust_error;
 
	 // �������
   if(ROBOT_REAL_POS_DATA.POS_YAW*Target_angle >= 0)
   {
      YawAdjust_error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
   }
   else
   {
		 if(ABS(ROBOT_REAL_POS_DATA.POS_YAW)+ABS(Target_angle) <= 180) YawAdjust_error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
		 else 
		 {
				AngleLimit(&YawAdjust_error);
		 }
   }
   
   // ֱ������PID������ٶ�
   PID_position_PID_calculation_by_error(&yaw_pid, YawAdjust_error);
  Robot_Chassis.World_V[2]= -yaw_pid.output;	// ���̽��ٶ� ��λ��rad/s
	 
	  if(ABS(YawAdjust_error)<0.5)return 1;
	 else 
	 {
		 return 0;
	 }
}	

/**
* @brief  AngleLimit�Ƕ��޷�
* @note		���Ƕ�������-180�㵽180��
* @param  angle:Ҫ���Ƶ�ֵ
* @retval 
*/

void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}

///**
//* @brief  ���ι滮
//* @note		���Ƕ�������-180�㵽180��
//* @param  
//* @retval 
//*/
//int TrapezoidPlaning_set(float real_time,float T,float POS_X,float POS_Y,float POS_YAW,float V_max,float V_ac,float V_de)
//{
//	YawAdjust(POS_YAW);
//	float error,error_0;
//	float t_1,t_2,t_3;
//	float target_POS,target_X,target_Y;
//	float ac_rate,de_rate;
//	//�������
//	error = sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));  // �������
//  //����һЩ����
//	ac_rate=V_max/(V_ac*T);
//	de_rate=V_max/(V_de*T);
//	
//	if(real_time/T<=V_ac)//���ٶ�
//	{
//		target_POS = ac_rate*real_time*real_time/2.0f;
//	}
//	if((real_time/T>=V_ac)&&(real_time/T<=(1-V_de)))//���ٶ�
//	{
//		target_POS = ac_rate*(V_ac*T)*(V_ac*T)/2.0f+V_max*(real_time-V_ac*T);
//	}
//	if(real_time/T>(1-V_de))//���ٶ�
//	{
//		target_POS = ac_rate*(V_ac*T)*(V_ac*T)/2.0f+V_max*(real_time-V_ac*T);
//	}
//	
//  //�ֽ��ٶ�
//	target_X =(ROBOT_REAL_POS_DATA.POS_X - POS_X) /error;//x��
//	target_Y =(ROBOT_REAL_POS_DATA.POS_Y - POS_Y) /error;//y��
//	
//	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X)<10&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y)<10)
//	{
//		
//		return 1;
//	}
//	else return 0;
//		
//}
//void TrapezoidPlaning(float t_real)

/**
* @brief  ���ι滮
* @note		���Ƕ�������-180�㵽180��
* @param  
* @retval 
*/
//int TrapezoidPlaning(float real_time,
//										 float T,
//										 float POS_X,float POS_Y,float POS_YAW,
//										 float V_start,
//										 float V_end,
//										 float V_max,
//										 float R_ac,
//										 float R_de,
//										 int* first_time_flag)
//{	
////	YawAdjust(POS_YAW);
//	float error,error_0;
//	float target_POS,target_X,target_Y;

//	
//	float Ssu;   //��·��
//	float Sac;   //����·��
//	float Sde;   //����·��
//	float Sco;   //����·��
//	float Aac;   //���ټ��ٶ�
//	float Ade;   //���ټ��ٶ�
//	float S;     //��ǰ·��
//	float output_V;//������ٶ�
//	float real_error;//��ʵ���
//	
//  //����һЩ����
//	if(*first_time_flag)//��һ��
//	{
//		error_0=sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));  // �������
//		*first_time_flag=0;
//	}

//	////	// �����������������ִ���ٶȹ滮		
//	if((R_ac > 1) || (R_ac < 0) ||		//����·�̵ı���
//		 (R_de > 1) || (R_de < 0) ||	//����·�̵ı���
//		 (V_max < V_start) )			//�����ٶ�<��ʼ���ٶ� 
//	{
//		Robot_Chassis.World_V[1]=0;  // ���˶�
//		Robot_Chassis.World_V[0]=0;
//		return 1;
//	}
//		// ����һЩ����
//	Ssu = ABS(error_0); 	//��·��   
//	Sac = Ssu * R_ac;		//����·�� =	��·�� * ����·�̵ı���
//	Sde = Ssu * R_de;		//����·�� =	��·�� * ����·�̵ı���
//	Sco = Ssu - Sac - Sde;		//����·�� = ��·�� - ����·�� - ����·��
//	Aac = (V_max * V_max - V_start * V_start) / (2.0f * Sac);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
//	Ade = (V_end * V_end - V_max *   V_max) / (2.0f * Sde);	
//	real_error=sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));
//	// �����쳣���
//	if(error_0<real_error)		//����ʵ�����������ʱ
//	{
//		output_V = V_start;	//TARGET_RPM = ��ʼ���ٶ�
//	}

//	else
//	{
//		S = ABS(error_0 - sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y)));      //��ʼλ��
//		
//		// �滮RPM
//		if     (S < Sac)       output_V = sqrt(2.0f * Aac * S + V_start * V_start);               // ���ٽ׶�
//		else if(S < (Sac+Sco)) output_V = V_max;                                                        // ���ٽ׶�
//		else                   output_V = sqrt(V_end * V_end - 2.0f * Ade * ABS(Ssu - S));  // ���ٽ׶�
//	}
//	 
//  //�ֽ��ٶȣ���������ʵ�������
//	Robot_Chassis.World_V[1] =(output_V * 1.0f*(POS_X - ROBOT_REAL_POS_DATA.POS_X) /real_error);//x��
//	Robot_Chassis.World_V[0] = (output_V * 1.0f*(POS_Y - ROBOT_REAL_POS_DATA.POS_Y) /real_error);//y��
//	
//	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X)<3&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y)<3)
//	{
//		
//		return 1;
//		output_V=V_end;
//	}
//	else return 0;
//		
//}
//	

int chassis_TrapezoidPlaning(float POS_X_start,
	                    float POS_Y_start,
											float POS_X_end,
											float POS_Y_end,
											float POS_YAW,
											float V_start,
											float V_end,
											float V_max,
											float R_ac,
											float R_de)
{
//�����������
	float Ssu_chassis;   //��·��
	float Sac_chassis;   //����·��
	float Sde_chassis;   //����·��
	float Sco_chassis;   //����·��
	float Aac_chassis;   //���ټ��ٶ�
	float Ade_chassis;   //���ټ��ٶ�
	float S_chassis;     //��ǰ·��
	float output_V;//������ٶ�
	float real_error;//��ʵ���


	
	YawAdjust(POS_YAW);
		// �����������������ִ���ٶȹ滮		
	if((R_ac > 1) || (R_ac < 0) ||		//����·�̵ı���
		 (R_de > 1) || (R_de < 0) ||	//����·�̵ı���
		 (V_max < V_start) )			//�����ٶ�<��ʼ���ٶ� 
	{
		Robot_Chassis.World_V[1]=0;  // ���˶�
		Robot_Chassis.World_V[0]=0;
		return 1;
	}
	//�����г̱���
	Ssu_chassis=sqrt((POS_X_end-POS_X_start)*(POS_X_end-POS_X_start)+(POS_Y_end-POS_Y_start)*(POS_Y_end-POS_Y_start));
	Sac_chassis=Ssu_chassis*R_ac;
	Sde_chassis=Ssu_chassis*R_de;
	Sco_chassis=Ssu_chassis-Sac_chassis-Sde_chassis;
	Aac_chassis = (V_max * V_max - V_start * V_start) / (2.0f * Sac_chassis);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
//  	if(Aac_chassis>1800)
//		Aac_chassis=1200;//500mm/s
	Ade_chassis = (V_end * V_end - V_max *   V_max) / (2.0f * Sde_chassis);	//���ټ��ٶ�
//	  if(Ade_chassis>600)
//		Ade_chassis=600;//500mm/s
	real_error=sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X_end) * (ROBOT_REAL_POS_DATA.POS_X - POS_X_end) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y_end) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y_end));
		//�����쳣���
		if(Ssu_chassis<S_chassis)
		{
		output_V = -V_start;	//TARGET_RPM = ��ʼ���ٶ�
		}
		
			else
	{
		S_chassis = sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X_start) * (ROBOT_REAL_POS_DATA.POS_X - POS_X_start) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y_start) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y_start));   //��ʼλ��
		
		// �滮RPM
		if     (S_chassis < Sac_chassis)       output_V = sqrt(2.0f * Aac_chassis * S_chassis + V_start * V_start);               // ���ٽ׶�
		else if(S_chassis < (Sac_chassis+Sco_chassis)) output_V = sqrt(2.0f * Aac_chassis * Sac_chassis + V_start * V_start);                                                        // ���ٽ׶�
		else                   output_V = sqrt(V_end * V_end - 2.0f * Ade_chassis * ABS(Ssu_chassis - S_chassis));  // ���ٽ׶�
	}
	
	//�ֽ��ٶȣ���������ʵ�������
	Robot_Chassis.World_V[1] =(output_V * 1.0f*(POS_X_end - ROBOT_REAL_POS_DATA.POS_X) /real_error);//x��
	Robot_Chassis.World_V[0] = (output_V * 1.0f*(POS_Y_end - ROBOT_REAL_POS_DATA.POS_Y) /real_error);//y��
	
	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X_end)<100&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y_end)<100)//��ǰ����
	{
		testflag=1;
		output_V=0;
		Robot_Chassis.World_V[1]=0;
		Robot_Chassis.World_V[0]=0;
		return 1;
		
	}
	else return 0;
	
	
}



//��������
int near_pillar(float x,float y,float POS_YAW,float V_max)
{
	  
	 //�������
	 
		  if(ABS(y)>5||ABS(x)>5||ABS(POS_YAW)>0.1)
			{
   PID_position_PID_calculation_by_error(&point_traker_ladar_y_pid,y);
	 PID_position_PID_calculation_by_error(&point_traker_ladar_x_pid,x);
	 PID_position_PID_calculation_by_error(&yaw_pid_ladar,POS_YAW);
	 point_traker_ladar_y_pid.outputmax = ABS(V_max);
	 point_traker_ladar_x_pid.outputmax=ABS(V_max);
	 
	
	Robot_Chassis.World_V[0]=point_traker_ladar_y_pid.output;
	Robot_Chassis.World_V[1]=point_traker_ladar_x_pid.output;
	Robot_Chassis.World_V[2]=-yaw_pid_ladar.output;
			}
			else return 1;
			
	
		
		
		return 0;
		
}


  float S;     //��ǰ·��
int	laser_speed_case=0;
int laser_speed_control(float Xstart,float Ystart,//��ʼʱ�������ֵ
												float Xend,  float Yend,  //����ʱĿ��ļ�����ֵ
												int Xnow,  int Ynow,	//ʵʱ�ļ�����ֵ
												float Vmax,  float Vstart,float Vend,float POS_YAW)//����ٶȣ���ʼ�ٶȣ������ٶ�
{
	YawAdjust(POS_YAW);
	  float Ssu;   //��·��
	  float Sac;   //����·��
	  float Sde;   //����·��
	  float Sco;   //����·��
	  float Aac;   //���ټ��ٶ�
	  float Ade;   //���ټ��ٶ�
	
	  
	float output;
	int laser_ok=0;
//	  float S_laser;
	
		Ssu = sqrt((Xstart-Xend)*(Xstart-Xend)-
							 (Ystart-Yend)*(Ystart-Yend));//��·��
		S = sqrt((Xnow-Xstart)*(Xnow-Xstart)-
						 (Ynow-Ystart)*(Ynow-Ystart));//��ǰ·��
		Aac = 500;	//���ټ��ٶ� 
		
	  if(Vstart > Vmax)   //�����ٶȿ�������ٶ� �Ƚ���
		{
			Aac = -ABS(Aac) ;	//���ٶȱ���� 
		}
	  Ade = -500; //���ټ��ٶ�
	  Sac = ( Vmax * Vmax - Vstart * Vstart ) / (2.0f * Aac);
	  Sde = ( Vend * Vend -  Vmax  *  Vmax  ) / (2.0f * Ade);
		Sco = Ssu - Sac - Sde ;//����·��
		
laser_speed_case=1;
	
		if((Sac + Sde ) > Ssu )//���벻��  �������ǰ���Դﵽ������ٶ�Vmax �ٸ���Vmax���·���·��
		{
			Aac = ABS(Aac);	
			Vmax = sqrt((2*Aac*Ade*Ssu+Ade*Vstart*Vstart-Aac*Vend*Vend)/(Ade-Aac));
		 if(Vstart > Vmax)    //�����ٶȿ�������ٶ�  ֱ��ȫ�̼���
		 {
			 Sde = Ssu;
			 Ade = ( Vend * Vend -  Vstart  *  Vstart ) / (2.0f * Sde);
			 Sco = 0;
			 Sac = 0;
laser_speed_case=2;
		 }
		 else  //�����ټ���
		 {
			Sac = ( Vmax * Vmax - Vstart * Vstart ) / (2.0f * Aac) ;  //����·��
	    Sde = ( Vend * Vend -  Vmax  *  Vmax  ) / (2.0f * Ade) ;  //����·��
		  Sco = 0;
     laser_speed_case=3;
		 }
		}
   if     (S < Sac)            { output = sqrt(2.0f * Aac * S + Vstart * Vstart); laser_ok=0;}             	// ���ٽ׶�
		else if(S < (Sac+Sco))      { output = Vmax; laser_ok=0;}                                                 	// ���ٽ׶�
		else if(S < (Sac+Sco+Sde))  { output = sqrt(Vend * Vend - 2.0f * Ade * ABS(Ssu - S)); laser_ok=0;}         // ���ٽ׶�
		else                        { output = Vend; laser_ok=1;}                                                 // ֹͣ�׶�
		//ע��xy���ٶȶ�Ӧ��xy
		Robot_Chassis.World_V[1] = -output*0.1f*(Xstart-Xnow)/ABS(Ssu-S);//x���/�����
		Robot_Chassis.World_V[0]=  -output*0.1f*(Ystart-Ynow)/ABS(Ssu-S);//y���/�����
		return laser_ok;
	
	}
		
//�����������
	float Ssu_laser;   //��·��
	float Sac_laser;   //����·��
	float Sde_laser;   //����·��
	float Sco_laser;   //����·��
	float Aac_laser;   //���ټ��ٶ�
	float Ade_laser;   //���ټ��ٶ�
	float S_laser;     //��ǰ·��
	float output_V_laser;//������ٶ�
	float real_error_laser;//��ʵ���
int chassis_TrapezoidPlaning_laser(float POS_X_start,
	                    float POS_Y_start,
											float POS_X_end,
											float POS_Y_end,
											float POS_YAW,
											float V_start,
											float V_end,
											float V_max,
											float R_ac,
											float R_de,
												int POS_X_now,
                        int POS_Y_now)
{
	
	//�ǶȽ���
	YawAdjust(POS_YAW);
	// �����������������ִ���ٶȹ滮	
	if((R_ac > 1) || (R_ac < 0) ||		//����·�̵ı���
		 (R_de > 1) || (R_de < 0) ||	//����·�̵ı���
		 (V_max < V_start) )			//�����ٶ�<��ʼ���ٶ� 
	{
		testflag=1;
		Robot_Chassis.World_V[1]=0;  // ���˶�
		Robot_Chassis.World_V[0]=0;
		return 0;
	}
	//�����г̱���
	Ssu_laser=sqrt((POS_X_end-POS_X_start)*(POS_X_end-POS_X_start)+(POS_Y_end-POS_Y_start)*(POS_Y_end-POS_Y_start));
	Sac_laser=Ssu_laser*R_ac;
	Sde_laser=Ssu_laser*R_de;
	Sco_laser=Ssu_laser-Sac_laser-Sde_laser;
	Aac_laser = (V_max * V_max - V_start * V_start) / (2.0f * Sac_laser);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
//  	if(Aac_chassis>1800)
//		Aac_chassis=1200;//500mm/s
	Ade_laser = (V_end * V_end - V_max *   V_max) / (2.0f * Sde_laser);	//���ټ��ٶ�
//	  if(Ade_chassis>600)
//		Ade_chassis=600;//500mm/s
	real_error_laser=sqrt((POS_X_now - POS_X_end) * (POS_X_now - POS_X_end) + (POS_Y_now - POS_Y_end) * (POS_Y_now - POS_Y_end));
	//�����쳣���
		if(Ssu_laser<S_laser)
		{
		output_V_laser = V_start;	//TARGET_RPM = ��ʼ���ٶ�
		}
	
			else
	 {
		S_laser = sqrt((POS_X_now - POS_X_start) * (POS_X_now - POS_X_start) + (POS_Y_now - POS_Y_start) * (POS_Y_now- POS_Y_start));   //��ʼλ��
		
		// �滮RPM
		if     (S_laser < Sac_laser)       output_V_laser = sqrt(2.0f * Aac_laser * S_laser + V_start * V_start);               // ���ٽ׶�
		else if(S_laser < (Sac_laser+Sco_laser)) output_V_laser = sqrt(2.0f * Aac_laser * Sac_laser + V_start * V_start);                                                        // ���ٽ׶�
		else if (S_laser<Ssu_laser)                 output_V_laser = sqrt(V_end * V_end - 2.0f * Ade_laser * ABS(Ssu_laser - S_laser));  // ���ٽ׶�
 	  else                                     output_V_laser=V_end;
	 }
	 	//�ֽ��ٶȣ���������ʵ�������
	if(direction==2)//����������������ұ��ƶ���kΪx,bΪy
	{Robot_Chassis.World_V[1] =-(output_V_laser * 1.0f*(POS_X_end - POS_X_now) /real_error_laser);//x��
	Robot_Chassis.World_V[0] = (output_V_laser * 1.0f*(POS_Y_end - POS_Y_now) /real_error_laser);//y��
	testflag=0;
	}
	else if(direction==1)//���ߣ�������������ƶ���b��xΪx,k��y Ϊy
	{
	Robot_Chassis.World_V[1] =(output_V_laser * 1.0f*(POS_X_end - POS_X_now) /real_error_laser);//x��
	Robot_Chassis.World_V[0] = (output_V_laser * 1.0f*(POS_Y_end - POS_Y_now) /real_error_laser);//y��
	

	}
	else
	{
	Robot_Chassis.World_V[1] =(output_V_laser * 1.0f*(POS_X_end - POS_X_now) /real_error_laser);//x��
	Robot_Chassis.World_V[0] = -(output_V_laser * 1.0f*(POS_Y_end - POS_Y_now) /real_error_laser);//y��

	}
	if(ABS(POS_X_now - POS_X_end)<5&&ABS(POS_Y_now - POS_Y_end)<5)
	{
		testflag=1;
		output_V_laser=V_end;
		return 1;
		
	}
	else return 0;
}



////�޸�����ȡ����λ,ĩ�ٶȲ�Ϊ0
//int Laser_calibration_point(float k, float b,float yaw,float v_max,int direction_1)
//{
//	float error_K_point,error_B_point,ERROR_point;

//	YawAdjust(yaw);
//	//ת��ָ���Ƕ�
//	error_K_point =  location_k - k;
//	error_B_point =  location_b - b;
//	ERROR_point=sqrt(error_K_point*error_K_point+error_B_point*error_B_point);
////�жϾ����Ƿ����
//	if(error_K_point<1500&&error_B_point<1500)
//	{
//		if(ABS(error_K_point)>8||ABS(error_K_point)>8)
//		{
////pidֱ����� 
//			laser_K_pid.outputmax = ABS(v_max);
//			PID_position_PID_calculation_by_error(&laser_K_pid, ERROR_point);
//			if(direction_1==1)
//			{
//			Robot_Chassis.World_V[0] = -laser_K_pid.output*1.0f*error_K_point/ERROR_point;
//			Robot_Chassis.World_V[1]= -laser_K_pid.output*1.0f*error_B_point/ERROR_point;
//			}
//			if(direction_1==2)
//			{
//			Robot_Chassis.World_V[1] = laser_K_pid.output*1.0f*error_K_point/ERROR_point;
//			Robot_Chassis.World_V[0]= -laser_K_pid.output*1.0f*error_B_point/ERROR_point;
//			}
//			error_K_point =  location_k - k;
//			error_B_point =  location_b - k;
//		}
//		else
//			return 1;
//	}
//   
//	return 0;
//}
//	

