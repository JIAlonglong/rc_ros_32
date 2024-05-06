//#ifndef __ELMO_H
//#define __ELMO_H

//#include "stm32f4xx.h"

////ELMO������CAN�㲥ID��
//#define ELMO_BROADCAST_ID  (0x000)
////ELMO����������ID��ַ 
//#define ELMO_DEVICE_BASEID (0x300)
////ELMO����������ID��ַ
//#define SDO_RESPONSE_COB_ID_BASE 0x280

///******************����������ģʽ************************/
//#define SPEED_CONTROL_MODE    (2)
//#define POSITION_CONTROL_MODE    (5)
//#define HOMING_MODE   (6)
///*********************λ�û�����ģʽ**********************/
//#define ABSOLUTE_MODE (0)

///*******************************��������������************************************/
///**
//* @brief  Elmo��������ʼ��
//* @param  CANx����ʹ�õ�CANͨ�����
//* @author ACTION
//*/
//void ElmoInit(CAN_HandleTypeDef* hcan);

///**
//* @brief  ���ʹ�ܣ�ͨ�磩
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @author ACTION
//* @note ELMO������Ĭ�ϳ�ʼ״̬Ϊ���ʧ�ܣ�ʹ�õ��ʱ��Ҫ�������ʹ��
//*       ����������������Ҫ�ڵ��ʧ��״̬�²ſ�������
//*/
//void MotorOn(CAN_HandleTypeDef* hcan, uint8_t ElmoNum);

///**
//* @brief  ���ʧ�ܣ��ϵ磩
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @author ACTION
//*/
//void MotorOff(CAN_HandleTypeDef* hcan, uint8_t ElmoNum);

///**
//* @brief  �������ٶȻ���ʼ��
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  acc�����ٶȣ���λ������ÿ���η���
//* @param  dec�����ٶȣ���λ������ÿ���η���
//* @author ACTION
//* @note ���ٶȻ���ʼ����ſ���ʹ�ܵ������
//*/
//void VelLoopCfg(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec);

///**
//* @brief  ������λ�û���ʼ��
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  acc�����ٶȣ���λ������ÿ���η���
//* @param  dec�����ٶȣ���λ������ÿ���η���
//* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
//* @author ACTION
//* @note ��λ�û���ʼ����ſ���ʹ�ܵ������
//*/
//void PosLoopCfg(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec,uint32_t vel);

///**
//* @brief  ����ٶȿ���
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
//* @author ACTION
//*/
//void VelCrl(CAN_HandleTypeDef* hcan, uint8_t ElmoNum,int32_t vel);

///**
//* @brief  ���λ�ÿ���
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  posMode: λ�û�����ģʽ����Χ��
//				ABSOLUTE_MODE: ����λ��ģʽ
//				RELATIVE_MODE: ���λ��ģʽ
//* @param  pos:λ�������λ�����壬��Χ�����λ�����Ƶ���Сλ������
//* @author ACTION
//*/
//void PosCrl(CAN_HandleTypeDef* hcan, uint8_t ElmoNum,uint8_t posMode,int32_t pos);

///**
//* @brief  ���ü��ٶ�����ٶ�
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  acc�����ٶȣ���λ������ÿ���η���
//* @param  dec�����ٶȣ���λ������ÿ���η���
//* @author ACTION
//* @note
//*/
//void SetAccAndDec(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec);

///**
//* @brief  ����λ�û���������ٶ�
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
//* @author ACTION
//* @note���ٶ������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
//*/
//void SetPosLoopVel(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, int32_t vel);

///**
//* @brief  ��������������ģʽ
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  unitMode������������ģʽ����Χ��
//			TORQUE_CONTROL_MODE�����ؿ���ģʽ���ڸ�ģʽ�¿���ִ��TC��������
//			SPEED_CONTROL_MODE���ٶȿ���ģʽ���ڸ�ģʽ��ͨ������JVֵ�����ٶ�
//			MICRO_STEPPER_MODE��ֱ���������ʹ�ø�ģʽ
//			DUAL_POSITION_MODE��˫λ�ñջ�ģʽ
//			SINGLE_POSITION_MODE����λ�ñջ�ģʽ���ڸ�ģʽ�¿�������PA��PR��JV��PT��PVT�˶�
//* @author ACTION
//* @note ֻ���ڵ��ʧ��ʱ�������øò���
//*/
//void SetUnitMode(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint8_t unitMode);

///**
//* @brief  ���������ٶ�
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
//* @author ACTION
//* @note���ٶ������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
//*/
//void SetJoggingVel(CAN_HandleTypeDef* hcan, uint8_t ElmoNum,int32_t vel);

///**
//* @brief  ����λ�û�����
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  posMode: λ�û�����ģʽ����Χ��
//				ABSOLUTE_MODE: ����λ��ģʽ
//				RELATIVE_MODE: ���λ��ģʽ
//* @param  pos:λ�������λ�����壬��Χ�����λ�����Ƶ���Сλ������
//* @author ACTION
//* @note��λ�������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
//*/
//void SendPosCmd(CAN_HandleTypeDef* hcan, uint8_t ElmoNum,uint8_t posMode,int32_t pos);

///**********************************��ȡ��������������*************************************/

///**
//* @brief  ��ȡ���λ��
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @author ACTION
// * @note�����ձ�ʶ��Ϊ��0x00005850
//*/
//void ReadActualPos(CAN_HandleTypeDef* hcan, uint8_t ElmoNum);

///**
//* @brief  ��ȡ����ٶ�
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @author ACTION
// * @note�����ձ�ʶ��Ϊ��0x00005856
//*/
//void ReadActualVel(CAN_HandleTypeDef* hcan, uint8_t ElmoNum);

//#endif
