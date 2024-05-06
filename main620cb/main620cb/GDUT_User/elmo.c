//
//JIAlonglong 2023.2.17
//
#include "elmo.h"
#include "stm32f4xx_hal.h"
////GDUT_JIAlonglong m3508��չ��
///*******************************��������������************************************/
///**
//* @brief  Elmo��������ʼ��
//* @param  CANx����ʹ�õ�CANͨ�����
//* @author ACTION
//*/
//void ElmoInit(CAN_HandleTypeDef* hcan)
//{
//    uint32_t data[1][2] = {0x00000001, 0x00000000};
//    uint32_t TxMailbox;
//    CAN_TxHeaderTypeDef TxMessage;
//    uint8_t TxData[8];
//		
//    TxMessage.StdId = ELMO_BROADCAST_ID;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 8;

//    TxData[0] = *(unsigned long*)&data[0][0] & 0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0] >> 8) & 0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0] >> 16) & 0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0] >> 24) & 0xff;
//    TxData[4] = *(unsigned long*)&data[0][1] & 0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1] >> 8) & 0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1] >> 16) & 0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1] >> 24) & 0xff;

//    if (HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &TxMailbox) != HAL_OK) {
//        // Failed to add message to the transmit mailbox
//    }
//}


///**
//* @brief  ���ʹ�ܣ�ͨ�磩
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @author ACTION
//* @note ELMO������Ĭ�ϳ�ʼ״̬Ϊ���ʧ�ܣ�ʹ�õ��ʱ��Ҫ�������ʹ��
//*       ����������������Ҫ�ڵ��ʧ��״̬�²ſ�������
//*/
//void MotorOn(CAN_HandleTypeDef* hcan, uint8_t ElmoNum)
//{
//	//��һ��������MO����ڶ���������1�����ʹ�ܣ�ͨ�磩
//	uint32_t data[1][2] = {
//		{ 0x00004F4D, 0x00000001 },
//	};
//						 
//	CAN_TxHeaderTypeDef TxHeader;
//	uint8_t TxData[8];
//	TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;	// standard identifier=0
//	TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;	// extended identifier=StdId
//	TxHeader.IDE = CAN_ID_STD;						// type of identifier for the message is Standard
//	TxHeader.RTR = CAN_RTR_DATA;						// the type of frame for the message that will be transmitted
//	TxHeader.DLC = 8;

//	uint32_t TxMailbox;
//	
//	//������ݵ�TxHeader.Data����
//	TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//	TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//	TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//	TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//	TxData[4] = *(unsigned long*)&data[0][1]&0xff;
//	TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//	TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//	TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

//	//��������
//	if(HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//	{
//		//����ʧ�ܣ��������
//	}

//	//�ȴ����ͳɹ�
//	uint16_t timeout = 0;
//	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3 && timeout < 60000)
//	{
//		timeout++;
//	}
//	if (timeout >= 60000) {
//		//���ͳ�ʱ���������
//	}
//}

///**
//* @brief  ���ʧ�ܣ��ϵ磩
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @author ACTION GDUT-JIAlonglong
//*/
//void MotorOff(CAN_HandleTypeDef* hcan, uint8_t ElmoNum) {
//    //��һ�����ݷ���MO����ڶ������ݷ���0�����ʧ�ܣ��ϵ磩
//    uint32_t data[1][2] = {
//        0x00004F4D,0x00000000,  //MO  0
//    };
//    uint32_t mbox;
//    CAN_TxHeaderTypeDef TxHeader;
//    uint8_t TxData[8];
//                     
//    TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;   // standard identifier=0
//    TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;   // extended identifier=StdId
//    TxHeader.IDE = CAN_ID_STD;                      // type of identifier for the message is Standard
//    TxHeader.RTR = CAN_RTR_DATA;                     // the type of frame for the message that will be transmitted
//    TxHeader.DLC = 8;

//    TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//    TxData[4] = *(unsigned long*)&data[0][1]&0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

//    //��������
//    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &mbox) != HAL_OK) {
//        // ����ʧ�ܴ���
//    }
//    
//    //�ȴ����ͳɹ�
//    uint16_t timeout = 0;
//    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3) {
//        timeout++;
//        if (timeout > 60000) {
//            // ����ʱ
//            break;
//        }
//    }
//}

///**
//* @brief  �������ٶȻ���ʼ��
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  acc�����ٶȣ���λ������ÿ���η���
//* @param  dec�����ٶȣ���λ������ÿ���η���
//* @author ACTION
//* @note ���ٶȻ���ʼ����ſ���ʹ�ܵ������
//*/
//void VelLoopCfg(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
//{
//  SetUnitMode(hcan, ElmoNum, SPEED_CONTROL_MODE);
//  SetAccAndDec(hcan, ElmoNum, acc, dec);
//}

///**
//* @brief  ���ü��ٶ�����ٶ�
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  acc�����ٶȣ���λ������ÿ���η���
//* @param  dec�����ٶȣ���λ������ÿ���η���
//* @author ACTION
//* @note
//*/
//void SetAccAndDec(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
//{
//    //��һ�����ݷ���AC\DC����ڶ������ݷ�������ֵ
//    uint32_t data[2][2]={
//                            0x00004341,0x00000000,    //AC
//                            0x00004344,0x00000000    //DC
//    };
//    uint32_t TxMailbox;
//    CAN_TxHeaderTypeDef TxMessage;
//		uint8_t TxData[8];

//    TxMessage.StdId = ELMO_DEVICE_BASEID + ElmoNum;       // standard identifier=0
//    TxMessage.ExtId = ELMO_DEVICE_BASEID + ElmoNum;       // extended identifier=StdId
//    TxMessage.IDE = CAN_ID_STD;                          // type of identifier for the message is Standard
//    TxMessage.RTR = CAN_RTR_DATA;                         // the type of frame for the message that will be transmitted
//    TxMessage.DLC = 8;

//    data[0][1] = acc;
//    data[1][1] = dec;

//    for(uint8_t i = 0; i < 2; i++)
//    {
//				TxData[0] = *(unsigned long*)&data[i][0]&0xff;
//				TxData[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
//				TxData[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
//				TxData[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
//				TxData[4] = *(unsigned long*)&data[i][1]&0xff;
//				TxData[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
//				TxData[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
//				TxData[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;

//        if (HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &TxMailbox) != HAL_OK) {
//            // ������Ӧ�����쳣����
//        }
//        
//        //�ȴ����ͳɹ�
//        uint16_t timeout = 0;
//        while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//        {
//            timeout++;
//            if(timeout > 60000)
//            {
//                // ������Ӧ�����쳣����
//            }
//        }
//    }
//}


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
//void PosLoopCfg(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint32_t acc, uint32_t dec,uint32_t vel)
//{
//    // set unit mode to position control mode
//    SetUnitMode(hcan, ElmoNum, POSITION_CONTROL_MODE);

//    // set acceleration and deceleration
//    SetAccAndDec(hcan, ElmoNum, acc, dec);

//    // set velocity for position loop
//    SetPosLoopVel(hcan, ElmoNum, vel);
//}


///**
//* @brief  ����λ�û���������ٶ�
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
//* @author ACTION
//* @note���ٶ������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
//*/
//void SetPosLoopVel(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, int32_t vel)
//{
//	//��һ�����ݷ���SP����ڶ������ݷ�������ֵ
//	uint32_t data[1][2]={
//		0x00005053,0x00000000,		//SP
//	};
//	
//	CAN_TxHeaderTypeDef TxHeader;
//	uint8_t TxData[8];
//	uint32_t TxMailbox;
//	
//	TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;
//	TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;
//	TxHeader.IDE = CAN_ID_STD;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.DLC = 8;
//	
//	data[0][1] = vel;
//	
//	TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//	TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//	TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//	TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//	TxData[4] =  *(unsigned long*)&data[0][1]&0xff;
//	TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//	TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//	TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

//	HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox);
//	
//	//�ȴ����ͳɹ�
//	uint16_t timeout = 0;
//	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//	{
//		timeout++;
//		if(timeout > 60000)
//		{
//			//������Ӧ�����쳣����
//		
//		}
//	}
//}


///**
//* @brief  ����ٶȿ���
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
//* @author ACTION
//*/
//void VelCrl(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, int32_t vel)
//{
//	SetJoggingVel(hcan, ElmoNum, vel);
//}

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
//void PosCrl(CAN_HandleTypeDef* hcan, uint8_t ElmoNum,uint8_t posMode,int32_t pos)
//{
//	SendPosCmd(hcan, ElmoNum, posMode, pos);
//}

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

//void SetUnitMode(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint8_t unitMode)
//{
//	//��һ�����ݷ���UM����ڶ������ݷ���ģʽ
//	uint32_t data[1][2]={
//						0x00004D55,0x00000000,      //UM
//					 };
//	uint32_t mailbox;
//	CAN_TxHeaderTypeDef TxHeader;
//	TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
//	TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;		// extended identifier=StdId
//	TxHeader.IDE = CAN_ID_STD;							// type of identifier for the message is Standard
//	TxHeader.RTR = CAN_RTR_DATA;							// the type of frame for the message that will be transmitted
//	TxHeader.DLC = 8;
//	uint8_t TxData[8];
//	data[0][1] = unitMode;
//	TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//	TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//	TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//	TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//	TxData[4] = *(unsigned long*)&data[0][1]&0xff;
//	TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//	TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//	TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
//	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &mailbox);
//	
//	//�ȴ����ͳɹ�
//	uint16_t timeout = 0;
//	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//	{
//		timeout++;
//		if(timeout > 60000)
//		{
//			//��ʱ����
//			break;
//		}
//	}
//}



///**
//* @brief  ���������ٶ�
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @param  vel: �ٶȣ���λ������ÿ�룬��Χ����С�ٶ����Ƶ�����ٶ�����
//* @author ACTION
//* @note���ٶ������Ŵ�����ת�ķ��򣬴�����Ϊ������С����Ϊ������
//*/
//void SetJoggingVel(CAN_HandleTypeDef *hcan, uint8_t ElmoNum, int32_t vel)
//{
//  // ��һ�����ݷ���JV����ڶ������ݷ�������ֵ
//  uint32_t data[1][2] = {
//      0x0000564A, 0x00000000,  // JV
//  };
//  uint32_t mbox;
//  CAN_TxHeaderTypeDef TxHeader;
//  uint8_t TxData[8];

//  TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;   // standard identifier=0
//  TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;   // extended identifier=StdId
//  TxHeader.IDE = CAN_ID_STD;                      // type of identifier for the message is Standard
//  TxHeader.RTR = CAN_RTR_DATA;                     // the type of frame for the message that will be transmitted
//  TxHeader.DLC = 8;

//  data[0][1] = vel;

//  TxData[0] = *(unsigned long *)&data[0][0] & 0xff;
//  TxData[1] = (*(unsigned long *)&data[0][0] >> 8) & 0xff;
//  TxData[2] = (*(unsigned long *)&data[0][0] >> 16) & 0xff;
//  TxData[3] = (*(unsigned long *)&data[0][0] >> 24) & 0xff;
//  TxData[4] = *(unsigned long *)&data[0][1] & 0xff;
//  TxData[5] = (*(unsigned long *)&data[0][1] >> 8) & 0xff;
//  TxData[6] = (*(unsigned long *)&data[0][1] >> 16) & 0xff;
//  TxData[7] = (*(unsigned long *)&data[0][1] >> 24) & 0xff;

//  HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &mbox);

//  while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)
//  {
//  }
//}


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
//void SendPosCmd(CAN_HandleTypeDef* hcan, uint8_t ElmoNum, uint8_t posMode, int32_t pos)
//{
//    uint32_t data[1][2] = {0x00000000, 0x00000000};  //PA
//    uint32_t mailbox;
//    CAN_TxHeaderTypeDef TxHeader;
//    uint8_t TxData[8];

//    TxHeader.StdId = ELMO_DEVICE_BASEID + ElmoNum;
//    TxHeader.ExtId = ELMO_DEVICE_BASEID + ElmoNum;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    TxHeader.DLC = 8;

//    if (posMode == ABSOLUTE_MODE)
//    {
//        data[0][0] = 0x00004150;  //����
//    }

//    data[0][1] = pos;

//    TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//    TxData[4] =  *(unsigned long*)&data[0][1]&0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

//		
//    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &mailbox) != HAL_OK)
//    {
//        // ���������
//    }

//    uint16_t timeout = 0;
//    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3)  // �ȴ���Ϣ�������
//    {
//        timeout++;
//        if (timeout > 60000)
//        {
//            // ��ʱ����
//        }
//    }
//}



///**********************************��ȡ��������������*************************************/

///**
//* @brief  ��ȡ���λ��
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @author ACTION
// * @note�����ձ�ʶ��Ϊ��0x00005850
//*/
//void ReadActualPos(CAN_HandleTypeDef* hcan, uint8_t ElmoNum)
//{
//    uint32_t data[1][2] = {
//        0x40005850, 0x00000000 //PX
//    };
//    uint32_t mailbox;
//    CAN_TxHeaderTypeDef TxMessage;
//		uint8_t TxData[8];
//    
//    TxMessage.StdId = ELMO_DEVICE_BASEID + ElmoNum;   // standard identifier=0
//    TxMessage.ExtId = ELMO_DEVICE_BASEID + ElmoNum;   // extended identifier=StdId
//    TxMessage.IDE = CAN_ID_STD;                     // type of identifier for the message is Standard
//    TxMessage.RTR = CAN_RTR_DATA;                    // the type of frame for the message that will be transmitted
//    TxMessage.DLC = 8;

//    TxData[0] = *(unsigned long*)&data[0][0] & 0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0] >> 8) & 0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0] >> 16) & 0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0] >> 24) & 0xff;
//    TxData[4] = *(unsigned long*)&data[0][1] & 0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1] >> 8) & 0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1] >> 16) & 0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1] >> 24) & 0xff;

//    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &mailbox);;

//    uint16_t timeout = 0;
//    while (status != HAL_OK)
//    {
//        timeout++;
//        if (timeout > 60000)
//        {
//            // handle timeout error
//        }
//        status = HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &mailbox);
//    }
//}


///**
//* @brief  ��ȡ����ٶ�
//* @param  CANx����ʹ�õ�CANͨ�����
//* @param  ElmoNum��������ID�ţ���Χ��0~128��0Ϊ�㲥��ID��
//* @author ACTION
// * @note�����ձ�ʶ��Ϊ��0x00005856
//*/
//void ReadActualVel(CAN_HandleTypeDef* hcan, uint8_t ElmoNum)
//{
//    uint32_t data[1][2] = {
//        0x40005856, 0x00000000,      // VX
//    };
//    CAN_TxHeaderTypeDef TxMessage;
//    uint8_t TxData[8];
//		uint32_t TxMailbox;
//    
//    TxMessage.StdId = ELMO_DEVICE_BASEID + ElmoNum;
//    TxMessage.ExtId = ELMO_DEVICE_BASEID + ElmoNum;
//    TxMessage.IDE = CAN_ID_STD;
//    TxMessage.RTR = CAN_RTR_DATA;
//    TxMessage.DLC = 8;
//    
//    TxData[0] = *(unsigned long*)&data[0][0]&0xff;
//    TxData[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
//    TxData[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
//    TxData[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
//    TxData[4] = *(unsigned long*)&data[0][1]&0xff;
//    TxData[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
//    TxData[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
//    TxData[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
//    
//    HAL_CAN_AddTxMessage(hcan, &TxMessage, TxData, &TxMailbox);
//    
//    uint16_t timeout = 0;
//    while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 3) {
//        timeout++;
//        if(timeout > 60000) {
//            // handle timeout
//        }
//    }
//}
