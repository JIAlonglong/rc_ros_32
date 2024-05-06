/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "moto.h"
#include "MIT.h"
CAN_TxHeaderTypeDef	TxHeader;      //����
uint8_t	RxData[8];  //���ݽ������飬can������ֻ֡��8֡
uint8_t	RxData2[8];
int Rx_Flag =1;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN1_Filter_Init();
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*CAN��������ʼ��*/
void CAN1_Filter_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
  sFilterConfig.FilterBank = 0;                    /* ��������0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* ����λģʽ */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* 32λ��*/
  
  sFilterConfig.FilterIdHigh         = (((uint32_t)CAN_RxExtId<<3)&0xFFFF0000)>>16;				/* Ҫ���˵�ID��λ *///0x0000
  sFilterConfig.FilterIdLow          = (((uint32_t)CAN_RxExtId<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; /* Ҫ���˵�ID��λ *///0x0000
//  sFilterConfig.FilterMaskIdHigh     = 0xFFFF;			/* ��������16λÿλ����ƥ�� */
//  sFilterConfig.FilterMaskIdLow      = 0xFFFF;			/* ��������16λÿλ����ƥ�� */
	sFilterConfig.FilterMaskIdHigh     = 0x0000;			
  sFilterConfig.FilterMaskIdLow      = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;           /* ��������������FIFO 0 */
  sFilterConfig.FilterActivation = ENABLE;          /* ʹ�ܹ����� */ 
  sFilterConfig.SlaveStartFilterBank = 14;
	
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
		/* Filter configuration Error */
    Error_Handler();
  }
	
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	TxHeader.ExtId=CAN_TxExtId;        //��չ��ʶ��(29λ)
	TxHeader.IDE=CAN_ID_EXT;    //ʹ�ñ�׼֡
	TxHeader.RTR=CAN_RTR_DATA;  //����֡
	TxHeader.DLC=8; 
	TxHeader.TransmitGlobalTime = DISABLE;
}

/*CAN��������ʼ��*/
void CAN2_Filter_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
  sFilterConfig.FilterBank = 0;                    /* ��������0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* ����λģʽ */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* 32λ��*/
  
  sFilterConfig.FilterIdHigh         = (((uint32_t)CAN_RxExtId<<3)&0xFFFF0000)>>16;				/* Ҫ���˵�ID��λ *///0x0000
  sFilterConfig.FilterIdLow          = (((uint32_t)CAN_RxExtId<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; /* Ҫ���˵�ID��λ *///0x0000
//  sFilterConfig.FilterMaskIdHigh     = 0xFFFF;			/* ��������16λÿλ����ƥ�� */
//  sFilterConfig.FilterMaskIdLow      = 0xFFFF;			/* ��������16λÿλ����ƥ�� */
	sFilterConfig.FilterMaskIdHigh     = 0x0000;			
  sFilterConfig.FilterMaskIdLow      = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;           /* ��������������FIFO 0 */
  sFilterConfig.FilterActivation = ENABLE;          /* ʹ�ܹ����� */ 
  sFilterConfig.SlaveStartFilterBank = 14;
	
	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
		/* Filter configuration Error */
    Error_Handler();
  }
	
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	  /*##-4- Activate CAN RX notification #######################################*/
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	TxHeader.ExtId=CAN_TxExtId;        //��չ��ʶ��(29λ)
	TxHeader.IDE=CAN_ID_EXT;    //ʹ�ñ�׼֡
	TxHeader.RTR=CAN_RTR_DATA;  //����֡
	TxHeader.DLC=8; 
	TxHeader.TransmitGlobalTime = DISABLE;
	

}


int o=0;
/*CAN�����жϺ���*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

		
	 uint16_t m;
		
	if(hcan == &hcan1)
	{
	CAN_RxHeaderTypeDef	RxHeader;      //����
		Rx_Flag =1;  //���ձ�־λ
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
		m3508_update_m3508_info(&RxHeader,RxData);  // M3508������ݴ���
		for(m=0;m<8;m++)
			M3508AngleIntegral(&MOTOR_REAL_INFO[m]);//�ǶȻ���
		o++;
	}
	if(hcan == &hcan2)
	{
	  CAN_RxHeaderTypeDef	RxHeader;      //����
		Rx_Flag =1;  //���ձ�־λ
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData2);
		U8_update_info(&RxHeader,RxData2);  // U8������ݴ���
		DM43_update_info(&RxHeader,RxData2);
	}
	
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
