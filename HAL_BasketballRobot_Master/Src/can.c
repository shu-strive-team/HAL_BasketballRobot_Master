/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */


u8 txData[8];
u8 canRxDataBuf[8];
uint32_t pTxMailbox;
CAN_TxHeaderTypeDef  Tx1Message;		//发送配置参数
CAN_RxHeaderTypeDef  Rx1Message;		//发送配置参数
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
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
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 3);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void CAN1_Init(void)						
{
	CAN_FilterTypeDef canfilter;
	
	//canfilter.FilterNumber = 0;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	
	//  //filtrate any ID you want here
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;
  
	canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;
	  //use different filter for can1&can2
	canfilter.FilterBank=0;
//    hcan1.pTxMsg = &Tx1Message;
//    hcan1.pRxMsg = &Rx1Message;
  

	HAL_CAN_ConfigFilter(&hcan1,&canfilter);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan1);
}

//底盘电调ID：201到204
void CAN_GetMotorData(CAN_RxHeaderTypeDef *pHeader, uint8_t aData[])
{
	switch (pHeader->StdId)
	{
		case 0x201:
		Motor[0].Angle = aData[0] << 8 | aData[1];
		Motor[0].Speed = aData[2] << 8 | aData[3];
		Motor[0].Current = aData[4] << 8 | aData[5];
		Motor[0].Temperature = aData[6];
		break;

    	case 0x202:
		Motor[1].Angle = aData[0] << 8 | aData[1];
		Motor[1].Speed = aData[2] << 8 | aData[3];
		Motor[1].Current = aData[4] << 8 | aData[5];
		Motor[1].Temperature = aData[6];
		break;

	    case 0x203:
		Motor[2].Angle = aData[0] << 8 | aData[1];
		Motor[2].Speed = aData[2] << 8 | aData[3];
		Motor[2].Current = aData[4] << 8 | aData[5];
		Motor[2].Temperature = aData[6];
		break;

	    case 0x204:
		Motor[3].Angle = aData[0] << 8 | aData[1];
		Motor[3].Speed = aData[2] << 8 | aData[3];
		Motor[3].Current = aData[4] << 8 | aData[5];
		Motor[3].Temperature = aData[6];
		break;
	}
}

//发送数据
//底盘发送数据时，标识符为0x200
void CAN_SetMotorCurrent(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
	
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	txData[0] = iq1 >> 8;
	txData[1] = iq1;
	txData[2] = iq2 >> 8;
	txData[3] = iq2;
	txData[4] = iq3 >> 8;
	txData[5] = iq3;
	txData[6] = iq4 >> 8;
	txData[7] = iq4;
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message, txData, &pTxMailbox);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
