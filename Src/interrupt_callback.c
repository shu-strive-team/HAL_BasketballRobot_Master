#include "interrupt_callback.h"
#include "control.h"
#include "usart.h"
#include "can.h"
#include "math.h"
#include <string.h>

uint8_t times;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, canRxDataBuf);
		CAN_GetMotorData(&Rx1Message, canRxDataBuf);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		times ++;
		if (times % 5 == 0)
		{
			SetMotorSpeed_pid();
		    SetMotorAngle_pid();
			Motor[2].AngleOutput = 0;
			Motor[3].AngleOutput = 0;
			Motor[0].AngleOutput = 0;
			CAN_SetMotorCurrent(Motor[0].AngleOutput,Motor[1].AngleOutput,Motor[2].AngleOutput,Motor[3].AngleOutput);

		}
	}
}

//调底盘电机PID参数
char charBuf[4];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*************PID参数串口数据处理***********/
	if (huart->Instance == UART4)
	{
		rxPID.Buf[rxPID.Count & 0x7f] = rxPID.pidReadBuf;
		//是否开始接收
		if ((rxPID.Count & 0x7f) == 0 && rxPID.Buf[0] != '$')
			return;

		rxPID.Count++;

		if ((rxPID.Count & 0x7f) == 8)
		{
			//接收正确
			if (rxPID.Sum == rxPID.pidReadBuf)
			{
				for (int i = 0; i < 4; i++)
					charBuf[i] = rxPID.Buf[i + 3];

				switch (rxPID.Buf[1])
				{
				case 'p':
					memcpy(&rxPID.pidAdjust->p, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->p = -rxPID.pidAdjust->p;
					break;
				case 'i':
					memcpy(&rxPID.pidAdjust->i, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->i = -rxPID.pidAdjust->i;
					break;
				case 'd':
					memcpy(&rxPID.pidAdjust->d, charBuf, 4);
					if (rxPID.Buf[2] == '-')
						rxPID.pidAdjust->d = -rxPID.pidAdjust->d;
					break;
				}
				rxPID.Sum = 0;
				rxPID.Count = 0;
			}
			else
			{
				rxPID.Sum = 0;
				rxPID.Count = 0;
			}
		}
		else
			rxPID.Sum += rxPID.pidReadBuf;
	}
}
