#include "interrupt_callback.h"
#include "control.h"
#include "usart.h"
#include "can.h"
#include "math.h"
#include <string.h>
#include "get_info.h"
#include "lcd.h"

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
		GetYaw();
		Get_Position();
		if (times % 5 == 0)
		{
			Calc_MotorSpeed_pid();
//		    Calc_MotorAngle_pid();
//			CAN_SetMotorCurrent(Motor[0].SpeedOutput + Motor[0].AngleOutput,
//			                    Motor[1].SpeedOutput + Motor[1].AngleOutput,
//			                    Motor[2].SpeedOutput + Motor[2].AngleOutput,
//			                    Motor[3].SpeedOutput + Motor[3].AngleOutput);
			CAN_SetMotorCurrent(Motor[0].SpeedOutput,
			                    Motor[1].SpeedOutput,
			                    Motor[2].SpeedOutput,
			                    Motor[3].SpeedOutput);
		}
		if (times % 10 == 0)
		{
			ReadEncoder();
		}
	}
}

//调底盘电机PID参数
char charBuf[4];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	u8 sum1 = 0,sum2 = 0,sum3 = 0,i = 0;
	
	if (huart->Instance == USART1)
	{
		
	}
	
//接收陀螺仪数据
//陀螺仪通行协议看OneNote
	if (huart->Instance == USART2)
	{
//		ReceiveIMUData();
		
		if ((USART2_RX_STA & 0x8000) == 0)  //接收未完成
		{
			USART2_RX_BUF[USART2_RX_STA & 0x3FFF] = aRxBuffer2[0];
			
			if ((USART2_RX_STA & 0x3FFF) == 0 && (USART2_RX_BUF[0] != 0x55))
				return;  //第一个数据不是帧头0x55，跳过
			if ((USART2_RX_STA & 0x3FFF) == 1 && (USART2_RX_BUF[1] != 0x53))
				return;  //第二个数据不是角度标志位0x53，跳过
			
			USART2_RX_STA ++;
			
			if ((USART2_RX_STA & 0x3FFF) == 11)
			{
				for (i = 0;i < 10; i ++)
				    sum2 += USART2_RX_BUF[i];
				
				if (sum2 == USART2_RX_BUF[10])
					USART2_RX_STA |= 0x8000;  //接收完成
				else
					USART2_RX_STA = 0;        //数据错误，重新接收
			}
		}
	}
	
	if (huart->Instance == USART3)
	{
//		ReceiveRadarData();
		if ((Radar.RX_STA & 0x8000) == 0)
		{
			Radar.RX_BUF[Radar.RX_STA & 0X3FFF] = aRxBuffer3[0];
			
			if ((Radar.RX_STA & 0X3FFF) == 0 && Radar.RX_BUF[0] != '#')
				return;  //第一个数据不是'#'
			if ((Radar.RX_STA & 0X3FFF) == 1 && Radar.RX_BUF[1] != '&')
			{            //第二个数据不是'&'
				Radar.RX_STA = 0;
				return;
			}
			if ((Radar.RX_STA & 0X3FFF) ==2 && Radar.RX_BUF[2] != 'r')
			{            //第三个数据不是'r'
				Radar.RX_STA = 0;
				return;
			}
			
			Radar.RX_STA ++;
			
			for (i = 0;i < 10;i++)
			    sum3 += Radar.RX_BUF[i];
			if (sum3 == Radar.RX_BUF[9])
				Radar.RX_STA |= 0x8000;  //接收完成
			else
				Radar.RX_STA = 0;        //数据错误，重新接收
		}
	}
		
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
