#include "interrupt_callback.h"
#include "control.h"
#include "usart.h"
#include "can.h"
#include "math.h"
#include <string.h>
#include "get_info.h"
#include "lcd.h"
#include "remote.h"
#include "lcd.h"

uint16_t times = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, canRxDataBuf);
		CAN_GetMotorData(&Rx1Message, canRxDataBuf);
	}
}

//��ʱ�����£��洦���жϻص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		if(RmtSta & 0x80)//�ϴ������ݱ����յ���
		{	
			RmtSta &= ~0X10;			        //ȡ���������Ѿ���������
			if((RmtSta & 0X0F) == 0X00)
				RmtSta |= 1 << 6;               //����Ѿ����һ�ΰ����ļ�ֵ��Ϣ�ɼ�
			if((RmtSta & 0X0F) < 14)
				RmtSta ++;
			else
			{
				RmtSta &= ~(1 << 7);                //���������ʶ
				RmtSta &= 0XF0;	                //��ռ�����	
			}						 	   	
		}							    
	}
	
	if (htim->Instance == TIM5)
	{
		times ++;
		
		if (times % 5 == 0)
		{
//			Calc_MotorAngle_pid();
//			CAN_SetMotorCurrent(Motor[0].Double_LoopOutput,Motor[1].Double_LoopOutput,
//								Motor[2].Double_LoopOutput,Motor[3].Double_LoopOutput);
			Calc_MotorSpeed_pid();
			CAN_SetMotorCurrent(Motor[0].Single_LoopOutput,Motor[1].Single_LoopOutput,
								Motor[2].Single_LoopOutput,Motor[3].Single_LoopOutput);
		}
		
		if (times % 10 == 0)
		{
//			output[0]=BasketballRobot.ThetaD-BasketballRobot.LastTheta;
//			output[1]=BasketballRobot.ThetaD;
//			output[2]=BasketballRobot.LastTheta;
//			sendware(output,sizeof(output));
			GetRadarData();
			GetYaw();
		    Get_Position();
		}
		
		if (times % 50 == 2)
		{
			LCD_Show_Obo();
			LCD_Show_setspeed();
			LCD_Show_getspeed();
			LCD_Show_V();
			LCD_Show_position();
		}
		
		if (times % 200 == 0)
			motortime ++;
		
		if (motortime > 60000)
			motortime = 0;
		
		if (times > 60000)
			times = 0;
	}
}

char charBuf[4];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	u8 sum1 = 0,sum2 = 0,sum3 = 0,i;
	
	if (huart->Instance == USART1)  //����Ǵ���1
	{
		if ((Vision.RX_STA & 0x8000) == 0)  //����δ���
		{
			Vision.RX_BUF[Vision.RX_STA & 0X3FFF] = aRxBuffer1[0];
			
			if ((Vision.RX_STA & 0X3FFF) == 0 && Vision.RX_BUF[0] != '@')
				return;
			if ((Vision.RX_STA & 0X3FFF) == 1 && Vision.RX_BUF[1] != '^')
			{
				Vision.RX_STA = 0;
				return;
			}
			if ((Vision.RX_STA & 0X3FFF) == 2 && Vision.RX_BUF[2] != 'v')
			{
				Vision.RX_STA = 0;
				return;
			}
			
			Vision.RX_STA ++;
			
			if ((Vision.RX_STA & 0X3FFF) == 10)
			{
				for (i = 0; i < 9; i++)
					sum1 += Vision.RX_BUF[i];
				if (sum1 == Vision.RX_BUF[9])
					Vision.RX_STA |= 0x8000;
				else
					Vision.RX_STA = 0;
			}		
		}
	}
	
//��������������
//������ͨ��Э�鿴OneNote
	if (huart->Instance == USART2)  //����Ǵ���2
	{
//		ReceiveIMUData();
		if ((USART2_RX_STA & 0x8000) == 0)  //����δ���
		{
			USART2_RX_BUF[USART2_RX_STA & 0x3FFF] = aRxBuffer2[0];
			
			if ((USART2_RX_STA & 0x3FFF) == 0 && (USART2_RX_BUF[0] != 0x55))
				return;  //��һ�����ݲ���֡ͷ0x55������
			if ((USART2_RX_STA & 0x3FFF) == 1 && (USART2_RX_BUF[1] != 0x53))
				return;  //�ڶ������ݲ��ǽǶȱ�־λ0x53������
			
			USART2_RX_STA ++;
			
			if ((USART2_RX_STA & 0x3FFF) == 11)
			{
				for (i = 0;i < 10; i ++)
				    sum2 += USART2_RX_BUF[i];
				
				if (sum2 == USART2_RX_BUF[10])
					USART2_RX_STA |= 0x8000;  //�������
				else
					USART2_RX_STA = 0;        //���ݴ������½���
			}
		}
	}
	
	if (huart->Instance == USART3)  //����Ǵ���3
	{
//		ReceiveRadarData();
		if ((Radar.RX_STA & 0x8000) == 0)
		{
			Radar.RX_BUF[Radar.RX_STA & 0X3FFF] = aRxBuffer3[0];
			
			if ((Radar.RX_STA & 0X3FFF) == 0 && Radar.RX_BUF[0] != '@')
				return;  //��һ�����ݲ���'#'
			if ((Radar.RX_STA & 0X3FFF) == 1 && Radar.RX_BUF[1] != '^')
			{            //�ڶ������ݲ���'&'
				Radar.RX_STA = 0;
				return;
			}
			if ((Radar.RX_STA & 0X3FFF) ==2 && Radar.RX_BUF[2] != 'r')
			{            //���������ݲ���'r'
				Radar.RX_STA = 0;
				return;
			}
			
			Radar.RX_STA ++;
			
			if ((Radar.RX_STA & 0X3FFF) == 10)
			{
				for (i = 0;i < 9;i++)
					sum3 += Radar.RX_BUF[i];
				if (sum3 == Radar.RX_BUF[9])
					Radar.RX_STA |= 0x8000;  //�������
				else
					Radar.RX_STA = 0;        //���ݴ������½���
		    }
		}
	}

	//�����̵��PID����
	/*************PID�����������ݴ���***********/
	if (huart->Instance == UART4)
	{
		rxPID.Buf[rxPID.Count & 0x7f] = rxPID.pidReadBuf;
		//�Ƿ�ʼ����
		if ((rxPID.Count & 0x7f) == 0 && rxPID.Buf[0] != '$')
			return;

		rxPID.Count++;

		if ((rxPID.Count & 0x7f) == 8)
		{
			//������ȷ
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

//
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	u8 i = 0;
	
	if (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_ORE) != RESET)
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer2,1);
	}
}

//��ʱ�����벶���жϻص�����
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)  //�����жϷ���ִ��
{
	if (htim -> Instance == TIM1)
	{
		if (RDATA)  //�����ز���
		{
			TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1);  //һ��Ҫ�����ԭ�������ã�
			TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,
			                        TIM_ICPOLARITY_FALLING);  //CC1P = 1������Ϊ�½��ز���
			__HAL_TIM_SET_COUNTER(&htim1,0);                  //��ն�ʱ��ֵ
			RmtSta |= 0x10;                                   //����������ѱ�����	
		}
		else        //�½��ز���
		{
			Dval = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);  //��ȡCCR1Ҳ������CC1IF��־λ
			TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1);         //һ��Ҫ�����ԭ�������ã�
			TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,
			                        TIM_ICPOLARITY_RISING);          //����TIM1ͨ��1�����ز���
			
			if (RmtSta & 0X10)  //���һ�θߵ�ƽ����
			{
				if (RmtSta & 0X80)  //���յ���������
				{
					if (Dval > 300 && Dval < 800)  //560Ϊ��׼ֵ��560us
					{
						RmtRec <<= 1;      //����һλ
						RmtRec |= 0;       //���յ�0
					}
					else if (Dval >1400 && Dval < 1800)  //1680Ϊ��׼ֵ��1680us
					{
						RmtRec <<= 1;      //����һλ
						RmtRec |= 1;       //���յ�1
					}
					else if (Dval >2200 && Dval < 2600)  //�õ�������ֵ����Ϣ2500Ϊ��׼ֵ��2.5ms
					{
						RmtCnt ++;         //������������1��
						RmtSta &= 0XF0;    //��ռ�ʱ��
					}
					else if (Dval >4200 && Dval < 4700)  //4500Ϊ��׼ֵ��4.5ms
					{
						RmtSta |= 1 << 7;  //��ǳɹ�������������
						RmtCnt = 0;        //�����������������
					}
				}
				RmtSta &= ~(1 << 4);
			}
		}
	}
}
