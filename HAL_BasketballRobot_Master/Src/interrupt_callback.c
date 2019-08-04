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

//定时器更新（益处）中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		if(RmtSta & 0x80)//上次有数据被接收到了
		{	
			RmtSta &= ~0X10;			        //取消上升沿已经被捕获标记
			if((RmtSta & 0X0F) == 0X00)
				RmtSta |= 1 << 6;               //标记已经完成一次按键的键值信息采集
			if((RmtSta & 0X0F) < 14)
				RmtSta ++;
			else
			{
				RmtSta &= ~(1 << 7);                //清空引导标识
				RmtSta &= 0XF0;	                //清空计数器	
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
	
	if (huart->Instance == USART1)  //如果是串口1
	{
		if ((Vision.RX_STA & 0x8000) == 0)  //接收未完成
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
	
//接收陀螺仪数据
//陀螺仪通行协议看OneNote
	if (huart->Instance == USART2)  //如果是串口2
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
	
	if (huart->Instance == USART3)  //如果是串口3
	{
//		ReceiveRadarData();
		if ((Radar.RX_STA & 0x8000) == 0)
		{
			Radar.RX_BUF[Radar.RX_STA & 0X3FFF] = aRxBuffer3[0];
			
			if ((Radar.RX_STA & 0X3FFF) == 0 && Radar.RX_BUF[0] != '@')
				return;  //第一个数据不是'#'
			if ((Radar.RX_STA & 0X3FFF) == 1 && Radar.RX_BUF[1] != '^')
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
			
			if ((Radar.RX_STA & 0X3FFF) == 10)
			{
				for (i = 0;i < 9;i++)
					sum3 += Radar.RX_BUF[i];
				if (sum3 == Radar.RX_BUF[9])
					Radar.RX_STA |= 0x8000;  //接收完成
				else
					Radar.RX_STA = 0;        //数据错误，重新接收
		    }
		}
	}

	//调底盘电机PID参数
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

//定时器输入捕获中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)  //捕获中断发生执行
{
	if (htim -> Instance == TIM1)
	{
		if (RDATA)  //上升沿捕获
		{
			TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1);  //一定要先清楚原来的设置！
			TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,
			                        TIM_ICPOLARITY_FALLING);  //CC1P = 1，设置为下降沿捕获
			__HAL_TIM_SET_COUNTER(&htim1,0);                  //清空定时器值
			RmtSta |= 0x10;                                   //标记上升沿已被捕获	
		}
		else        //下降沿捕获
		{
			Dval = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);  //读取CCR1也可以清CC1IF标志位
			TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1);         //一定要先清楚原来的设置！
			TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,
			                        TIM_ICPOLARITY_RISING);          //配置TIM1通道1上升沿捕获
			
			if (RmtSta & 0X10)  //完成一次高电平捕获
			{
				if (RmtSta & 0X80)  //接收到了引导码
				{
					if (Dval > 300 && Dval < 800)  //560为标准值，560us
					{
						RmtRec <<= 1;      //左移一位
						RmtRec |= 0;       //接收到0
					}
					else if (Dval >1400 && Dval < 1800)  //1680为标准值，1680us
					{
						RmtRec <<= 1;      //左移一位
						RmtRec |= 1;       //接收到1
					}
					else if (Dval >2200 && Dval < 2600)  //得到按键键值的信息2500为标准值，2.5ms
					{
						RmtCnt ++;         //按键次数增加1次
						RmtSta &= 0XF0;    //清空计时器
					}
					else if (Dval >4200 && Dval < 4700)  //4500为标准值，4.5ms
					{
						RmtSta |= 1 << 7;  //标记成功接收了引导码
						RmtCnt = 0;        //清除按键次数计数器
					}
				}
				RmtSta &= ~(1 << 4);
			}
		}
	}
}
