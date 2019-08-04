#include "get_info.h"
#include "usart.h"
#include "gpio.h"

void ReadEncoder(void)
{
//	int16_t nEncoder1,nEncoder2;
	
	//读取CNT数值后清零
	BasketballRobot.w[0] = TIM2 -> CNT;
	TIM2 -> CNT = 0;
//	nEncoder1 = __HAL_TIM_GetCounter(&htim2);
//	__HAL_TIM_SetCounter(&htim2,0);
	
	BasketballRobot.w[1] = TIM4 -> CNT;
	TIM4 -> CNT = 0;
//	nEncoder2 = __HAL_TIM_GetCounter(&htim4);
//	__HAL_TIM_SetCounter(&htim4,0);
//	
//	BasketballRobot.w[0] = nEncoder1;
//	BasketballRobot.w[1] = nEncoder2;
}

//接收陀螺仪数据
//陀螺仪通行协议看OneNote
//舍弃不用，写在中断回调函数中
void ReceiveIMUData(void)
{
	uint8_t sum = 0,i = 0;
	if ((USART2_RX_STA & 0x8000) == 0)  //接收未完成
	{
		USART2_RX_BUF[USART2_RX_STA & 0X3FFF] = aRxBuffer2[0];
		
		if ((USART2_RX_STA & 0X3FFF) == 0 && USART2_RX_BUF[0] != 0x55)
			return;  //第一个数据不是帧头0x55，跳过
		if ((USART2_RX_STA & 0X3FFF) == 1 && USART2_RX_BUF[1] != 0x53)
			return;  //第二个数据不是角度标志位0x53，跳过
		
		USART2_RX_STA ++;
		
		if ((USART2_RX_STA & 0X3FFF) == 11)
		{
			for (i = 0;i < 10;i ++)
			    sum += USART2_RX_BUF[i];
			
			if (sum == USART2_RX_BUF[10])
				USART2_RX_STA |= 0x8000;  //接收完成
			else
				USART2_RX_STA = 0;        //数据错误，重新接收
		}
	}
}

//获取偏航角
void GetYaw(void)
{
	if (USART2_RX_STA & 0x8000)
	{
		BasketballRobot.LastTheta = BasketballRobot.ThetaD;
		
		BasketballRobot.ThetaD = (float)((USART2_RX_BUF[7] << 8) | (USART2_RX_BUF[6])) / 32768 * 180;
		BasketballRobot.ThetaR = BasketballRobot.ThetaD * PI / 180;
		
		while (BasketballRobot.ThetaD < 0)
			BasketballRobot.ThetaD += 360;
		while (BasketballRobot.ThetaD > 360)
			BasketballRobot.ThetaD -= 360;
		while (BasketballRobot.ThetaR < 0)
			BasketballRobot.ThetaR += 2 * PI;
		while (BasketballRobot.ThetaR > 2 * PI)
			BasketballRobot.ThetaR -= 2*PI;
		
		USART2_RX_STA = 0;
		
		LED0 = !LED0;
		LED1 = !LED1;
		
//		printf("yaw: %.2f   tim : %d \r\n    ",BasketballRobot.ThetaD, TIM5->CNT);
	}
}

////舍弃不用，写在中断回调函数中
void ReceiveRadarData(void)
{
	uint8_t sum = 0,i = 0;
	
	if ((Radar.RX_STA & 0x8000) == 0)  //接收未完成
	{
		Radar.RX_BUF[Radar.RX_STA & 0x3FFF] = aRxBuffer3[0];
		
		if ((Radar.RX_STA & 0X3FFF) == 0 && Radar.RX_BUF[0] != '#')
			return;  //第一个数据不是'#'
		if ((Radar.RX_STA & 0X3FFF) == 1 && Radar.RX_BUF[1] != '&')
		{            //第二个数据不是'&'
			Radar.RX_STA = 0;
			return;
		}
		if ((Radar.RX_STA & 0X3FFF) == 2 && Radar.RX_BUF[2] != 'r')
		{            //第二个数据不是'r'
			Radar.RX_STA = 0;
			return;
		}
		
		Radar.RX_STA ++;
		
		if ((Radar.RX_STA & 0X3FFF) == 10)
		{
			for (i = 0;i < 9;i ++)
			    sum += Radar.RX_BUF[i];
			if (sum == USART3_RX_BUF[9])
				Radar.RX_STA |= 0x8000;  //接收完成
			else
				Radar.RX_STA = 0;        //数据错误，重新接收
		}
	}
}

u32 a,d;
//雷达数据处理
u8 GetRadarData(void)
{
	
	if(Radar.RX_STA&0x8000)
	{
		a = (Radar.RX_BUF[3]<<8)|Radar.RX_BUF[4];
		d = (Radar.RX_BUF[5]<<8)|Radar.RX_BUF[6];
		Radar.RX_STA=0;
		
	}
	if(a<240 || a >300||d>4000||d<10) //原来&&
	{
		Radar.State = 0;
		return 0;
	}	
	else
	{
		
		Radar.Angle = a;
		Radar.Distance = d;
		Radar.State = 1;
		
		LCD_ShowString(30+200,660,200,16,16,"Radar:rad");	
		LCD_ShowNum(30+200+48+8+45,660,Radar.Angle,4,16);		
		LCD_ShowString(30+200,680,200,16,16,"Radar:length");	
		LCD_ShowNum(30+200+48+8+45,680,Radar.Distance,4,16);
		return 1;
	}
	
}

//舍弃不用，写在中断回调函数中
void ReceiveVisionData(void)
{
	uint8_t sum = 0,i = 0;
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
				sum += Vision.RX_BUF[i];
			if (sum == Vision.RX_BUF[9])
				Vision.RX_STA |= 0x8000;
			else
				Vision.RX_STA = 0;
		}		
	}
}

//视觉数据处理
u8 GetVisionData(void)
{
	u32 x,d;
	
	if (Vision.RX_STA & 0X8000)
	{
		x = (Vision.RX_BUF[3] << 8) | Vision.RX_BUF[4];
		d = (Vision.RX_BUF[5] << 8) | Vision.RX_BUF[6];
		Vision.RX_STA = 0;
	}
	
	if (x < 10 || x > 630 || d < 500)
	{
		Vision.State = 0;
		return 0;
	}
	else
	{
		Vision.X = x;
		Vision.Depth = d;
		Vision.State = 1;
		
		LCD_ShowString(30 + 200, 420, 200, 16, 16, "View :pix");
		LCD_ShowNum(30 + 200 + 48 + 8 + 45, 420, Vision.X, 4, 16);
		LCD_ShowString(30 + 200, 440, 200, 16, 16, "View :length");
		LCD_ShowNum(30 + 200 + 48 + 8 + 45, 440, Vision.Depth, 4, 16);
		
		return 1;
	}
}

//float sum1 = 0,sum2 = 0,sum3 = 0;
//float asd = 0;
//坐标转换，里程计定位
//具体计算推导去看论文
void Get_Position(void)
{
	//根据速度运算球场坐标
	float nW_1,nW_2,nX,nY;
	
	float l1,l2;            //减去自转偏差后的里程计值
	
	float theta_inv[2][2];  //坐标变换矩阵
	
	float d_theta;
	
	d_theta = BasketballRobot.ThetaD - BasketballRobot.LastTheta;
	//asd = d_theta;
	//sum3 += d_theta;
	
	if (d_theta > 300)
		d_theta -= 360;
	if (d_theta < -300)
		d_theta += 360;
	
	ReadEncoder();
	
	//theta_inv
	theta_inv[0][0] = cos(BasketballRobot.ThetaR);
	theta_inv[0][1] = -sin(BasketballRobot.ThetaR);
	theta_inv[1][0] = -theta_inv[0][1];
	theta_inv[1][1] = theta_inv[0][0];
	
//使用两个里程计定位
	
	//由陀螺仪得自转偏差
	nW_1 = d_theta / 180 * 8688; //7145、7370分别为为机器人转180°X轴和Y轴编码器的总计数
	nW_2 = d_theta / 180 * 8794;
	
	//sum1 += nW_1;
	//sum2 += nW_2;
	
	//除去自传偏差
	l1 = BasketballRobot.w[0] + nW_1;
	l2 = BasketballRobot.w[1] - nW_2;
	
	//23540 通过实际测试得出 ，所得坐标单位只是大概为1m
	nX = l1 / 23522;//23540.23526
	nY = l2 / 23522;
	
	BasketballRobot.X += nX * theta_inv[0][0] + nY * theta_inv[0][1];
	BasketballRobot.Y += nX * theta_inv[1][0] + nY * theta_inv[1][1];
	
	BasketballRobot.encoderCount[0] += BasketballRobot.w[0];
	BasketballRobot.encoderCount[1] += BasketballRobot.w[1];
}
