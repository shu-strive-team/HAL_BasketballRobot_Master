#include "get_info.h"
#include "usart.h"
#include "gpio.h"

void ReadEncoder(void)
{
//	int16_t nEncoder1,nEncoder2;
	
	//��ȡCNT��ֵ������
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

//��������������
//������ͨ��Э�鿴OneNote
void ReceiveIMUData(void)
{
	uint8_t sum = 0,i = 0;
	if ((USART2_RX_STA & 0x8000) == 0)  //����δ���
	{
		USART2_RX_BUF[USART2_RX_STA & 0X3FFF] = aRxBuffer2[0];
		
		if ((USART2_RX_STA & 0X3FFF) == 0 && USART2_RX_BUF[0] != 0x55)
			return;  //��һ�����ݲ���֡ͷ0x55������
		if ((USART2_RX_STA & 0X3FFF) == 1 && USART2_RX_BUF[1] != 0x53)
			return;  //�ڶ������ݲ��ǽǶȱ�־λ0x53������
		
		USART2_RX_STA ++;
		
		if ((USART2_RX_STA & 0X3FFF) == 11)
		{
			for (i = 0;i < 10;i ++)
			    sum += USART2_RX_BUF[i];
			
			if (sum == USART2_RX_BUF[10])
				USART2_RX_STA |= 0x8000;  //�������
			else
				USART2_RX_STA = 0;        //���ݴ������½���
		}
	}
}

//��ȡƫ����
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

void ReceiveRadarData(void)
{
	uint8_t sum = 0,i = 0;
	
	if ((Radar.RX_STA & 0x8000) == 0)  //����δ���
	{
		Radar.RX_BUF[Radar.RX_STA & 0x3FFF] = aRxBuffer3[0];
		
		if ((Radar.RX_STA & 0X3FFF) == 0 && Radar.RX_BUF[0] != '#')
			return;  //��һ�����ݲ���'#'
		if ((Radar.RX_STA & 0X3FFF) == 1 && Radar.RX_BUF[1] != '&')
		{            //�ڶ������ݲ���'&'
			Radar.RX_STA = 0;
			return;
		}
		if ((Radar.RX_STA & 0X3FFF) == 2 && Radar.RX_BUF[2] != 'r')
		{            //�ڶ������ݲ���'r'
			Radar.RX_STA = 0;
			return;
		}
		
		Radar.RX_STA ++;
		
		if ((Radar.RX_STA & 0X3FFF) == 10)
		{
			for (i = 0;i < 9;i ++)
			    sum += Radar.RX_BUF[i];
			if (sum == USART3_RX_BUF[9])
				Radar.RX_STA |= 0x8000;  //�������
			else
				Radar.RX_STA = 0;        //���ݴ������½���
		}
	}
}

//�״����ݴ���
u8 GetRadarData(void)
{
	u32 angle,distance;
	
	if (Radar.RX_STA & 0x8000)
	{
		angle = (Radar.RX_BUF[3] << 8) | Radar.RX_BUF[4];
		distance = (Radar.RX_BUF[5] << 8) | Radar.RX_BUF[5];
		Radar.RX_STA = 0;
	}
	
	if (angle >240 || angle < 300 || distance > 4000 || distance < 10)
	{
		Radar.State = 0;
		return 0;
	}
	else
	{
		Radar.Angle = angle;
		Radar.Distance = distance;
		Radar.State = 1;
		
		LCD_ShowString(30 + 200, 460, 200, 16, 16, "Radar:rad");
		LCD_ShowNum(30 + 200 + 48 + 8 + 45, 460, Radar.Angle, 4, 16);
		LCD_ShowString(30 + 200, 480, 200, 16, 16, "Radar:length");
		LCD_ShowNum(30 + 200 + 48 + 8 + 45, 480, Radar.Distance, 4, 16);
		
		return 1;
	}
}

void ReceiveVisionData(void)
{
	uint8_t sum = 0,i = 0;
	if ((USART1_RX_STA & 0x8000) == 0)  //����δ���
	{
		
	}
}

//�Ӿ����ݴ���
u8 GetVisionData(void)
{
	
}

//����ת������̼ƶ�λ
//��������Ƶ�ȥ������
void Get_Position(void)
{
	//�����ٶ�����������
	float nW,nX,nY;
	
	float l1,l2;            //
	
	float theta_inv[2][2];  //
	
	float d_theta;
	
	d_theta = BasketballRobot.ThetaD - BasketballRobot.LastTheta;
	
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
	
//ʹ��������̼ƶ�λ
	
	//�������ǵ���תƫ��
	nW = d_theta / 182 * 14640;
	
	//��ȥ�Դ�ƫ��
	l1 = BasketballRobot.w[0] - nW;
	l2 = BasketballRobot.w[1] - nW;
	
	nX = -l1 / 22400;
	nY = (2 * l2 + l1) / 1.7320508f / 22400;
	
	BasketballRobot.X += nX * theta_inv[0][0] + nY * theta_inv[0][1];
	BasketballRobot.Y += nX * theta_inv[0][1] + nY - theta_inv[1][1];
	
	BasketballRobot.encoderCount[0] += BasketballRobot.w[0];
	BasketballRobot.encoderCount[1] += BasketballRobot.w[1];
}
