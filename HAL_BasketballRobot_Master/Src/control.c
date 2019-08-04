#include "control.h"
#include "get_info.h"

u16 motortime = 0;

RxPID rxPID;     //���ڵ���pid
BLOCKING Blocking;  //��λpid

float output[6];

struct RADAR Radar;
struct VISION Vision;

MOTOR Motor[4];
ROBOT BasketballRobot;
PID_t *pidAdjust;

void Control_Init(void)
{
	BasketballRobot.X = 0;       //������������ϵ��x����
	BasketballRobot.Y = 0;       //������������ϵ��y����
	BasketballRobot.ThetaD = 0;  //�������������y��н� �Ƕ�
	BasketballRobot.ThetaR = 0;  //�������������y��н� ����
	
	BasketballRobot.Vx = 0;      //������������ϵx�����ٶ�
	BasketballRobot.Vy = 0;      //������������ϵy�����ٶ�
	BasketballRobot.W = 0;       //�����˽��ٶȣ�˳ʱ��������
	
	BasketballRobot.w[0] = 0;    //��һ���������ٶ�
	BasketballRobot.w[1] = 0;     //�ڶ����������ٶ�
	
	BasketballRobot.encoderCount[0] = 0;  //��һ����������ܼ���
	BasketballRobot.encoderCount[1] = 0;  //�ڶ�����������ܼ���
	
	BasketballRobot.LastTheta = 0;  //��һʱ�̣�������theta��
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  //����������ͨ��
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	
	//��������
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim5);

	HAL_UART_Receive_IT(&huart1,(u8 *)aRxBuffer1, 1);
	HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer2, 1);
	HAL_UART_Receive_IT(&huart3,(u8 *)aRxBuffer3, 1);
	HAL_UART_Receive_IT(&huart4, &rxPID.pidReadBuf, 1);
	
	IMU_Init();  //�����ǳ�ʼ��
	
	Set_MotorSpeed(0,0,0,0);
}

//���õ�����ٶȣ����Ƶ�����
void Set_MotorSpeed(float V1,float V2,float V3,float V4)
{
	Motor[0].SetSpeed = V1;
	Motor[1].SetSpeed = V2;
	Motor[2].SetSpeed = V3;
	Motor[3].SetSpeed = V4;
}

void AllPID_Init(void)
{
	u8 i;
	for (i = 0;i < 4;i ++)
	{
	    PID_Init(&(Motor[i].SpeedPID), POSITION_PID, 5000, 3000,
					 5,1,0);
//	    PID_Init(&(Motor[i].AnglePID), POSITION_PID, ANGLE_MAX, 5000,0.25,0,0);
		
		PID_Init(&Blocking.Vx_adjust, POSITION_PID, 2000, 1000, 900, 0, 20);
		PID_Init(&Blocking.Vy_adjust, POSITION_PID, 2000, 1000, 900, 0, 20);
	}
}
//����PID���Ƴ���
//�����ٶ�PID��λ��PID����
void Calc_MotorSpeed_pid(void)
{
	u8 i;
	
	for (i = 0;i < 4;i ++)
	    Motor[i].Single_LoopOutput = PID_Calc(&(Motor[i].SpeedPID),
		                                  Motor[i].Speed, Motor[i].SetSpeed);
}

//void Calc_MotorAngle_pid(void)
//{
//	u8 i;
//	DoubleLoop_Flag = 1;
//	
//	for (i = 0;i < 4;i ++)
//	{
//	    Motor[i].SetSpeed = PID_Calc(&(Motor[i].AnglePID),
//											     Motor[i].Angle, Motor[i].SetAngle);
//		DoubleLoop_Flag = 0;
//		
//	    Motor[i].Double_LoopOutput = PID_Calc(&(Motor[i].SpeedPID),
//											     Motor[i].Speed, Motor[i].SetSpeed);
//	}
//}

//����������ϵ�ٶ�������ӵ��ٶ�
//�����ķ�����˶��ֽ�ʱע��ÿ�����ӵ�������
//vx�����������x���ٶ�
//vy�����������y���ٶ�
//w:  ������ԭ����ת�Ľ��ٶ�
void GetMotorVelocity_Self(float vx, float vy, float w)
{
	Motor[0].SetSpeed = vx + vy + w * PLATFORM_L;
	Motor[1].SetSpeed = vx - vy + w * PLATFORM_L;
	Motor[2].SetSpeed = - vx - vy + w * PLATFORM_L;
	Motor[3].SetSpeed = - vx + vy  + w * PLATFORM_L;
	
	LCD_Show_setspeed();
}

//�����������ٶ�������ӵ��ٶ�
//vx���������x���ٶ�
//vy���������y���ٶ�
//w:������ԭ����ת�Ľ��ٶ�
void GetMotorVelocity(float vx_a, float vy_a, float w_a)
{
    float vx,vy,w;
	
	vx = vx_a * cos(BasketballRobot.ThetaR) + vy_a * sin(BasketballRobot.ThetaR);
	vy = -vx_a * sin(BasketballRobot.ThetaR) + vy_a * cos(BasketballRobot.ThetaR);
	w = w_a;
	
	Motor[0].SetSpeed = vx + vy + w * PLATFORM_L;
	Motor[1].SetSpeed = vx - vy + w * PLATFORM_L;
	Motor[2].SetSpeed = - vx - vy + w * PLATFORM_L;
	Motor[3].SetSpeed = - vx + vy  + w * PLATFORM_L;
	
	LCD_Show_setspeed();
}

//��ȡ���⿪��״̬
void GetInfraredState(void)
{
	while (1)
	{
		if (!INFRARED)
			break;
	}
}

//������״̬
void ShoveMotor(shovemotor t)
{
	
}

//��е���½�
void Robot_ArmDown(void)
{
	
}

//��е������
void Robot_ArmUp(void)
{
	
}

//PD�������ٶ�
static float adjustAngleVw_PD(float D_Theta)
{
	float Vw = 0;
	
	if (D_Theta > 0 && D_Theta <= 180)
		Vw = -D_Theta * 30;
	else if (D_Theta > 180)
	{
		D_Theta = 360 - D_Theta;
		Vw = D_Theta * 30;
	}
	else if (D_Theta < 0 && D_Theta > -180)
		Vw = -D_Theta * 30;
	else if (D_Theta <= -180)
	{
		D_Theta = 360 + D_Theta;
		Vw = -D_Theta * 30;
	}
	
	if (Vw > 900)
		Vw = 900;
	else if (Vw < -900)
		Vw = -900;
	if (Vw > 0 && Vw < 90)
		Vw = 90;
	else if (Vw > -90 && Vw < 0)
		Vw = -90;
	
	return Vw;
}

//PD����X���ٶ�
static float adjustVx_PD(float D_X)
{
	float sx,Now_DX;
	static float Last_DX;
	Now_DX = D_X;
	
	if (Now_DX > 0.05f)
	{
		sx = Now_DX * 1000 + 200;
		
		
		
		if (Now_DX < 1)
			sx =Now_DX * 1000 + 5 * (Now_DX - Last_DX) + 600;
		
		if (sx > 1500)
			sx = 1500;
	}
	
	else if (Now_DX < -0.05f)
	{
		sx = -Now_DX * 1000 - 200;
		
		if (Now_DX > -1)
			sx = -Now_DX * 1000 -  5 * (Now_DX - Last_DX) - 800;
		
		if (sx < -1500)
			sx = -1500;
	}
	
	else
		sx = 0;
	
	Last_DX = Now_DX;

/*
    float sx;
	
	if (fabs(D_X) > 0.05f)
	{
		Blocking.Vx_adjust.err[NOW] = D_X;
		sx = (err_PID_Calc(&Blocking.Vx_adjust,Blocking.Vx_adjust.err[NOW]) + 300);
	}
	else
		sx = 0;
	
	if (sx > 1500)
		sx = 1500;
	if (sx < -1500)
		sx = -1500;
*/	
	return sx;	
}
//PD����Y���ٶ�
static float adjustVy_PD(float D_Y)
{
	float sy,Now_DY;
	static float Last_DY;
	Now_DY = D_Y ;
	
	if (Now_DY > 0.05f)
	{
		//sy = Now_DY * 1000 + 200;
		sy =(7.5- Now_DY) * 250  + 200;
		
		if (Now_DY < 1)//����Ŀ��1m
			sy = Now_DY * 1000 + 5 * (Now_DY - Last_DY) + 600;
		
		if (sy > 1500)
			sy = 1500;
	}
	
	else if (Now_DY < -0.05f)
	{
		sy = Now_DY * 1000 - 200;
		
		if (Now_DY > -1)
			sy = -Now_DY * 1000 -  5 * (Now_DY - Last_DY) - 600;
		
		if (sy < -1500)
			sy = -1500;
	}
	
	else
		sy = 0;
	
	Last_DY = Now_DY;
/*
    float sy;
	
	if (fabs(D_Y) > 0.05f)
	{
		Blocking.Vy_adjust.err[NOW] = D_Y;
		sy = err_PID_Calc(&Blocking.Vy_adjust,Blocking.Vy_adjust.err[NOW]) + 300;
	}
	else
		sy = 0;
	
	if (sy > 1500)
		sy = 1500;
	if (sy < -1500)
		sy = -1500;
*/
	return sy;
}

//תһ��ʱ�Ľ�y�ȵ���
//������ adjustAngleVw_PD �����в���ʵ��תһ�ܣ�������תһ�ܵ���ʵ��
static float adjust_circle(float D_Theta)
{
	float Vw = 0;

	if (D_Theta > 0 && D_Theta <= 180)
		Vw = -D_Theta * 30;
	else if (D_Theta > 180)
	{
		D_Theta = 360 - D_Theta;
		Vw = -D_Theta * 30 - 90;
	}
	
	if (Vw < -900)
		Vw = -900;
	else if (Vw > -90 && Vw < 0)
		Vw = -90;
	
	return Vw;
}

//��תһ��
void RobotRotate_circle(void)
{
	float D_Theta;
	float Vw = 0;  //w > 0��˳ʱ��
	
	D_Theta = 360 - BasketballRobot.ThetaD;
	
	Vw = adjust_circle(D_Theta);

	while (D_Theta > 1 || D_Theta < -1)
	{
		Set_MotorSpeed(Vw,Vw,Vw,Vw);
		
		D_Theta = 360 - BasketballRobot.ThetaD -2;
		
		Vw = adjust_circle(D_Theta);
		
		LCD_Show_setspeed();
	}
	
	Set_MotorSpeed(0,0,0,0);
}

//�����˶����������Ƕȣ��Զ�����
void RobotRotate(float theta)
{
	float D_Theta;
	float Vw = 0;  //w > 0��˳ʱ��
	
	while (theta < 0)
		theta = theta + 360;
	
	while (theta > 360)
		theta = theta - 360;
	
	D_Theta = theta - BasketballRobot.ThetaD;
	
	Vw = adjustAngleVw_PD(D_Theta);

	while (D_Theta > 1 || D_Theta < -1)
	{
		Set_MotorSpeed(Vw,Vw,Vw,Vw);
		
		D_Theta = theta - BasketballRobot.ThetaD + 2;
		
		Vw = adjustAngleVw_PD(D_Theta);
		
		LCD_Show_setspeed();
	}
	
	Set_MotorSpeed(0,0,0,0);
}

//����Ŀ�ĵ�
//X_I:Ŀ�ĵ������X
//Y_I:Ŀ�ĵ������Y
//Theta_I:Ŀ�ĵ�����ĽǶ�
void RobotGoTo(float X_I,float Y_I,float Theta_I)
{
	float D_Theta,D_X,D_Y,Vw = 0,sx = 0,sy = 0;
	
	D_Theta = Theta_I - BasketballRobot.ThetaD;
	D_X = X_I - BasketballRobot.X;
	D_Y = Y_I - BasketballRobot.Y;
	while (fabs(D_Y) > 0.05f || fabs(D_X) > 0.05f)
	{
		sy = adjustVy_PD(D_Y);
		sx = adjustVx_PD(D_X);
		Vw = adjustAngleVw_PD(D_Theta);
		
		GetMotorVelocity(sx,sy,Vw);
		
		Set_MotorSpeed(Motor[0].SetSpeed,Motor[1].SetSpeed,
		               Motor[2].SetSpeed,Motor[3].SetSpeed);
		
		D_Theta = Theta_I - BasketballRobot.ThetaD;
	    D_X = X_I - BasketballRobot.X;
	    D_Y = Y_I - BasketballRobot.Y;
	}
	RobotRotate(Theta_I);
	
	Set_MotorSpeed(0,0,0,0);
}

//����ֱ��
//ֱ��1m
void RobotGoAvoidance(void)
{
	//float w = 100;
	float theta = BasketballRobot.ThetaD, D_theta = 0;
	u8 time = 1;

	Set_MotorSpeed(0,0,0,0);
	LCD_Show_getspeed();
	
	Radar.RX_STA = 0;
	
	do
	{
		while((Radar.RX_STA&0x8000) == 0);//����δ���
		
		if (!GetRadarData())
		{
			if (time == 0)
			{
			}
			else if (time++ < 5)
			{
				Set_MotorSpeed(0, 0, 0, 0);
				continue;
			}
			else if (time != 0)
				time = 0;
		}
		else
			time = 1;

		//		if(Radar.Distance < 10)
		//			continue;
	
		if (time == 0)
		{
			GetMotorVelocity_Self(0, 1000, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
			delay_ms(5000);
			Set_MotorSpeed(0,0,0,0);
			break;
		}
		
		
		if (Radar.Angle < RADAR_MID - 15)
		{
			GetMotorVelocity_Self(-800, 0, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Angle > RADAR_MID + 15)
		{
			GetMotorVelocity_Self(800, 0, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Distance > 500)
		{
			GetMotorVelocity_Self(0, 800, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Angle < RADAR_MID - 10)
		{
			GetMotorVelocity_Self(-500, 0, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if ((Radar.Angle > RADAR_MID + 10))
		{
			GetMotorVelocity_Self(500, 0, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		
		else if (Radar.Angle < RADAR_MID - 5)
		{
			GetMotorVelocity_Self(-300, 0, 0); //ԭ��-80 0 0
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Angle > RADAR_MID + 5)
		{
			GetMotorVelocity_Self(300, 0, 0); //ԭ��80 0 0
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else
		{
			if(BasketballRobot.X >= 2.5f)
				RobotGoTo(BasketballRobot.X - 0.55,BasketballRobot.Y,BasketballRobot.ThetaD);
			else
				RobotGoTo(BasketballRobot.X + 0.55,BasketballRobot.Y,BasketballRobot.ThetaD);
			
			GetMotorVelocity_Self(0, 800, 0);
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
			delay_ms(4000);
			Set_MotorSpeed(0,0,0,0);
			break;
		}
		LCD_Show_getspeed();
	} while (1);

	//	GetMotorVelocity_Self(0, 100, 0);
	//	SetPWM(BasketballRobot.Velocity[0], BasketballRobot.Velocity[1], BasketballRobot.Velocity[2]);


	Set_MotorSpeed(0,0,0,0);
	LCD_Show_getspeed();	
	

}
