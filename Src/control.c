#include "control.h"

RxPID rxPID;  //串口调节pid

struct RADAR Radar;

MOTOR Motor[4];
ROBOT BasketballRobot;
PID_t *pidAdjust;

void Control_Init(void)
{
	BasketballRobot.X = 0;       //机器人在坐标系中x坐标
	BasketballRobot.Y = 0;       //机器人在坐标系中y坐标
	BasketballRobot.ThetaD = 0;  //机器人正方向和y轴夹角 角度
	BasketballRobot.ThetaR = 0;  //机器人正方向和y轴夹角 弧度
	
	BasketballRobot.Vx = 0;      //机器人在坐标系x方向速度
	BasketballRobot.Vy = 0;      //机器人在坐标系y方向速度
	BasketballRobot.W = 0;       //机器人角速度，顺时针正方向
	
	BasketballRobot.w[0] = 0;    //第一个编码器速度
	BasketballRobot.w[1] = 0;    //第二个编码器速度
	
	BasketballRobot.encoderCount[0] = 0;  //第一个编编码器总计数
	BasketballRobot.encoderCount[1] = 0;  //第二个编编码器总计数
	
	BasketballRobot.LastTheta = 0;  //上一时刻，机器人theta角
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  //开启解码器通道
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	
	//开启外设
	HAL_TIM_Base_Start_IT(&htim5);
	
	HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer2, 1);
	HAL_UART_Receive_IT(&huart3,(u8 *)aRxBuffer3, 1);
	HAL_UART_Receive_IT(&huart4, &rxPID.pidReadBuf, 1);
	
	IMU_Init();  //陀螺仪初始化
	
	Set_MotorSpeed(0,0,0,0);
}

//设置电机的速度
void Set_MotorSpeed(float V1,float V2,float V3,float V4)
{
	Motor[0].SetSpeed = V1;
	Motor[1].SetSpeed = V2;
	Motor[2].SetSpeed = V3;
	Motor[3].SetSpeed = V4;
	
//	LCD_Show_setspeed();
//	LCD_Show_getspeed();
}

void MOTOR_ControlInit()
{
	u8 i;
	for (i = 0;i < 4;i ++)
	{
		Motor[i].SetAngle = 0;
	
	    PID_Init(&(Motor[i].SpeedPID), POSITION_PID, CURRENT_MAX, 1000,
					 2.0,0.1,0);
	    PID_Init(&(Motor[i].AnglePID), POSITION_PID, ANGLE_MAX, 1000,
				     0.04,0.001,0);
	}
}
//底盘PID控制程序
//包含速度PID与位置PID控制
void Calc_MotorSpeed_pid(void)
{
	u8 i;
	for (i = 0;i < 4;i ++)
	    Motor[i].SpeedOutput = PID_Calc(&(Motor[i].SpeedPID),
		                                  Motor[i].Speed, Motor[i].SetSpeed);
}

void Calc_MotorAngle_pid(void)
{
	u8 i;
	for (i = 0;i < 4;i ++)
	{
//	    Motor[i].SetSpeed = PID_Calc(&(Motor[i].AnglePID),
//											     Motor[i].Angle, Motor[i].SetAngle);
//	    Motor[i].SpeedOutput = PID_Calc(&(Motor[i].SpeedPID),
//											     Motor[i].Speed, Motor[i].SetSpeed);
		Motor[i].AngleOutput = PID_Calc(&(Motor[i].AnglePID),
		                                  Motor[i].Angle,Motor[i].SetAngle);
	}
}

//给自身坐标系速度求得轮子的速度
//vx：自身坐标的x轴速度
//vy：自身坐标的y轴速度
//w:  机器人原地旋转的角速度
void GetMotorVelocity_Self(float vx, float vy, float w)
{
//	u8 i, j, k;
//	float L[4][3];
//	float theta[3][3];
//	float V[3];
//	float temp[4][3];
//	
//	//     1    1    -(PLATFORM_X+PLATFORM_Y)
//	//L = -1    1    (PLATFORM_X+PLATFORM_Y)
//	//     1    1	 (PLATFORM_X+PLATFORM_Y)
//	//    -1    1    -(PLATFORM_X+PLATFORM_Y)
//	L[0][0] = 1;
//	L[0][1] = 1;
//	L[0][2] = -(PLATFORM_X+PLATFORM_Y);
//	L[1][0] = -1;
//	L[1][1] = 1;
//	L[1][2] = -(PLATFORM_X+PLATFORM_Y);
//	L[2][0] = 1;
//	L[2][1] = 1;
//	L[2][2] = (PLATFORM_X+PLATFORM_Y);
//	L[3][0] = -1;
//	L[3][1] = 1;
//	L[3][2] = -(PLATFORM_X+PLATFORM_Y);
//	//        cos(0)    sin(0)    0
//	//theta = sin(0)    cos(0)    0
//	//        0         0         1
//	theta[0][0] = 1;
//	theta[0][1] = 0;
//	theta[0][2] = 0;
//	theta[1][0] = 0;
//	theta[1][1] = 1;
//	theta[1][2] = 0;
//	theta[2][0] = 0;
//	theta[2][1] = 0;
//	theta[2][2] = 1;
//	//V
//	V[0] = vx;
//	V[1] = vy;
//	V[2] = w;
//	
//	for (i = 0; i < 4; i++)
//	{
//		for(j = 0; j < 3; j++)
//		{
//			temp[i][j] = 0;
//			for(k = 0; k < 3; k++)
//				temp[i][j] =+ L[i][k] * theta[k][j];
//		}
//	}
//	for (i = 0; i < 4; i++)
//	{
//		Motor[i].SetSpeed = 0;
//		for (j = 0; j < 3; j++)
//		    Motor[i].SetSpeed += temp[i][j] * V[j];
//	}
	
	
}

//给定球场坐标速度求得轮子的速度
//vx：球场坐标的x轴速度
//vy：球场坐标的y轴速度
//w:机器人原地旋转的角速度
void GetMotorVelocity(float vx, float vy, float w)
{
//	u8 i, j, k;
//	float L[4][3];
//	float theta[3][3];
//	float V[3];
//	float temp[4][3];
//	
//	//     1    1    -(PLATFORM_X+PLATFORM_Y)
//	//L = -1    1    (PLATFORM_X+PLATFORM_Y)
//	//     1    1	 (PLATFORM_X+PLATFORM_Y)
//	//    -1    1    -(PLATFORM_X+PLATFORM_Y)
//	L[0][0] = 1;
//	L[0][1] = 1;
//	L[0][2] = -(PLATFORM_X+PLATFORM_Y);
//	L[1][0] = -1;
//	L[1][1] = 1;
//	L[1][2] = -(PLATFORM_X+PLATFORM_Y);
//	L[2][0] = 1;
//	L[2][1] = 1;
//	L[2][2] = (PLATFORM_X+PLATFORM_Y);
//	L[3][0] = -1;
//	L[3][1] = 1;
//	L[3][2] = -(PLATFORM_X+PLATFORM_Y);
//	//        cos(theta)    sin(theta)    0
//	//theta = sin(theta)    cos(theta)    0
//	//        0             0             1
//	theta[0][0] = cos(BasketballRobot.ThetaR);
//	theta[0][1] = sin(BasketballRobot.ThetaR);;
//	theta[0][2] = 0;
//	theta[1][0] = sin(BasketballRobot.ThetaR);;
//	theta[1][1] = cos(BasketballRobot.ThetaR);;
//	theta[1][2] = 0;
//	theta[2][0] = 0;
//	theta[2][1] = 0;
//	theta[2][2] = 1;
//	//V
//	V[0] = vx;
//	V[1] = vy;
//	V[2] = w;
//	
//	for (i = 0; i < 4; i++)
//	{
//		for(j = 0; j < 3; j++)
//		{
//			temp[i][j] = 0;
//			for(k = 0; k < 3; k++)
//				temp[i][j] =+ L[i][k] * theta[k][j];
//		}
//	}
//	for (i = 0; i < 4; i++)
//	{
//		Motor[i].SetSpeed = 0;
//		for (j = 0; j < 3; j++)
//		    Motor[i].SetSpeed += temp[i][j] * V[j];
//	}
}

//机械臂下降
void Robot_ArmDown(void)
{
	
}

//机械臂上升
void Robot_ArmUp(void)
{
	
}

//PD调整角速度
static float adjustAngleVw_PD(float D_Theta)
{
	float Vw = 0;
	if (D_Theta >= 0 && D_Theta < 180)
		Vw = D_Theta * 2;
	else if (D_Theta >= 180)
	{
		D_Theta = 360 - D_Theta;
		Vw = -D_Theta * 2;
	}
	else if (D_Theta < 0 && D_Theta > -180)
		Vw = D_Theta * 5;
	else if (D_Theta <= -180)
	{
		D_Theta = 360 + D_Theta;
		Vw = D_Theta * 2;
	}
	
	if (Vw > 300)
		Vw = 300;
	else if (Vw < -300)
		Vw = -300;
	if (Vw > 0 && Vw < 5)
		Vw = 5;
	if (Vw > -5 && Vw < 0)
		Vw = -5;
	
	return Vw;
}

//PD调整X轴速度
static float adjustVx_PD(float D_X)
{
	float sx,Now_DX;
	static float Last_DX;
	Now_DX = D_X;
	
	if (Now_DX > 0.05f)
	{
		sx = Now_DX * 35;
		
		if (Now_DX < 1)
			sx = Now_DX * 35 + 10 * (Now_DX - Last_DX) + 15;
		
		if (sx > 80)
			sx = 80;
	}
	
	else if (Now_DX < -0.05f)
	{
		sx = Now_DX * 35;
		
		if (Now_DX > -1)
			sx = Now_DX * 35 + 10 * (Now_DX - Last_DX) - 15;
		
		if (sx < -80)
			sx = -80;
	}
	
	else
		sx = 0;
	
	Last_DX = Now_DX;
	delay_ms(10);
	return sx;	
}
//PD调整Y轴速度
static float adjustVy_PD(float D_Y)
{
	float sy;
	
	if (D_Y > 0.05f)
	{
		sy = D_Y * 400;
		
		if (D_Y < 1)
			sy = D_Y * 400 + 100;
		
		if (sy > 800)
			sy = 800;
	}
	
	else if (D_Y < -0.05f)
	{
		sy = D_Y * 400;
		
		if (D_Y > -1)
			sy = D_Y * 400 - 100;
		
		if (sy < -800)
			sy = -800;
	}
	
	else
		sy = 0;
	
	return sy;	
}

//自旋运动，根据误差角度，自动调节
void RobotRotate(float theta)
{
	float D_Theta;
	float Vw = 0;
	
	while (theta < 0)
		theta += 360;
	
	while (theta > 360)
		theta -= 360;
	
	D_Theta = theta - BasketballRobot.ThetaD;
	
	Vw = adjustAngleVw_PD(D_Theta);
	
	while (D_Theta > 5 || D_Theta < -5)
	{
		Set_MotorSpeed(Vw,Vw,Vw,Vw);
		
		D_Theta = theta - BasketballRobot.ThetaD;
		
		Vw = adjustAngleVw_PD(D_Theta);
		
		LCD_Show_setspeed();
	}
	
	Set_MotorSpeed(0,0,0,0);
}

//行至目的地
//X_I:目的地坐标的X
//Y_I:目的地坐标的Y
//Theta_I:目的地坐标的角度
void RobotGoToDestn(float X_I,float Y_I,float Theta_I)
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
		
		Set_MotorSpeed(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],
		               BasketballRobot.Velocity[2],BasketballRobot.Velocity[3]);
		
		D_Theta = Theta_I - BasketballRobot.ThetaD;
	    D_X = X_I - BasketballRobot.X;
	    D_Y = Y_I - BasketballRobot.Y;
	}
	RobotRotate(Theta_I);
	
	Set_MotorSpeed(0,0,0,0);
}
