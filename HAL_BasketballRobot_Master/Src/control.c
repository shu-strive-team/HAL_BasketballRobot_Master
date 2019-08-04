#include "control.h"
#include "get_info.h"

u16 motortime = 0;

RxPID rxPID;     //串口调节pid
BLOCKING Blocking;  //走位pid

float output[6];

struct RADAR Radar;
struct VISION Vision;

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
	BasketballRobot.w[1] = 0;     //第二个编码器速度
	
	BasketballRobot.encoderCount[0] = 0;  //第一个编编码器总计数
	BasketballRobot.encoderCount[1] = 0;  //第二个编编码器总计数
	
	BasketballRobot.LastTheta = 0;  //上一时刻，机器人theta角
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  //开启解码器通道
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	
	//开启外设
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim5);

	HAL_UART_Receive_IT(&huart1,(u8 *)aRxBuffer1, 1);
	HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer2, 1);
	HAL_UART_Receive_IT(&huart3,(u8 *)aRxBuffer3, 1);
	HAL_UART_Receive_IT(&huart4, &rxPID.pidReadBuf, 1);
	
	IMU_Init();  //陀螺仪初始化
	
	Set_MotorSpeed(0,0,0,0);
}

//设置电机的速度（控制电流）
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
//底盘PID控制程序
//包含速度PID与位置PID控制
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

//给自身坐标系速度求得轮子的速度
//麦克纳姆轮在运动分解时注意每个轮子的正方向
//vx：自身坐标的x轴速度
//vy：自身坐标的y轴速度
//w:  机器人原地旋转的角速度
void GetMotorVelocity_Self(float vx, float vy, float w)
{
	Motor[0].SetSpeed = vx + vy + w * PLATFORM_L;
	Motor[1].SetSpeed = vx - vy + w * PLATFORM_L;
	Motor[2].SetSpeed = - vx - vy + w * PLATFORM_L;
	Motor[3].SetSpeed = - vx + vy  + w * PLATFORM_L;
	
	LCD_Show_setspeed();
}

//给定球场坐标速度求得轮子的速度
//vx：球场坐标的x轴速度
//vy：球场坐标的y轴速度
//w:机器人原地旋转的角速度
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

//获取红外开关状态
void GetInfraredState(void)
{
	while (1)
	{
		if (!INFRARED)
			break;
	}
}

//铲球电机状态
void ShoveMotor(shovemotor t)
{
	
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

//PD调整X轴速度
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
//PD调整Y轴速度
static float adjustVy_PD(float D_Y)
{
	float sy,Now_DY;
	static float Last_DY;
	Now_DY = D_Y ;
	
	if (Now_DY > 0.05f)
	{
		//sy = Now_DY * 1000 + 200;
		sy =(7.5- Now_DY) * 250  + 200;
		
		if (Now_DY < 1)//距离目标1m
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

//转一周时的角y度调节
//由于在 adjustAngleVw_PD 函数中不能实现转一周，所以自转一周单独实现
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

//自转一周
void RobotRotate_circle(void)
{
	float D_Theta;
	float Vw = 0;  //w > 0，顺时针
	
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

//自旋运动，根据误差角度，自动调节
void RobotRotate(float theta)
{
	float D_Theta;
	float Vw = 0;  //w > 0，顺时针
	
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

//行至目的地
//X_I:目的地坐标的X
//Y_I:目的地坐标的Y
//Theta_I:目的地坐标的角度
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

//避障直行
//直行1m
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
		while((Radar.RX_STA&0x8000) == 0);//接收未完成
		
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
			GetMotorVelocity_Self(-300, 0, 0); //原来-80 0 0
			Set_MotorSpeed(Motor[0].SetSpeed, Motor[1].SetSpeed, Motor[2].SetSpeed,Motor[3].SetSpeed);
		}
		else if (Radar.Angle > RADAR_MID + 5)
		{
			GetMotorVelocity_Self(300, 0, 0); //原来80 0 0
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
