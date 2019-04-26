#include "control.h"

RxPID rxPID;  //串口调节pid

MOTOR Motor[4];
ROBOT BasketballRobot;
PID_t *pidAdjust;

//设置电机的速度
void Set_MotorSpeed(float V1,float V2,float V3,float V4)
{
	Motor[0].SetSpeed = V1;
	Motor[1].SetSpeed = V2;
	Motor[2].SetSpeed = V3;
	Motor[3].SetSpeed = V4;
}

void MOTOR_ControlInit()
{
	PID_StructInit(&(Motor[1].SpeedPID), POSITION_PID, CURRENT_LIM, 1000,
					 1.0f,0,0);
	PID_StructInit(&(Motor[1].AnglePID), POSITION_PID, CURRENT_LIM, 1000,
				     0.5f,0,0);
}
//底盘PID控制程序
//包含速度PID（内环）与位置（角度）PID控制（外环）
void SetMotorSpeed_pid(void)  //内环
{
	Motor[1].SpeedOutput = PID_Calc(&(Motor[1].SpeedPID),
		                                  Motor[1].Speed, Motor[1].SetSpeed);
}

void SetMotorAngle_pid(void)  //外环
{
	Motor[1].SetAngle = Motor[1].SpeedOutput;
	Motor[1].AngleOutput = PID_Calc(&(Motor[1].AnglePID),
											 Motor[1].Angle, Motor[1].SetAngle);
}

//给自身坐标系速度求得轮子的速度
//vx：自身坐标的x轴速度
//vy：自身坐标的y轴速度
//w:  机器人原地旋转的角速度
void GetMotorVelocity_Self(float vx, float vy, float w)
{
	u8 i, j, k;
	float L[4][3];
	float theta[3][3];
	float V[3];
	float temp[4][3];
	
	//     1    1    -(PLATFORM_X+PLATFORM_Y)
	//L = -1    1    (PLATFORM_X+PLATFORM_Y)
	//     1    1	 (PLATFORM_X+PLATFORM_Y)
	//    -1    1    -(PLATFORM_X+PLATFORM_Y)
	L[0][0] = 1;
	L[0][1] = 1;
	L[0][2] = -(PLATFORM_X+PLATFORM_Y);
	L[1][0] = -1;
	L[1][1] = 1;
	L[1][2] = -(PLATFORM_X+PLATFORM_Y);
	L[2][0] = 1;
	L[2][1] = 1;
	L[2][2] = (PLATFORM_X+PLATFORM_Y);
	L[3][0] = -1;
	L[3][1] = 1;
	L[3][2] = -(PLATFORM_X+PLATFORM_Y);
	//        cos(0)    sin(0)    0
	//theta = sin(0)    cos(0)    0
	//        0         0         1
	theta[0][0] = 1;
	theta[0][1] = 0;
	theta[0][2] = 0;
	theta[1][0] = 0;
	theta[1][1] = 1;
	theta[1][2] = 0;
	theta[2][0] = 0;
	theta[2][1] = 0;
	theta[2][2] = 1;
	//V
	V[0] = vx;
	V[1] = vy;
	V[2] = w;
	
	for (i = 0; i < 4; i++)
	{
		for(j = 0; j < 3; j++)
		{
			temp[i][j] = 0;
			for(k = 0; k < 3; k++)
				temp[i][j] =+ L[i][k] * theta[k][j];
		}
	}
	for (i = 0; i < 4; i++)
	{
		Motor[i].SetSpeed = 0;
		for (j = 0; j < 3; j++)
		    Motor[i].SetSpeed += temp[i][j] * V[j];
	}
}

//给定球场坐标速度求得轮子的速度
//vx：球场坐标的x轴速度
//vy：球场坐标的y轴速度
//w:机器人原地旋转的角速度
void GetMotorVelocity(float vx, float vy, float w)
{
	u8 i, j, k;
	float L[4][3];
	float theta[3][3];
	float V[3];
	float temp[4][3];
	
	//     1    1    -(PLATFORM_X+PLATFORM_Y)
	//L = -1    1    (PLATFORM_X+PLATFORM_Y)
	//     1    1	 (PLATFORM_X+PLATFORM_Y)
	//    -1    1    -(PLATFORM_X+PLATFORM_Y)
	L[0][0] = 1;
	L[0][1] = 1;
	L[0][2] = -(PLATFORM_X+PLATFORM_Y);
	L[1][0] = -1;
	L[1][1] = 1;
	L[1][2] = -(PLATFORM_X+PLATFORM_Y);
	L[2][0] = 1;
	L[2][1] = 1;
	L[2][2] = (PLATFORM_X+PLATFORM_Y);
	L[3][0] = -1;
	L[3][1] = 1;
	L[3][2] = -(PLATFORM_X+PLATFORM_Y);
	//        cos(theta)    sin(theta)    0
	//theta = sin(theta)    cos(theta)    0
	//        0             0             1
	theta[0][0] = cos(BasketballRobot.ThetaR);
	theta[0][1] = sin(BasketballRobot.ThetaR);;
	theta[0][2] = 0;
	theta[1][0] = sin(BasketballRobot.ThetaR);;
	theta[1][1] = cos(BasketballRobot.ThetaR);;
	theta[1][2] = 0;
	theta[2][0] = 0;
	theta[2][1] = 0;
	theta[2][2] = 1;
	//V
	V[0] = vx;
	V[1] = vy;
	V[2] = w;
	
	for (i = 0; i < 4; i++)
	{
		for(j = 0; j < 3; j++)
		{
			temp[i][j] = 0;
			for(k = 0; k < 3; k++)
				temp[i][j] =+ L[i][k] * theta[k][j];
		}
	}
	for (i = 0; i < 4; i++)
	{
		Motor[i].SetSpeed = 0;
		for (j = 0; j < 3; j++)
		    Motor[i].SetSpeed += temp[i][j] * V[j];
	}
}

