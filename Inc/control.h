#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f4xx_HAL.h"
#include "sys.h"
#include "pid.h"
#include <math.h>
#include "tim.h"
#include "usart.h"
#include "lcd.h"
#include "mpu6050.h"
#include "delay.h"

#define CURRENT_MAX 8000				//电流最大值
#define ANGLE_MAX   8191                //转子机械角度最大值，对应转子机械角度360°
#define PI 3.141592654f

#define PLATFORM_X 0.8f  //平台车轮长方向中心距离的一半
#define PLATFORM_Y 0.5f  //平台车轮宽方向中心距离的一半
#define WHEEL_R 0.02     //小辊子中心到麦轮中心的距离

enum{
    SPEED = 0,
    CURRENT 	,
    POSITION 	,
};

typedef struct
{
	uint16_t Angle;		  //转子机械角度
	int16_t Speed;	      //转子转速
	int16_t Current;      //转矩电流
	int16_t Temperature;  //电机温度
	int16_t SetSpeed;
	int16_t SetAngle;
	
	PID_t SpeedPID;
	PID_t AnglePID;
	
	int16_t SpeedOutput;
	int16_t AngleOutput;

}MOTOR;

struct RADAR
{
	uint16_t RX_STA;
	
	uint8_t RX_BUF[20];
	
	uint32_t Distance;  //距离
	
	uint32_t Angle;     //角度
	
	u8 State;           //状态
	
};

typedef struct
{
    uint8_t Count;
	uint8_t Buf[20];
	u8		Sum;
	u8		pidReadBuf;
	PID_t* 	pidAdjust;
} RxPID;

typedef struct
{
	float X;                  //机器人在坐标系中y坐标
	float Y;		          //机器人在坐标系中y坐标
	float PX;		          //点的x坐标
	float PY;		          //点的y坐标
	float ThetaR;             //机器人正方向和x轴夹角 弧度
	float ThetaD;	          //机器人正方向和y轴夹角 角度
	
	float Vx;		          //机器人在坐标系x方向速度
	float Vy;		          //机器人在坐标系y方向速度	
	float W;		          //机器人角速度，顺时针正方向
	
	int16_t w[2];             //编码器速度
	int64_t encoderCount[2];  //编码器总计数
	
	float Velocity[4];	      //轮子的速度
	
	float LastTheta;          //上一时刻，机器人Theta角
	
}ROBOT;

//extern PID_t* pidAdjust;
extern MOTOR Motor[4];
extern ROBOT BasketballRobot;
extern RxPID rxPID;
extern struct RADAR Radar;

void Control_Init(void);
void MOTOR_ControlInit(void);
void Calc_MotorSpeed_pid(void);
void Calc_MotorAngle_pid(void);
void Set_MotorSpeed(float V1,float V2,float V3,float V4);
void GetMotorVelocity_Self(float vx, float vy, float w);
void GetMotorVelocity(float vx, float vy, float w);
void Robot_ArmDown(void);
void Robot_ArmUp(void);
static float adjustAngleVw_PD(float D_Theta);
static float adjustVy_PD(float D_Y);
static float adjustVx_PD(float D_X);
void RobotRotate(float theta);
void RobotGoToDestn(float X_I,float Y_I,float Theta_I);

#endif
