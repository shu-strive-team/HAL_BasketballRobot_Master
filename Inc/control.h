#ifndef _CONTROL_H
#define _CONTROL_H

#include "stm32f4xx_HAL.h"
#include "sys.h"
#include "pid.h"
#include <math.h>

#define CURRENT_LIM 1000				//电流最小值
#define CURRENT_MAX 16000				//电流最大值
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
	float X;        //机器人在坐标系中y坐标
	float Y;		//机器人在坐标系中y坐标
	float PX;		//点的x坐标
	float PY;		//点的y坐标
	float ThetaR;   //机器人正方向和x轴夹角 弧度
	float ThetaD;	//机器人正方向和y轴夹角 角度
	
	float Vx;		//机器人在坐标系x方向速度
	float Vy;		//机器人在坐标系y方向速度	
	float W;		//机器人角速度，顺时针正方向
	
}ROBOT;

//extern PID_t* pidAdjust;
extern MOTOR Motor[4];
extern ROBOT BasketballRobot;
extern RxPID rxPID;

void MOTOR_ControlInit(void);
void SetMotorSpeed_pid(void);
void SetMotorAngle_pid(void);
void GetMotorVelocity_Self(float vx, float vy, float w);
void GetMotorVelocity(float vx, float vy, float w);
void Set_MotorSpeed(float V1,float V2,float V3,float V4);

#endif
