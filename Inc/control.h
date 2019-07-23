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

#define CURRENT_MAX 8000				//�������ֵ
#define ANGLE_MAX   8191                //ת�ӻ�е�Ƕ����ֵ����Ӧת�ӻ�е�Ƕ�360��
#define PI 3.141592654f

#define PLATFORM_X 0.8f  //ƽ̨���ֳ��������ľ����һ��
#define PLATFORM_Y 0.5f  //ƽ̨���ֿ������ľ����һ��
#define WHEEL_R 0.02     //С�������ĵ��������ĵľ���

enum{
    SPEED = 0,
    CURRENT 	,
    POSITION 	,
};

typedef struct
{
	uint16_t Angle;		  //ת�ӻ�е�Ƕ�
	int16_t Speed;	      //ת��ת��
	int16_t Current;      //ת�ص���
	int16_t Temperature;  //����¶�
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
	
	uint32_t Distance;  //����
	
	uint32_t Angle;     //�Ƕ�
	
	u8 State;           //״̬
	
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
	float X;                  //������������ϵ��y����
	float Y;		          //������������ϵ��y����
	float PX;		          //���x����
	float PY;		          //���y����
	float ThetaR;             //�������������x��н� ����
	float ThetaD;	          //�������������y��н� �Ƕ�
	
	float Vx;		          //������������ϵx�����ٶ�
	float Vy;		          //������������ϵy�����ٶ�	
	float W;		          //�����˽��ٶȣ�˳ʱ��������
	
	int16_t w[2];             //�������ٶ�
	int64_t encoderCount[2];  //�������ܼ���
	
	float Velocity[4];	      //���ӵ��ٶ�
	
	float LastTheta;          //��һʱ�̣�������Theta��
	
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
