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
#include "gpio.h"

#define CURRENT_MAX 8000				//�������ֵ
#define ANGLE_MAX   8191                //ת�ӻ�е�Ƕ����ֵ����Ӧת�ӻ�е�Ƕ�360��
#define PI 3.141592654f

#define PLATFORM_L 1.0f  //ƽ̨���ֳ��������ľ����һ����ƽ̨���ֿ������ľ����һ��֮��
#define WHEEL_R 0.02f     //С�������ĵ��������ĵľ���
#define VISION_MID 256   //�Ӿ���λ����
#define RADAR_MID 268    //�״ﶨλ����

extern u16 motortime;

extern float output[6];

enum{
    SPEED = 0,
    CURRENT 	,
    POSITION 	,
};

typedef struct
{
	PID_t Vx_adjust;
	PID_t Vy_adjust;
	
}BLOCKING;

typedef struct
{
	uint16_t Angle;		  //ת�ӻ�е�Ƕ�
	int16_t Speed;	      //ת��ת��
	int16_t Current;      //ת�ص���
	int16_t Temperature;  //����¶�
	int16_t SetSpeed;
	int16_t SetAngle;
	
	PID_t SpeedPID;
//	PID_t AnglePID;
	
	int16_t Single_LoopOutput;
//	int16_t Double_LoopOutput;
//	int16_t AngleOutput;
	int CNT;

}MOTOR;


//�����״����ݣ�������
struct RADAR
{
	uint16_t RX_STA;
	
	uint8_t RX_BUF[20];
	
	uint32_t Distance;  //����
	
	uint32_t Angle;     //�Ƕ�
	
	u8 State;           //״̬
	
};

//�����Ӿ�����
struct VISION
{
	uint16_t RX_STA;
	uint16_t RX_BUF[20];
	uint32_t Depth;  //��ȣ�����
	uint32_t X;      //Xλ�ã�����
	u8 State;        //״̬
};

//����������״̬
typedef enum
{
	STOP = 0,
	UP,
	DOWN
}shovemotor;

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
	float ThetaD;	          //�������������x��н� �Ƕ�
	
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
extern BLOCKING Blocking;
extern struct RADAR Radar;
extern struct VISION Vision;

void Control_Init(void);
void AllPID_Init(void);
void Calc_MotorSpeed_pid(void);
//void Calc_MotorAngle_pid(void);
void Set_MotorSpeed(float V1,float V2,float V3,float V4);  //���õ�����ٶȣ����Ƶ�����
void GetMotorVelocity_Self(float vx, float vy, float w);
void GetMotorVelocity(float vx, float vy, float w);
void GetInfraredState(void);    //��ȡ���⿪��״̬
void ShoveMotor(shovemotor t);  //������״̬
void Robot_ArmDown(void);       //��е���½�
void Robot_ArmUp(void);         //��е������
static float adjustAngleVw_PD(float D_Theta);  //PD�������ٶ�
static float adjustVy_PD(float D_Y);           //PD����Y���ٶ�
static float adjustVx_PD(float D_X);           //PD����X���ٶ�
static float adjust_circle(float D_Theta);
void RobotRotate_circle(void);                 //��תһ��
void RobotRotate(float theta);                 //�����˶����������Ƕȣ��Զ�����
void RobotGoTo(float X_I,float Y_I,float Theta_I);  //����Ŀ�ĵ�
void RobotGoAvoidance(void);



#endif
