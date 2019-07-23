#include "mpu6050.h"
#include "usart.h"
#include "delay.h"

/*******************************

Ϊģ�黯��MPU6050��д����ģ���Դ��������˲��㷨
ʹ��ǰ����ͨ����λ���Լ��ٶȡ����ٶ�������
MPU_Init()����ΪZ��У׼

********************************/

void IMU_Init()
{
	//�Ƕȳ�ʼ��ָ�ʹZ��Ƕȹ��㣺0XFF,0XAA,0X76,0X00,0X00
	u8 cmd[5] = {0XFF,0XAA,0X76,0X00,0X00};
	HAL_UART_Transmit(&huart2,cmd,5,1000);
	
//	delay_ms(10);
	delay_ms(10);
}


