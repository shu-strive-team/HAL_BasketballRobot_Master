#ifndef __GETPOSITION_H
#define __GETPOSITION_H

#include "control.h"
#include "tim.h"

void ReadEncoder(void);
void ReceiveIMUData(void);
void Get_Position(void);  //����ת������̼ƶ�λ
void GetYaw(void);        //��ȡƫ����
void ReceiveRadarData(void);
u8 GetRadarData(void);    //�״����ݴ���
void ReceiveVisionData(void);
u8 GetVisionData(void);   //�Ӿ����ݴ���

#endif
