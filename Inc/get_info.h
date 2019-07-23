#ifndef __GETPOSITION_H
#define __GETPOSITION_H

#include "control.h"
#include "tim.h"

void ReadEncoder(void);
void ReceiveIMUData(void);
void Get_Position(void);  //坐标转换，里程计定位
void GetYaw(void);        //获取偏航角
void ReceiveRadarData(void);
u8 GetRadarData(void);    //雷达数据处理
void ReceiveVisionData(void);
u8 GetVisionData(void);   //视觉数据处理

#endif
