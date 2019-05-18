/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：imu.h
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/
#ifndef __IMU_H
#define __IMU_H
//外部文件引用
#include "stdint.h"
#include "mpu6050.h"

//宏定义区



//数据结构声明
//数据结构声明
typedef struct
{
    float x;
    float y;
    float z;
}Vectorf_t;

typedef struct{
    float roll;
    float pitch;
    float yaw;
}Attitude_t;

typedef struct {  //四元数
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion_t;

typedef struct
{
    Quaternion_t    Quat;
    Attitude_t      Attitude;
    Vectorf_t       Acc;
    Vectorf_t       Gyro;
    Vectorf_t       Gravity;
    Vectorf_t       vecxZ,vecyZ,veczZ;
    
    float           ConvergenceKp;
    float           DCM[3][3];
    float           DCM_T[3][3];
}IMU_t;

//Extern引用
extern Attitude_t g_Attitude;    //当前角度姿态值


//函数声明
void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt);
void GetAngle(Attitude_t *pAngE);
void ResetAttitude();
#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

