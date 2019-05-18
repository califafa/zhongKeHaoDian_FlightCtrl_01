/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�imu.h
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/
#ifndef __IMU_H
#define __IMU_H
//�ⲿ�ļ�����
#include "stdint.h"
#include "mpu6050.h"

//�궨����



//���ݽṹ����
//���ݽṹ����
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

typedef struct {  //��Ԫ��
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

//Extern����
extern Attitude_t g_Attitude;    //��ǰ�Ƕ���ֵ̬


//��������
void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt);
void GetAngle(Attitude_t *pAngE);
void ResetAttitude();
#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

