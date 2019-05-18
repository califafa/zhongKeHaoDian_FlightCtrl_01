/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�imu.c
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
imu������ֱ�ӵ���GetAngle,��̬���ݴ洢��g_Attitude��


*/
//�ⲿ�ļ�����
#include "imu.h"
#include "myMath.h"
#include <math.h>
#include "height_control.h"

//�궨����



//Extern����
typedef struct 
{
  float x;
  float y;
  float z;
}Vector_t; 


//˽�к�����



//˽�б�����
Attitude_t g_Attitude;    //��ǰ�Ƕ���ֵ̬
static Quaternion_t NumQ = {1, 0, 0, 0};
float vecxZ,vecyZ,veczZ;
float wz_acc_tmp[2];
/******************************************************************************
  * �������ƣ�GetAngle
  * �������������ݴ��������ݼ���õ���ǰ��̬
  * ��    �룺const MPU6050Manager_t *pMpu��ԭʼ����������
               float dt����λ����ʱ��
  * ��    ����Attitude_t *pAngE������Ƕ�ֵ
  * ��    �أ�void
  * ��    ע��null    
  *    
  *
******************************************************************************/
void ATT_Update(const MPU6050Manager_t *pMpu,Attitude_t *pAngE, float dt) 
{    
    Vector_t Gravity,Acc,Gyro,AccGravity;
    static Vector_t GyroIntegError = {0};
    static float KpDef = 0.8f ; //��Ԫ������ֵ
    static float KiDef = 0.0003f; 
    float q0_t,q1_t,q2_t,q3_t;
    float NormQuat; 
    float HalfTime = dt * 0.5f;

    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);                
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);              
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);  
    // ���ٶȹ�һ����
    NormQuat = Q_rsqrt(squa(g_MPUManager.accX)+ squa(g_MPUManager.accY) +squa(g_MPUManager.accZ));

    Acc.x = pMpu->accX * NormQuat; //��һ��ɻ�Ϊ��λ�����·������
    Acc.y = pMpu->accY * NormQuat;  
    Acc.z = pMpu->accZ * NormQuat;  

    //������˵ó���ֵ����˺���Եõ���ת����������������µļ��ٶȷ����ϵ�ƫ��
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    //���ٶ��ںϼ��ٶȱ�������ֵ�����������乲ͬ�γ���PI�������õ�������Ľ��ٶ�ֵ
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//�����ƣ��˴��������ǽ��ٶȵ�Ư��
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;    
    // һ�����������, ������Ԫ��
    //������Ľ��ٶ�ֵ���֣��õ�������̬��������Ԫ��һ��ʵ��Q0�������鲿Q1~3��ֵ�ı仯
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t; //���ֺ��ֵ�ۼӵ��ϴε���Ԫ���У����µ���Ԫ��
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;
    
    // ������Ԫ����һ�����õ���λ������
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3)); //�õ���Ԫ����ģ��
    NumQ.q0 *= NormQuat; //ģ��������Ԫ��ֵ
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;
    
    /*��������ϵ�µ�Z��������*/
    float NormAccz = -pMpu->accX * vecxZ+ pMpu->accY * vecyZ + pMpu->accZ * veczZ;  /*Z�ᴹֱ�����ϵļ��ٶȣ���ֵ��������бʱ��Z����ٶȵ������ͣ����ǵ���������Ӧ�ó���ֵ*/        
    wz_acc_tmp[0] = (NormAccz - 8192) * 0.1196f;// cm/ss //0.1196f;����ÿ���η���
    wz_acc_tmp[1] += 0.1f *(wz_acc_tmp[0] - wz_acc_tmp[1]);//LPF
    HeightInfo.Z_Acc = wz_acc_tmp[1];
}

/******************************************************************************
  * �������ƣ�GetNormAccz
  * ��������������Z�᷽��ļ��ٶ�ֵ
  * ��    �룺void
  * ��    ��������Z�᷽��ļ��ٶ�ֵ
  * ��    �أ�void
  * ��    ע��null    
  *
  *
******************************************************************************/
float GetNormAccz(void)
{
    return 0;
}

/******************************************************************************
  * �������ƣ�GetAngle
  * �������������½Ƕ�
  * ��    �룺Attitude_t *pAngE���ǶȽṹ��ָ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
  *
  *
******************************************************************************/

void GetAngle(Attitude_t *pAngE)
{
    vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*����(3,1)��*///��������ϵ�µ�X�����������
    vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*����(3,2)��*///��������ϵ�µ�Y�����������
    veczZ = NumQ.q0 * NumQ.q0 - NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3;  /*����(3,3)��*///��������ϵ�µ�Z����������� 
    
    pAngE->yaw = atan2f((2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3),(1 - 2 * (NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3))) * RtA;
    pAngE->pitch = asin(vecxZ)* RtA;   //������          
    pAngE->roll  = atan2f(vecyZ,veczZ) * RtA;  //�����
}

void ResetAttitude()
{
    NumQ.q0 = 1;
    NumQ.q1 = 0;
    NumQ.q2 = 0;
    NumQ.q3 = 0;
}
/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
