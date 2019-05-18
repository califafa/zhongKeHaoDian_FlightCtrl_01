/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�myMath.c
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
�����ѧ���㺯����ֱ�Ӳ�ѯ����ʹ�ü���


*/
//�ⲿ�ļ�����
#include "myMath.h"
#include <math.h>


//�궨����



//Extern����



//˽�к�����



//˽�б�����
const float M_PI1 = 3.1415926535f;
const float RtA = 57.2957795f;
const float AtR = 0.0174532925f;
const float Gyro_G = 0.03051756f * 2;          
const float Gyro_Gr = 0.0005326f * 2;    
const float PI_2 = 1.570796f;

/******************************************************************************
  * �������ƣ�sine
  * ������������������ֵ
  * ��    �룺float x:�Ƕ�ֵ
  * ��    ����void
  * ��    �أ�����ֵ
  * ��    ע��null
  *    
  *
******************************************************************************/
float sine(float x)          // (-M_PI , M_PI) ???? 0.0005
{
    float Q = 0.775f;
    float P = 0.225f;
    float B =  1.273239544;
    float tmp_c = -0.405284f;
    float y = B * x + tmp_c * x * fabs(x); 
    return (Q * y + P * y * fabs(y));
}

/******************************************************************************
  * �������ƣ�cosine
  * ������������������ֵ
  * ��    �룺float x:�Ƕ�
  * ��    ����
  * ��    �أ�����ֵ 
  * ��    ע��cos(x)=sin(M_PI / 2 + x)=sin(M_PI / 2 - x)  
  *    
  *
******************************************************************************/
float cosine(float x)
{
    return sine(x + M_PI / 2);
}

/******************************************************************************
  * �������ƣ�arctan
  * ���������������к���
  * ��    �룺����������
  * ��    ����void
  * ��    �أ�������ֵ
  * ��    ע���������������չ��ʽ ����Խ�ߣ�ֵԽ׼ȷ70��������׼ȷ��  
  *    
  *
******************************************************************************/
float arctan(float x)  //  (-1 , +1)    6? ?? 0.002958 
{
    float t = x;
    float result = 0;
    float X2 = x * x;
    unsigned char cnt = 1;
    do
    {
        result += t / ((cnt << 1) - 1);
        t = -t;
        t *= X2;
        cnt++;
    }while(cnt <= 6);
    
    return result;
}

/******************************************************************************
  * �������ƣ�arcsin
  * ���������������Һ���
  * ��    �룺float x:����������
  * ��    ����void
  * ��    �أ�����������
  * ��    ע���������������չ��ʽ -1 < x < +1     42��������׼ȷ��    
  *    
  *
******************************************************************************/
float arcsin(float x)
{
    float d = 1;
    float t = x;
    unsigned char cnt = 1;
    float result = 0;    
    float X2 = x * x;
    
    if (x >= 1.0f) 
    {
        return PI_2;
    }
    if (x <= -1.0f) 
    {
        return -PI_2;
    }
    do
    {
        result += t / (d * ((cnt << 1) - 1));
        t *= X2 * ((cnt << 1) - 1);//
        d *= (cnt << 1);//2 4 6 8 10 ...
        cnt++;
    }while(cnt <= 6);

    return result;
}

/******************************************************************************
  * �������ƣ�Q_rsqrt
  * �������������ټ��� 1 / Sqrt(x) 
  * ��    �룺float number:Ҫ���������
  * ��    ����void
  * ��    �أ�1 / Sqrt(x) 
  * ��    ע��null
  *    
  *
******************************************************************************/
float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;
 
    x2 = number * 0.5F;
    y  = number;
    i  = *(long*) &y;                      
    i  = 0x5f3759df - ( i >> 1 );               
    y  = *(float*) &i;
    y  = y * (threehalfs - (x2 * y * y ));   // 1st iteration ����һ��ţ�ٵ�����
    return y;
}


/******************************************************************************
  * �������ƣ�data_limit
  * ���������������޷�
  * ��    �룺float data:Ҫ���������� 
              float toplimit:����
              float lowerlimit:����
  * ��    ����
  * ��    �أ� 
  * ��    ע��    
  *    
  *
******************************************************************************/
float data_limit(float data, float toplimit, float lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
    return data;
}


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
