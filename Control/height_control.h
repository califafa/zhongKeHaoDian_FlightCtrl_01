/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�height_control.h
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
#ifndef __HEIGHT_CONTROL_H
#define __HEIGHT_CONTROL_H
//�ⲿ�ļ�����
#include "stdint.h"


//�궨����



//���ݽṹ����
typedef struct
{
    float Z_Speed;
    float Z_Acc;
    float Z_Postion;

    float Alt;
    uint16_t Thr;
}HeightInfo_t;


//Extern����
extern HeightInfo_t HeightInfo;


//��������
void ALT_Ctrl(float dT_s);
#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/