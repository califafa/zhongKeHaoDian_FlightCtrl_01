/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�SPL06.h
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
#ifndef SPL06_H
#define SPL06_H
//�ⲿ�ļ�����
#include "stdint.h"
#include "stdbool.h"
//�궨����
#define HW_ADR_L                    0x76//SDO LOW
#define HW_ADR                      HW_ADR_L<<1
#define CONTINUOUS_PRESSURE         1
#define CONTINUOUS_TEMPERATURE      2
#define CONTINUOUS_P_AND_T          3
#define PRESSURE_SENSOR             0
#define TEMPERATURE_SENSOR          1
#define OFFSET_COUNT                30

//���ݽṹ����

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

struct spl0601_calib_param_t {	
    int c0;
    int c1;
    long c00;
    long c10;
    int c01;
    int c11;
    int c20;
    int c21;
    int c30;       
};

struct spl0601_t {	
    struct spl0601_calib_param_t calib_param;/**<calibration data*/	
    uint8_t 			chip_id; /**<chip id*/	
    long 	i32rawPressure;
    long 	i32rawTemperature;
    long 	i32kP;    
    long 	i32kT;
};
typedef struct
{    
    int i16C0;
    int i16C1;
    long i32C00;
    long i32C10;
    int i16C01;
    int i16C11;
    int i16C20;
    int i16C21;
    int i16C30;       
}SPL06Param_t0;

typedef struct
{    
    SPL06Param_t0 Param;
    uint8_t u8Chip_id;
    long i32RawPressure;
    long i32RawTemperature;
    long i32KP;
    long i32KT;
    
    long fGround_Alt;
    long fALT;                  //height above sea level        
    float fRelative_Alt;
    
    float fTemperature;
    float fPressure;
    float fLast_Pressure;
    
    float fOffset;
    bool Check;
}SPL06Manager_t;
//Extern����
extern SPL06Manager_t g_SPL06Manager;

void Drv_SPL06CSPin_Init(void);
uint8_t Drv_Spl0601_Init(void);
float Drv_Spl0601_Read(void);
void ResetAlt(void);
void UpdateSPL06Info(void);

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
