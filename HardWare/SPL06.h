/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：SPL06.h
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
#ifndef SPL06_H
#define SPL06_H
//外部文件引用
#include "stdint.h"
#include "stdbool.h"
//宏定义区
#define HW_ADR_L                    0x76//SDO LOW
#define HW_ADR                      HW_ADR_L<<1
#define CONTINUOUS_PRESSURE         1
#define CONTINUOUS_TEMPERATURE      2
#define CONTINUOUS_P_AND_T          3
#define PRESSURE_SENSOR             0
#define TEMPERATURE_SENSOR          1
#define OFFSET_COUNT                30

//数据结构声明

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
//Extern引用
extern SPL06Manager_t g_SPL06Manager;

void Drv_SPL06CSPin_Init(void);
uint8_t Drv_Spl0601_Init(void);
float Drv_Spl0601_Read(void);
void ResetAlt(void);
void UpdateSPL06Info(void);

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
