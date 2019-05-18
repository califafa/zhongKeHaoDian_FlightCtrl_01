/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：SPL06.c
  * 摘    要：本文件用以驱动spl06气压计，获取气压信息，并解算出高度
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
SPL06驱动可按如下方式使用：
1.调用 SPL06_Init() 函数，以初始化硬件设备；
2.固定频率调用 UpdateSPL06Info() 函数，以更新气压值和高度值；
*/
#ifndef SPL06_C
#define SPL06_C


//外部文件引用
#include "SPL06.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>


//宏定义区
#define PRS_CFG                 0x06
#define TMP_CFG                 0x07
#define MEAS_CFG                0x08
#define SPL06_REST_VALUE        0x09
#define PRODUCT_ID              0X0D

//(1 / 5.25588f) Pressure factor
#define CONST_PF                0.1902630958    
#define FIX_TEMP                25     
#define SPL06_Check             I2C_Read_Byte(HW_ADR, 0x0D)

//Extern引用


//私有函数区

static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

void spl0601_get_calib_param ( void );

//私有变量区
SPL06Manager_t g_SPL06Manager;

/*****************************************************************************
  * 函数名称:spl0601_write
  * 函数描述:写spl06寄存器信息
  * 输    入:
  * uint8_t regadr:寄存器地址
  * uint8_t val:写入寄存器的数据
  * 输    出:void
  * 返    回:void
  * 备    注:null
  *
  *
*****************************************************************************/
static void spl0601_write ( unsigned char regadr, unsigned char val )
{
    I2C_Write_Byte(HW_ADR, regadr, val);  
    
}

/*****************************************************************************
  * 函数名称:spl0601_read
  * 函数描述:读取spl06寄存器信息
  * 输    入:
  * uint8_t regadr:寄存器地址
  * 输    出:void
  * 返    回:void
  * 备    注:null
  *
  *
*****************************************************************************/
static uint8_t spl0601_read ( unsigned char regadr )
{
    uint8_t reg_data;
    reg_data = I2C_Read_Byte(HW_ADR, regadr);
    
    return reg_data;
}

/******************************************************************************
  * 函数名称:spl0601_rateset
  * 函数描述:设置温度传感器的每秒采样次数以及过采样率
  * 输    入:
  * uint8_t iSensor:过采样率,最大值为128
  * uint8_t u8SmplRate:每秒采样次数(Hz),最大值为128
  * uint8_t u8OverSmpl  :传感器选择
  *                     0:气压计
  *                     1:温度计
  * 输    出:void
  * 返    回:void
  * 备    注:null
  *
  *
******************************************************************************/
void spl0601_rateset ( uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl )
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch ( u8SmplRate )
    {
    case 2:
        reg |= ( 1 << 4 );
        break;
    case 4:
        reg |= ( 2 << 4 );
        break;
    case 8:
        reg |= ( 3 << 4 );
        break;
    case 16:
        reg |= ( 4 << 4 );
        break;
    case 32:
        reg |= ( 5 << 4 );
        break;
    case 64:
        reg |= ( 6 << 4 );
        break;
    case 128:
        reg |= ( 7 << 4 );
        break;
    case 1:
    default:
        break;
    }
    switch ( u8OverSmpl )
    {
    case 2:
        reg |= 1;
        i32kPkT = 1572864;
        break;
    case 4:
        reg |= 2;
        i32kPkT = 3670016;
        break;
    case 8:
        reg |= 3;
        i32kPkT = 7864320;
        break;
    case 16:
        i32kPkT = 253952;
        reg |= 4;
        break;
    case 32:
        i32kPkT = 516096;
        reg |= 5;
        break;
    case 64:
        i32kPkT = 1040384;
        reg |= 6;
        break;
    case 128:
        i32kPkT = 2088960;
        reg |= 7;
        break;
    case 1:
    default:
        i32kPkT = 524288;
        break;
    }

    if ( iSensor == 0 )
    {
        p_spl0601->i32kP = i32kPkT;
        spl0601_write ( 0x06, reg );
        if ( u8OverSmpl > 8 )
        {
            reg = spl0601_read ( 0x09 );
            spl0601_write ( 0x09, reg | 0x04 );
        }
    }
    if ( iSensor == 1 )
    {
        p_spl0601->i32kT = i32kPkT;
        spl0601_write ( 0x07, reg | 0x80 ); //Using mems temperature
        if ( u8OverSmpl > 8 )
        {
            reg = spl0601_read ( 0x09 );
            spl0601_write ( 0x09, reg | 0x08 );
        }
    }

}
/******************************************************************************
  * 函数名称：spl0601_get_calib_param
  * 函数描述：获取校准参数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null   
  *    
  *
******************************************************************************/
void spl0601_get_calib_param ( void )
{
    unsigned long h;
    unsigned long m;
    unsigned long l;
    h =  spl0601_read ( 0x10 );
    l  =  spl0601_read ( 0x11 );
    p_spl0601->calib_param.c0 = ( int ) h << 4 | l >> 4;
    p_spl0601->calib_param.c0 = ( p_spl0601->calib_param.c0 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c0 ) : p_spl0601->calib_param.c0;
    h =  spl0601_read ( 0x11 );
    l  =  spl0601_read ( 0x12 );
    p_spl0601->calib_param.c1 = ( int ) ( h & 0x0F ) << 8 | l;
    p_spl0601->calib_param.c1 = ( p_spl0601->calib_param.c1 & 0x0800 ) ? ( 0xF000 | p_spl0601->calib_param.c1 ) : p_spl0601->calib_param.c1;
    h =  spl0601_read ( 0x13 );
    m =  spl0601_read ( 0x14 );
    l =  spl0601_read ( 0x15 );
    p_spl0601->calib_param.c00 = ( int32_t ) h << 12 | ( int32_t ) m << 4 | ( int32_t ) l >> 4;
    p_spl0601->calib_param.c00 = ( p_spl0601->calib_param.c00 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c00 ) : p_spl0601->calib_param.c00;
    h =  spl0601_read ( 0x15 );
    m =  spl0601_read ( 0x16 );
    l =  spl0601_read ( 0x17 );
    p_spl0601->calib_param.c10 = ( int32_t ) h << 16 | ( int32_t ) m << 8 | l;
    p_spl0601->calib_param.c10 = ( p_spl0601->calib_param.c10 & 0x080000 ) ? ( 0xFFF00000 | p_spl0601->calib_param.c10 ) : p_spl0601->calib_param.c10;
    h =  spl0601_read ( 0x18 );
    l  =  spl0601_read ( 0x19 );
    p_spl0601->calib_param.c01 = ( int ) h << 8 | l;
    h =  spl0601_read ( 0x1A );
    l  =  spl0601_read ( 0x1B );
    p_spl0601->calib_param.c11 = ( int ) h << 8 | l;
    h =  spl0601_read ( 0x1C );
    l  =  spl0601_read ( 0x1D );
    p_spl0601->calib_param.c20 = ( int ) h << 8 | l;
    h =  spl0601_read ( 0x1E );
    l  =  spl0601_read ( 0x1F );
    p_spl0601->calib_param.c21 = ( int ) h << 8 | l;
    h =  spl0601_read ( 0x20 );
    l  =  spl0601_read ( 0x21 );
    p_spl0601->calib_param.c30 = ( int ) h << 8 | l;
}
/*****************************************************************************
  * 函数名称：spl0601_start_temperature
  * 函数描述：发起一次温度测量
  * 输    入：
  * 输    出：
  * 返    回：
  * 备    注：
  *    
  *
*****************************************************************************/
void spl0601_start_temperature ( void )
{
    spl0601_write ( 0x08, 0x02 );
}

/*****************************************************************************
  * 函数名称：spl0601_start_pressure
  * 函数描述：发起一次压力测量
  * 输    入：
  * 输    出：
  * 返    回：
  * 备    注：
  *    
  *
*****************************************************************************/
void spl0601_start_pressure ( void )
{
    spl0601_write ( 0x08, 0x01 );
}

/******************************************************************************
  * 函数名称：spl0601_start_continuous
  * 函数描述：Select node for the continuously measurement
  * 输    入：
  * uint8_t mode:模式选择
  *              1:气压模式;
  *              2:温度模式; 
  *              3:气压和温度模式;
  * 输    出：
  * 返    回：
  * 备    注：
  *    
  *
******************************************************************************/
void spl0601_start_continuous ( uint8_t mode )
{
    spl0601_write ( 0x08, mode + 4 );
}

/******************************************************************************
  * 函数名称：spl0601_get_raw_temp
  * 函数描述：获取温度的原始值，并转换成32Bits整数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void spl0601_get_raw_temp ( void )
{
    uint8_t h[3] = {0};

    h[0] = spl0601_read ( 0x03 );
    h[1] = spl0601_read ( 0x04 );
    h[2] = spl0601_read ( 0x05 );

    p_spl0601->i32rawTemperature = ( int32_t ) h[0] << 16 | ( int32_t ) h[1] << 8 | ( int32_t ) h[2];
    p_spl0601->i32rawTemperature = ( p_spl0601->i32rawTemperature & 0x800000 ) ? ( 0xFF000000 | p_spl0601->i32rawTemperature ) : p_spl0601->i32rawTemperature;
}

/******************************************************************************
  * 函数名称：spl0601_get_raw_pressure
  * 函数描述：获取压力原始值，并转换成32bits整数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null      
  *    
  *
******************************************************************************/
void spl0601_get_raw_pressure ( void )
{
    uint8_t h[3];

    h[0] = spl0601_read ( 0x00 );
    h[1] = spl0601_read ( 0x01 );
    h[2] = spl0601_read ( 0x02 );

    p_spl0601->i32rawPressure = ( int32_t ) h[0] << 16 | ( int32_t ) h[1] << 8 | ( int32_t ) h[2];
    p_spl0601->i32rawPressure = ( p_spl0601->i32rawPressure & 0x800000 ) ? ( 0xFF000000 | p_spl0601->i32rawPressure ) : p_spl0601->i32rawPressure;
}
/*****************************************************************************
  * 函数名称：Drv_Spl0601_Init
  * 函数描述：初始化SPL06
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null   
  *    
  *
*****************************************************************************/
uint8_t Drv_Spl0601_Init ( void )
{
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = spl0601_read ( 0x0D );// 0x34  0x10
	
    spl0601_get_calib_param();

    spl0601_rateset ( PRESSURE_SENSOR, 128, 16 );

    spl0601_rateset ( TEMPERATURE_SENSOR, 8, 8 );

    spl0601_start_continuous ( CONTINUOUS_P_AND_T );
	
    delay_ms(2000);
    g_SPL06Manager.fGround_Alt = (long)Drv_Spl0601_Read();

    if(p_spl0601->chip_id == 0x10)
    {
        g_SPL06Manager.Check = true;
        return 1;
    }
    else
    {
        return 0;
    }
}
/******************************************************************************
  * 函数名称：spl0601_get_temperature
  * 函数描述：在获取原始值的基础上，返回浮点校准后的温度值
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null   
  *    
  *
******************************************************************************/
float spl0601_get_temperature ( void )
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/******************************************************************************
  * 函数名称：spl0601_get_pressure
  * 函数描述：在获取原始值的基础上，返回浮点校准后的压力值
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null   
  *    
  *
******************************************************************************/
float spl0601_get_pressure ( void )
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / ( float ) p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * ( p_spl0601->calib_param.c20 + fPsc * p_spl0601->calib_param.c30 );
    qua3 = fTsc * fPsc * ( p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21 );
    //qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    //fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}

float baro_Offset, alt_3, height;
unsigned char baro_start;
float temperature, alt_high;
float baro_pressure;

float Drv_Spl0601_Read ( void )
{
    spl0601_get_raw_temp();
    temperature = spl0601_get_temperature();

    spl0601_get_raw_pressure();
    baro_pressure = spl0601_get_pressure();

    //使用三次函数拟合大气压函数
    alt_3 = ( 101400 - baro_pressure ) / 1000.0f;
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101400 - baro_pressure ) * 100.0f ;

    return height;
}

/******************************************************************************
  * 函数名称：ResetAlt
  * 函数描述：重置地面高度
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null     
  *    
  *
******************************************************************************/
void ResetAlt()
{
    g_SPL06Manager.fGround_Alt = g_SPL06Manager.fALT;
}

/******************************************************************************
  * 函数名称：UpdateSPL06Info
  * 函数描述：更新气压计高度信息
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null     
  *    
  *
******************************************************************************/
void UpdateSPL06Info()
{
//如果使用大气压公式计算高度
#if 0
    /* tropospheric properties (0-11km) for standard atmosphere */
    /* temperature at base height in Kelvin, [K] = [°C] + 273.15 */
    const double T1 = 15.0 + 273.15;

    /* temperature gradient in degrees per metre */    
    const double a = -6.5 / 1000;    
    
    /* gravity constant in m / s/s */
    const double g = 9.80665;    
    
    /* ideal gas constant in J/kg/K */
    const double R = 287.05;    
    
    /* current pressure at MSL in kPa */
    double p1 = 101325.0 / 1000.0;

    /* measured pressure in kPa */
    
    spl0601_get_raw_temp();
    float temperature = spl0601_get_temperature();

    spl0601_get_raw_pressure();
    float baro_pressure = spl0601_get_pressure();
    
    
    double p = baro_pressure / 1000.0f;

    //Altitude = (((exp((-(a * R) / g) * log((p / p1)))) * T1) - T1) / a;
    g_SPL06Manager.fALT = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
    g_SPL06Manager.fRelative_Alt = g_SPL06Manager.fALT - g_SPL06Manager.fGround_Alt;
#else
    
    g_SPL06Manager.fALT = (long)(1000 * Drv_Spl0601_Read ());
    g_SPL06Manager.fRelative_Alt = (g_SPL06Manager.fALT - g_SPL06Manager.fGround_Alt)/1000;
#endif
}

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
