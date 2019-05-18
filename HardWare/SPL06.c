/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�SPL06.c
  * ժ    Ҫ�����ļ���������spl06��ѹ�ƣ���ȡ��ѹ��Ϣ����������߶�
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
SPL06�����ɰ����·�ʽʹ�ã�
1.���� SPL06_Init() �������Գ�ʼ��Ӳ���豸��
2.�̶�Ƶ�ʵ��� UpdateSPL06Info() �������Ը�����ѹֵ�͸߶�ֵ��
*/
#ifndef SPL06_C
#define SPL06_C


//�ⲿ�ļ�����
#include "SPL06.h"
#include "i2c.h"
#include <math.h>
#include <stdio.h>


//�궨����
#define PRS_CFG                 0x06
#define TMP_CFG                 0x07
#define MEAS_CFG                0x08
#define SPL06_REST_VALUE        0x09
#define PRODUCT_ID              0X0D

//(1 / 5.25588f) Pressure factor
#define CONST_PF                0.1902630958    
#define FIX_TEMP                25     
#define SPL06_Check             I2C_Read_Byte(HW_ADR, 0x0D)

//Extern����


//˽�к�����

static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

void spl0601_get_calib_param ( void );

//˽�б�����
SPL06Manager_t g_SPL06Manager;

/*****************************************************************************
  * ��������:spl0601_write
  * ��������:дspl06�Ĵ�����Ϣ
  * ��    ��:
  * uint8_t regadr:�Ĵ�����ַ
  * uint8_t val:д��Ĵ���������
  * ��    ��:void
  * ��    ��:void
  * ��    ע:null
  *
  *
*****************************************************************************/
static void spl0601_write ( unsigned char regadr, unsigned char val )
{
    I2C_Write_Byte(HW_ADR, regadr, val);  
    
}

/*****************************************************************************
  * ��������:spl0601_read
  * ��������:��ȡspl06�Ĵ�����Ϣ
  * ��    ��:
  * uint8_t regadr:�Ĵ�����ַ
  * ��    ��:void
  * ��    ��:void
  * ��    ע:null
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
  * ��������:spl0601_rateset
  * ��������:�����¶ȴ�������ÿ����������Լ���������
  * ��    ��:
  * uint8_t iSensor:��������,���ֵΪ128
  * uint8_t u8SmplRate:ÿ���������(Hz),���ֵΪ128
  * uint8_t u8OverSmpl  :������ѡ��
  *                     0:��ѹ��
  *                     1:�¶ȼ�
  * ��    ��:void
  * ��    ��:void
  * ��    ע:null
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
  * �������ƣ�spl0601_get_calib_param
  * ������������ȡУ׼����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null   
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
  * �������ƣ�spl0601_start_temperature
  * ��������������һ���¶Ȳ���
  * ��    �룺
  * ��    ����
  * ��    �أ�
  * ��    ע��
  *    
  *
*****************************************************************************/
void spl0601_start_temperature ( void )
{
    spl0601_write ( 0x08, 0x02 );
}

/*****************************************************************************
  * �������ƣ�spl0601_start_pressure
  * ��������������һ��ѹ������
  * ��    �룺
  * ��    ����
  * ��    �أ�
  * ��    ע��
  *    
  *
*****************************************************************************/
void spl0601_start_pressure ( void )
{
    spl0601_write ( 0x08, 0x01 );
}

/******************************************************************************
  * �������ƣ�spl0601_start_continuous
  * ����������Select node for the continuously measurement
  * ��    �룺
  * uint8_t mode:ģʽѡ��
  *              1:��ѹģʽ;
  *              2:�¶�ģʽ; 
  *              3:��ѹ���¶�ģʽ;
  * ��    ����
  * ��    �أ�
  * ��    ע��
  *    
  *
******************************************************************************/
void spl0601_start_continuous ( uint8_t mode )
{
    spl0601_write ( 0x08, mode + 4 );
}

/******************************************************************************
  * �������ƣ�spl0601_get_raw_temp
  * ������������ȡ�¶ȵ�ԭʼֵ����ת����32Bits����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
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
  * �������ƣ�spl0601_get_raw_pressure
  * ������������ȡѹ��ԭʼֵ����ת����32bits����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null      
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
  * �������ƣ�Drv_Spl0601_Init
  * ������������ʼ��SPL06
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null   
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
  * �������ƣ�spl0601_get_temperature
  * �����������ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼����¶�ֵ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null   
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
  * �������ƣ�spl0601_get_pressure
  * �����������ڻ�ȡԭʼֵ�Ļ����ϣ����ظ���У׼���ѹ��ֵ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null   
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

    //ʹ�����κ�����ϴ���ѹ����
    alt_3 = ( 101400 - baro_pressure ) / 1000.0f;
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101400 - baro_pressure ) * 100.0f ;

    return height;
}

/******************************************************************************
  * �������ƣ�ResetAlt
  * �������������õ���߶�
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null     
  *    
  *
******************************************************************************/
void ResetAlt()
{
    g_SPL06Manager.fGround_Alt = g_SPL06Manager.fALT;
}

/******************************************************************************
  * �������ƣ�UpdateSPL06Info
  * ����������������ѹ�Ƹ߶���Ϣ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null     
  *    
  *
******************************************************************************/
void UpdateSPL06Info()
{
//���ʹ�ô���ѹ��ʽ����߶�
#if 0
    /* tropospheric properties (0-11km) for standard atmosphere */
    /* temperature at base height in Kelvin, [K] = [��C] + 273.15 */
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

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
