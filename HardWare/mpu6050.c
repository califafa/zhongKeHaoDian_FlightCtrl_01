/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：mpu6050.c
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

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
MPU6050驱动的使用方法如下：
1.调用MPU6050Init函数，查看当前MPU6050是否初始化成功；
2.固定周期调用GetMPU6050Data，以获取传感器数据；

PS：传感器数据存放在g_MPUManager中


*/
//外部文件引用
#include "include.h"
#include "mpu6050.h"
#include "filter.h"
#include <string.h>
#include "LED.h"
#include "myMath.h"
#include "kalman.h"
#include "i2c.h"


//宏定义区
#define SMPLRT_DIV          0x19    //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIGL             0x1A    //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG         0x1B    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define ACCEL_CONFIG        0x1C    //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define ACCEL_ADDRESS       0x3B
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_ADDRESS        0x43
#define GYRO_XOUT_L         0x44    
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define PWR_MGMT_1          0x6B    //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I            0x75    //IIC地址寄存器(默认数值0x68，只读)
#define MPU6050_PRODUCT_ID  0x68
#define MPU6052C_PRODUCT_ID 0x72
#define MPU6050_ADDRESS     0xD0    //0x68

#define Acc_Read()          I2C_Read_Bytes(MPU6050_ADDRESS, 0X3B, buffer, 6) //读取加速度
#define Gyro_Read()         I2C_Read_Bytes(MPU6050_ADDRESS, 0x43, &buffer[6], 6)  //  读取角速度
#define MPU6050_RawRead()     I2C_Read_Bytes(MPU6050_ADDRESS, 0X3B, buffer, 14) //读取加速度
//Extern引用



//私有函数区


//私有变量区
MPU6050Manager_t g_MPUManager;   //g_MPUManager原始数据
int16_t *pMpu = (int16_t *)&g_MPUManager;
/******************************************************************************
  * 函数名称：MPU6050Init
  * 函数描述：g_MPUManager的初始化
  * 输    入：void
  * 输    出：g_MPUManager初始化结果   
              0:初始化成功
              1:初始化失败
  * 返    回： 
  * 备    注：    
  *
  *
******************************************************************************/
bool MPU6050Init(void) //初始化
{   
    uint8_t check = 0;
    
    delay_ms(200);
    I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,    0x00);   //复位
    delay_ms(200);
    I2C_Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV,   0);   //陀螺仪采样率，0x00(333Hz)
    I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,   0x03);   //设置设备时钟源，陀螺仪Z轴
    delay_ms(200);
    I2C_Write_Byte(MPU6050_ADDRESS, CONFIGL,      0x04);   //低通滤波频率，0x03(42Hz)
    I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG,  0x18);   //+-2000deg/s
    I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x18);   //+-4G

    check = I2C_Read_Byte(MPU6050_ADDRESS, PWR_MGMT_1); 
    check =  I2C_Read_Byte(MPU6050_ADDRESS, GYRO_CONFIG);  //判断g_MPUManager地址
    check = I2C_Read_Byte(MPU6050_ADDRESS, ACCEL_CONFIG);  //判断g_MPUManager地址

    check = I2C_Read_Byte(MPU6050_ADDRESS, 0x75);  //判断g_MPUManager地址

    if(check != MPU6050_PRODUCT_ID) //如果地址不正确
    {
        g_MPUManager.Check = false;
        return false;
    }
    else
    {
        GetMPU6050Offset(); //调用校准数据
        g_MPUManager.Check = true;
        return true;
    }
}

/*****************************************************************************
  * 函数名称：GetMPU6050Data
  * 函数描述：读取陀螺仪和加速度计的数据并做滤波处理
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void GetMPU6050Data(void) 
{
    int i;
    uint8_t buffer[14];   
    static float mpu_filter[3][6];
    int16_t mpu_filter_tmp[6];
    
    Acc_Read();          
    Gyro_Read();
    
    for(i = 0; i < 6; i++)
    {
        //依次获取buffer中的传感器数据
        mpu_filter_tmp[i] = (((int16_t)buffer[i << 1] << 8) | buffer[(i << 1) + 1]) - g_MPUManager.Offset[i];
        
        //二阶LPF
        mpu_filter[0][i] += 0.25f * (mpu_filter_tmp[i] - mpu_filter[0][i]);
        mpu_filter[1][i] += 0.25f * (mpu_filter[0][i]  - mpu_filter[1][i]);
        
        pMpu[i] = (int16_t)mpu_filter[1][i];
    }
}

/******************************************************************************
  * 函数名称：GetMPU6050Offset
  * 函数描述：获取g_MPUManager静态下传感器偏差
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void GetMPU6050Offset(void) //校准
{
    int32_t buffer[6] = {0};
    int16_t i = 0;  
    uint8_t k = 30;
    const int8_t MAX_GYRO_QUIET = 5;
    const int8_t MIN_GYRO_QUIET = -5;    
    
    int16_t LastGyro[3] = {0};  /*wait for calm down*/
    int16_t ErrorGyro[3] = {0};        /*set offset initial to zero*/
    
    memset(g_MPUManager.Offset, 0, 12);
    g_MPUManager.Offset[2] = 8192;   //根据手册量程设定加速度标定值 

    while(k--)  //判断飞控是否处于静止状态
    {
        do
        {
            delay_ms(10);
            GetMPU6050Data();
            
            for(i = 0; i < 3; i++)
            {
                ErrorGyro[i] = pMpu[i + 3] - LastGyro[i];
                LastGyro[i] = pMpu[i + 3];    
            }
        }while ((ErrorGyro[0] > MAX_GYRO_QUIET) 
             || (ErrorGyro[0] < MIN_GYRO_QUIET)
             || (ErrorGyro[1] > MAX_GYRO_QUIET) 
             || (ErrorGyro[1] < MIN_GYRO_QUIET)
             || (ErrorGyro[2] > MAX_GYRO_QUIET)
             || (ErrorGyro[2] < MIN_GYRO_QUIET));
    }

    for(i = 0; i < 356; i++)  //取第100到第356组的平均值做为校准值
    {        
        GetMPU6050Data();
        
        if(100 <= i)
        {   int k;
            for(k = 0; k < 6; k++)
            {
                buffer[k] += pMpu[k];
            }
        }
    }

    for(i = 0; i < 6; i++)  //保存校准值
    {
        g_MPUManager.Offset[i] = buffer[i] >>8;
    }
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
