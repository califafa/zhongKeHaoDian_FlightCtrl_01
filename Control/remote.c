/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：remote.c
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
遥控器驱动程序调用方式如下：
1.AnalyRC函数由NRF接收驱动函数调用，用以分析遥控器发送过来的数据，不需要单独调用；
2.RCReceiveHandle函数用以处理遥控器接收处理函数；

*/
//外部文件引用
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "gcs.h"
#include "communication.h"
#include "pid.h"
#include <string.h>
#include "SPL06.h"

//宏定义区
extern Queue_t USB_Send_Queue;


//Extern引用
extern float PIDGroup_desired_yaw_pos_tmp;
extern float exp_hei;



//私有函数区
void RemoteUnlock(void);    
void RCReceiveHandle(void);


//私有变量区
uint8_t RC_rxData[32];
Remote_t Remote;

 
/******************************************************************************
  * 函数名称：AnalyRC
  * 函数描述：分析遥控器数据
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void AnalyRC(void)
{
    uint8_t CheckSum = 0;
    OldRemote_t OldRemote;
    
    deQueue(&NRF_Mannager.qRx, RC_rxData,&CheckSum);
    memcpy((uint8_t*)&OldRemote, RC_rxData, sizeof(OldRemote_t));   
    if(RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)
    {
        g_LedManager.emLEDStatus = StatusFlash;
        
        Remote.roll =  ((uint16_t)RC_rxData[4]<<8)  | RC_rxData[5];  //通道1，横滚
        Remote.pitch = ((uint16_t)RC_rxData[6]<<8)  | RC_rxData[7];  //通道2，俯仰
        Remote.thr =   ((uint16_t)RC_rxData[8]<<8)  | RC_rxData[9];   //通道3，油门
        Remote.yaw =   ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //通道4，偏航
        Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //通道5  左上角按键属于通道5,也就是AUX1，定高功能  
        Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //通道6  右上角按键属于通道6,也就是AUX2，一键起飞功能  
        Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //通道7  没有提到功能的按键都属于备用 
        Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //通道8  
        Remote.AUX5 =  ((uint16_t)RC_rxData[20]<<8) | RC_rxData[21];   //通道9  
        Remote.AUX6 =  ((uint16_t)RC_rxData[22]<<8) | RC_rxData[23];   //通道10 
        Remote.AUX7 =  ((uint16_t)RC_rxData[24]<<8) | RC_rxData[25];   //通道11 
        
        RCReceiveHandle();
    }
    
    if((OldRemote.Start == 0xAAAA) && (OldRemote.Stop == 0x5555))
    {
        g_LedManager.emLEDStatus = StatusFlash;
        Remote.roll = OldRemote.ROLL;
        Remote.pitch = OldRemote.PITCH;
        Remote.thr = OldRemote.THROTTLE;
        Remote.yaw =  OldRemote.YAW;
        Remote.AUX1 =  OldRemote.SW_THREE;
        Remote.AUX2 =  OldRemote.SW_TWO;
        Remote.AUX6 = OldRemote.ContrlBit;
        RCReceiveHandle();
    }
}
 
/******************************************************************************
  * 函数名称：RemoteUnlock
  * 函数描述：遥控器解锁操作
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void RemoteUnlock(void)
{
    volatile static uint8_t status = WAITING_1;

    if(Remote.thr < 1150 &&Remote.yaw < 1200)                         //油门遥杆左下角锁定
    {
        ResetAlt();
        status = EXIT_255;
    }
    
    switch(status)
    {
        case WAITING_1://等待解锁
            //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁            
            if(Remote.thr < 1100)  //第一步        
            {             
                 status = WAITING_2;                 
            }        
            break;
        case WAITING_2://第二步 
            if(Remote.thr > 1800)          
            {        
                static uint8_t cnt = 0;
                 cnt++;        
                if(cnt > 5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
                {    
                    cnt = 0;
                    status = WAITING_3;
                }
            }            
            break;
        case WAITING_3:
            if(Remote.thr < 1100)  //第三步        
            {             
                status = WAITING_4;                 
            }
            break;            
        case WAITING_4:    //解锁前准备                   
            g_FMUflg.unlock = 1;   //解锁标志位

            status = PROCESS_31;   //进入控制
            break;
        case PROCESS_31:    //进入解锁状态
            if(!g_FMUflg.unlock)                           //其它紧急情况可直接锁定飞控
            {
                status = EXIT_255;                
            }

            break;
        case EXIT_255: //进入锁定                           
            g_FMUflg.unlock = 0;           //锁定
            status = WAITING_1;     //返回等待解锁
            break;
        default:
            status = EXIT_255;
            break;
    }
}

/******************************************************************************
  * 函数名称：RCReceiveHandle
  * 函数描述：遥控器数据接收处理函数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *
******************************************************************************/
void RCReceiveHandle()
{
    const float roll_pitch_ratio = 0.04f;  //遥控控制姿态的量

    volatile int16_t temp = Remote.pitch - 1500;
    temp = temp * roll_pitch_ratio;
    PIDGroup[emPID_Pitch_Pos].desired = temp;
    
    temp = Remote.roll - 1500;
    temp = temp * roll_pitch_ratio;
    PIDGroup[emPID_Roll_Pos].desired  = temp;
    
    if(g_FMUflg.unlock == 1 && Remote.thr > 1300)
    {
        if(Remote.yaw > 1700 )
        {    //以下为遥控控制偏航角 +-号代表方向 0.75代表控制偏航角的旋转量                            
            PIDGroup_desired_yaw_pos_tmp -= 1;    
        }
        else if(Remote.yaw <1300)
        {
            PIDGroup_desired_yaw_pos_tmp += 1;
        }

        if(PIDGroup[emPID_Yaw_Pos].desired>=180)
        {
            PIDGroup_desired_yaw_pos_tmp -= 360;
        }
        else if(PIDGroup[emPID_Yaw_Pos].desired<-180)
        {
            PIDGroup_desired_yaw_pos_tmp += 360;
        }
    }
    
    if(Remote.AUX1<1200)
    {
        g_UAVinfo.UAV_Mode = Stabilize_Mode;
    }
    else if(Remote.AUX1<1600)
    {
        g_UAVinfo.UAV_Mode = Altitude_Hold;
    }
    else if(Remote.AUX1<2100)
    {
        g_UAVinfo.UAV_Mode = AutoLand;
    }
    else
    {
    
    }
    
    if(g_UAVinfo.UAV_Mode == Altitude_Hold)
    {

    }
    
    switch(Remote.AUX2)
    {
        case 2000:
            
            break;
        case 1000:
            g_FMUflg.unlock = 0;
            ResetPID();        
            ResetAlt();
            break;
    }

//一键起飞
    if(Remote.AUX6)
    {
        if(g_UAVinfo.UAV_Mode == Altitude_Hold)
        {
            g_FMUflg.unlock = 1;
            exp_hei = 50;
        }
    }

//内八上锁
    if(Remote.thr < 1300 &&Remote.yaw > 1700 && Remote.pitch < 1300 && Remote.roll < 1300)                         //油门遥杆左下角锁定
    {
        g_FMUflg.unlock = 0;
        ResetAlt();
    }
    
    RemoteUnlock();
}

/******************************************************************************
  * 函数名称：UpdateFMUToRemote
  * 函数描述：更新飞控数据到遥控器
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：此函数周期运行，通过遥控器把数据穿给上位机
  *    
  *
******************************************************************************/
void UpdateFMUToRemote()
{
    uint8_t Buff[QUEUE_DATA_MAXLENGTH];
    uint8_t length;

    if(deQueue(&USB_Send_Queue, Buff,&length))
    {
        NRF_Mannager.Hardware_Mannager->send_buff(Buff);
    }
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
