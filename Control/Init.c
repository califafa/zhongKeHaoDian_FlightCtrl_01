/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�Init.c
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
Ӳ����ʼ������ʹ��ʱֻ��Ҫ����Hadrware_Init����


*/
//�ⲿ�ļ�����
#include "include.h"
#include "gcs.h"
#include "led.h"
#include "spl06.h"
#include "communication.h"
#include "led.h"
#include "delay.h"
#include "uart.h"
#include "motor.h"
#include "timer.h"

//�궨����
#define HARDWARE_CHECK_LED    LED_STATUS_ON;LED_POWER_ON;delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;delay_ms(100);\
                              LED_STATUS_ON;LED_POWER_ON;delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;delay_ms(100);

#define HARDWARE_CHECK        g_MPUManager.Check && \
                              g_SPL06Manager.Check && \
                              g_NRFManager.Check

//Extern����


//˽�к�����
void PID_Init(void); 

//˽�б�����
uint32_t SysTick_count; //ϵͳʱ�����
Queue_t USB_Send_Queue;

/******************************************************************************
  * �������ƣ�Hadrware_Init
  * ������������ʼ������Ӳ���Ͳ���
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
  *    
  *
******************************************************************************/
void Hardware_Init(void)
{    
    System_Clock_Init();
    I2C_Init();
    Motor_Init();
    LEDInit();                      //LED���Ƴ�ʼ��
    MPU6050Init();                  //g_MPUManager��ʼ��
    Drv_Spl0601_Init();                   //SPL06��ʼ��
    NRF_Radio_Init();  
    if(HARDWARE_CHECK)              //Ӳ�����
    {
        g_LedManager.emLEDPower = PowerOn;
    }
    
    gcs_init();                     //����վͨ�ų�ʼ��
    PID_Init();                     //PID������ʼ��
    USART_Init(USCI_A_UART_CLOCKSOURCE_ACLK, 115200);
    Timer_Init();
}

/******************************************************************************
  * �������ƣ�PID_Init
  * ������������ʼ������PID����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null
  *    
  *
******************************************************************************/
void PID_Init(void)
{
    PIDGroup[emPID_Pitch_Pos].kp   = 5.0f;
    PIDGroup[emPID_Pitch_Spd].kp   = 1.2f;
    PIDGroup[emPID_Pitch_Spd].ki   = 1.0f;
    PIDGroup[emPID_Pitch_Spd].kd   = 0.03f;
    PIDGroup[emPID_Pitch_Spd].IntegLimitHigh = 10;
    PIDGroup[emPID_Pitch_Spd].IntegLimitLow = -10;

    PIDGroup[emPID_Roll_Pos].kp    = 5.0f;
    PIDGroup[emPID_Roll_Spd].kp    = 1.2f;
    PIDGroup[emPID_Roll_Spd].ki    = 1.0f;
    PIDGroup[emPID_Roll_Spd].kd    = 0.03f;
    PIDGroup[emPID_Roll_Spd].IntegLimitHigh = 10;
    PIDGroup[emPID_Roll_Spd].IntegLimitLow = -10;

    PIDGroup[emPID_Yaw_Pos].kp     = 8.0f;
    PIDGroup[emPID_Yaw_Spd].kp     = 3.0f;
    PIDGroup[emPID_Yaw_Spd].kd     = 0.00f;
    PIDGroup[emPID_Yaw_Spd].OutLimitHigh = 0;
    PIDGroup[emPID_Yaw_Spd].OutLimitLow = -0;


//====
    PIDGroup[emPID_Roll_Spd].IntegLimitHigh = 50;
    PIDGroup[emPID_Roll_Spd].IntegLimitLow = -50;

    //��ʼ��UAV�����Ϣ
    g_UAVinfo.UAV_Mode = Altitude_Hold;
}


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
