/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�height_control.c
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
�߶ȿ�������


*/
//�ⲿ�ļ�����
#include "height_control.h"
#include "control.h"
#include "SPL06.h"
#include "imu.h"
#include "mpu6050.h"
#include "fmuConfig.h"
#include "myMath.h"
#include "math.h"
#include "Remote.h"
#include "pid.h"

//�궨����
#define IN_RANGE(value, min, max)        ( (value)>(min) && (value)<(max) )
#define THROTTLE_BASE 600

//Extern����
extern SPL06Manager_t g_SPL06Manager;


//˽�к�����

//˽�б�����
HeightInfo_t HeightInfo;
float dt2 = 0;
bool Acc_Enable_Flag = false;

#include "gcs.h"

#define MAX_EXP_WZ_VEL_UP 200
#define MAX_EXP_WZ_VEL_DW 150
#define MAX_EXP_WZ_ACC    500

//wz_ctrl struct
static float exp_vel_transition[4];
static float exp_vel_d;

static float exp_acc;
static float fb_acc;
static float exp_vel;
static float fb_vel;
float exp_hei;
static float fb_hei;

//pid struct
static float hei_err,vel_err,acc_err,acc_err_i,acc_out,wz_out;

#define H_KP 1.5f
#define V_KP 5.0f
#define V_KD 0.05f
#define A_KP 0.4f
#define A_KI 0.6f

//fc����˼�Ƿɿ�
//extern uint8_t fc_state
uint8_t fc_state_take_off = 0;
void ALT_Ctrl(float dT_s)
{
    //==input calculate
    //fb = feedback ���·�����Ϣ
    fb_vel = HeightInfo.Z_Speed;
    fb_hei = HeightInfo.Z_Postion;
    fb_acc = HeightInfo.Z_Acc;
    
    //�����ٶ���ң��������ת������
    exp_vel_transition[0] = (Remote.thr - 1500)*0.0008f;
    
    //��������SSZQxaQxzz
    if(exp_vel_transition[0]<0.3f && exp_vel_transition[0]>-0.3f)
    {
        exp_vel_transition[0] = 0;
    }
    
    //����������Χ������������ٶ�
    if(exp_vel_transition[0] > 0)
    {
        exp_vel_transition[1] = exp_vel_transition[0] * MAX_EXP_WZ_VEL_UP;
    }
    else
    {
        exp_vel_transition[1] = exp_vel_transition[0] * MAX_EXP_WZ_VEL_DW;
    }
    
    //�������������ٶ�����
    float tmp = MAX_EXP_WZ_ACC * dT_s;
    
    //���������ٶ�����
    exp_vel_d = (exp_vel_transition[1] - exp_vel_transition[2]);
    
    //�����ٶ������޷�
    if(exp_vel_d > tmp)
    {
        exp_vel_d = tmp;
    }
    else if(exp_vel_d < -tmp)
    {
        exp_vel_d = -tmp;
    }
    
    //�����ٶ�Ϊ�����ٶ��������൱�ڼ��ٶȻ���
    exp_vel_transition[2] += exp_vel_d;
    
    //�����ٶ�LPF
    exp_vel_transition[3] += 0.2f *(exp_vel_transition[2] - exp_vel_transition[3]);
    
    //==exp_val state
    //
    if(g_UAVinfo.UAV_Mode >= Altitude_Hold && fc_state_take_off != 0 )
    {
        //���յ������ٶ�Ϊexp_vel
        exp_vel = exp_vel_transition[3];
        
        //�����߶�Ϊ�����ٶȵĻ���
        exp_hei += exp_vel * dT_s;
        
        //�����߶��޷�
        if(exp_hei > fb_hei+150)
        {
            exp_hei = fb_hei+150;
        }
        else if(exp_hei < fb_hei-150)
        {
            exp_hei = fb_hei-150;
        }
    }
    else
    {
        exp_vel = 0;
        exp_hei = fb_hei;
 
    }
    
    //==ctrl
    //�߶���� = �����߶���� - �������
    hei_err = (exp_hei - fb_hei);
    
    
    //�ٶ���� = (Kp * �߶���� + �����ٶ�) - (�����ٶ� + �������ٶ� * kd)
    //Ϊ�δ˴��ٶ���Ϊ��vel_err = exp_vel - fb_vel;
    //PD���ƣ����������������ٶ�
    vel_err = ((H_KP * hei_err + exp_vel) - (fb_vel + V_KD *fb_acc));
    
    //�������ٶ� = Kp * �ٶ�����P���ƣ������������������ٶ�
    exp_acc = (V_KP * vel_err);
    
    //���ٶ���� = �������ٶ� - �������ٶ�
    //�Լ��ٶȽ���PI����
    acc_err = exp_acc - fb_acc;
    acc_err_i += A_KI * acc_err * dT_s;
    acc_err_i = (acc_err_i > 600)?600:((acc_err_i<0)?0:acc_err_i);
    
    //�������Ϊ Kp * ���ٶ�����ֵ + ���ٶȻ���ֵ��PI���ƣ�
    acc_out = A_KP * exp_acc;
    wz_out = acc_out + acc_err_i;
    
    //����޷�
    wz_out = (wz_out > 1000)?1000:((wz_out < 0)?0:wz_out);
    HeightInfo.Thr = (uint16_t)wz_out;
    
    //unlock state
    //����ɻ�δ�������򽫼��ٶȵĻ��������㣬�߶�����ֵ�뷴��ֵ��ͬ
    if(g_FMUflg.unlock == 0)
    {
        int i;
        acc_err_i = 0;
        exp_hei = fb_hei;
        fc_state_take_off = 0;
         for(i = 0;i<4;i++)
        {
          exp_vel_transition[i] = 0;
        }
    }
    else
    {
        if(g_UAVinfo.UAV_Mode >= Altitude_Hold)
        {
            //������λ��״̬�л�Ϊ���
            if(exp_vel_transition[0]>0)
            {
                fc_state_take_off = 1;
            }
        }
        else//g_UAVinfo.UAV_Mode < Altitude_Hold
        {
            fc_state_take_off = 1;
        }
    }
}


/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
