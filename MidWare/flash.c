/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：flash.c
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
Flash操作函数



*/
//外部文件引用
#include "include.h"
#include "flash.h"
#include "string.h"


//宏定义区
#define LENGTH 50
#define INFOA_START   (0x1980)

//Extern引用



//私有函数区



//私有变量区




/******************************************************************************
  * 函数名称：Flash_Write
  * 函数描述：往Flash中写数据
  * 输    入：
  * uint8_t offset:写入数据偏移
  * uint8_t *ptr:要写入数据的地址
  * uint8_t length:要写入数据的长度
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void Flash_Write(uint8_t offset, uint8_t *ptr, uint8_t length)
{
    uint8_t temp[LENGTH];
    uint8_t status = 1;
    
    memcpy(temp, (uint8_t*)INFOA_START, LENGTH);
    memcpy(temp + offset, ptr, length);
    
    FlashCtl_unlockInfoA();
    do
    {
        FlashCtl_eraseSegment((uint8_t *)INFOA_START);
        status = FlashCtl_performEraseCheck((uint8_t *)INFOA_START,
            128);
    } while (status == STATUS_FAIL);

    FlashCtl_write8(temp,  (uint8_t *)INFOA_START, LENGTH);
    FlashCtl_lockInfoA();
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
