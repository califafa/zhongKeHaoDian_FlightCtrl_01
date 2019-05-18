/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：flash.h
  * 摘    要：
  *
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/

#ifndef _FLASH_H
#define _FLASH_H
//外部文件引用
#include "stdint.h"




//Extern引用



//函数声明
void Flash_Write(uint8_t offset, uint8_t *ptr, uint8_t length);

#endif
