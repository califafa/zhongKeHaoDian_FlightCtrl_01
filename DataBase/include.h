/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：include.h
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
#ifndef _INCLUDE_
#define _INCLUDE_
//外部文件引用
//第一批被引用的头文件
#include "driverlib.h"
#include "queue.h"
#include "delay.h"
#include "board.h"

//宏定义区
#define SUCCESS 0
#define FAILED  1

//READONLY
#include "stdint.h"
#include "stdbool.h"

void Hardware_Init(void);
void PollingKernel(void);
#endif
