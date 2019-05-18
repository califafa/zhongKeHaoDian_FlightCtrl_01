/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：UART.h
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
#ifndef _UART_H
#define _UART_H
//外部文件引用
#include "include.h"

typedef struct
{
    uint16_t tx_idle;     // 是否空闲(0: 指定的数据包还没有发送完毕)
    uint16_t tx_totle;    // 发送总量
    uint16_t tx_cnt;      // 已发送量
    uint16_t rx_cnt;
    
    uint8_t tx_buf[200];  // 发送缓存    
    uint8_t rx_buf[200];  // 接收缓存
}Usart_t;

extern Usart_t Usart1;
extern Usart_t Usart2;

void USART_Init(uint8_t ClockSource, uint32_t Baudrate);
void U2_UCA1_Send(uint8_t* pTx, uint8_t len);

void PollingUSART();

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

