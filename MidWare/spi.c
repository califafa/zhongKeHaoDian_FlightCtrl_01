/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：spi.c
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
调用SPI_init函数初始化SPI总线
调用SPI_RW函数从SPI总线中读取数据

*/

#include "spi.h"

#define SPI_MISO_PIN    GPIO_PIN2
#define SPI_MOSI_PIN    GPIO_PIN1
#define SPI_SLK_PIN     GPIO_PIN3

#define SPI_MISO        (P4IN & SPI_MISO_PIN)
#define SPI_MOSI_H      P4OUT |= SPI_MOSI_PIN
#define SPI_MOSI_L      P4OUT &= ~SPI_MOSI_PIN
#define SPI_SLK_H       P4OUT |= SPI_SLK_PIN
#define SPI_SLK_L       P4OUT &= ~SPI_SLK_PIN

/******************************************************************************
  * 函数名称：SPI_init
  * 函数描述：初始化SPI总线
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *
  *
******************************************************************************/
void SPI_init()
{
    GPIO_setAsOutputPin(GPIO_PORT_P4, SPI_MOSI_PIN);
    GPIO_setAsInputPin(GPIO_PORT_P4, SPI_MISO_PIN);
    GPIO_setAsOutputPin(GPIO_PORT_P4, SPI_SLK_PIN);
}

/******************************************************************************
  * 函数名称：SPI_RW
  * 函数描述：SPI总线读写
  * 输    入：uint8_t txdata:需要发送的数据
  * 输    出：void
  * 返    回：uint8_t :返回读取到的数据
  * 备    注：本函数一般不单独使用，需要配合片选信号线使用
  *
  *
******************************************************************************/
uint8_t SPI_RW(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if(data & 0x80)
        {
            SPI_MOSI_H;
        }else
        {
            SPI_MOSI_L;
        }
        
        data <<= 1;                           // 低一位移位到最高位
        
        SPI_SLK_H;
        if (SPI_MISO) 
        {
           data |= 0x01;
        }
        SPI_SLK_L;  
    }
    
    return (data); 
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
