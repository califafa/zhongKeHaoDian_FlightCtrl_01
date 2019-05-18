/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：UART.c
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
处理串口的相关事情

*/
//外部文件引用
#include "UART.h"
#include "include.h"
#include "communication.h"

#define USART_RX_TIMEOUT_MAX    20
bool lbUsartRx = false;
int TimeoutUSART = 0;

Usart_t Usart2 = 
{
    .tx_idle = 1,
    .tx_totle = 0,
    .tx_cnt = 0,
};

/******************************************************************************
  * 函数名称：USART_Init
  * 函数描述：串口初始化
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *
  *
******************************************************************************/
void USART_Init(uint8_t ClockSource, uint32_t Baudrate)
{
    USCI_A_UART_initParam USCI_A1_UART_initParam;
    
    float frequency = 0.0;
    float N_f = 0.0;
    
    if(USCI_A_UART_CLOCKSOURCE_ACLK == ClockSource)
    {
        frequency = UCS_getACLK();
    }
    else if(USCI_A_UART_CLOCKSOURCE_SMCLK == ClockSource)
    {
        frequency = UCS_getSMCLK();
    }
    else
    {
      return;
    }
    
    // Tx: P4.4     Rx: P4.5
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN5);
    
    N_f = frequency / Baudrate;
    
    USCI_A1_UART_initParam.selectClockSource = ClockSource;
    USCI_A1_UART_initParam.clockPrescalar = (uint16_t)(N_f / 16);    
    USCI_A1_UART_initParam.firstModReg = (uint16_t)(((N_f / 16) - (uint16_t)(N_f / 16)) * 16);
    USCI_A1_UART_initParam.secondModReg = 0;
    USCI_A1_UART_initParam.parity = USCI_A_UART_NO_PARITY;
    USCI_A1_UART_initParam.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    USCI_A1_UART_initParam.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    USCI_A1_UART_initParam.uartMode = USCI_A_UART_MODE;
    USCI_A1_UART_initParam.overSampling = USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
    
    USCI_A_UART_init(USCI_A1_BASE, &USCI_A1_UART_initParam);
    
    USCI_A_UART_enable(USCI_A1_BASE);
    
    /*
        http://bbs.eeworld.com.cn/forum.php?mod=viewthread&tid=430117&highlight=msp430%2B%B4%AE%BF%DA%D6%D0%B6%CF
        串口中断使能要放在串口初始化结束之后 否则不能进入中断
    */
    USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_TRANSMIT_INTERRUPT);
    USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
}

/******************************************************************************
  * 函数名称：PollingUSART
  * 函数描述：轮询串口接收信息
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：整个系统与上位机通信，此处为入口
  *
  *
******************************************************************************/
void PollingUSART()
{    
    if(lbUsartRx)
    {
        TimeoutUSART++;
        
        if(TimeoutUSART > USART_RX_TIMEOUT_MAX)
        {
            //数据入口
            SwitchPort(Connect_Port_USB, Usart2.rx_buf);
            
            TimeoutUSART = 0;
            Usart2.rx_cnt = 0;
            lbUsartRx = false;
        }
    }
}

/******************************************************************************
  * 函数名称：U2_UCA1_Send
  * 函数描述：以中断的方式发送数据
  * 输    入：
  * uint8_t *ptx:要发送的数据地址
  * uint8_t len:要发送的数据长度
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *
  *
******************************************************************************/
void U2_UCA1_Send(uint8_t* pTx, uint8_t len)
{
    uint8_t i;
    
    // 等待串口空闲
    while(!Usart2.tx_idle)
        ;
    
    // 占用串口
    Usart2.tx_idle = 0;
    
    // 缓存要发送的数据
    for(i=0; i<len; i++)
    {
        Usart2.tx_buf[i] = pTx[i];
    }
    
    // 启动发送
    Usart2.tx_totle = len;
    Usart2.tx_cnt = 0;
    
    if(Usart2.tx_cnt < Usart2.tx_totle)
    {
        UCA1TXBUF = Usart2.tx_buf[Usart2.tx_cnt++];
    }
}

/******************************************************************************
  * 函数名称：USCI_A1_ISR
  * 函数描述：USCI_AI中断服务函数
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：中断函数自动调用
  *
  *
******************************************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
    #pragma vector = USCI_A1_VECTOR
    __interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
    void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
    #error Compiler not supported!
#endif
{
    // USART Tx Interrupt
    if(USCI_A_UART_getInterruptStatus(USCI_A1_BASE, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG))
    {
        if(Usart2.tx_cnt < Usart2.tx_totle)
        {
            UCA1TXBUF = Usart2.tx_buf[Usart2.tx_cnt++];
        }
        else
        {
            Usart2.tx_idle = 1;
        }
        
        USCI_A_UART_clearInterrupt(USCI_A1_BASE, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
    }
    
    // USART Rx Interrupt
    if(USCI_A_UART_getInterruptStatus(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT_FLAG))
    {
        lbUsartRx = true;
        
        TimeoutUSART = 0;
        Usart2.rx_buf[Usart2.rx_cnt++] = USCI_A_UART_receiveData(USCI_A1_BASE);
        USCI_A_UART_clearInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    }
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
