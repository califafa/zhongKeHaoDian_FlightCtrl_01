#ifndef _ZKHD_LINK_H
#define _ZKHD_LINK_H

#include "stdint.h"

#pragma pack (1)
typedef struct
{
    uint8_t Start;
    uint16_t Length;
    
    uint8_t Send_ID;
    uint8_t Receive_ID;
    uint8_t Message_ID;
}ZKHD_Link_Head_t;
#pragma pack ()        

typedef enum{
    Device_GCS,
    Device_FMU,
    Device_Remote,
    
    Num_Device,
}emDevice_t;

typedef enum
{
    MsgID_0_Remote_Info,
    MsgID_1_Remote_Config,
    
    Num_Serarch,
}MSG_GCS_To_Remote_t;

typedef enum
{
    MsgID_0_Channel_Info = 0,
    MsgID_1_Config_Info,
    
    Num_Answer,
}MSG_Remote_To_GCS_t;

typedef enum
{
    MsgID_0_Attitude_Info = 0,
    MsgID_1_Motor_Info,
    MsgID_2_Sensor_Info,
    MsgID_3_Line_Info,
    MsgID_4_Alt_Info,
    MsgID_5_Hardware_Info,
    MsgID_6_PID_Info,
    MsgID_7_Freq,
    
    Num_MSG_FMU_To_GCS,
}MSG_FMU_To_GCS_t;

typedef enum
{
    temp = 0,
    
}MSG_GCS_To_FMU_t;

void ZKHD_Link_Handle(uint8_t  *Rx_Buff);
void ZKHD_Link_MakeFrame(uint8_t Send_ID,
                          uint8_t Receive_ID,
                          uint8_t Msg_ID,
                          uint8_t *ptr,
                          uint16_t Length,
                          uint8_t *Tx_Buff,uint8_t *);


#endif
