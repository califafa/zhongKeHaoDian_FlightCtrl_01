#include "ZKHD_Link.h"
#include "gcs.h"

void ZKHD_Link_GCS_To_Remote_Handle(ZKHD_Link_Head_t *,uint8_t *ptr,uint8_t length);
void ZKHD_Link_GCS_To_FMU_Handle(ZKHD_Link_Head_t *,uint8_t *ptr,uint8_t length);

void ZKHD_Link_MakeFrame(uint8_t Send_ID,
                        uint8_t Receive_ID,
                        uint8_t Msg_ID,
                        uint8_t *ptr,
                        uint16_t Length,
                        uint8_t *Tx_Buff,
                        uint8_t *Tx_Buff_Length)
{
    uint8_t Buff[100];
    int sum = 0;
    int i;
    ZKHD_Link_Head_t ZKHD_Link_Head;
    
    ZKHD_Link_Head.Send_ID = Send_ID;
    ZKHD_Link_Head.Receive_ID = Receive_ID;
    ZKHD_Link_Head.Start = 0xAA;
    ZKHD_Link_Head.Length = Length;
    ZKHD_Link_Head.Message_ID = Msg_ID;
    
    memcpy(Buff,&ZKHD_Link_Head.Start,sizeof(ZKHD_Link_Head_t));
    memcpy(Buff+sizeof(ZKHD_Link_Head_t),ptr,Length);
    
    for(i=0;i<Length + 6;i++)
    {
        sum += Buff[i];
    }
    
    Buff[Length + sizeof(ZKHD_Link_Head_t)] = sum & 0xFF;
    
    memcpy(Tx_Buff,Buff,Length + sizeof(ZKHD_Link_Head_t) + 1);
    *Tx_Buff_Length = Length + sizeof(ZKHD_Link_Head_t) + 1;
}

void ZKHD_Link_Handle(uint8_t *Rx_Buff)
{
    ZKHD_Link_Head_t *ZKHD_Link_Head_Ptr;
    
    ZKHD_Link_Head_Ptr = (ZKHD_Link_Head_t*)Rx_Buff;
    
    if(ZKHD_Link_Head_Ptr->Send_ID == Device_GCS)
    {
        if(ZKHD_Link_Head_Ptr->Receive_ID == Device_Remote)
        {
            ZKHD_Link_GCS_To_Remote_Handle(ZKHD_Link_Head_Ptr,&ZKHD_Link_Head_Ptr->Message_ID+1,ZKHD_Link_Head_Ptr->Length);
        }else if(ZKHD_Link_Head_Ptr->Receive_ID == Device_FMU)
        {
            ZKHD_Link_GCS_To_FMU_Handle(ZKHD_Link_Head_Ptr,&ZKHD_Link_Head_Ptr->Message_ID+1,ZKHD_Link_Head_Ptr->Length);
        }
    }        
}

void ZKHD_Link_GCS_To_Remote_Handle(ZKHD_Link_Head_t *ZKHD_Link_Head,uint8_t *ptr,uint8_t length)
{
    
}

void ZKHD_Link_GCS_To_FMU_Handle(ZKHD_Link_Head_t *ZKHD_Link_Head,uint8_t *ptr,uint8_t length)
{
    gcs_ReceiveHandle(ZKHD_Link_Head,ptr,length);
} 

