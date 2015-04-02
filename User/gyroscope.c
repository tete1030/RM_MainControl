#include "gyroscope.h"
#include "main.h"
void Gyroscope_Init(void)
{
      CanTxMsg Tx_message; 
      Tx_message.StdId = 0x404;
      Tx_message.IDE = CAN_Id_Standard;
      Tx_message.RTR = CAN_RTR_Data;
      Tx_message.DLC = 0x08;
    
      Tx_message.Data[0] = 0x00;
      Tx_message.Data[1] = 0x01;
      Tx_message.Data[2] = 0x02;
      Tx_message.Data[3] = 0x03;
      Tx_message.Data[4] = 0x04;
      Tx_message.Data[5] = 0x05;
      Tx_message.Data[6] = 0x06;
      Tx_message.Data[7] = 0x07;
    
    CAN_Transmit(CAN2,&Tx_message);
}
