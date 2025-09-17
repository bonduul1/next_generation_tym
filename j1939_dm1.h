/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __J1939_DM1_H
#define __J1939_DM1_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

  
#define J1939_DM1_CAN_ID        0x18FECA20
#define J1939_DM1_CM_CAN_ID     0x18ECFF20
#define J1939_DM1_DT_CAN_ID     0x18EBFF20

  
void dm_init_function();
void dm_transmit_function();
 
/*
  BAM Message structure --> CAN ID = 0x18FECA10, Source address is 0x10
    Message --> Byte 0
      Bit 7-6: Malfunction indicator lamp status
      Bit 5-4: Red stop lamp status
      Bit 3-2: Amber warning lamp status
      Bit 1-0: Protect lamp status
    Message --> Byte 1 = Reserved
    Message --> Byte 2 = SPN
    Message --> Byte 3 = SPN
    Message --> Byte 4
      Bit 7-5: SPN
      Bit 4-0: FMI
    Message --> Byte 5
      Bit 7  : SPN CM (Conversion Method)
      Bit 6-0: Occurance count
    Message --> Byte 6 = 0xFF (Reserved)
    Message --> Byte 7 = 0xFF (Reserved)


  TP.CM_BAM message structure --> CAN ID = 0x18ECFF10, Source address is 0x10
    Message --> Byte 0 = 0x20, (Control byte = Broadcast Announce Message)
    Message --> Byte 1 = BAM Size LOW
    Message --> Byte 2 = BAM Size HIGH
    Message --> Byte 3 = BAM packets
    Message --> Byte 4 = 0xFF, (Reserved byte)
    Message --> Byte 5 = BAM ID LOW 
    Message --> Byte 6 = BAM ID MID
    Message --> Byte 7 = BAM ID HIGH
      BAM DM1 PGN is 0x00FECA

  TP.DT_BAM message structure --> CAN ID = 0x18EBFF10, Source address is 0x10
    Message --> Byte 0 = 0 (Sequence number)
    Message --> Byte 1:N = Packetized data    
      Message --> Byte 1
        Bit 7-6: Malfunction indicator lamp status
        Bit 5-4: Red stop lamp status
        Bit 3-2: Amber warning lamp status
        Bit 1-0: Protect lamp status
      Message --> Byte 2 = Reserved
      
      Message --> Byte 3 = SPN-1
      Message --> Byte 4 = SPN-1
      Message --> Byte 5
        Bit 7-5: SPN-1
        Bit 4-0: FMI-1
      Message --> Byte 6
        Bit 7  : SPN CM-1 (Conversion Method)
        Bit 6-0: Occurance count-1
      
      Message --> Byte 7 = SPN-2

      Message --> Byte 0 = 1 (Sequence number)
      Message --> Byte 1 = SPN-2
      Message --> Byte 2
        Bit 7-5: SPN-2
        Bit 4-0: FMI-2
      Message --> Byte 3
        Bit 7  : SPN CM-2 (Conversion Method)
        Bit 6-0: Occurance count-2

      So on .................
*/
#ifdef __cplusplus
}
#endif

#endif /* __J1939_DM1_H */
