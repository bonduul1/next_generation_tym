#include "j1939_dm1.h"
#include "settings_dtc.h"
#include "can.h"
#include "main.h"

uint8_t dm1_data_buffer[TOTAL_NUMBER_OF_DTCS * 4];
uint8_t transmitData[80];                                                       // packetNumber * 8 = 80, maximum 10 packet can be transmitted into J1939
uint16_t bamSize;
uint8_t  sequenceNumber;
uint8_t  sequenceCounter;
uint8_t  transmitTP;
uint8_t  packetNumber;

void make_sequence(uint8_t size)
{
  uint8_t i;
  uint8_t index = 0;
  uint8_t dmIndex = 0;
  
  memset(transmitData, 0xFF, packetNumber * 8);
  
  // prepare packet
  for(i = 0; i < packetNumber; i++) {
    transmitData[index++] = i + 1;                      // Sequence number
    if(i == 0) {
      transmitData[index++] = 0x00;
      transmitData[index++] = 0xFF;
    }
    else {
      if(dmIndex == size)
        break;
      transmitData[index++] = dm1_data_buffer[dmIndex++];
      
      if(dmIndex == size)
        break;
      transmitData[index++] = dm1_data_buffer[dmIndex++];  
    }
    if(dmIndex == size)
        break;
    transmitData[index++] = dm1_data_buffer[dmIndex++];
    if(dmIndex == size)
        break;
    transmitData[index++] = dm1_data_buffer[dmIndex++];
    if(dmIndex == size)
        break;
    transmitData[index++] = dm1_data_buffer[dmIndex++];
    if(dmIndex == size)
        break;
    transmitData[index++] = dm1_data_buffer[dmIndex++];
    if(dmIndex == size)
        break;
    transmitData[index++] = dm1_data_buffer[dmIndex++];
  }
}

uint16_t create_packet_for_dm1()
{
  uint16_t i;
  uint16_t packetLen = 0;
  uint16_t dtcs = check_number_of_errors();
  
  if(dtcs == 0) {
    return 0;
  }

  for(i = 0; i < TOTAL_NUMBER_OF_DTCS; i++)
  {
    if(dtc[i].result == DTC_FAILED) {
      dm1_data_buffer[packetLen++] = (uint8_t)(dtc[i].spn);
      dm1_data_buffer[packetLen++] = (uint8_t)(dtc[i].spn >> 8);
      dm1_data_buffer[packetLen++] = (uint8_t)(((dtc[i].spn >> 11) & 0xE0) | (dtc[i].fmi));
      if(dtc[i].totalErrorCounter >= 0x7F)
        dm1_data_buffer[packetLen++] = 0x7F;
      else
        dm1_data_buffer[packetLen++] = (uint8_t)((dtc[i].totalErrorCounter) & 0x7F);
    }
  }
  return packetLen;
}

void transmit_dm()
{
  uint16_t size;
  uint8_t  data[8];
  
  size = create_packet_for_dm1();
  
  if(size == 0) {
    data[0] = 0x00;                             // No need to control lamp
    data[1] = 0xFF;                             // Reserved
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0xFF;
    data[7] = 0xFF;
    
    // send data
    can_transmit(CAN_CHANNEL_1, J1939_DM1_CAN_ID, data);
  }
  else if(size == 4) {
    data[0] = 0x00;                             // No need to control lamp
    data[1] = 0xFF;                             // Reserved
    data[2] = dm1_data_buffer[0];
    data[3] = dm1_data_buffer[1];
    data[4] = dm1_data_buffer[2];
    data[5] = dm1_data_buffer[3];
    data[6] = 0xFF;
    data[7] = 0xFF;
    
    // send data
    can_transmit(CAN_CHANNEL_1, J1939_DM1_CAN_ID, data);
  }
  else {
    // BAM should use
    bamSize = size + 2;
    packetNumber = bamSize / 7;
    
    if((bamSize % 7) != 0) {
      packetNumber++;
    }
    
    if(packetNumber > 10) {
      packetNumber = 10;
    }
    
    data[0] = 0x20;                             // BAM Message
    data[1] = (uint8_t)(bamSize);               // message len
    data[2] = (uint8_t)(bamSize >> 8);          // message len
    data[3] = packetNumber;                     // Number of packets
    data[4] = 0xFF;                             // Reserved
    data[5] = 0xCA;                             // DM1 PGN
    data[6] = 0xFE;                             // DM1 PGN
    data[7] = 0x00;                             // DM1 PGN
    
    sequenceNumber = 1;
    sequenceCounter = 0;
    transmitTP = TRUE;
    
    make_sequence(size);
    
    can_transmit(CAN_CHANNEL_1, J1939_DM1_CM_CAN_ID, data);
  }
}

uint8_t transmit_dm_dt()
{
  uint8_t i;
  uint8_t data[8];
  if(sequenceCounter >= packetNumber)
    return FALSE;
  
  for(i = 0; i < 8; i++) {
    data[i] = transmitData[i + sequenceCounter * 8];
  }
  sequenceCounter++;
  
  can_transmit(CAN_CHANNEL_1, J1939_DM1_DT_CAN_ID, data);
  return TRUE;
}

void dm_init_function()
{
  bamSize = 0;
  sequenceNumber = 0;
  sequenceCounter = 0;
  transmitTP = FALSE;
  packetNumber = 0;
}

void dm_transmit_function()
{
  static uint16_t timer = 10000;                // To start directly, changed the init value on 2024.01.30
  static uint16_t timerDT = 10000;              // To start directly, changed the init value on 2024.01.30
  
  if(flagTimer.tenMs == TRUE) {
    timer += 10;
    timerDT += 10;
  }
    
  if(transmitTP == FALSE) {
    if(timer >= 1000) {
      timer = 0;
      timerDT = 0;
      transmit_dm();
    }
  }
  else {
    if(timerDT >= 100) {
      timerDT = 0;
      if(transmit_dm_dt() == FALSE) {
        dm_init_function();
      }
    }
  }
}

