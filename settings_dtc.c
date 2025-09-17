#include "settings_dtc.h"
#include "fram.h"
#include "defines.h"
#include "watchdog.h"
#include "sensors.h"
#include "main.h"
#include "uds_main.h"

const _dtc_number_typedef dtcNumber[TOTAL_NUMBER_OF_DTCS] = {
  // DID,       SPN,            DID,     Snapshot Record Number,
  // SYSTEM DTC
  { 0x900100,   0x7F008,        9101,   1       },             // PUDS_SVC_PARAM_DTC_BATTERY_VOLTAGE_ABOVE_NORMAL
  { 0x900101,   0x7F008,        9102,   1       },             // PUDS_SVC_PARAM_DTC_BATTERY_VOLTAGE_BELOW_NORMAL
  
  { 0x900200,   0x7F009,        9105,   1       },             // PUDS_SVC_PARAM_DTC_SENSOR_VOLTAGE_ABOVE_NORMAL
  { 0x900201,   0x7F009,        9106,   1       },             // PUDS_SVC_PARAM_DTC_SENSOR_VOLTAGE_BELOW_NORMAL
  // SYSTEM DTC
  
  // SENSOR & INPUT DTC
  { 0x910103,   0x7F328,        9203,   2       },             // PUDS_SVC_PARAM_DTC_SHUTTLE_SENSOR_SHORTED_HIGH
  { 0x910104,   0x7F328,        9204,   2       },             // PUDS_SVC_PARAM_DTC_SHUTTLE_SENSOR_SHORTED_LOW

  { 0x910203,   0x7F329,        9207,   2       },             // PUDS_SVC_PARAM_DTC_CLUTCH_SENSOR_SHORTED_HIGH
  { 0x910204,   0x7F329,        9208,   2       },             // PUDS_SVC_PARAM_DTC_CLUTCH_SENSOR_SHORTED_LOW
  
  { 0x910B03,   0x7F332,        9246,   2       },             // PUDS_SVC_PARAM_DTC_HAND_THROTTLE_SHORTED_HIGH
  { 0x910B04,   0x7F332,        9247,   2       },             // PUDS_SVC_PARAM_DTC_HAND_THROTTLE_SHORTED_LOW

  { 0x910C03,   0x7F333,        9250,   2       },             // PUDS_SVC_PARAM_DTC_FOOT_THROTTLE_SHORTED_HIGH
  { 0x910C04,   0x7F333,        9251,   2       },             // PUDS_SVC_PARAM_DTC_FOOT_THROTTLE_SHORTED_LOW
  
  { 0x910D03,   0x7F334,        9254,   2       },             // PUDS_SVC_PARAM_DTC_HITCH_LEVER_SHORTED_HIGH
  { 0x910D04,   0x7F334,        9255,   2       },             // PUDS_SVC_PARAM_DTC_HITCH_LEVER_SHORTED_LOW
  
  { 0x910E03,   0x7F335,        9258,   2       },             // PUDS_SVC_PARAM_DTC_HITCH_POSITION_SENSOR_SHORTED_HIGH
  { 0x910E04,   0x7F335,        9259,   2       },             // PUDS_SVC_PARAM_DTC_HITCH_POSITION_SENSOR_SHORTED_LOW
  
  { 0x910F03,   0x7F336,        9262,   2       },             // PUDS_SVC_PARAM_DTC_HITCH_DRAFT_SENSOR_SHORTED_HIGH
  { 0x910F04,   0x7F336,        9263,   2       },             // PUDS_SVC_PARAM_DTC_HITCH_DRAFT_SENSOR_SHORTED_LOW
  // SENSOR & INPUT DTC
  
  // OUTPUT DTC
  { 0x920103,   0x7F710,        9301,   3       },             // PUDS_SVC_PARAM_DTC_FORWARD_OUTPUT_SHORTED_HIGH
  { 0x920104,   0x7F710,        9302,   3       },             // PUDS_SVC_PARAM_DTC_FORWARD_OUTPUT_SHORTED_LOW
  
  { 0x920203,   0x7F711,        9303,   3       },             // PUDS_SVC_PARAM_DTC_REVERSE_OUTPUT_SHORTED_HIGH
  { 0x920204,   0x7F711,        9304,   3       },             // PUDS_SVC_PARAM_DTC_REVERSE_OUTPUT_SHORTED_LOW
  
  { 0x920703,   0x7F716,        9313,   3       },             // PUDS_SVC_PARAM_DTC_HITCH_UP_OUTPUT_SHORTED_HIGH
  { 0x920704,   0x7F716,        9314,   3       },             // PUDS_SVC_PARAM_DTC_HITCH_UP_OUTPUT_SHORTED_LOW
  
  { 0x920803,   0x7F717,        9315,   3       },             // PUDS_SVC_PARAM_DTC_HITCH_DOWN_OUTPUT_SHORTED_HIGH
  { 0x920804,   0x7F717,        9316,   3       },             // PUDS_SVC_PARAM_DTC_HITCH_DOWN_OUTPUT_SHORTED_LOW

  { 0x921103,   0x7F720,        9333,   3       },             // PUDS_SVC_PARAM_DTC_PTO_OUTPUT_SHORTED_HIGH
  { 0x921104,   0x7F720,        9334,   3       }              // PUDS_SVC_PARAM_DTC_PTO_OUTPUT_SHORTED_LOW
  // OUTPUT DTC
};

_dtc_typedef dtc[TOTAL_NUMBER_OF_DTCS];
_dtc_typedef firstFailedDTC;
_dtc_typedef firstConfirmedDTC;
_dtc_typedef lastFailedDTC;
_dtc_typedef lastConfirmedDTC;

/**
 * Write a corresponding DTC counter into FRAM
 *
 *
 * @param DTC counter index
 * @return TRUE/FALSE
 */
uint8_t write_dtc_counter(uint8_t index, uint32_t pData)
{
  uint8_t tempWriteData[4];
  
  if(index >= TOTAL_NUMBER_OF_DTCS)
    return FALSE;
  
  tempWriteData[0] = (uint8_t)(pData);
  tempWriteData[1] = (uint8_t)(pData >> 8);
  tempWriteData[2] = (uint8_t)(pData >> 16);
  tempWriteData[3] = (uint8_t)(pData >> 24);
  fram_write(NV_START_ADDRESS_OF_DTC_STATE_COUNTER + (index * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  return TRUE;
}

uint8_t read_dtc_counter(uint8_t index, uint32_t *pData)
{
  uint8_t tempReadData[4];
  
  if(index >= TOTAL_NUMBER_OF_DTCS)
    return FALSE;
  
  fram_read(NV_START_ADDRESS_OF_DTC_STATE_COUNTER + (index * DIAGNOSTIC_DATA_SIZE), tempReadData);
  *pData = tempReadData[0] + (tempReadData[1] << 8) + (tempReadData[2] << 16) + (tempReadData[3] << 24);
  
  return TRUE;
}

uint8_t write_dtc_time(uint8_t index, uint32_t pData)
{
  uint8_t tempWriteData[4];
  
  if(index >= TOTAL_NUMBER_OF_DTCS)
    return FALSE;
  
  tempWriteData[0] = (uint8_t)(pData);
  tempWriteData[1] = (uint8_t)(pData >> 8);
  tempWriteData[2] = (uint8_t)(pData >> 16);
  tempWriteData[3] = (uint8_t)(pData >> 24);
  fram_write(NV_START_ADDRESS_OF_DTC_TIME + (index * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  return TRUE;
}

uint8_t read_dtc_time(uint8_t index, uint32_t *pData)
{
  uint8_t tempReadData[4];
  
  if(index >= TOTAL_NUMBER_OF_DTCS)
    return FALSE;
  
  fram_read(NV_START_ADDRESS_OF_DTC_TIME + (index * DIAGNOSTIC_DATA_SIZE), tempReadData);
  *pData = tempReadData[0] + (tempReadData[1] << 8) + (tempReadData[2] << 16) + (tempReadData[3] << 24);
  
  return TRUE;
}

uint8_t previousResult[TOTAL_NUMBER_OF_DTCS];
uint8_t timerResult[TOTAL_NUMBER_OF_DTCS];

void update_dtc_test_result()                                                   // This is called every 10 msseconds
{
  static uint16_t timer = 0;
  uint8_t i;
  
  timer += 10;
  if(timer < 1000) {
    return;
  }
  timer = 0;
  
  for(i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    dtc[i].result = DTC_PASSED;
  }

  // --------------------------------------- System ---------------------------------------
  if(flagErrorSensorsHigh.battery == TRUE)        dtc[DTC_BATTERY_VOLTAGE_ABOVE_NORMAL].result = DTC_FAILED;
  else if(flagErrorSensorsLow.battery == TRUE)    dtc[DTC_BATTERY_VOLTAGE_BELOW_NORMAL].result = DTC_FAILED;
  
  // --------------------------------------- System ---------------------------------------
  
  // --------------------------------------- Sensor ---------------------------------------
  if(flagErrorSensorsHigh.shuttle == TRUE)        dtc[DTC_SHUTTLE_SENSOR_SHORTED_HIGH].result = DTC_FAILED;
  else if(flagErrorSensorsLow.shuttle == TRUE)    dtc[DTC_SHUTTLE_SENSOR_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensorsHigh.clutch == TRUE)         dtc[DTC_CLUTCH_SENSOR_SHORTED_HIGH].result = DTC_FAILED;
  else if(flagErrorSensorsLow.clutch == TRUE)     dtc[DTC_CLUTCH_SENSOR_SHORTED_LOW].result = DTC_FAILED;
  
  
  if(flagErrorSensorsHigh.handPedal == TRUE)      dtc[DTC_HAND_THROTTLE_SHORTED_HIGH].result = DTC_FAILED;
  else if(flagErrorSensorsLow.handPedal == TRUE)  dtc[DTC_HAND_THROTTLE_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensorsHigh.footPedal == TRUE)      dtc[DTC_FOOT_THROTTLE_SHORTED_HIGH].result = DTC_FAILED;
  else if(flagErrorSensorsLow.footPedal == TRUE)  dtc[DTC_FOOT_THROTTLE_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensorsHigh.threePLever == TRUE)            dtc[DTC_HITCH_LEVER_SHORTED_HIGH].result = DTC_FAILED;
  else if(flagErrorSensorsLow.threePLever == TRUE)        dtc[DTC_HITCH_LEVER_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensorsHigh.threePPosition == TRUE)         dtc[DTC_HITCH_POSITION_SENSOR_SHORTED_HIGH].result = DTC_FAILED;
  else if(flagErrorSensorsLow.threePPosition == TRUE)     dtc[DTC_HITCH_POSITION_SENSOR_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensorsHigh.threePDraft == TRUE)            dtc[DTC_HITCH_DRAFT_SENSOR_SHORTED_HIGH].result = DTC_FAILED;
  else if(flagErrorSensorsLow.threePDraft == TRUE)        dtc[DTC_HITCH_DRAFT_SENSOR_SHORTED_LOW].result = DTC_FAILED;
  // --------------------------------------- Sensor ---------------------------------------
  
  // --------------------------------------- Valve ---------------------------------------
  if(flagErrorSensors.forwardValve == TRUE)       dtc[DTC_FORWARD_OUTPUT_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensors.backwardValve == TRUE)      dtc[DTC_REVERSE_OUTPUT_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensors.threePUp == TRUE)           dtc[DTC_HITCH_UP_OUTPUT_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensors.threePDown == TRUE)         dtc[DTC_HITCH_DOWN_OUTPUT_SHORTED_LOW].result = DTC_FAILED;
  
  if(flagErrorSensors.pto == TRUE)                dtc[DTC_PTO_OUTPUT_SHORTED_LOW].result = DTC_FAILED;
  // --------------------------------------- Valve ---------------------------------------
  
  for(i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    if((previousResult[i] == DTC_PASSED) && (dtc[i].result == DTC_FAILED)) {
      dtc[i].totalErrorCounter++;
      dtc[i].lastErrorTime = tractorOperationTime;
      write_dtc_counter(i, dtc[i].totalErrorCounter);
      write_dtc_time(i, dtc[i].lastErrorTime);
    }

    previousResult[i] = dtc[i].result;
  }
}

// This DTC state diagram running every 2ms for one DTC, Totally 28 DTCs are using, which means totally 56ms is using to check whole DTCs
void dtc_state_diagram_run(uint8_t index)
{
  if(index >= TOTAL_NUMBER_OF_DTCS)
    return;
  
  switch(dtc[index].operationCycle)
  {
    case DTC_OP_IDLE:

      dtc[index].spn = dtcNumber[index].spn;
      dtc[index].fmi = (uint8_t)(dtcNumber[index].dtcNumber & 0x1F);
      dtc[index].snapshotRecordNumber = dtcNumber[index].snapshotRecordNumber;
      dtc[index].snapshotRecordDID = dtcNumber[index].snapshotRecordDID;
      
      dtc[index].dtcNumber = (uint8_t)(dtc[index].spn);
      dtc[index].dtcNumber <<= 8;
      dtc[index].dtcNumber |= (uint8_t)(dtc[index].spn >> 8);;
      dtc[index].dtcNumber <<= 8;
      dtc[index].dtcNumber |= (uint8_t)(((dtc[index].spn >> 11) & 0xE0) | (dtc[index].fmi));
      
// Disabled Enkhbat      printf("%d = SPN: %d, FMI: %d, DTC: %d\r\n", index, dtc[index].spn, dtc[index].fmi, dtc[index].dtcNumber);
      
      read_dtc_counter(index, &(dtc[index].totalErrorCounter));
      read_dtc_time(index, &(dtc[index].lastErrorTime));
    
      dtc[index].result = DTC_PASSED;
      
      dtc[index].testResult = DTC_NO_RESULT;
      dtc[index].statusOfDTC = 0;
      
      dtc[index].tripCounter = 0;
      dtc[index].tripThreshold = 127;                                      // This value should be checked again.
      
      dtc[index].agingCounter = 0;
      dtc[index].agingThreshold = 127;
      
      dtc[index].operationCycle = DTC_OP_START;
      dtc[index].operationTime = 0;
            
      dtc[index].statusOfDTC |= TEST_NOT_COMPLETED_SINCE_LAST_CLEAR;
      dtc[index].statusOfDTC |= TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE;
      
      // Cleared status bit
      firstFailedDTC.testResult = DTC_NO_RESULT;
      firstFailedDTC.statusOfDTC = 0;
      
      firstConfirmedDTC.testResult = DTC_NO_RESULT;
      firstConfirmedDTC.statusOfDTC = 0;
      
      lastFailedDTC.testResult = DTC_NO_RESULT;
      lastFailedDTC.statusOfDTC = 0;
      
      lastFailedDTC.testResult = DTC_NO_RESULT;
      lastFailedDTC.statusOfDTC = 0;
      
      break;
      
    case DTC_OP_START:
      dtc[index].statusOfDTC &= ~TEST_FAILED_THIS_OPERATION_CYCLE;
      dtc[index].statusOfDTC |= TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE;
      
      dtc[index].testResult = DTC_NO_RESULT;
      
      dtc[index].operationTime = 6000;
      dtc[index].operationCycle = DTC_OP_RUN;
      break;
      
    case DTC_OP_RUN:
      dtc[index].testResult = dtc[index].result;
      
      dtc[index].statusOfDTC &= ~TEST_NOT_COMPLETED_SINCE_LAST_CLEAR;
      dtc[index].statusOfDTC &= ~TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE;
      
      if(dtc[index].testResult == DTC_FAILED)
      {

        if((dtc[index].statusOfDTC & TEST_FAILED_THIS_OPERATION_CYCLE) == 0)
        {
          dtc[index].tripCounter++;
          if(dtc[index].tripCounter == dtc[index].tripThreshold)
          {
            dtc[index].statusOfDTC |= CONFIRMED_DTC;
            dtc[index].tripCounter = 0;

            if((firstConfirmedDTC.statusOfDTC & CONFIRMED_DTC) == 0)                      // Copy information
            {
              firstConfirmedDTC.testResult = DTC_FAILED;
              firstConfirmedDTC.statusOfDTC = dtc[index].statusOfDTC;
              firstConfirmedDTC.dtcNumber = dtc[index].dtcNumber;
              firstConfirmedDTC.tripCounter = dtc[index].tripCounter;
            }
            lastConfirmedDTC.testResult = DTC_FAILED;
            lastConfirmedDTC.statusOfDTC = dtc[index].statusOfDTC;
            lastConfirmedDTC.dtcNumber = dtc[index].dtcNumber;
            lastConfirmedDTC.tripCounter = dtc[index].tripCounter;
          }
        }

        dtc[index].statusOfDTC |= TEST_FAILED;
        dtc[index].statusOfDTC |= TEST_FAILED_THIS_OPERATION_CYCLE;
        dtc[index].statusOfDTC |= TEST_FAILED_SINCE_LAST_CLEAR;
        dtc[index].statusOfDTC |= PENDING_DTC;
        
        if((firstFailedDTC.statusOfDTC & TEST_FAILED) == 0)                      // Copy information
        {
          firstFailedDTC.testResult = DTC_FAILED;
          firstFailedDTC.statusOfDTC = dtc[index].statusOfDTC;
          firstFailedDTC.dtcNumber = dtc[index].dtcNumber;
          firstFailedDTC.tripCounter = dtc[index].tripCounter;
        }
        lastFailedDTC.testResult = DTC_FAILED;
        lastFailedDTC.statusOfDTC = dtc[index].statusOfDTC;
        lastFailedDTC.dtcNumber = dtc[index].dtcNumber;
        lastFailedDTC.tripCounter = dtc[index].tripCounter;
      }
      else {
        dtc[index].statusOfDTC &= ~TEST_FAILED;                                 // Clear bit
        
        if(((dtc[index].statusOfDTC & TEST_FAILED_THIS_OPERATION_CYCLE) == 0) &&
           ((dtc[index].statusOfDTC & TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE) == 0))
        {
          dtc[index].statusOfDTC &= ~PENDING_DTC;
        }
      }

      dtc[index].operationTime -= TOTAL_NUMBER_OF_DTCS;                         // reduced by 56, each DTC is checked in 1ms
      if(dtc[index].operationTime <= 0) {                                       // 60,000 ms = 60 seconds = 1 minute
        dtc[index].operationCycle = DTC_OP_STOP;
      }
      break;
    case DTC_OP_STOP:

      if((dtc[index].statusOfDTC & TEST_FAILED_THIS_OPERATION_CYCLE) == 0) {    // In this operation cycle there is NO error is detected
        dtc[index].agingCounter++;
        if(dtc[index].agingCounter == dtc[index].agingThreshold) {
          dtc[index].statusOfDTC &= ~CONFIRMED_DTC;
        }
      }

//      printf("dtc[%d].agingCounter = %d, dtc[%d].tripCounter = %d \r\n", index, dtc[index].agingCounter, index, dtc[index].tripCounter);
      
      dtc[index].operationCycle = DTC_OP_START;
      
      break;
    default:
      dtc[index].operationCycle = DTC_OP_IDLE;
      break;
  }
}

void dtc_run()
{
  static uint8_t index = 0;
  
  dtc_state_diagram_run(index);
  
  index++;
  if(index >= TOTAL_NUMBER_OF_DTCS) {
    index = 0;
  }
}

void dtc_init()
{
  uint8_t i = 0;
  
// Disabled Enkhbat  printf("TOTAL_NUMBER_OF_DTCS = %d\r\n", TOTAL_NUMBER_OF_DTCS);
  
  for(i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    previousResult[i] = DTC_PASSED;
	timerResult[i] = 0;
    dtc[i].operationCycle = DTC_OP_IDLE;
    dtc_state_diagram_run(i);
    watchdog_trigger();
  }
}

uint16_t check_number_of_errors()
{
  uint16_t i;
  uint16_t errorNumber = 0;
    
  for(i = 0; i < TOTAL_NUMBER_OF_DTCS; i++)
  {
    if(dtc[i].result == DTC_FAILED) {
      errorNumber++;
    }
  }
  return errorNumber;
}
                     
/*-------------------------------------------------- UDS Communication Start ----------------------------------------------------------------------------*/
uint8_t func_clearDiagnosticInformation(uint8_t *pData)
{
  uint32_t dtcNumber = (pData[0] << 16) | (pData[1] << 8) + (pData[2]);
  uint8_t i;
  
  for (i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    if(dtc[i].dtcNumber == dtcNumber)
    {
      dtc[i].totalErrorCounter = 0;
      dtc[i].lastErrorTime = 0;
      write_dtc_counter(i, dtc[i].totalErrorCounter);
      write_dtc_time(i, dtc[i].lastErrorTime);
      dtc[i].operationCycle = DTC_OP_IDLE;
      dtc_state_diagram_run(i);
// Disabled Enkhbat      printf("dtc[%d] is cleared.\r\n", i);
      return TRUE;
    }
  }
  
// Disabled Enkhbat  printf("DTC Number is NOT matched.\r\n");
  
  return FALSE;
}

uint16_t func_reportNumberOfDTCByStatusMask(uint8_t DTCStatusMask)
{
  uint16_t i;
  uint16_t numDTC = 0;
  for (i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    // Check if the status byte matches the given status mask
    if ((dtc[i].statusOfDTC & DTCStatusMask) == DTCStatusMask) {
      numDTC++;
    }
  }
  return numDTC;
}

uint16_t func_reportDTCByStatusMask(uint8_t subFunction, uint8_t DTCStatusMask, uint8_t* pData)
{
  const uint8_t packetLen = 4;
  uint16_t i;
  uint16_t numDTC = 0;
  uint8_t statusMask;
  uint8_t packetCounter = 0;
  
  if((subFunction == reportDTCByStatusMask) || 
     (subFunction == reportMirrorMemoryDTCByStatusMask)) {
    statusMask = DTCStatusMask;
  }
  else if(subFunction == reportSupportedDTC) {
    statusMask = 0;
  }
  else if(subFunction == reportFirstTestFailedDTC) {
    if((firstFailedDTC.statusOfDTC & TEST_FAILED) == TEST_FAILED) {
      pData[0] = (uint8_t)(firstFailedDTC.dtcNumber >> 16);
      pData[1] = (uint8_t)(firstFailedDTC.dtcNumber >> 8);
      pData[2] = (uint8_t)(firstFailedDTC.dtcNumber);
      pData[3] = (uint8_t)(firstFailedDTC.statusOfDTC);
      numDTC += packetLen;
    }
    return numDTC;
  }
  else if(subFunction == reportFirstConfirmedDTC) {
    if((firstConfirmedDTC.statusOfDTC & CONFIRMED_DTC) == CONFIRMED_DTC) {
      pData[0] = (uint8_t)(firstConfirmedDTC.dtcNumber >> 16);
      pData[1] = (uint8_t)(firstConfirmedDTC.dtcNumber >> 8);
      pData[2] = (uint8_t)(firstConfirmedDTC.dtcNumber);
      pData[3] = (uint8_t)(firstConfirmedDTC.statusOfDTC);
      numDTC += packetLen;
    }
    return numDTC;
  }
  else if(subFunction == reportMostRecentTestFailedDTC) {
    if((lastFailedDTC.statusOfDTC & TEST_FAILED) == TEST_FAILED) {
      pData[0] = (uint8_t)(lastFailedDTC.dtcNumber >> 16);
      pData[1] = (uint8_t)(lastFailedDTC.dtcNumber >> 8);
      pData[2] = (uint8_t)(lastFailedDTC.dtcNumber);
      pData[3] = (uint8_t)(lastFailedDTC.statusOfDTC);
      numDTC += packetLen;
    }
    return numDTC;
  }
  else if(subFunction == reportMostRecentConfirmedDTC) {
    if((lastConfirmedDTC.statusOfDTC & CONFIRMED_DTC) == CONFIRMED_DTC) {
      pData[0] = (uint8_t)(lastConfirmedDTC.dtcNumber >> 16);
      pData[1] = (uint8_t)(lastConfirmedDTC.dtcNumber >> 8);
      pData[2] = (uint8_t)(lastConfirmedDTC.dtcNumber);
      pData[3] = (uint8_t)(lastConfirmedDTC.statusOfDTC);
      numDTC += packetLen;
    }
    return numDTC;
  }

  // Loop through all the stored DTCs
  for (i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    if ((dtc[i].statusOfDTC & statusMask) == statusMask) {
      // Add the DTC to the list
      pData[(packetCounter * packetLen) + 0] = (uint8_t)(dtc[i].dtcNumber >> 16);
      pData[(packetCounter * packetLen) + 1] = (uint8_t)(dtc[i].dtcNumber >> 8);
      pData[(packetCounter * packetLen) + 2] = (uint8_t)(dtc[i].dtcNumber);
      pData[(packetCounter * packetLen) + 3] = (uint8_t)(dtc[i].statusOfDTC);
      numDTC += packetLen;
      packetCounter++;
    }
  }
  return numDTC;
}

uint16_t func_reportDTCSnapshotIdentification(uint8_t* pData)
{
  const uint8_t packetLen = 4;
  uint16_t i;
  uint16_t numDTC = 0;
  
  // Loop through all the stored DTCs
  for (i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    // Add the DTC to the list
    pData[(i * packetLen) + 0] = (uint8_t)(dtc[i].dtcNumber >> 16);
    pData[(i * packetLen) + 1] = (uint8_t)(dtc[i].dtcNumber >> 8);
    pData[(i * packetLen) + 2] = (uint8_t)(dtc[i].dtcNumber);
    pData[(i * packetLen) + 3] = (uint8_t)(dtc[i].snapshotRecordNumber);
    numDTC += packetLen;
  }
  return numDTC;
}

uint16_t func_reportDTCSnapshotRecordByDTCNumber(uint8_t* pData)
{
  const uint8_t packetLen = 13;
  uint16_t numDTC = 0;
  uint16_t i;
  uint32_t dtcMaskRecord = (uint32_t)((pData[0] << 16) | (pData[1] << 8) | pData[2]);
  uint8_t snapshotRecordNumber = pData[3];
  uint8_t packetCounter = 0;
  
  // Loop through all the stored DTCs
  for (i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    // Add the DTC to the list
    if((dtc[i].dtcNumber == dtcMaskRecord) && (dtc[i].snapshotRecordNumber == snapshotRecordNumber)) {
      pData[(packetCounter * packetLen) + 0] = (uint8_t)(dtc[i].dtcNumber >> 16);
      pData[(packetCounter * packetLen) + 1] = (uint8_t)(dtc[i].dtcNumber >> 8);
      pData[(packetCounter * packetLen) + 2] = (uint8_t)(dtc[i].dtcNumber);
      pData[(packetCounter * packetLen) + 3] = (uint8_t)(dtc[i].statusOfDTC);
      pData[(packetCounter * packetLen) + 4] = (uint8_t)(dtc[i].snapshotRecordNumber);
      pData[(packetCounter * packetLen) + 5] = 1;                                           // DTC Snapshot record number of Identifiers
      pData[(packetCounter * packetLen) + 6] = (uint8_t)(dtc[i].snapshotRecordDID >> 8);    // DID MSB
      pData[(packetCounter * packetLen) + 7] = (uint8_t)(dtc[i].snapshotRecordDID);         // DID LSB
      pData[(packetCounter * packetLen) + 8] = (uint8_t)(dtc[i].totalErrorCounter);         // Total error counter is using 1 byte
      pData[(packetCounter * packetLen) + 9] = (uint8_t)(dtc[i].lastErrorTime >> 24);       // Time of last error, ms
      pData[(packetCounter * packetLen) + 10] = (uint8_t)(dtc[i].lastErrorTime >> 16);      // Time of last error, ms
      pData[(packetCounter * packetLen) + 11] = (uint8_t)(dtc[i].lastErrorTime >> 8);       // Time of last error, ms
      pData[(packetCounter * packetLen) + 12] = (uint8_t)(dtc[i].lastErrorTime);            // Time of last error, ms
      numDTC += packetLen;
      packetCounter++;
    }
  }
  
  return numDTC;
}

uint16_t func_reportDTCStoredDataByRecordNumber(uint8_t* pData)
{
  const uint8_t packetLen = 13;
  uint16_t numDTC = 0;
  uint16_t i;
  uint8_t snapshotRecordNumber = pData[0];
  uint8_t packetCounter = 0;
  
  // Loop through all the stored DTCs
  for (i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    // Add the DTC to the list
    if(dtc[i].snapshotRecordNumber == snapshotRecordNumber) {
      pData[(packetCounter * packetLen) + 0] = (uint8_t)(dtc[i].snapshotRecordNumber);
      pData[(packetCounter * packetLen) + 1] = (uint8_t)(dtc[i].dtcNumber >> 16);
      pData[(packetCounter * packetLen) + 2] = (uint8_t)(dtc[i].dtcNumber >> 8);
      pData[(packetCounter * packetLen) + 3] = (uint8_t)(dtc[i].dtcNumber);
      pData[(packetCounter * packetLen) + 4] = (uint8_t)(dtc[i].statusOfDTC);
      pData[(packetCounter * packetLen) + 5] = 1;                                           // DTC Snapshot record number of Identifiers
      pData[(packetCounter * packetLen) + 6] = (uint8_t)(dtc[i].snapshotRecordDID >> 8);    // DID MSB
      pData[(packetCounter * packetLen) + 7] = (uint8_t)(dtc[i].snapshotRecordDID);         // DID LSB
      pData[(packetCounter * packetLen) + 8] = (uint8_t)(dtc[i].totalErrorCounter);         // Total error counter is using 1 byte
      pData[(packetCounter * packetLen) + 9] = (uint8_t)(dtc[i].lastErrorTime >> 24);       // Time of last error, ms
      pData[(packetCounter * packetLen) + 10] = (uint8_t)(dtc[i].lastErrorTime >> 16);      // Time of last error, ms
      pData[(packetCounter * packetLen) + 11] = (uint8_t)(dtc[i].lastErrorTime >> 8);       // Time of last error, ms
      pData[(packetCounter * packetLen) + 12] = (uint8_t)(dtc[i].lastErrorTime);            // Time of last error, ms
      numDTC += packetLen;
      packetCounter++;
    }
  }
  
  return numDTC;
}

uint16_t func_reportDTCFaultDetectionCounter(uint8_t* pData)
{
  const uint8_t packetLen = 4;
  uint16_t i;
  uint16_t numDTC = 0;
  
  for (i = 0; i < TOTAL_NUMBER_OF_DTCS; i++) {
    // Add the DTC to the list
    pData[(i * packetLen) + 0] = (uint8_t)(dtc[i].dtcNumber >> 16);
    pData[(i * packetLen) + 1] = (uint8_t)(dtc[i].dtcNumber >> 8);
    pData[(i * packetLen) + 2] = (uint8_t)(dtc[i].dtcNumber);
    if(dtc[i].testResult == DTC_FAILED)
      pData[(i * packetLen) + 3] = 127;
    else if(dtc[i].testResult == DTC_PASSED)
      pData[(i * packetLen) + 3] = (uint8_t)(-128);
    else
      pData[(i * packetLen) + 3] = 0;    
    numDTC += packetLen;
  }
  return numDTC;
}

/*-------------------------------------------------- UDS Communication End ----------------------------------------------------------------------------*/