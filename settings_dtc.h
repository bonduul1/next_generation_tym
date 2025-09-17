#ifndef UDS_SETTINGS_DTC_H
#define UDS_SETTINGS_DTC_H 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "settings.h"

enum uds_dtc_subfunction
{
  reportNumberOfDTCByStatusMask = 0x01,                                         // Supported
  reportDTCByStatusMask = 0x02,                                                 // Supported
  reportDTCSnapshotIdentification = 0x03,                                       // Supported
  reportDTCSnapshotRecordByDTCNumber = 0x04,                                    // Supported
  reportDTCStoredDataByRecordNumber = 0x05,                                     // Supported
  reportDTCExtDataRecordByDTCNumber = 0x06,                                     // NOT Supported
  reportNumberOfDTCBySeverityMaskRecord = 0x07,                                 // NOT Supported
  reportDTCBySeverityMaskRecord = 0x08,                                         // NOT Supported
  reportSeverityInformationOfDTC = 0x09,                                        // NOT Supported
  reportSupportedDTC = 0x0A,                                                    // Supported
  reportFirstTestFailedDTC = 0x0B,                                              // Supported
  reportFirstConfirmedDTC = 0x0C,                                               // Supported
  reportMostRecentTestFailedDTC = 0x0D,                                         // Supported
  reportMostRecentConfirmedDTC = 0x0E,                                          // Supported
  reportMirrorMemoryDTCByStatusMask = 0x0F,                                     // Supported
  reportMirrorMemoryDTCExtDataRecordByDTCNumber = 0x010,                        // NOT Supported
  reportNumberOfMirrorMemoryDTCByStatusMask = 0x11,                             // Supported
  reportNumberOfEmissionsOBDDTCByStatusMask = 0x12,                             // NOT Supported
  reportEmissionsOBDDTCByStatusMask = 0x13,                                     // NOT Supported
  reportDTCFaultDetectionCounter = 0x14,                                        // Supported
  reportDTCWithPermanentStatus = 0x15,                                          // NOT Supported
  reportDTCExtDataRecordByRecordNumber = 0x16,                                  // NOT Supported
  reportUserDefMemoryDTCByStatusMask = 0x17,                                    // NOT Supported
  reportUserDefMemoryDTCSnapshotRecordByDTCNumber = 0x18,                       // NOT Supported
  reportUserDefMemoryDTCExtDataRecordByDTCNumber = 0x19,                        // NOT Supported
  // 0x1A - 0x41 = ISOSAEReserved
  reportWWHOBDDTCByMaskRecord = 0x42,                                           // NOT Supported
  // 0x43 - 0x54 = ISOSAEReserved
  reportWWHOBDDTCWithPermanentStatus = 0x55,                                    // NOT Supported
  // 0x56 - 0x7F = ISOSAEReserved
};

enum uds_dtc_format_identifier
{
  SAE_J2012_DA_DTCFormat_00 = 0x00,
  ISO_14229_1_DTCFormat = 0x01,
  SAE_J1939_73_DTCFormat = 0x02,
  ISO_11992_4_DTCFormat = 0x03,
  SAE_J2012_DA_DTCFormat_04 = 0x04,
  // 0x05 - 0xFF = ISOSAEReserved
};

enum uds_dtc_status_mask
{
  // The DTCStatusMask contains eight (8) DTC status bits. 
  TEST_FAILED = 0x01,                                           // DTC is no longer failed at the time of the request
  TEST_FAILED_THIS_OPERATION_CYCLE = 0x02,                      // DTC never failed on the current operation cycle
  PENDING_DTC = 0x04,                                           // DTC failed on the current or previous operation cycle
  CONFIRMED_DTC = 0x08,                                         // DTC is not confirmed at the time of the request
  TEST_NOT_COMPLETED_SINCE_LAST_CLEAR = 0x10,                   // DTC test were completed since the last code clear
  TEST_FAILED_SINCE_LAST_CLEAR = 0x20,                          // DTC test failed at least once since last code clear
  TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE = 0x40,               // DTC test completed this operation cycle
  WARNING_INDICATOR_REQUESTED = 0x80                            // Server is not requesting warningIndicator to be active
}; 

// This should be checked again
#define DTS_STATUS_AVAILABILITY_MASK    (TEST_FAILED | \
                                         TEST_FAILED_THIS_OPERATION_CYCLE | \
                                         PENDING_DTC | \
                                         CONFIRMED_DTC | \
                                         TEST_NOT_COMPLETED_SINCE_LAST_CLEAR | \
                                         TEST_FAILED_SINCE_LAST_CLEAR | \
                                         TEST_NOT_COMPLETED_THIS_OPERATION_CYCLE )
enum uds_dtc_snapshot
{
  SNAPSHOT_COUNTER = 1,
  SNAPSHOT_TIME = 2,
};

enum uds_dtc_operation_cycle
{
  DTC_OP_IDLE = 0,
  DTC_OP_START,
  DTC_OP_RUN,
  DTC_OP_STOP
};

enum uds_dtc_test_result
{
  DTC_FAILED = 0,
  DTC_NO_RESULT = 1,
  DTC_PASSED = 2
};

enum total_dtcs
{
  // SYSTEM DTC
  DTC_BATTERY_VOLTAGE_ABOVE_NORMAL = 0,
  DTC_BATTERY_VOLTAGE_BELOW_NORMAL,
  
  DTC_SENSOR_VOLTAGE_ABOVE_NORMAL,
  DTC_SENSOR_VOLTAGE_BELOW_NORMAL,
  // SYSTEM DTC
  
  // SENSOR & INPUT DTC
  DTC_SHUTTLE_SENSOR_SHORTED_HIGH,
  DTC_SHUTTLE_SENSOR_SHORTED_LOW,
  
  DTC_CLUTCH_SENSOR_SHORTED_HIGH,
  DTC_CLUTCH_SENSOR_SHORTED_LOW,
    
  DTC_HAND_THROTTLE_SHORTED_HIGH,
  DTC_HAND_THROTTLE_SHORTED_LOW,
  
  DTC_FOOT_THROTTLE_SHORTED_HIGH,
  DTC_FOOT_THROTTLE_SHORTED_LOW,
  
  DTC_HITCH_LEVER_SHORTED_HIGH,
  DTC_HITCH_LEVER_SHORTED_LOW,
  
  DTC_HITCH_POSITION_SENSOR_SHORTED_HIGH,
  DTC_HITCH_POSITION_SENSOR_SHORTED_LOW,
  
  DTC_HITCH_DRAFT_SENSOR_SHORTED_HIGH,
  DTC_HITCH_DRAFT_SENSOR_SHORTED_LOW,
  // SENSOR & INPUT DTC
  
  // OUTPUT DTC
  DTC_FORWARD_OUTPUT_SHORTED_HIGH,
  DTC_FORWARD_OUTPUT_SHORTED_LOW,
  
  DTC_REVERSE_OUTPUT_SHORTED_HIGH,
  DTC_REVERSE_OUTPUT_SHORTED_LOW,

  DTC_HITCH_UP_OUTPUT_SHORTED_HIGH,
  DTC_HITCH_UP_OUTPUT_SHORTED_LOW,
  
  DTC_HITCH_DOWN_OUTPUT_SHORTED_HIGH,
  DTC_HITCH_DOWN_OUTPUT_SHORTED_LOW,

  DTC_PTO_OUTPUT_SHORTED_HIGH,
  DTC_PTO_OUTPUT_SHORTED_LOW,
  // OUTPUT DTC
  TOTAL_NUMBER_OF_DTCS
};

typedef struct {
  const uint32_t dtcNumber;                             // ISO14229-1 DTC NUMBER
  const uint32_t spn;                                   // J1939 SPN
  const uint16_t snapshotRecordDID;                     // DID of Snapshot record number
  const uint8_t  snapshotRecordNumber;                  // Snapshot record number
}_dtc_number_typedef;

typedef struct {
  uint32_t dtcNumber;                                   // ISO14229-1 DTC NUMBER
  uint32_t spn;                                         // J1939 SPN
  uint32_t totalErrorCounter;                           // Total error counter: This should be saved into NV
  uint32_t lastErrorTime;                               // Last error time: This should be saved into NV
  int32_t  operationTime;                               // We should configure this time before running operation cycle
  uint8_t  operationCycle;                              // what kind of operation cycle is going on
  uint8_t  result;                                      // Real-Time testing result state
  uint8_t  testResult;                                  // Testing result state
  uint8_t  statusOfDTC;                                 // current operation cycle's status of DTC
  uint8_t  tripCounter;                                 // 
  uint8_t  tripThreshold;                               // This value should be used when trip should be reset
  uint8_t  agingCounter;                                // 
  uint8_t  agingThreshold;                              // This value should be used when trip should be reset
  uint8_t  snapshotRecordNumber;                        // Snapshot record number
  uint16_t snapshotRecordDID;                           // DID of Snapshot record
  uint8_t  fmi;                                         // J1939 FMI
} _dtc_typedef;

extern _dtc_typedef dtc[TOTAL_NUMBER_OF_DTCS];

uint8_t write_dtc_counter(uint8_t index, uint32_t pData);
uint8_t read_dtc_counter(uint8_t index, uint32_t *pData);
uint8_t write_dtc_time(uint8_t index, uint32_t pData);
uint8_t read_dtc_time(uint8_t index, uint32_t *pData);

void update_dtc_test_result();
uint16_t check_number_of_errors();
void dtc_run();
void dtc_init();

uint8_t func_clearDiagnosticInformation(uint8_t *pData);

uint16_t func_reportNumberOfDTCByStatusMask(uint8_t DTCStatusMask);
uint16_t func_reportDTCByStatusMask(uint8_t subFunction, uint8_t DTCStatusMask, uint8_t* pData);
uint16_t func_reportDTCSnapshotIdentification(uint8_t* pData);
uint16_t func_reportDTCSnapshotRecordByDTCNumber(uint8_t* pData);
uint16_t func_reportDTCStoredDataByRecordNumber(uint8_t* pData);
uint16_t func_reportDTCFaultDetectionCounter(uint8_t* pData);
#endif /* UDS_SETTINGS_DTC_H */