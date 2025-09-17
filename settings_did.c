

#include "settings_did.h"
#include "fram.h"
#include "defines.h"
#include "watchdog.h"

//----------------------------------------------------------------------------------------------------------------
uint8_t nvECUManufacturingDate[8];                                              // Actually 8 bytes are used
uint8_t nvECUSerialNumber[16];                                                  // Actually 15 bytes are used
uint8_t nvVIN[20];                                                              // Actually 17 bytes are used
uint8_t nvProgrammingDate[8];                                                   // Actually 8 bytes are used
uint8_t nvCalibrationDate[8];                                                   // Actually 8 bytes are used
uint8_t nvIdentificationOptionSystemSupplierSpecific[4];                        // Actually 4 bytes are used
uint8_t nvIsApplicationExist[4];                                                // Actually 4 bytes are used
uint8_t nvActiveSession;                                                        // Actually 1 bytes are used

uint8_t update_ECUMDDID(uint8_t *data)
{
  uint8_t tempWriteData[4];
  uint8_t i = 0;

  for(i = 0; i < NV_DATA_LENGTH_OF_ECUMDDID; i++) {
    nvECUManufacturingDate[i] = data[i];
  }

  for(i = 0; i < 2; i++) {
    tempWriteData[0] = nvECUManufacturingDate[i * 4 + 0];
    tempWriteData[1] = nvECUManufacturingDate[i * 4 + 1];
    tempWriteData[2] = nvECUManufacturingDate[i * 4 + 2];
    tempWriteData[3] = nvECUManufacturingDate[i * 4 + 3];
    fram_write(NV_START_ADDRESS_OF_ECUMDDID + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  }
  return TRUE;
}

uint8_t update_ECUSNDID(uint8_t *data)
{
  uint8_t tempWriteData[4];
  uint8_t i = 0;

  for(i = 0; i < NV_DATA_LENGTH_OF_ECUSNDID; i++) {
    nvECUSerialNumber[i] = data[i];
  }

  for(i = 0; i < 4; i++) {
    tempWriteData[0] = nvECUSerialNumber[i * 4 + 0];
    tempWriteData[1] = nvECUSerialNumber[i * 4 + 1];
    tempWriteData[2] = nvECUSerialNumber[i * 4 + 2];
    tempWriteData[3] = nvECUSerialNumber[i * 4 + 3];
    fram_write(NV_START_ADDRESS_OF_ECUSNDID + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  }
  return TRUE;
}

uint8_t update_VINDID(uint8_t *data)
{
  uint8_t tempWriteData[4];
  uint8_t i = 0;

  for(i = 0; i < NV_DATA_LENGTH_OF_VINDID; i++) {
    nvVIN[i] = data[i];
  }

  for(i = 0; i < 5; i++) {
    tempWriteData[0] = nvVIN[i * 4 + 0];
    tempWriteData[1] = nvVIN[i * 4 + 1];
    tempWriteData[2] = nvVIN[i * 4 + 2];
    tempWriteData[3] = nvVIN[i * 4 + 3];
    fram_write(NV_START_ADDRESS_OF_VINDID + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  }
  return TRUE;
}

uint8_t update_CDDID(uint8_t *data)
{
  uint8_t tempWriteData[4];
  uint8_t i = 0;
    
  for(i = 0; i < NV_DATA_LENGTH_OF_CDDID; i++) {
    nvCalibrationDate[i] = data[i];
  }
  
  for(i = 0; i < 2; i++) {
    tempWriteData[0] = nvCalibrationDate[i * 4 + 0];
    tempWriteData[1] = nvCalibrationDate[i * 4 + 1];
    tempWriteData[2] = nvCalibrationDate[i * 4 + 2];
    tempWriteData[3] = nvCalibrationDate[i * 4 + 3];
    fram_write(NV_START_ADDRESS_OF_CDDID + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  }
  return TRUE;
}

uint8_t update_AS(uint8_t data)
{
  uint8_t tempWriteData[4];

  nvActiveSession = data;
  
  tempWriteData[0] = nvActiveSession;
  tempWriteData[1] = 0;
  tempWriteData[2] = 0;
  tempWriteData[3] = 0;
  fram_write(NV_START_ADDRESS_OF_AS, tempWriteData);
  return TRUE;
}

uint8_t read_dids()
{
  uint8_t i = 0;
  uint8_t tempReadData[4];

  for(i = 0; i < 2; i++) {
    fram_read(NV_START_ADDRESS_OF_ECUMDDID + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    nvECUManufacturingDate[i * 4 + 0] = tempReadData[0];
    nvECUManufacturingDate[i * 4 + 1] = tempReadData[1];
    nvECUManufacturingDate[i * 4 + 2] = tempReadData[2];
    nvECUManufacturingDate[i * 4 + 3] = tempReadData[3];
  }  
  
  for(i = 0; i < 4; i++) {
    fram_read(NV_START_ADDRESS_OF_ECUSNDID + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    nvECUSerialNumber[i * 4 + 0] = tempReadData[0];
    nvECUSerialNumber[i * 4 + 1] = tempReadData[1];
    nvECUSerialNumber[i * 4 + 2] = tempReadData[2];
    nvECUSerialNumber[i * 4 + 3] = tempReadData[3];
  }

  for(i = 0; i < 5; i++) {
    fram_read(NV_START_ADDRESS_OF_VINDID + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    nvVIN[i * 4 + 0] = tempReadData[0];
    nvVIN[i * 4 + 1] = tempReadData[1];
    nvVIN[i * 4 + 2] = tempReadData[2];
    nvVIN[i * 4 + 3] = tempReadData[3];
  }
  
  watchdog_trigger();

  for(i = 0; i < 2; i++) {
    fram_read(NV_START_ADDRESS_OF_PDDID + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    nvProgrammingDate[i * 4 + 0] = tempReadData[0];
    nvProgrammingDate[i * 4 + 1] = tempReadData[1];
    nvProgrammingDate[i * 4 + 2] = tempReadData[2];
    nvProgrammingDate[i * 4 + 3] = tempReadData[3];
  }
  
  for(i = 0; i < 2; i++) {
    fram_read(NV_START_ADDRESS_OF_CDDID + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    nvCalibrationDate[i * 4 + 0] = tempReadData[0];
    nvCalibrationDate[i * 4 + 1] = tempReadData[1];
    nvCalibrationDate[i * 4 + 2] = tempReadData[2];
    nvCalibrationDate[i * 4 + 3] = tempReadData[3];
  }
  
  for(i = 0; i < 1; i++) {
    fram_read(NV_START_ADDRESS_OF_IDOSSS_BV + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    nvIdentificationOptionSystemSupplierSpecific[i * 4 + 0] = tempReadData[0];
    nvIdentificationOptionSystemSupplierSpecific[i * 4 + 1] = tempReadData[1];
    nvIdentificationOptionSystemSupplierSpecific[i * 4 + 2] = tempReadData[2];
    nvIdentificationOptionSystemSupplierSpecific[i * 4 + 3] = tempReadData[3];
  }
  
  fram_read(NV_START_ADDRESS_OF_AS, tempReadData);
  nvActiveSession = tempReadData[0];
  
  watchdog_trigger();
  return TRUE;
}
