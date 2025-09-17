#ifndef UDS_SETTINGS_DID_H
#define UDS_SETTINGS_DID_H 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "settings.h"
#include "main.h"

#define NV_START_ADDRESS_OF_ECUMDDID            NV_START_ADDRESS_OF_UDS_STANDARD_DID + 200      // uint8_t nvECUManufacturingDate[8];
#define NV_START_ADDRESS_OF_ECUSNDID            NV_START_ADDRESS_OF_UDS_STANDARD_DID + 220      // uint8_t nvECUSerialNumber[15];
#define NV_START_ADDRESS_OF_VINDID              NV_START_ADDRESS_OF_UDS_STANDARD_DID + 240      // uint8_t nvVIN[17];
#define NV_START_ADDRESS_OF_PDDID               NV_START_ADDRESS_OF_UDS_STANDARD_DID + 380      // uint8_t nvProgrammingDate[8];
#define NV_START_ADDRESS_OF_CDDID               NV_START_ADDRESS_OF_UDS_STANDARD_DID + 400      // uint8_t nvCalibrationDate[8];
#define NV_START_ADDRESS_OF_IDOSSS_BV           NV_START_ADDRESS_OF_UDS_STANDARD_DID + 420      // uint8_t nvIdentificationOptionSystemSupplierSpecific[4];
#define NV_START_ADDRESS_OF_APP_CHECK           NV_START_ADDRESS_OF_UDS_STANDARD_DID + 440      // 
#define NV_START_ADDRESS_OF_AS                  NV_START_ADDRESS_OF_UDS_STANDARD_DID + 460      // uint8_t nvActiveSession;

#define NV_DATA_LENGTH_OF_ECUMDDID              8                               // uint8_t nvECUManufacturingDate[8];
#define NV_DATA_LENGTH_OF_ECUSNDID              15                              // uint8_t nvECUSerialNumber[15];
#define NV_DATA_LENGTH_OF_VINDID                17                              // uint8_t nvVIN[17];
#define NV_DATA_LENGTH_OF_PDDID                 8                               // uint8_t nvProgrammingDate[8];
#define NV_DATA_LENGTH_OF_CDDID                 8                               // uint8_t nvCalibrationDate[8];
#define NV_DATA_LENGTH_OF_IDOSSS_BV             4                               // uint8_t nvIdentificationOptionSystemSupplierSpecific[4];
#define NV_DATA_LENGTH_OF_APP_CHECK             4                               // uint8_t nvIsApplicationExist[4];
#define NV_DATA_LENGTH_OF_AS_BV                 1                               // uint8_t nvActiveSession;

extern uint8_t nvECUManufacturingDate[8];                                       // Actually 8 bytes are used
extern uint8_t nvECUSerialNumber[16];                                           // Actually 15 bytes are used
extern uint8_t nvVIN[20];                                                       // Actually 17 bytes are used
extern uint8_t nvProgrammingDate[8];                                            // Actually 8 bytes are used
extern uint8_t nvCalibrationDate[8];                                            // Actually 8 bytes are used
extern uint8_t nvIdentificationOptionSystemSupplierSpecific[4];                 // Actually 4 bytes are used
extern uint8_t nvIsApplicationExist[4];                                         // Actually 4 bytes are used
extern uint8_t nvActiveSession;                                                 // Actually 1 bytes are used

uint8_t update_ECUMDDID(uint8_t *data);
uint8_t update_ECUSNDID(uint8_t *data);
uint8_t update_VINDID(uint8_t *data);
uint8_t update_CDDID(uint8_t *data);
uint8_t update_PDDID(uint8_t *data);
uint8_t update_IDOSSS_BV();
uint8_t update_APP_CHECK(uint32_t data);
uint8_t update_AS(uint8_t data);

uint8_t read_dids();

#endif /* UDS_SETTINGS_DID_H */


