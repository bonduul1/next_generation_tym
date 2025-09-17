
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
  
/* Defines -------------------------------------------------------------------*/
#define CAN_CHANNEL_1                   1
#define CAN_CHANNEL_2                   2

/*--------------------- CAN1 IDs --------------------- */
#define CAN1_UDS_PHYSICAL_REQUEST       0x18DA20FA
#define CAN1_UDS_PHYSICAL_RESPONSE      0x18DAFA20
#define CAN1_UDS_FUNCTIONAL_REQUEST     0x18DBFFFA

#define CAN1_RX_ECU_J1939_EEC2          0x0CF00300                              // Engine ECU --> TCU
#define CAN1_RX_ECU_J1939_EEC1          0x0CF00400                              // Engine ECU --> TCU
#define CAN1_TX_ECU_J1939_TSC1          0x0C000003                              // TCU --> Engine ECU
 
#define CAN1_TX_SKC                     0x19FFD020                              // TCU --> SKC
#define CAN1_TX_IP1                     0x19FFA020                              // TCU --> IP
#define CAN1_TX_IP2                     0x19FFA120                              // TCU --> IP

#define CAN1_RX_ACU                     0x19FFB050                              // ACU --> TCU
#define CAN1_TX_ACU                     0x19FFB020                              // TCU --> ACU

#define CAN1_RX_TBOX1                   0x19FFE040                              // TBOX --> TCU
#define CAN1_RX_TBOX2                   0x19FFE140                              // TBOX --> TCU

#define CAN1_RX_CPG1                    0x19FFE055                              // CPG --> TCU
#define CAN1_RX_CPG2                    0x19FFE056                              // CPG --> TCU
#define CAN1_RX_CPG3                    0x19FFE057                              // CPG --> TCU
  
#define CAN1_TX_PA                      0x18FF0203                              // TCU --> ECU
#define CAN1_TX_EAT                     0x18FF4403                              // TCU --> ECU

/*--------------------- CAN2 IDs --------------------- */
#define CAN2_RX_ARMREST                 0x18FE2111

#define CAN2_RX_DASH                    0x18FE2112
#define CAN2_TX_DASH_SETTING            0x18D01221
#define CAN2_TX_DASH                    0x18A71221

#define CAN2_RX_PILLAR                  0x18FE2113
#define CAN2_TX_PILLAR_SETTING          0x18D01321
#define CAN2_TX_PILLAR                  0x18A71321

#define CAN2_RX_FENDER_A                0x18FE2114  
#define CAN2_TX_FENDER_A_SETTING        0x18D01421
#define CAN2_TX_FENDER_A                0x18A71421

#define CAN2_RX_FENDER_B                0x18FE2115
#define CAN2_TX_FENDER_B_SETTING        0x18D01521
#define CAN2_TX_FENDER_B                0x18A71521

#define CAN2_RX_DIAL                    0x18FE211A

#define CAN2_RX_ANGLE                   0x10FF5350
#define CAN2_RX_GYRO                    0x10FF5451
  
// CAN1 message types
typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canRxEEC2_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    
    uint8_t engineSpeedLSB                : 8;
    uint8_t engineSpeedMSB                : 8;

    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canRxEEC1_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canTxTSC1_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  temperature        : 8;
    
    uint8_t  ptoSwitch          : 1;
    uint8_t  parkingBrakeSwitch : 1;
    uint8_t  brakeSwitch        : 2;
    uint8_t  clutchSwitch       : 1;
    uint8_t  seatSwitch         : 1;
    uint8_t  res1               : 1;
    uint8_t  modelSwitch        : 1;
    
    uint8_t  fnr                : 2;
    uint8_t  readingLamp        : 1;
    uint8_t  res2               : 5;
    
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  flag       : 8;
  };
} canTxSKC_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  pto                : 1;
    uint8_t  fourWD             : 1;
    uint8_t  forward            : 1;
    uint8_t  turnUp             : 1;
    uint8_t  quickTurn          : 1;
    uint8_t  backUp             : 1;
    uint8_t  startCheck         : 1;
    uint8_t  res1               : 1;
    
    uint8_t  reverse            : 1;
    uint8_t  hitchUp            : 1;
    uint8_t  hitchDown          : 1;
    uint8_t  res2               : 1;
    uint8_t  draft              : 1;
    uint8_t  depth              : 1;
    uint8_t  res3               : 2;
    
    uint8_t  res4               : 1;
    uint8_t  quickTurnFlashing  : 1;
    uint8_t  speedCheck         : 1;
    uint8_t  rpmCruise          : 1;
    uint8_t  sensorCheck        : 1;
    uint8_t  hitchUpFlashing    : 1;
    uint8_t  hitchDownFlashing  : 1;
    uint8_t  res5               : 1;
    
    uint8_t  depthFlashing      : 1;
    uint8_t  res6               : 5;
    uint8_t  speedUnit          : 1;
    uint8_t  dtc                : 1;
    
    uint8_t  subTransmission    : 2;
    uint8_t  mainTransmission   : 3;
    uint8_t  res7               : 2;
    uint8_t  yususu             : 1;
    
    uint8_t  temperature        : 8;
    
    uint8_t  res9               : 8;
    uint8_t  res10              : 8;
    uint8_t  flag               : 8;
  };
} canTxIP1_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  fnr                : 2;
    uint8_t  fourWD             : 2;
    uint8_t  hitchMode          : 3;
    uint8_t  startCheck         : 1;
    
    uint8_t  tsc1Mode           : 2;
    uint8_t  res2               : 6;
    
    uint8_t  res3               : 8;
    uint8_t  res4               : 8;
    uint8_t  res5               : 8;
    uint8_t  res6               : 8;
    uint8_t  res7               : 8;
    uint8_t  res8               : 8;
    uint8_t  flag               : 8;
  };
} canTxIP2_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canRxACU_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canTxACU_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canRxTBOX1_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canRxTBOX2_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canRxCPG1_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canRxCPG2_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  res1       : 8;
    uint8_t  res2       : 8;
    uint8_t  res3       : 8;
    uint8_t  res4       : 8;
    uint8_t  res5       : 8;
    uint8_t  res6       : 8;
    uint8_t  res7       : 8;
    uint8_t  res8       : 8;
    uint8_t  flag       : 8;
  };
} canRxCPG3_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  engineTorqueMapSelector    : 4;
    uint8_t  res1                       : 4;
    
    uint8_t  droopSelector              : 2;
    uint8_t  res2                       : 6;
    
    uint8_t  res3                       : 8;
    uint8_t  res4                       : 8;
    uint8_t  res5                       : 8;
    uint8_t  res6                       : 8;
    uint8_t  res7                       : 8;
    uint8_t  res8                       : 8;
    uint8_t  flag                       : 8;
  };
} canTxPA_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  regenerationStop           : 2;
    uint8_t  regenerationRelease        : 2;
    uint8_t  res1                       : 4;
    
    uint8_t  startEOLRoutine            : 8;
    uint8_t  EOLRoutine                 : 8;
    
    uint8_t  res2                       : 2;
    uint8_t  inhibitHeatMode            : 2;
    uint8_t  stationarySwitch           : 2;
    uint8_t  EATOverrideSwitch          : 2;
    
    uint8_t  res3                       : 8;
    uint8_t  res4                       : 8;
    uint8_t  res5                       : 8;
    
    uint8_t  messageCounter             : 4;
    uint8_t  messageCheckSum            : 4;
    
    uint8_t  flag                       : 8;
  };
} canTxEAT_t;

// CAN2 message types
typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  acceleratorPosition        : 8;                    // 0x00 ~ 0xFA, Scale = 0.4, Offset = 0, 0xFE = Error    
    uint8_t  hitchLeverLSB              : 8;                    // 0x00 ~ 0x3E8, Scale = 0.1, Offset = 0, 0x3F8 = Error
    uint8_t  hitchLeverMSB              : 8;                    // 0x00 ~ 0x3E8, Scale = 0.1, Offset = 0, 0x3F8 = Error
    
    // Byte 3
    uint8_t  hitchUpButton              : 1;                    // one touch up
    uint8_t  hitchDownButton            : 1;                    // one touch down
    uint8_t  rpmDecreaseButton          : 1;                    // RPM decrease
    uint8_t  rpmIncreaseButton          : 1;                    // RPM increase
    uint8_t  rpmCruiseAButton           : 1;                    // RPM cruise A
    uint8_t  rpmCruiseBButton           : 1;                    // RPM cruise B
    uint8_t  reservedJButton            : 1;                    // J button for reserved
    uint8_t  reservedKButton            : 1;                    // K button for reserved
    
    uint8_t  res5                       : 8;
    uint8_t  res6                       : 8;
    uint8_t  programVersion             : 8;                    // program version
    uint8_t  checkSum                   : 8;                    // checksum + count
    uint8_t  flag                       : 8;
  };
} canRxArmRest_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  screenTransitionButton     : 2;
    uint8_t  regenButton                : 2;
    uint8_t  quickTurnButton            : 2;
    uint8_t  fourWDButton               : 2;
    
    uint8_t  res2                       : 8;                    // should be 0xFF
    uint8_t  res3                       : 8;                    // should be 0xFF
    uint8_t  res4                       : 8;                    // should be 0xFF
    uint8_t  res5                       : 8;                    // should be 0xFF
    uint8_t  res6                       : 8;                    // should be 0xFF
    uint8_t  res7                       : 8;                    // should be 0xFF
    uint8_t  res8                       : 8;                    // should be 0xFF
    uint8_t  flag                       : 8;
  };
} canRxDash_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  functionLightBrightness    : 8;                    // 0 ~ 250, Scale = 0.4, Unit = %
    uint8_t  backLightBrightness        : 8;                    // 0 ~ 250, Scale = 0.4, Unit = %
    uint8_t  ledBlinkingPeriod          : 8;                    // 5 ~ 250, Scale = 10, Unit = ms
    uint8_t  res4                       : 8;                    // should be 0xFF
    uint8_t  res5                       : 8;                    // should be 0xFF
    uint8_t  res6                       : 8;                    // should be 0xFF
    uint8_t  res7                       : 8;                    // should be 0xFF
    uint8_t  res8                       : 8;                    // should be 0xFF
    uint8_t  flag                       : 8;
  };
} canTxDashSetting_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  quickTurn                  : 2;
    uint8_t  led19                      : 2;
    uint8_t  fourWDManual               : 2;
    uint8_t  fourWDAuto                 : 2;
    
    // Byte 1
    uint8_t  led24                      : 2;
    uint8_t  regen                      : 2;
    uint8_t  led22                      : 2;
    uint8_t  led21                      : 2;
    
    // Byte 2
    uint8_t  led28                      : 2;
    uint8_t  led27                      : 2;
    uint8_t  screenTransition           : 2;
    uint8_t  led25                      : 2;
    
    uint8_t  res4                       : 8;
    uint8_t  res5                       : 8;
    uint8_t  res6                       : 8;
    uint8_t  res7                       : 8;
    uint8_t  res8                       : 8;
    uint8_t  flag                       : 8;
  };
} canTxDash_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  topLinkDownButton          : 2;
    uint8_t  topLinkUpButton            : 2;
    uint8_t  hitchDownButton            : 2;
    uint8_t  hitchUpButton              : 2;
    
    // Byte 1
    uint8_t  res2                       : 4;
    uint8_t  balanceDownButton          : 2;
    uint8_t  balanceUpButton            : 2;
    
    uint8_t  res3                       : 8;                    // should be 0xFF
    uint8_t  res4                       : 8;                    // should be 0xFF
    uint8_t  res5                       : 8;                    // should be 0xFF
    uint8_t  res6                       : 8;                    // should be 0xFF
    uint8_t  res7                       : 8;                    // should be 0xFF
    uint8_t  res8                       : 8;                    // should be 0xFF
    uint8_t  flag                       : 8;
  };
} canRxPillar_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  functionLightBrightness    : 8;                    // 0 ~ 250, Scale = 0.4, Unit = %
    uint8_t  backLightBrightness        : 8;                    // 0 ~ 250, Scale = 0.4, Unit = %
    uint8_t  ledBlinkingPeriod          : 8;                    // 5 ~ 250, Scale = 10, Unit = ms
    uint8_t  res4                       : 8;                    // should be 0xFF
    uint8_t  res5                       : 8;                    // should be 0xFF
    uint8_t  res6                       : 8;                    // should be 0xFF
    uint8_t  res7                       : 8;                    // should be 0xFF
    uint8_t  res8                       : 8;                    // should be 0xFF
    uint8_t  flag                       : 8;
  };
} canTxPillarSetting_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  hitchDown                  : 2;
    uint8_t  led19                      : 2;
    uint8_t  led18                      : 2;
    uint8_t  hitchUp                    : 2;
    
    // Byte 1
    uint8_t  led24                      : 2;
    uint8_t  topLinkUp                  : 2;
    uint8_t  led22                      : 2;
    uint8_t  led21                      : 2;
    
    // Byte 2
    uint8_t  led28                      : 2;
    uint8_t  led27                      : 2;
    uint8_t  topLinkDown                : 2;
    uint8_t  led25                      : 2;
    
    // Byte 3
    uint8_t  balanceDown                : 2;
    uint8_t  led31                      : 2;
    uint8_t  led30                      : 2;
    uint8_t  balanceUp                  : 2;

    uint8_t  res5                       : 8;
    uint8_t  res6                       : 8;
    uint8_t  res7                       : 8;
    uint8_t  res8                       : 8;
    uint8_t  flag                       : 8;
  };
} canTxPillar_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  stationaryButton           : 2;
    uint8_t  readingButton              : 2;
    uint8_t  downRpmButton              : 2;
    uint8_t  upHitchButton              : 2;
    
    // Byte 1
    uint8_t  res2                       : 4;
    uint8_t  autoStopButton             : 2;
    uint8_t  hitchModeButton            : 2;
    
    uint8_t  res3                       : 8;                    // should be 0xFF
    uint8_t  res4                       : 8;                    // should be 0xFF
    uint8_t  res5                       : 8;                    // should be 0xFF
    uint8_t  res6                       : 8;                    // should be 0xFF
    uint8_t  res7                       : 8;                    // should be 0xFF
    uint8_t  res8                       : 8;                    // should be 0xFF
    uint8_t  flag                       : 8;
  };
} canRxFenderA_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  functionLightBrightness    : 8;                    // 0 ~ 250, Scale = 0.4, Unit = %
    uint8_t  backLightBrightness        : 8;                    // 0 ~ 250, Scale = 0.4, Unit = %
    uint8_t  ledBlinkingPeriod          : 8;                    // 5 ~ 250, Scale = 10, Unit = ms
    uint8_t  res4                       : 8;                    // should be 0xFF
    uint8_t  res5                       : 8;                    // should be 0xFF
    uint8_t  res6                       : 8;                    // should be 0xFF
    uint8_t  res7                       : 8;                    // should be 0xFF
    uint8_t  res8                       : 8;                    // should be 0xFF
    uint8_t  flag                       : 8;
  };
} canTxFenderASetting_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  turnDownRpm                : 2;
    uint8_t  led19                      : 2;
    uint8_t  backUpHitch                : 2;
    uint8_t  turnUpHitch                : 2;
    
    // Byte 1
    uint8_t  led24                      : 2;
    uint8_t  reading                    : 2;
    uint8_t  led22                      : 2;
    uint8_t  backDownRpm                : 2;
    
    // Byte 2
    uint8_t  led28                      : 2;
    uint8_t  led27                      : 2;
    uint8_t  stationary                 : 2;
    uint8_t  led25                      : 2;
    
    // Byte 3
    uint8_t  autoStop                   : 2;
    uint8_t  hitchDepth                 : 2;
    uint8_t  hitchDraft                 : 2;
    uint8_t  hitchPosition              : 2;

    uint8_t  res5                       : 8;
    uint8_t  res6                       : 8;
    uint8_t  res7                       : 8;
    uint8_t  res8                       : 8;
    uint8_t  flag                       : 8;
  };
} canTxFenderA_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  balanceSensitiveButton     : 2;
    uint8_t  balanceModeButton          : 2;
    uint8_t  reservedButton             : 2;
    uint8_t  powerAssistButton          : 2;
    
    uint8_t  res2                       : 8;                    // should be 0xFF
    uint8_t  res3                       : 8;                    // should be 0xFF
    uint8_t  res4                       : 8;                    // should be 0xFF
    uint8_t  res5                       : 8;                    // should be 0xFF
    uint8_t  res6                       : 8;                    // should be 0xFF
    uint8_t  res7                       : 8;                    // should be 0xFF
    uint8_t  res8                       : 8;                    // should be 0xFF
    uint8_t  flag                       : 8;
  };
} canRxFenderB_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  functionLightBrightness    : 8;                    // 0 ~ 250, Scale = 0.4, Unit = %
    uint8_t  backLightBrightness        : 8;                    // 0 ~ 250, Scale = 0.4, Unit = %
    uint8_t  ledBlinkingPeriod          : 8;                    // 5 ~ 250, Scale = 10, Unit = ms
    uint8_t  res4                       : 8;                    // should be 0xFF
    uint8_t  res5                       : 8;                    // should be 0xFF
    uint8_t  res6                       : 8;                    // should be 0xFF
    uint8_t  res7                       : 8;                    // should be 0xFF
    uint8_t  res8                       : 8;                    // should be 0xFF
    uint8_t  flag                       : 8;
  };
} canTxFenderBSetting_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  led20                      : 2;
    uint8_t  led19                      : 2;
    uint8_t  led18                      : 2;
    uint8_t  powerAssist                : 2;
    
    // Byte 1
    uint8_t  balanceFlat                : 2;
    uint8_t  balanceManual              : 2;
    uint8_t  led22                      : 2;
    uint8_t  led21                      : 2;
    
    // Byte 2
    uint8_t  balanceSlow                : 2;
    uint8_t  balanceNormal              : 2;
    uint8_t  balanceFast                : 2;
    uint8_t  balanceSlope               : 2;
    
    uint8_t  res4                       : 8;
    uint8_t  res5                       : 8;
    uint8_t  res6                       : 8;
    uint8_t  res7                       : 8;
    uint8_t  res8                       : 8;
    uint8_t  flag                       : 8;
  };
} canTxFenderB_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  upLimit                    : 8;                    // 0x00 ~ 0xFA, Scale = 0.4, Offset = 0, 0xFE = Error
    uint8_t  subSetting                 : 8;                    // 0x00 ~ 0xFA, Scale = 0.4, Offset = 0, 0xFE = Error
    uint8_t  hitchDownSpeed             : 8;                    // 0x00 ~ 0xFA, Scale = 0.4, Offset = 0, 0xFE = Error
    uint8_t  balanceSetting             : 8;                    // 0x00 ~ 0xFA, Scale = 0.4, Offset = 0, 0xFE = Error
    uint8_t  shuttleDial                : 8;                    // 0x00 ~ 0xFA, Scale = 0.4, Offset = 0, 0xFE = Error
    uint8_t  ptoStopPosition            : 8;                    // 0x00 ~ 0xFA, Scale = 0.4, Offset = 0, 0xFE = Error
    uint8_t  programVersion             : 8;                    // Scale = 0.1, Offset = 0
    uint8_t  checkSum                   : 8;                    // Checksum + Count
    uint8_t  flag                       : 8;
  };
} canRxDial_t;

typedef union {
  uint8_t data[9];
  struct {
    uint8_t  rollAngleLSB               : 8;
    uint8_t  rollAngleMSB               : 8;
    
    uint8_t  pitchAngleLSB              : 8;
    uint8_t  pitchAngleMSB              : 8;
    
    uint8_t  rollAccLSB                 : 8;
    uint8_t  rollAccMSB                 : 8;
    
    uint8_t  pitchAccLSB                : 8;
    uint8_t  pitchAccMSB                : 8;
    
    uint8_t  flag                       : 8;
  };
} canRxAngle_t;

typedef union {
  uint8_t data[9];
  struct {
    // Byte 0
    uint8_t  gyroXAxisLSB               : 8;
    uint8_t  gyroXAxisMSB               : 8;
    
    uint8_t  gyroYAxisLSB               : 8;
    uint8_t  gyroYAxisMSB               : 8;

    uint8_t  gyroZAxisLSB               : 8;
    uint8_t  gyroZAxisMSB               : 8;
    
    uint8_t  res7                       : 8;
    uint8_t  counter                    : 8;
    uint8_t  flag                       : 8;
  };
} canRxGyro_t;

typedef union {
  uint8_t data[11];
  struct {
    // Byte 0
    uint8_t  fourWD                     : 2;
    uint8_t  quickTurn                  : 2;
    uint8_t  regen                      : 2;
    uint8_t  screenTransaction          : 2;
    
    uint8_t  hitchUpPilar               : 2;
    uint8_t  hitchDownPilar             : 2;
    uint8_t  balanceUpPillar            : 2;
    uint8_t  balanceDownPillar          : 2;
    
    uint8_t  toplinkUpPilar             : 2;
    uint8_t  toplinkDownPilar           : 2;
    uint8_t  turnUpHitch                : 2;
    uint8_t  backUpHitch                : 2;
    
    uint8_t  turnDownRpm                : 2;
    uint8_t  backDownRpm                : 2;
    uint8_t  readingLight               : 2;
    uint8_t  stationary                 : 2;
    
    uint8_t  hitchControlMode           : 2;
    uint8_t  autoStop                   : 2;
    uint8_t  powerAssist                : 2;
    uint8_t  res00                      : 2;
    
    uint8_t  balanceControlMode         : 2;
    uint8_t  balanceSensitivityMode     : 2;
    uint8_t  autoBrakeMode              : 2;
    uint8_t  saveLastData               : 1;
    uint8_t  res1                       : 1;
    
    uint8_t  engineLoad                 : 8;
    uint8_t  engineSpeedLSB             : 8;
    uint8_t  engineSpeedMSB             : 8;
    
    uint8_t  fourWDPrevious             : 2;
    uint8_t  res2                       : 6;
    
    uint8_t  flag                       : 8;
  };
} canFlag_t;

extern uint16_t engineSpeed;

#define CAN_SP_BUTTON_OFF               0x00
#define CAN_SP_BUTTON_ON                0x01
#define CAN_SP_BUTTON_ERROR             0x02
#define CAN_SP_BUTTON_NOT_AVAILABLE     0x03

#define CAN_SP_LED_OFF                  0x00
#define CAN_SP_LED_ON                   0x01
#define CAN_SP_LED_BLINK                0x02
#define CAN_SP_LED_INVALID              0x03


extern canRxEEC2_t                      canRxEEC2;
extern canRxEEC2_t                      canRxEEC2Previous;
extern canRxEEC1_t                      canRxEEC1;
extern canRxEEC1_t                      canRxEEC1Previous;
extern canTxTSC1_t                      canTxTSC1;
extern canTxSKC_t                       canTxSKC;

extern canTxIP1_t                       canTxIP1;
extern canTxIP2_t                       canTxIP2;

extern canRxACU_t                       canRxACU;
extern canRxACU_t                       canRxACUPrevious;
extern canTxACU_t                       canTxACU;

extern canRxTBOX1_t                     canRxTBOX1;
extern canRxTBOX1_t                     canRxTBOX1Previous;
extern canRxTBOX2_t                     canRxTBOX2;
extern canRxTBOX2_t                     canRxTBOX2Previous;
extern canRxCPG1_t                      canRxCPG1;
extern canRxCPG1_t                      canRxCPG1Previous;
extern canRxCPG2_t                      canRxCPG2;
extern canRxCPG2_t                      canRxCPG2Previous;
extern canRxCPG3_t                      canRxCPG3;
extern canRxCPG3_t                      canRxCPG3Previous;

extern canRxArmRest_t                   canRxArmRest;
extern canRxArmRest_t                   canRxArmRestPrevious;
extern canRxArmRest_t                   canRxArmRestPreviousApp;

extern canRxDash_t                      canRxDash;
extern canRxDash_t                      canRxDashPrevious;
extern canTxDashSetting_t               canTxDashSetting;
extern canTxDash_t                      canTxDash;

extern canRxPillar_t                    canRxPillar;
extern canRxPillar_t                    canRxPillarPrevious;
extern canTxPillarSetting_t             canTxPillarSetting;
extern canTxPillar_t                    canTxPillar;

extern canRxFenderA_t                   canRxFenderA;
extern canRxFenderA_t                   canRxFenderAPrevious;
extern canTxFenderASetting_t            canTxFenderASetting;
extern canTxFenderA_t                   canTxFenderA;

extern canRxFenderB_t                   canRxFenderB;
extern canRxFenderB_t                   canRxFenderBPrevious;
extern canTxFenderBSetting_t            canTxFenderBSetting;
extern canTxFenderB_t                   canTxFenderB;

extern canRxDial_t                      canRxDial;
extern canRxDial_t                      canRxDialPrevious;

extern canRxAngle_t                     canRxAngle;
extern canRxAngle_t                     canRxAnglePrevious;
extern canRxGyro_t                      canRxGyro;
extern canRxGyro_t                      canRxGyroPrevious;

extern canFlag_t                        canFlag;
extern canFlag_t                        canFlagPrevious;


void can_reinit();
void can_init(uint8_t channel);

void can_data_handler(uint32_t ID, uint8_t *rxData);
void init_can_variables();
void can_receive_process();

void can_transmit_process_2();

uint8_t can_transmit_tsc1(uint8_t *data);
uint8_t can_transmit_skc(uint8_t *data);
uint8_t can_transmit_ip1(uint8_t *data);
uint8_t can_transmit_ip2(uint8_t *data);
uint8_t can_transmit_acu(uint8_t *data);
uint8_t can_transmit_power_assist(uint8_t *data);
uint8_t can_transmit_eat(uint8_t *data);

uint8_t can_transmit(uint8_t channel, uint32_t id, uint8_t _data[]);


#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
