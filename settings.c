#include <stdlib.h>
#include "settings.h"
#include "settings_did.h"
#include "fram.h"
#include "can.h"
#include "input.h"
#include "output.h"
#include "main.h"
#include "watchdog.h"
#include "sensors.h"
#include "shuttle.h"
#include "rpmController.h"
#include "algorithm.h"
#include "hitch.h"
#include "uds_main.h"

/*--------------------------------------------------------- MAP number typedef and Variables Start ---------------------------------------------------*/
transmissionFillMapNumberState_t        mapNumberInfo[NUMBER_OF_MAP_STAGE_NUMBER];
transmissionFillMapNumberState_t        mapNumberShiftInfo[NUMBER_OF_MAP_STAGE_SHIFT_NUMBER];
trainFillMapNumberState_t               mapNumberTrainInfo[NUMBER_OF_MAP_STAGE_TRAIN_NUMBER];
transmissionMapDataState_t              mapDataInfo[NUMBER_OF_MAP_NUMBER][NUMBER_OF_MAP_POINT];

/*--------------------------------------------------------- MAP number typedef and Variables End -----------------------------------------------------*/
int16_t *nvFillCurrentFirst;
int16_t *nvFillCurrentTemperatureFirst;
int16_t *nvFillTimeFirst;
int16_t *nvFillTimeTemperatureFirst;
  
int16_t *nvFillCurrentSecond;
int16_t *nvFillCurrentTemperatureSecond;
int16_t *nvFillTimeSecond;
int16_t *nvFillTimeTemperatureSecond;
  
int16_t *nvForwardShuttleCompensationCurrent;
int16_t *nvForwardShuttleCompensationTemp;
int16_t *nvBackwardShuttleCompensationCurrent;
int16_t *nvBackwardShuttleCompensationTemp;
  
int16_t *nvDragOnCurrent;
int16_t *nvDragOnTime;
int16_t *nvDragOffCurrent;
int16_t *nvDragOffTime;

int16_t *nvPtoOnCurrentPercent;
int16_t *nvPtoOnTime;
int16_t *nvPtoOffCurrentPercent;
int16_t *nvPtoOffTime;

//int16_t *nvClutchMapCurrent;
//int16_t *nvClutchMapTime;
  
int16_t *nvClutchForwardFillCurrent;
int16_t *nvClutchForwardFillTime;
int16_t *nvClutchBackwardFillCurrent;
int16_t *nvClutchBackwardFillTime;

int16_t *nvClutchForwardPositionCurrent;
int16_t *nvClutchForwardPositionSensor;
int16_t *nvClutchBackwardPositionCurrent;
int16_t *nvClutchBackwardPositionSensor;

int16_t *nvThreePUpCurrent;
int16_t *nvThreePUpPosition;
int16_t *nvThreePDownCurrent;
int16_t *nvThreePDownPosition;

int16_t *nvBalanceUpCurrent;
int16_t *nvBalanceUpPosition;
int16_t *nvBalanceDownCurrent;
int16_t *nvBalanceDownPosition;
  
int16_t *nvThreePDraftUpCurrent;
int16_t *nvThreePDraftUpPosition;
int16_t *nvThreePDraftDownCurrent;
int16_t *nvThreePDraftDownPosition;

int16_t *nvThreePDepthUpCurrent;
int16_t *nvThreePDepthUpPosition;
int16_t *nvThreePDepthDownCurrent;
int16_t *nvThreePDepthDownPosition;

int16_t *nvPTOTemperatureCompensationDuty;
int16_t *nvPTOTemperatureCompensationDutyTemperature;

int16_t  nvX[NUMBER_OF_MAP_USED_FOR_OTHER_PFUNCTION][NUMBER_OF_MAP_POINT];      // First 24 map data is copied into this two dimensional array
int16_t  nvY[NUMBER_OF_MAP_USED_FOR_OTHER_PFUNCTION][NUMBER_OF_MAP_POINT];      // First 24 map data is copied into this two dimensional array
uint8_t  isUpdateHappenedMapData[NUMBER_OF_MAP_USED_FOR_OTHER_PFUNCTION];       // If update is occured in the map data this flag should be TRUE
uint8_t  isUpdateHappenedMapNumber;                                             // If update is occured in the map number (transmission) this flag should be TRUE

int16_t  nvTransmissionFillTime[NUMBER_OF_MAP_POINT];                           // There is no need pointer
int16_t  nvTransmissionFillCurrent[NUMBER_OF_MAP_POINT];
int16_t  nvTransmissionMapTime[NUMBER_OF_MAP_POINT];
int16_t  nvTransmissionMapCurrent[NUMBER_OF_MAP_POINT];
int16_t  nvTransmissionClutchToShuttleTime[NUMBER_OF_MAP_POINT];
int16_t  nvTransmissionClutchToShuttleCurrent[NUMBER_OF_MAP_POINT];


// NV Control data 
  // Shuttle & other control part
uint16_t nvPwmFrequency;
uint16_t nvDitherFrequency;
uint16_t nvDitherCurrent;
uint16_t nvShuttleSensetiveUpLimit;
uint16_t nvShuttleSensetiveMiddleLimit;
uint16_t nvShuttleSensetiveDownLimit;
uint16_t nvForwardReverseSwitchingSpeed;
uint16_t nvForwardReverseSwitchingTime;
uint16_t nvClutchPedalPushTime;
uint16_t nvClutchPedalFreeTime;
uint16_t nvEngineLoad;
uint16_t nvPtoHigher3Pposition;
uint16_t nvPtoMedium3Pposition;
uint16_t nvPtoLower3Pposition;
uint16_t nvRpmCruiseA;
uint16_t nvDragOnTemperature;
uint16_t nvDragOffTemperature;
uint16_t nvDragAutoConnectSpeed;
uint16_t nvDragAutoDisconnectSpeed;
uint16_t nv4WdAutoConnectSpeed;
uint16_t nv4WdAutoDisconnectSpeed;
uint16_t nvQtAutoConnectSpeed;
uint16_t nvQtAutoDisconnectSpeed;
uint16_t nvDiffAutoConnectSpeed;
uint16_t nvDiffAutoDisconnectSpeed;
uint16_t nvRpmCruiseB;
uint16_t nvSteeringRight;
uint16_t nvSteeringLeft;
uint16_t nvClutchGap;
uint16_t nvForwardBaseCurrent;
uint16_t nvBackwardBaseCurrent;
uint16_t nvPtoOnWhenHitchDown;
uint16_t nvHitchDownWhenTurnBackUpOn;

  // Hitch control part
uint16_t nv3pLeverUpLimit;
uint16_t nv3pLeverDownLimit;
uint16_t nv3pPositionUpLimit;
uint16_t nv3pPositionDownLimit;
uint16_t nv3pManualUpMinDuty;
uint16_t nv3pManualDownMinDuty;
uint16_t nv3pManualUpMaxDuty;
uint16_t nv3pManualDownMaxDuty;
uint16_t nv3pManualUpCycle;
uint16_t nv3pManualDownCycle;
uint16_t nv3pAutoUpMinDuty;
uint16_t nv3pAutoDownMinDuty;
uint16_t nv3pAutoUpMidDuty;
uint16_t nv3pAutoDownMidDuty;
uint16_t nv3pAutoUpMaxDuty;
uint16_t nv3pAutoDownMaxDuty;
uint16_t nv3pSettingUpDuty;
uint16_t nv3pSettingDownDuty;
uint16_t nv3pUpStepDuty;
uint16_t nv3pDownStepDuty;
uint16_t nv3pQuickUpStepDuty;
uint16_t nv3pQuickDownStepDuty;
uint16_t nv3pQuickDownDuty;
uint16_t nv3pStallStepDuty;

uint16_t nvDepthSensorSamplingTime;
uint16_t nvDepthTime;
uint16_t nvDepthSwitchOutputTime;
uint16_t nvDepthDownLimit;

uint16_t nvStrokeRightLimit;
uint16_t nvStrokeLeftLimit;
uint16_t nvStrokeCenter;
uint16_t nvRollingCenter;
uint16_t nvBalanceSamplingTimeHigh;
uint16_t nvBalanceSamplingTimeMid;
uint16_t nvBalanceSamplingTimeLow;
uint16_t nvBalanceDeadbandHigh;
uint16_t nvBalanceDeadbandMid;
uint16_t nvBalanceDeadbandLow;


uint16_t nvAdLeftHigh;
uint16_t nvAdLeftMid;
uint16_t nvAdLeftLow;
uint16_t nvAdRightHigh;
uint16_t nvAdRightMid;
uint16_t nvAdRightLow;
uint16_t nvAdDutyHigh;
uint16_t nvAdDutyMid;
uint16_t nvAdDutyLow;
uint16_t nvAdAutoConnectSpeed;
uint16_t nvAdAutoDisconnectSpeed;
uint16_t nvAdOnTime;
uint16_t nvAdOffTime;
uint16_t nvAdDeadBand;
uint16_t nvAdPWMDeadBand;
uint16_t nvAdNeutralRange;

uint16_t nvPanelData;
uint16_t nvSpeedUnit;

nvData_t controlSettingsData[NUMBER_OF_CONTROL_PACKET] = {
  // DID,       min,       max,    default, value
  
  { 4001,       100,        3500,   1000,   &nvPwmFrequency                 },              // 1    --> Hz
  { 4002,       50,         350,    100,    &nvDitherFrequency              },              // 2    --> Hz
  { 4003,       0,          400,    130,    &nvDitherCurrent                },              // 3    --> mA
  { 4004,       0,          200,    150,    &nvShuttleSensetiveUpLimit      },              // 4    --> mA          --> Offset = -100
  { 4005,       0,          200,    100,    &nvShuttleSensetiveMiddleLimit  },              // 5    --> mA          --> Offset = -100
  { 4006,       0,          200,    90,     &nvShuttleSensetiveDownLimit    },              // 6    --> mA          --> Offset = -100
  { 4007,       0,          300,    70,     &nvForwardReverseSwitchingSpeed },              // 7    --> km/h        --> Scale = 0.1
  { 4008,       0,          1500,   300,    &nvForwardReverseSwitchingTime  },              // 8    --> ms              VAC_DIRECTION_CHANGE_TIME
  { 4009,       10,         1500,   1000,   &nvClutchPedalPushTime          },              // 9    --> ms  
  { 4010,       10,         1500,   400,    &nvClutchPedalFreeTime          },              // 10   --> ms              VAC_CLUTCH_PUSH_FREE
  { 4011,       0,          100,    80,     &nvEngineLoad                   },              // 11   --> %
  { 4012,       615,        984,    885,    &nvPtoHigher3Pposition          },              // 12   --> 10Bit ADC (data * 4.88 = mV)
  { 4013,       615,        984,    804,    &nvPtoMedium3Pposition          },              // 13   --> 10Bit ADC (data * 4.88 = mV)
  { 4014,       615,        984,    717,    &nvPtoLower3Pposition           },              // 14   --> 10Bit ADC (data * 4.88 = mV)
  { 4015,       1000,       2350,   2200,   &nvRpmCruiseA                   },              // 15   --> RPM
  { 4016,       0,          175,    120,    &nvDragOnTemperature            },              // 16   --> Celsius     --> Offset = -25
  { 4017,       0,          175,    125,    &nvDragOffTemperature           },              // 17   --> Celsius     --> Offset = -25
  { 4018,       0,          300,    2,      &nvDragAutoConnectSpeed         },              // 18   --> km/h        --> Scale = 0.1
  { 4019,       0,          300,    2,      &nvDragAutoDisconnectSpeed      },              // 19   --> km/h        --> Scale = 0.1
  { 4020,       0,          300,    90,     &nv4WdAutoConnectSpeed          },              // 20   --> km/h        --> Scale = 0.1
  { 4021,       0,          300,    100,    &nv4WdAutoDisconnectSpeed       },              // 21   --> km/h        --> Scale = 0.1
  { 4022,       0,          300,    80,     &nvQtAutoConnectSpeed           },              // 22   --> km/h        --> Scale = 0.1
  { 4023,       0,          300,    80,     &nvQtAutoDisconnectSpeed        },              // 23   --> km/h        --> Scale = 0.1
  { 4024,       0,          300,    80,     &nvDiffAutoConnectSpeed         },              // 24   --> km/h        --> Scale = 0.1
  { 4025,       0,          300,    80,     &nvDiffAutoDisconnectSpeed      },              // 25   --> km/h        --> Scale = 0.1
  { 4026,       1000,       2350,   1400,   &nvRpmCruiseB                   },              // 26   --> RPM
  { 4027,       0,          1023,   785,    &nvSteeringRight                },              // 27   --> ADC
  { 4028,       0,          1023,   340,    &nvSteeringLeft                 },              // 28   --> ADC
  { 4029,       40,         200,    60,     &nvClutchGap                    },              // 29   --> ADC         --> Scale = 1
  { 4030,       0,          400,    210,    &nvForwardBaseCurrent           },              // 30   --> mA          --> Offset = -200
  { 4031,       0,          400,    210,    &nvBackwardBaseCurrent          },              // 31   --> mA          --> Offset = -200
  { 4032,       0,          1,      1,      &nvPtoOnWhenHitchDown           },              // 32   --> -           --> Offset = 0
  { 4033,       0,          1,      1,      &nvHitchDownWhenTurnBackUpOn    },              // 33   --> -           --> Offset = 0

  { 4101,       0,          1000,   900,    &nv3pLeverUpLimit               },              // 1    --> %        --> Scale = 0.1
  { 4102,       0,          1000,   100,    &nv3pLeverDownLimit             },              // 2    --> %        --> Scale = 0.1
  { 4103,       838,        982,    920,    &nv3pPositionUpLimit            },              // 3    --> 10Bit ADC (data * 4.88 = mV) // 4.1V, 4.8V, 4.50V
  { 4104,       21,         105,    39,     &nv3pPositionDownLimit          },              // 4    --> 10Bit ADC (data * 4.88 = mV) // 0.1V, 0.5V, 0.19V
  
  { 4105,       0,          1000,   230,    &nv3pManualUpMinDuty            },              // 5    --> %           --> Scale = 0.1
  { 4106,       0,          1000,   290,    &nv3pManualDownMinDuty          },              // 6    --> %           --> Scale = 0.1
  { 4107,       0,          1000,   350,    &nv3pManualUpMaxDuty            },              // 7    --> %           --> Scale = 0.1
  { 4108,       0,          1000,   490,    &nv3pManualDownMaxDuty          },              // 8    --> %           --> Scale = 0.1
  { 4109,       0,          1000,   600,    &nv3pManualUpCycle              },              // 9    --> ms
  { 4110,       0,          1000,   800,    &nv3pManualDownCycle            },              // 10   --> ms
  { 4111,       0,          1000,   220,    &nv3pAutoUpMinDuty              },              // 11   --> %           --> Scale = 0.1
  { 4112,       0,          1000,   220,    &nv3pAutoDownMinDuty            },              // 12   --> %           --> Scale = 0.1
  { 4113,       0,          1000,   380,    &nv3pAutoUpMidDuty              },              // 13   --> %           --> Scale = 0.1
  { 4114,       0,          1000,   380,    &nv3pAutoDownMidDuty            },              // 14   --> %           --> Scale = 0.1
  { 4115,       0,          1000,   700,    &nv3pAutoUpMaxDuty              },              // 15   --> %           --> Scale = 0.1
  { 4116,       0,          1000,   450,    &nv3pAutoDownMaxDuty            },              // 16   --> %           --> Scale = 0.1
  { 4117,       0,          1000,   500,    &nv3pSettingUpDuty              },              // 17   --> %           --> Scale = 0.1
  { 4118,       0,          1000,   450,    &nv3pSettingDownDuty            },              // 18   --> %           --> Scale = 0.1
  { 4119,       1,          30,     2,      &nv3pUpStepDuty                 },              // 19   --> %           --> Scale = 0.1
  { 4120,       1,          30,     2,      &nv3pDownStepDuty               },              // 20   --> %           --> Scale = 0.1
  { 4121,       1,          300,    20,     &nv3pQuickUpStepDuty            },              // 21   --> %           --> Scale = 0.1
  { 4122,       1,          300,    30,     &nv3pQuickDownStepDuty          },              // 22   --> %           --> Scale = 0.1
  { 4123,       1,          1000,   900,    &nv3pQuickDownDuty              },              // 23   --> %           --> Scale = 0.1
  { 4124,       1,          500,    60,     &nv3pStallStepDuty              },              // 24   --> %           --> Scale = 0.1
  { 4125,       5,          500,    20,     &nvDepthSensorSamplingTime      },              // 25   --> ms
  { 4126,       100,        2000,   1000,   &nvDepthTime                    },              // 26   --> ms
  { 4127,       0,          500,    100,    &nvDepthSwitchOutputTime        },              // 27   --> ms
  { 4128,       35,         102,    51,     &nvDepthDownLimit               },              // 28   --> ADC
  
  { 4201,       800,        1000,   928,    &nvStrokeRightLimit             },              // 1    --> 10Bit ADC (data * 4.88 = mV)
  { 4202,       50,         200,    115,    &nvStrokeLeftLimit              },              // 2    --> 10Bit ADC (data * 4.88 = mV)
  { 4203,       452,        572,    512,    &nvStrokeCenter                 },              // 3    --> 10Bit ADC (data * 4.88 = mV)
  { 4204,       452,        572,    512,    &nvRollingCenter                },              // 4    --> 10Bit ADC (data * 4.88 = mV)
  { 4205,       10,         1000,   300,    &nvBalanceSamplingTimeHigh      },              // 5    --> ms
  { 4206,       10,         1000,   221,    &nvBalanceSamplingTimeMid       },              // 6    --> ms
  { 4207,       10,         1000,   150,    &nvBalanceSamplingTimeLow       },              // 7    --> ms
  { 4208,       8,          123,    47,     &nvBalanceDeadbandHigh          },              // 8    --> 10Bit ADC (data * 4.88 = mV)
  { 4209,       8,          123,    28,     &nvBalanceDeadbandMid           },              // 9    --> 10Bit ADC (data * 4.88 = mV)
  { 4210,       8,          123,    20,     &nvBalanceDeadbandLow           },              // 10   --> 10Bit ADC (data * 4.88 = mV)

  { 4301,       21,         348,    117,    &nvAdLeftHigh                   },              // 1    --> 10Bit ADC (data * 4.88 = mV)
  { 4302,       21,         348,    173,    &nvAdLeftMid                    },              // 2    --> 10Bit ADC (data * 4.88 = mV)
  { 4303,       21,         348,    229,    &nvAdLeftLow                    },              // 3    --> 10Bit ADC (data * 4.88 = mV)
  { 4304,       676,        984,    907,    &nvAdRightHigh                  },              // 4    --> 10Bit ADC (data * 4.88 = mV)
  { 4305,       676,        984,    851,    &nvAdRightMid                   },              // 5    --> 10Bit ADC (data * 4.88 = mV)
  { 4306,       676,        984,    795,    &nvAdRightLow                   },              // 6    --> 10Bit ADC (data * 4.88 = mV)
  { 4307,       0,          1000,   900,    &nvAdDutyHigh                   },              // 7    --> %           --> Scale = 0.1
  { 4308,       0,          1000,   850,    &nvAdDutyMid                    },              // 8    --> %           --> Scale = 0.1
  { 4309,       0,          1000,   800,    &nvAdDutyLow                    },              // 9    --> %           --> Scale = 0.1
  { 4310,       0,          300,    80,     &nvAdAutoConnectSpeed           },              // 10   --> km/h        --> Scale = 0.1
  { 4311,       0,          300,    80,     &nvAdAutoDisconnectSpeed        },              // 11   --> km/h        --> Scale = 0.1
  { 4312,       0,          1000,   200,    &nvAdOnTime                     },              // 12   --> ms
  { 4313,       0,          1000,   200,    &nvAdOffTime                    },              // 13   --> ms
  { 4314,       8,          102,    20,     &nvAdDeadBand                   },              // 14   --> 10Bit ADC (data * 4.88 = mV)
  { 4315,       8,          287,    143,    &nvAdPWMDeadBand                },              // 15   --> 10Bit ADC (data * 4.88 = mV)
  { 4316,       492,        532,    512,    &nvAdNeutralRange               },              // 16   --> 10Bit ADC (data * 4.88 = mV)

  { 4401,       0,          65535,  0,      &nvPanelData                    },              // 1    --> Panel data
  { 4402,       0,          1,      0,      &nvSpeedUnit                    },              // 2    --> Speed unit
};

/*--------------------------------------------------------- Local functions Start ----------------------------------------------*/

/*--------------------------------------------------------- Local functions End -------------------------------------------------*/

void init_pointers()
{ 
  nvFillCurrentFirst = &(nvY[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][0]);                                    // 1
  nvFillCurrentTemperatureFirst = &(nvX[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][0]);                         // 1        
  nvFillTimeFirst = &(nvY[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][0]);                                          // 2
  nvFillTimeTemperatureFirst = &(nvX[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][0]);                               // 2
  
  nvFillCurrentSecond = &(nvY[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][0]);                                  // 3
  nvFillCurrentTemperatureSecond = &(nvX[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][0]);                       // 3
  nvFillTimeSecond = &(nvY[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][0]);                                        // 4
  nvFillTimeTemperatureSecond = &(nvX[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][0]);                             // 4
  
  nvForwardShuttleCompensationCurrent = &(nvY[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][0]);                 // 5
  nvForwardShuttleCompensationTemp = &(nvX[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][0]);                    // 5
  nvBackwardShuttleCompensationCurrent = &(nvY[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][0]);               // 6
  nvBackwardShuttleCompensationTemp = &(nvX[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][0]);                  // 6
  
  nvDragOnCurrent = &(nvY[MAP_NUMBER_DRAG_ON - 1][0]);                                                          // 7
  nvDragOnTime = &(nvX[MAP_NUMBER_DRAG_ON - 1][0]);                                                             // 7
  nvDragOffCurrent = &(nvY[MAP_NUMBER_DRAG_OFF - 1][0]);                                                        // 8
  nvDragOffTime = &(nvX[MAP_NUMBER_DRAG_OFF - 1][0]);                                                           // 8
  
  nvPtoOnCurrentPercent = &(nvY[MAP_NUMBER_PTO_ON - 1][0]);                                                     // 9
  nvPtoOnTime = &(nvX[MAP_NUMBER_PTO_ON - 1][0]);                                                               // 9
  nvPtoOffCurrentPercent = &(nvY[MAP_NUMBER_PTO_OFF - 1][0]);                                                   // 10
  nvPtoOffTime = &(nvX[MAP_NUMBER_PTO_OFF - 1][0]);                                                             // 10
  
  nvThreePUpCurrent = &(nvY[MAP_NUMBER_THREEP_UP - 1][0]);                                                      // 11
  nvThreePUpPosition = &(nvX[MAP_NUMBER_THREEP_UP - 1][0]);                                                     // 11
  nvThreePDownCurrent = &(nvY[MAP_NUMBER_THREEP_DOWN - 1][0]);                                                  // 12
  nvThreePDownPosition = &(nvX[MAP_NUMBER_THREEP_DOWN - 1][0]);                                                 // 12
  
  nvBalanceUpCurrent = &(nvY[MAP_NUMBER_BALANCE_UP - 1][0]);                                                    // 13
  nvBalanceUpPosition = &(nvX[MAP_NUMBER_BALANCE_UP - 1][0]);                                                   // 13
  nvBalanceDownCurrent = &(nvY[MAP_NUMBER_BALANCE_DOWN - 1][0]);                                                // 14
  nvBalanceDownPosition = &(nvX[MAP_NUMBER_BALANCE_DOWN - 1][0]);                                               // 14

  nvClutchForwardFillCurrent = &(nvY[FILL_NUMBER_CLUTCH_FORWARD - 1][0]);                                       // 15
  nvClutchForwardFillTime = &(nvX[FILL_NUMBER_CLUTCH_FORWARD - 1][0]);                                          // 15
  nvClutchBackwardFillCurrent = &(nvY[FILL_NUMBER_CLUTCH_BACKWARD - 1][0]);                                     // 16
  nvClutchBackwardFillTime = &(nvX[FILL_NUMBER_CLUTCH_BACKWARD - 1][0]);                                        // 16
  
  nvClutchForwardPositionCurrent = &(nvY[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][0]);                           // 17
  nvClutchForwardPositionSensor = &(nvX[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][0]);                            // 17
  nvClutchBackwardPositionCurrent = &(nvY[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][0]);                         // 18
  nvClutchBackwardPositionSensor = &(nvX[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][0]);                          // 18

  nvPTOTemperatureCompensationDuty = &(nvY[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][0]);                            // 19
  nvPTOTemperatureCompensationDutyTemperature = &(nvX[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][0]);                 // 19
 
  nvThreePDraftUpCurrent = &(nvY[20][0]);                                                                       // 20
  nvThreePDraftUpPosition = &(nvX[20][0]);                                                                      // 20
    
  nvThreePDraftDownCurrent = &(nvY[21][0]);                                                                     // 21
  nvThreePDraftDownPosition = &(nvX[21][0]);                                                                    // 21
    
  nvThreePDepthUpCurrent = &(nvY[22][0]);                                                                       // 22
  nvThreePDepthUpPosition = &(nvX[22][0]);                                                                      // 22
    
  nvThreePDepthDownCurrent = &(nvY[23][0]);                                                                     // 23
  nvThreePDepthDownPosition = &(nvX[23][0]);                                                                    // 23

}

uint8_t clutch_setting()
{
  uint8_t tempWriteData[4];
  uint8_t i;
  uint8_t j;

  uint16_t clutchValue;
  int16_t diff;
  
  if((get_clutch_sensor_average() >= 80) && (get_clutch_sensor_average() <= 500))
  {         
    clutchValue = get_clutch_sensor_average() + nvClutchGap;
    
    for( i = MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1; i < MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1 + 2; i++)
    {
      isUpdateHappenedMapData[i] = TRUE;
      
      diff = clutchValue - mapDataInfo[i][0].x;          // 90 - 70 = 20
      
      for( j = 0; j < NUMBER_OF_MAP_POINT; j++)
      {
        mapDataInfo[i][j].x += diff; 
      }
      
      for( j = 0; j < NUMBER_OF_MAP_POINT; j++)
      {
        tempWriteData[0] = mapDataInfo[i][j].x;
        tempWriteData[1] = mapDataInfo[i][j].x >> 8;
        tempWriteData[2] = mapDataInfo[i][j].y;
        tempWriteData[3] = mapDataInfo[i][j].y >> 8;
        fram_write(NV_MAP_POINT_START_ADDRESS + ((i * NUMBER_OF_MAP_POINT + j) * DIAGNOSTIC_DATA_SIZE), tempWriteData);
      }
    }
    return TRUE;
  }
  else
  {
    return FALSE;  
  }
}

void set_default_setting(uint8_t isForce)
{
  uint16_t i, j;
  uint8_t tempWriteData[4];
  uint8_t tempReadData[4];
  uint8_t flagCheck = FALSE;
  uint16_t framUpdateCounter;
  
  fram_read(NV_CONTROL_START_ADDRESS, tempReadData);
  framUpdateCounter = ((uint16_t)tempReadData[1] << 8) + ((uint16_t)tempReadData[0]);

  if(framUpdateCounter == 0) {
    flagCheck = TRUE;
  }
  else if(framUpdateCounter == 0xFFFF) {
    flagCheck = TRUE;
    framUpdateCounter = 0;
  }
  
  if(isForce == TRUE) {
    flagCheck = TRUE;
  }
  
  if(flagCheck == FALSE) {      // No force write, data is saved
    return;
  }
    
  framUpdateCounter += 1;
  tempWriteData[0] = (uint8_t)(framUpdateCounter);
  tempWriteData[1] = (uint8_t)(framUpdateCounter >> 8);
  tempWriteData[2] = 0;
  tempWriteData[3] = 0;
  fram_write(NV_CONTROL_START_ADDRESS, tempWriteData);
  

  // Updating map stage information from FRAM
  i = 0;                                                                        // Forward 1단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L                       // DID = 500
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 4;                       // Map number = 34      L                       // DID = 501
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M                       // DID = 508
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 2;                       // Map Number = 32      M                       // DID = 509
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 2단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 4;                       // Map number = 34      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 1;                       // Map Number = 31      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 3단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 3;                       // Map number = 33      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 1;                       // Map Number = 31      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 4단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 3;                       // Map number = 33      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 5단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 6단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 1단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 2단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 3단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 4단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  watchdog_trigger();
  
  i++;                                                                          // Forward 5단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 6단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  // Backward
  i++;                                                                          // Backward 1단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 7;                       // Map number = 37      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 6;                       // Map Number = 36      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 2단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 7;                       // Map number = 37      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 6;                       // Map Number = 36      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 3단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 7;                       // Map number = 37      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 6;                       // Map Number = 36      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 4단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 7;                       // Map number = 37      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 6;                       // Map Number = 36      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 5단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 6단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      M
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 1단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 5;                       // Map number = 35      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 2단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 5;                       // Map number = 35      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  watchdog_trigger();
  
  i++;                                                                          // Backward 3단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 5;                       // Map number = 35      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 4단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 5;                       // Map number = 35      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 5단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 6단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map number = 30      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION;                           // Map Number = 30      Not used
  mapNumberInfo[i].hl_map  = tempWriteData[0];
  mapNumberInfo[i].hl_fill = tempWriteData[1];
  mapNumberInfo[i].cm_map   = tempWriteData[2];
  mapNumberInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);

  // Transmission shift map is beginning
  
  i = 0;                                                                        // Forward 1-->2 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map Number = 41      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 2-->3 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map Number = 41      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 3-->4 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 12;                      // Map Number = 42      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 2-->1 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 3-->2 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  watchdog_trigger();
  
  i++;                                                                          // Forward 4-->3 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map Number = 41      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 1-->2 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 12;                      // Map number = 42      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 2-->3 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 13;                      // Map number = 43      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 3-->4 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 13;                      // Map number = 43      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 2-->1 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map number = 41      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 3-->2 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map number = 41      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Forward 4-->3 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map number = 41      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  // Backward
  i++;                                                                          // Backward 1-->2 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map Number = 41      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 2-->3 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map Number = 41      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 3-->4 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map Number = 41      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  watchdog_trigger();
  
  i++;                                                                          // Backward 2-->1 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);

  i++;                                                                          // Backward 3-->2 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 4-->3 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     L
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      L
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     M
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      M
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 1-->2 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map number = 41      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 2-->3 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map number = 41      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 3-->4 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map number = 41      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 2-->1 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 3-->2 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 11;                      // Map number = 41      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  i++;                                                                          // Backward 4-->3 단
  tempWriteData[1] = FILL_NUMBER_DEFAULT_TRANSMISSION + 1;                      // Fill number = 21     H
  tempWriteData[0] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map number = 40      H
  tempWriteData[3] = FILL_NUMBER_DEFAULT_TRANSMISSION;                          // Fill number = 20     Not used
  tempWriteData[2] = MAP_NUMBER_DEFAULT_TRANSMISSION + 10;                      // Map Number = 40      Not used
  mapNumberShiftInfo[i].hl_map  = tempWriteData[0];
  mapNumberShiftInfo[i].hl_fill = tempWriteData[1];
  mapNumberShiftInfo[i].cm_map   = tempWriteData[2];
  mapNumberShiftInfo[i].cm_fill  = tempWriteData[3];
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  watchdog_trigger();
  
  // Updating map data information from FRAM
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][0].x = 14;       // Fill temperature (X-Axis - Temperature) - 1.1
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][1].x = 22;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][2].x = 30;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][3].x = 38;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][4].x = 42;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][5].x = 66;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][6].x = 100;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][7].x = 120;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][0].y = 1000;     // Fill current (Y-Axis - Current) - 1.1
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][1].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][2].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][3].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][4].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][5].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][6].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_FIRST - 1][7].y = 1000;
  
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][0].x = 14;          // Fill temperature (X-Axis - Temperature) - 1.2
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][1].x = 22;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][2].x = 30;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][3].x = 38;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][4].x = 42;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][5].x = 66;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][6].x = 100;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][7].x = 120;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][0].y = 1200;        // Fill Time (Y-Axis - Time) - 1.2
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][1].y = 950;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][2].y = 800;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][3].y = 580;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][4].y = 480;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][5].y = 370;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][6].y = 350;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_FIRST - 1][7].y = 330;

  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][0].x = 14;      // Fill temperature (X-Axis - Temperature) - 2.1
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][1].x = 22;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][2].x = 30;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][3].x = 38;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][4].x = 42;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][5].x = 66;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][6].x = 125;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][7].x = 135;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][0].y = 1000;    // Fill current (Y-Axis - Current) - 2.1
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][1].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][2].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][3].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][4].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][5].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][6].y = 1000;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_CURRENT_SECOND - 1][7].y = 1000;
  
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][0].x = 14;         // Fill temperature (X-Axis - Temperature) - 1.1
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][1].x = 22;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][2].x = 30;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][3].x = 38;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][4].x = 42;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][5].x = 66;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][6].x = 125;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][7].x = 135;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][0].y = 1000;       // Fill temperature (Y-Axis - Time) - 1.1
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][1].y = 650;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][2].y = 520;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][3].y = 460;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][4].y = 440;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][5].y = 440;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][6].y = 400;
  mapDataInfo[FILL_NUMBER_TEMPERATURE_TIME_SECOND - 1][7].y = 390;
  
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][0].x = 0;     // 5 - Forward Shuttle compensation map
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][1].x = 10;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][2].x = 30;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][3].x = 40;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][4].x = 50;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][5].x = 60;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][6].x = 125;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][7].x = 135;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][0].y = 255;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][1].y = 253;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][2].y = 250;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][3].y = 240;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][4].y = 225;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][5].y = 210;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][6].y = 200;
  mapDataInfo[MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION - 1][7].y = 200;
  
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][0].x = 0;    // 6 - Backward Shuttle compensation map
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][1].x = 10;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][2].x = 30;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][3].x = 40;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][4].x = 50;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][5].x = 60;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][6].x = 125;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][7].x = 135;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][0].y = 258;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][1].y = 258;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][2].y = 255;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][3].y = 245;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][4].y = 230;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][5].y = 210;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][6].y = 200;
  mapDataInfo[MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION - 1][7].y = 200;
  
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][0].x = 0;                                 // 7 - DRAG ON MAP                                  (7)
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][1].x = 100;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][2].x = 300;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][3].x = 750;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][4].x = 1100;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][5].x = 1500;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][6].x = 1800;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][7].x = 2000;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][0].y = 0;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][1].y = 400;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][2].y = 450;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][3].y = 500;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][4].y = 550;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][5].y = 600;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][6].y = 650;
  mapDataInfo[MAP_NUMBER_DRAG_ON - 1][7].y = 700;
  
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][0].x = 10;                               // 8 - DRAF OFF MAP                                      (8)
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][1].x = 11;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][2].x = 15;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][3].x = 17;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][4].x = 20;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][5].x = 22;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][6].x = 23;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][7].x = 25;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][0].y = 150;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][1].y = 100;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][2].y = 50;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][3].y = 0;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][4].y = 0;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][5].y = 0;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][6].y = 0;
  mapDataInfo[MAP_NUMBER_DRAG_OFF - 1][7].y = 0;

  mapDataInfo[MAP_NUMBER_PTO_ON - 1][0].x = 35;                                 // 9 - PTO ON Map
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][1].x = 36;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][2].x = 100;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][3].x = 200;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][4].x = 1000;                      
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][5].x = 1250;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][6].x = 1500;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][7].x = 2000;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][0].y = 360;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][1].y = 360;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][2].y = 360;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][3].y = 270;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][4].y = 270;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][5].y = 270;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][6].y = 330;
  mapDataInfo[MAP_NUMBER_PTO_ON - 1][7].y = 1000;
  
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][0].x = 100;                           // 10 - PTO OFF
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][1].x = 300;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][2].x = 500;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][3].x = 550;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][4].x = 600;                      
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][5].x = 601;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][6].x = 602;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][7].x = 603;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][0].y = 300;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][1].y = 280;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][2].y = 250;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][3].y = 100;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][4].y = 0;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][5].y = 0;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][6].y = 0;
  mapDataInfo[MAP_NUMBER_PTO_OFF - 1][7].y = 0;

  watchdog_trigger();
  
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][0].x = 6;                         // 11 - 3P hitch controller up direction, x - position, y - output duty
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][1].x = 8;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][2].x = 28;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][3].x = 45;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][4].x = 150;                      
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][5].x = 200;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][6].x = 320;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][7].x = 520;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][0].y = 190;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][1].y = 210;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][2].y = 250;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][3].y = 290;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][4].y = 410;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][5].y = 440;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][6].y = 500;
  mapDataInfo[MAP_NUMBER_THREEP_UP - 1][7].y = 560;

  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][0].x = 4;                       // 12 - 3P hitch controller down direction, x - position, y - output duty
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][1].x = 8;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][2].x = 28;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][3].x = 40;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][4].x = 150;                      
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][5].x = 208;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][6].x = 320;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][7].x = 460;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][0].y = 210;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][1].y = 230;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][2].y = 280;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][3].y = 320;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][4].y = 400;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][5].y = 430;                     // map data is updated on 2021.12.10 in the field
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][6].y = 480;
  mapDataInfo[MAP_NUMBER_THREEP_DOWN - 1][7].y = 500;  

  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][0].x = 2;                        // 13 - Balance controller up direction, x - position, y - output duty
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][1].x = 5;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][2].x = 8;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][3].x = 20;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][4].x = 40;                      
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][5].x = 60;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][6].x = 80;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][7].x = 105;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][0].y = 220;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][1].y = 240;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][2].y = 260;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][3].y = 290;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][4].y = 350;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][5].y = 520;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][6].y = 700;
  mapDataInfo[MAP_NUMBER_BALANCE_UP - 1][7].y = 800;
  
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][0].x = 2;                      // 14 - Balance controller down direction, x - position, y - output duty
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][1].x = 5;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][2].x = 8;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][3].x = 20;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][4].x = 40;                      
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][5].x = 60;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][6].x = 80;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][7].x = 105;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][0].y = 220;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][1].y = 240;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][2].y = 260;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][3].y = 290;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][4].y = 350;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][5].y = 520;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][6].y = 700;
  mapDataInfo[MAP_NUMBER_BALANCE_DOWN - 1][7].y = 800;
    
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][0].x = 1;                   // 15 - Clutch Forward X-Axis data - Time            it is fill temperature map (first or second)
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][1].x = 180;
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][2].x = 181;
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][3].x = 182;
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][4].x = 212;                      
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][5].x = 1200;                // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][6].x = 1200;                // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][7].x = 1200;                // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][0].y = 800;                // Clutch Forward Y-Axis data - Current - unused
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][1].y = 800;
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][2].y = 800;
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][3].y = 600;
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][4].y = 600;
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][5].y = 170;                 // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][6].y = 170;                 // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_FORWARD - 1][7].y = 170;                 // NOT used point

  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][0].x = 1;                  // 16 - Clutch Backward X-Axis data - Time
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][1].x = 180;                // clutch fill
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][2].x = 181;
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][3].x = 182;
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][4].x = 212;
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][5].x = 1200;               // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][6].x = 1200;               // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][7].x = 1200;               // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][0].y = 850;               // Clutch Backward Y-Axis data - Current - unused
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][1].y = 850;
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][2].y = 850;
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][3].y = 600;
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][4].y = 600;
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][5].y = 170;                // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][6].y = 170;                // NOT used point
  mapDataInfo[FILL_NUMBER_CLUTCH_BACKWARD - 1][7].y = 170;                // NOT used point
  
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][0].x = 415;         // 17 - Clutch Forward position data X-Axis - sensor data (0 to 1023)
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][1].x = 455;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][2].x = 495;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][3].x = 535;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][4].x = 575;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][5].x = 615;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][6].x = 655;         
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][7].x = 735;         // pedal free
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][0].y = 505;         // Clutch Forward position data Y-Axis - Current
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][1].y = 515;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][2].y = 540;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][3].y = 550;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][4].y = 560;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][5].y = 570;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][6].y = 575;
  mapDataInfo[MAP_NUMBER_CLUTCH_FORWARD_POSITION - 1][7].y = 580;

  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][0].x = 415;        // 18 - Clutch Backward position data X-Axis - sensor data (0 to 1023)
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][1].x = 455;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][2].x = 495;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][3].x = 535;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][4].x = 575;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][5].x = 615;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][6].x = 655;               
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][7].x = 735;        // pedal free
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][0].y = 505;        // Clutch Backward position data Y-Axis - Current
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][1].y = 515;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][2].y = 540;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][3].y = 550;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][4].y = 560;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][5].y = 570;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][6].y = 575;
  mapDataInfo[MAP_NUMBER_CLUTCH_BACKWARD_POSITION - 1][7].y = 580;
    
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][0].x = 0;                    // PTO Duty-temperature compensation map                 (19)
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][1].x = 10;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][2].x = 25;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][3].x = 65;                   // 65 - 25 = 40
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][4].x = 85;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][5].x = 115;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][6].x = 125;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][7].x = 135;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][0].y = 500;                  // OFFSet 500
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][1].y = 500;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][2].y = 500;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][3].y = 500;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][4].y = 510;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][5].y = 520;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][6].y = 520;
  mapDataInfo[MAP_NUMBER_PTO_TEMPERATURE_DUTY - 1][7].y = 520;
  
  watchdog_trigger();
  
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][0].x = 1;             // Testing Fill data            (20)
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][1].x = 205;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][2].x = 206;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][3].x = 207;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][4].x = 227;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][5].x = 1200;          // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][6].x = 1200;          // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][7].x = 1200;          // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][0].y = 850;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][1].y = 850;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][2].y = 850;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][3].y = 600;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][4].y = 600;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][5].y = 170;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][6].y = 170;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 1][7].y = 170;           // Not used point
  
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][0].x = 1;             // Testing Fill data            (21)
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][1].x = 205;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][2].x = 206;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][3].x = 207;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][4].x = 247;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][5].x = 1200;          // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][6].x = 1200;          // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][7].x = 1200;          // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][0].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][1].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][2].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][3].y = 600;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][4].y = 600;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][5].y = 170;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][6].y = 170;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION - 0][7].y = 170;           // Not used point
  
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][0].x = 2;             // Testing Fill data            (22)
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][1].x = 100;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][2].x = 150;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][3].x = 200;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][4].x = 201;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][5].x = 202;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][6].x = 210;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][7].x = 400;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][0].y = 622;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][1].y = 600;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][2].y = 600;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][3].y = 300;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][4].y = 300;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][5].y = 300;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][6].y = 300;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 1][7].y = 300;           // Not used point

  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][0].x = 55;             // Testing Fill data            (23)
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][1].x = 85;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][2].x = 115;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][3].x = 125;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][4].x = 135;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][5].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][6].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][7].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][0].y = 400;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][1].y = 250;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][2].y = 200;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][3].y = 180;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][4].y = 170;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][5].y = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][6].y = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 2][7].y = 0;           // Not used point

  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][0].x = 55;             // Testing Fill data            (24)
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][1].x = 85;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][2].x = 150;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][3].x = 250;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][4].x = 300;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][5].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][6].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][7].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][0].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][1].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][2].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][3].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][4].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][5].y = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][6].y = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 3][7].y = 0;           // Not used point

  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][0].x = 200;             // Testing Fill data            (25)
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][1].x = 300;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][2].x = 401;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][3].x = 450;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][4].x = 500;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][5].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][6].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][7].x = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][0].y = 260;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][1].y = 300;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][2].y = 420;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][3].y = 800;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][4].y = 1000;
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][5].y = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][6].y = 0;           // Not used point
  mapDataInfo[FILL_NUMBER_DEFAULT_TRANSMISSION + 4][7].y = 0;           // Not used point

  watchdog_trigger();
  
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][0].x = 8;                   // 3P Draft up map              (26)
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][1].x = 12;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][2].x = 20;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][3].x = 28;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][4].x = 36;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][5].x = 40;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][6].x = 44;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][7].x = 48;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][0].y = 250;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][1].y = 270;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][2].y = 300;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][3].y = 340;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][4].y = 350;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][5].y = 360;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][6].y = 370;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][7].y = 380;
  
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][0].x = 8;                 // 3P Draft down map            (27)
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][1].x = 12;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][2].x = 20;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][3].x = 28;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][4].x = 36;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][5].x = 40;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][6].x = 44;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][7].x = 48;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][0].y = 250;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][1].y = 270;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][2].y = 300;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][3].y = 340;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][4].y = 350;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][5].y = 360;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][6].y = 370;
  mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][7].y = 380;
  
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][0].x = 20;                   // 3P depth up map              (28)
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][1].x = 28;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][2].x = 32;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][3].x = 36;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][4].x = 40;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][5].x = 52;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][6].x = 60;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][7].x = 80;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][0].y = 230;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][1].y = 240;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][2].y = 240;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][3].y = 240;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][4].y = 250;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][5].y = 260;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][6].y = 260;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][7].y = 270;
  
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][0].x = 20;                 // 3P depth down map            (29)
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][1].x = 28;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][2].x = 36;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][3].x = 40;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][4].x = 48;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][5].x = 56;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][6].x = 64;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][7].x = 80;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][0].y = 240;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][1].y = 250;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][2].y = 260;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][3].y = 270;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][4].y = 280;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][5].y = 300;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][6].y = 320;
  mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][7].y = 340;
  
  watchdog_trigger();
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][0].x = 200;              // Testing Map data - Time      (30)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][1].x = 250;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][2].x = 410;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][3].x = 600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][4].x = 1200;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][5].x = 1600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][6].x = 1900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][7].x = 2500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][0].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][1].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][2].y = 540;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][3].y = 570;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][4].y = 606;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][5].y = 685;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][6].y = 740;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 1][7].y = 990;

  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][0].x = 140;            // Testing Map data - Time      (31)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][1].x = 180;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][2].x = 900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][3].x = 1100;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][4].x = 1300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][5].x = 1500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][6].x = 1800;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][7].x = 2200;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][0].y = 595;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][1].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][2].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][3].y = 542;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][4].y = 555;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][5].y = 582;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][6].y = 645;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION - 0][7].y = 1015;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][0].x = 140;            // Testing Map data - Time      (32)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][1].x = 180;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][2].x = 1100;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][3].x = 1300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][4].x = 1500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][5].x = 1700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][6].x = 1900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][7].x = 2000;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][0].y = 570;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][1].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][2].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][3].y = 545;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][4].y = 560;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][5].y = 582;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][6].y = 630;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 1][7].y = 1025;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][0].x = 140;            // Testing Map data - Time      (33)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][1].x = 180;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][2].x = 1100;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][3].x = 1300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][4].x = 1500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][5].x = 1700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][6].x = 1750;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][7].x = 1800;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][0].y = 595;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][1].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][2].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][3].y = 540;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][4].y = 555;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][5].y = 588;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][6].y = 630;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 2][7].y = 1025;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][0].x = 140;            // Testing Map data - Time      (34)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][1].x = 180;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][2].x = 900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][3].x = 1100;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][4].x = 1300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][5].x = 1500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][6].x = 1550;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][7].x = 1800;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][0].y = 600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][1].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][2].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][3].y = 545;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][4].y = 565;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][5].y = 595;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][6].y = 635;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 3][7].y = 1035;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][0].x = 200;            // Testing Map data - Time      (35)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][1].x = 250;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][2].x = 410;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][3].x = 600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][4].x = 1200;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][5].x = 1600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][6].x = 1900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][7].x = 2500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][0].y = 600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][1].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][2].y = 540;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][3].y = 570;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][4].y = 606;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][5].y = 685;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][6].y = 740;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 4][7].y = 1000;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][0].x = 140;            // Testing Map data - Time      (36)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][1].x = 180;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][2].x = 900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][3].x = 1000;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][4].x = 1200;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][5].x = 1400;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][6].x = 1700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][7].x = 2300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][0].y = 600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][1].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][2].y = 535;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][3].y = 542;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][4].y = 555;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][5].y = 582;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][6].y = 646;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 5][7].y = 1000;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][0].x = 140;            // Testing Map data - Time      (37)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][1].x = 180;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][2].x = 1100;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][3].x = 1300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][4].x = 1500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][5].x = 1700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][6].x = 1900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][7].x = 1910;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][0].y = 635;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][1].y = 505;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][2].y = 505;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][3].y = 509;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][4].y = 523;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][5].y = 549;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][6].y = 615;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 6][7].y = 985;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][0].x = 1300;            // Testing Map data - Time      (38)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][1].x = 1600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][2].x = 1900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][3].x = 2100;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][4].x = 2300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][5].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][6].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][7].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][0].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][1].y = 14;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][2].y = 9;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][3].y = 5;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][4].y = 2;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][5].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][6].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 7][7].y = 0;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][0].x = 0;            // Testing Map data - Time      (39)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][1].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][2].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][3].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][4].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][5].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][6].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][7].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][0].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][1].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][2].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][3].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][4].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][5].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][6].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 8][7].y = 0;
  
  watchdog_trigger();
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][0].x = 0;              // Testing Map data - Time      (40)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][1].x = 10;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][2].x = 900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][3].x = 1300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][4].x = 1500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][5].x = 1800;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][6].x = 1850;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][7].x = 1900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][0].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][1].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][2].y = 550;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][3].y = 585;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][4].y = 620;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][5].y = 700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][6].y = 750;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 9][7].y = 1000;

  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][0].x = 0;             // Testing Map data - Time      (41)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][1].x = 10;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][2].x = 800;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][3].x = 1000;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][4].x = 1400;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][5].x = 1600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][6].x = 1700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][7].x = 1900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][0].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][1].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][2].y = 550;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][3].y = 585;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][4].y = 650;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][5].y = 700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][6].y = 750;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 10][7].y = 1000;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][0].x = 0;             // Testing Map data - Time      (42)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][1].x = 10;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][2].x = 300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][3].x = 500;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][4].x = 900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][5].x = 1100;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][6].x = 1200;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][7].x = 1400;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][0].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][1].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][2].y = 550;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][3].y = 585;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][4].y = 650;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][5].y = 700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][6].y = 750;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 11][7].y = 1000;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][0].x = 0;             // Testing Map data - Time      (43)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][1].x = 10;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][2].x = 200;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][3].x = 300;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][4].x = 600;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][5].x = 800;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][6].x = 900;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][7].x = 1100;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][0].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][1].y = 530;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][2].y = 550;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][3].y = 575;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][4].y = 650;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][5].y = 700;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][6].y = 750;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 12][7].y = 1000;
    
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][0].x = 0;            // Testing Map data - Time      (44)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][1].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][2].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][3].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][4].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][5].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][6].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][7].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][0].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][1].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][2].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][3].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][4].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][5].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][6].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 13][7].y = 0;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][0].x = 0;            // Testing Map data - Time      (45)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][1].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][2].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][3].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][4].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][5].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][6].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][7].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][0].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][1].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][2].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][3].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][4].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][5].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][6].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 14][7].y = 0;
  
  //MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][0].x = 0;              // Clutch to Shuttle       (46), // INDEX = 46 - 1 = 45
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][1].x = 11;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][2].x = 12;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][3].x = 13;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][4].x = 1000;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][5].x = 2000;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][6].x = 2300;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][7].x = 2500;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][0].y = 300;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][1].y = 301;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][2].y = 302;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][3].y = 303;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][4].y = 350;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][5].y = 600;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][6].y = 700;
  mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][7].y = 1000;
    
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][0].x = 0;            // Testing Map data - Time      (47)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][1].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][2].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][3].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][4].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][5].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][6].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][7].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][0].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][1].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][2].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][3].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][4].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][5].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][6].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 16][7].y = 0;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][0].x = 0;            // Testing Map data - Time      (48)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][1].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][2].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][3].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][4].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][5].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][6].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][7].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][0].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][1].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][2].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][3].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][4].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][5].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][6].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 17][7].y = 0;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][0].x = 0;            // Testing Map data - Time      (49)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][1].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][2].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][3].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][4].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][5].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][6].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][7].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][0].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][1].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][2].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][3].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][4].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][5].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][6].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 18][7].y = 0;
  
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][0].x = 0;            // Testing Map data - Time      (50)
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][1].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][2].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][3].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][4].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][5].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][6].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][7].x = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][0].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][1].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][2].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][3].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][4].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][5].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][6].y = 0;
  mapDataInfo[MAP_NUMBER_DEFAULT_TRANSMISSION + 19][7].y = 0;
  
  watchdog_trigger();
  
  for( i = 0; i < NUMBER_OF_MAP_NUMBER; i++)
  {
    for( j = 0; j < NUMBER_OF_MAP_POINT; j++)
    {
      tempWriteData[0] = mapDataInfo[i][j].x;
      tempWriteData[1] = mapDataInfo[i][j].x >> 8;
      tempWriteData[2] = mapDataInfo[i][j].y;
      tempWriteData[3] = mapDataInfo[i][j].y >> 8;
      fram_write(NV_MAP_POINT_START_ADDRESS + ((i * NUMBER_OF_MAP_POINT + j) * DIAGNOSTIC_DATA_SIZE), tempWriteData);
    }
    if((i % 10) == 0) {
      watchdog_trigger();
    }
  }
  
  for(i = 0; i < NUMBER_OF_CONTROL_PACKET; i++)
  {
    // The default setting is updated
    check_control_data(TRUE, i, controlSettingsData[i].defaultValue);
    if((i % 10) == 0) {
      watchdog_trigger();
    }
  }
  
  init_diagnostic_variables();                                                  // Added on 2023.05.24
  
  nvRpmCruiseNewA = nvRpmCruiseA;
  nvRpmCruiseNewB = nvRpmCruiseB;
  
  lever_rate_calculate();
  position_rate_calculate();
}

void init_diagnostic_variables()
{
  uint16_t i, j;
  uint8_t tempReadData[4];
  
  uint8_t updateFram = 0;

  set_default_setting(FALSE);
  
   
  updateFram = FALSE;
     
  // Updating map stage information from FRAM
  for( i = 0; i < NUMBER_OF_MAP_STAGE_NUMBER; i++)
  {
    fram_read(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    
    mapNumberInfo[i].hl_map  = tempReadData[0];
    mapNumberInfo[i].hl_fill = tempReadData[1];
    mapNumberInfo[i].cm_map   = tempReadData[2];
    mapNumberInfo[i].cm_fill  = tempReadData[3];
    
    
    if((mapNumberInfo[i].hl_map < MAP_NUMBER_START_TRANSMISSION) || (mapNumberInfo[i].hl_map > MAP_NUMBER_END_TRANSMISSION)) {
      mapNumberInfo[i].hl_map = MAP_NUMBER_DEFAULT_TRANSMISSION;
      updateFram = TRUE;
    }
    if((mapNumberInfo[i].hl_fill < FILL_NUMBER_START_TRANSMISSION) || (mapNumberInfo[i].hl_fill > FILL_NUMBER_END_TRANSMISSION)) {
      mapNumberInfo[i].hl_fill = FILL_NUMBER_DEFAULT_TRANSMISSION;
      updateFram = TRUE;
    }
    
    if((i <= 5) || ((i >= 12) && (i <= 17))) {
      if((mapNumberInfo[i].cm_map < MAP_NUMBER_START_TRANSMISSION) || (mapNumberInfo[i].cm_map > MAP_NUMBER_END_TRANSMISSION)) {
        mapNumberInfo[i].cm_map = MAP_NUMBER_DEFAULT_TRANSMISSION;
        updateFram = TRUE;
      }
      if((mapNumberInfo[i].cm_fill < FILL_NUMBER_START_TRANSMISSION) || (mapNumberInfo[i].cm_fill > FILL_NUMBER_END_TRANSMISSION)) {
        mapNumberInfo[i].cm_fill = FILL_NUMBER_DEFAULT_TRANSMISSION;
        updateFram = TRUE;
      }
    }
    
    if(updateFram == TRUE) {
      tempReadData[0] = mapNumberInfo[i].hl_map;
      tempReadData[1] = mapNumberInfo[i].hl_fill;
      tempReadData[2] = mapNumberInfo[i].cm_map;
      tempReadData[3] = mapNumberInfo[i].cm_fill;
      fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
      updateFram = FALSE;
    }
    if((i % 10) == 0) {                                                         // Added on 2021.09.14
      watchdog_trigger();
    }
  }
  
  updateFram = FALSE;
  // Updating 
  for( i = 0; i < NUMBER_OF_MAP_STAGE_SHIFT_NUMBER; i++)
  {
    fram_read(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);

    mapNumberShiftInfo[i].hl_map  = tempReadData[0];
    mapNumberShiftInfo[i].hl_fill = tempReadData[1];
    mapNumberShiftInfo[i].cm_map   = tempReadData[2];
    mapNumberShiftInfo[i].cm_fill  = tempReadData[3];
    
    
    if((mapNumberShiftInfo[i].hl_map < MAP_NUMBER_START_TRANSMISSION) || (mapNumberShiftInfo[i].hl_map > MAP_NUMBER_END_TRANSMISSION)) {
      mapNumberShiftInfo[i].hl_map = MAP_NUMBER_DEFAULT_TRANSMISSION;
      updateFram = TRUE;
    }
    if((mapNumberShiftInfo[i].hl_fill < FILL_NUMBER_START_TRANSMISSION) || (mapNumberShiftInfo[i].hl_fill > FILL_NUMBER_END_TRANSMISSION)) {
      mapNumberShiftInfo[i].hl_fill = FILL_NUMBER_DEFAULT_TRANSMISSION;
      updateFram = TRUE;
    }
    
    if((i <= 5) || ((i >= 12) && (i <= 17))) {
      if((mapNumberShiftInfo[i].cm_map < MAP_NUMBER_START_TRANSMISSION) || (mapNumberShiftInfo[i].cm_map > MAP_NUMBER_END_TRANSMISSION)) {
        mapNumberShiftInfo[i].cm_map = MAP_NUMBER_DEFAULT_TRANSMISSION;
        updateFram = TRUE;
      }
      if((mapNumberShiftInfo[i].cm_fill < FILL_NUMBER_START_TRANSMISSION) || (mapNumberShiftInfo[i].cm_fill > FILL_NUMBER_END_TRANSMISSION)) {
        mapNumberShiftInfo[i].cm_fill = FILL_NUMBER_DEFAULT_TRANSMISSION;
        updateFram = TRUE;
      }
    }
    
    if(updateFram == TRUE) {
      tempReadData[0] = mapNumberShiftInfo[i].hl_map;
      tempReadData[1] = mapNumberShiftInfo[i].hl_fill;
      tempReadData[2] = mapNumberShiftInfo[i].cm_map;
      tempReadData[3] = mapNumberShiftInfo[i].cm_fill;
      fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
      updateFram = FALSE;
    }
    if((i % 10) == 0) {                                                         // Added on 2021.09.14
      watchdog_trigger();
    }
  }

  // Updating map data information from FRAM
  for( i = 0; i < NUMBER_OF_MAP_NUMBER; i++)
  {
    for( j = 0; j < NUMBER_OF_MAP_POINT; j++)
    {
      fram_read(NV_MAP_POINT_START_ADDRESS + ((i * NUMBER_OF_MAP_POINT + j) * DIAGNOSTIC_DATA_SIZE), tempReadData);
      mapDataInfo[i][j].x = (tempReadData[1] << 8) + tempReadData[0];
      mapDataInfo[i][j].y = (tempReadData[3] << 8) + tempReadData[2];
      
      // map data info checking MAXIMUM and MINIMUM
      // How about minimum and maximum
      
      // Fill map
      
    }
    if((i % 10) == 0) {
      watchdog_trigger();
    }
  }
  
  for(i = 0; i < NUMBER_OF_CONTROL_PACKET; i++)
  {
    fram_read(NV_CONTROL_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempReadData);
    
    *(controlSettingsData[i].value) = (tempReadData[1] << 8) + tempReadData[0];

    check_control_data(FALSE, i, *(controlSettingsData[i].value));
    if((i % 10) == 0) {
      watchdog_trigger();
    }
  }
  
  read_dids();                                                                  // Added on 2023.02.23
  
  init_pointers();
 
  watchdog_trigger();
}

// The saving processes
uint8_t save_nvThreePLeverUpLimit(uint8_t _isSave, uint16_t _nvUpLimit)
{
  // 72 --> up limit
  return check_control_data(_isSave, NV_3P_LEVER_UP_LIMIT, _nvUpLimit);
}

uint8_t save_nvThreePLeverDownLimit(uint8_t _isSave, uint16_t _nvDownLimit)
{
  // 73 --> down limit
  return check_control_data(_isSave, NV_3P_LEVER_DOWN_LIMIT, _nvDownLimit);
}

uint8_t save_nvThreePPositionLimit(uint8_t _isSave, uint16_t _nvUpLimit, uint16_t _nvDownLimit)
{
  // Source address is 38 so 38 - 1 = 37 --> 74, 75 index
  check_control_data(_isSave, NV_3P_POSITION_UP_LIMIT, _nvUpLimit);
  return check_control_data(_isSave, NV_3P_POSITION_DOWN_LIMIT, _nvDownLimit);
}

uint8_t save_nvBalanceStroke(uint8_t _isSave, uint16_t _nvBalance, uint16_t _nvStroke)
{
  check_control_data(_isSave, NV_ROLLING_CENTER, _nvBalance);
  return check_control_data(_isSave, NV_STROKE_CENTER, _nvStroke);
}

uint8_t save_nvRpmCruiseA(uint8_t _isSave, uint16_t _newRpmCruise)
{
  return check_control_data(_isSave, NV_RPM_CRUISE_A, _newRpmCruise);
}

uint8_t save_nvRpmCruiseB(uint8_t _isSave, uint16_t _newRpmCruise)
{
  return check_control_data(_isSave, NV_RPM_CRUISE_B, _newRpmCruise);
}

uint8_t save_nvPanelData(uint8_t _isSave, uint16_t _newValue)
{
  return check_control_data(_isSave, NV_PANEL_DATA, _newValue);
}

uint16_t get_control_minimum_data(uint8_t index)
{
  if(index >= NUMBER_OF_CONTROL_PACKET)
    return 0xFFFF;
  return (controlSettingsData[index].minValue);
}

uint16_t get_control_maximum_data(uint8_t index)
{
  if(index >= NUMBER_OF_CONTROL_PACKET)
    return 0xFFFF;
  return (controlSettingsData[index].maxValue);
}

uint16_t get_control_default_data(uint8_t index)
{
  if(index >= NUMBER_OF_CONTROL_PACKET)
    return 0xFFFF;
  return (controlSettingsData[index].defaultValue);
}

uint8_t get_control_data(uint16_t did, uint16_t *data)
{
  uint16_t index;
  
  for(index = 0; index < NUMBER_OF_CONTROL_PACKET; index++)
  {
    if(did == controlSettingsData[index].didValue)
    {
      data[0] = *(controlSettingsData[index].value);
      return TRUE;
    }
  }
  
  return FALSE;
}

uint8_t check_control_data(uint8_t _isSave, uint8_t index, uint16_t data)
{      
  uint8_t isSave = FALSE;
  uint8_t tempWriteData[4];
  uint8_t i = index;
  
  if(_isSave == TRUE) {                          // Which means the data should be updated
    if(*(controlSettingsData[i].value) == data) {
      // there no update is needed because the saved data is NOT changed
      return TRUE;
    }
    
    isSave = TRUE;
    
    if((data <= controlSettingsData[i].maxValue) && (data >= controlSettingsData[i].minValue)) {
      *(controlSettingsData[i].value) = data;
    }
    else {
      isSave = FALSE;
    }
  }
  else {
    if((*(controlSettingsData[i].value) > controlSettingsData[i].maxValue) || (*(controlSettingsData[i].value) < controlSettingsData[i].minValue)) {
      *(controlSettingsData[i].value) = controlSettingsData[i].defaultValue;
      isSave = TRUE;
    }
  }
  
  if(isSave == TRUE) {
    tempWriteData[0] = (uint8_t)(*(controlSettingsData[i].value));
    tempWriteData[1] = (uint8_t)(*(controlSettingsData[i].value) >> 8);
    tempWriteData[2] = 0;
    tempWriteData[3] = 0;
    fram_write(NV_CONTROL_START_ADDRESS + (i * DIAGNOSTIC_DATA_SIZE), tempWriteData);
    return TRUE;
  }
  
  return FALSE;
}

uint8_t write_control_data(uint16_t did, uint16_t data)
{
  uint8_t tempWriteData[4];
  uint8_t index;
  
  for(index = 0; index < NUMBER_OF_CONTROL_PACKET; index++)
  {
    if(did == controlSettingsData[index].didValue) {
      if(*(controlSettingsData[index].value) == data) {
        return TRUE;
      }
      if((data <= controlSettingsData[index].maxValue) && (data >= controlSettingsData[index].minValue)) {
        *(controlSettingsData[index].value) = data;
        tempWriteData[0] = (uint8_t)(*(controlSettingsData[index].value));
        tempWriteData[1] = (uint8_t)(*(controlSettingsData[index].value) >> 8);
        tempWriteData[2] = 0;
        tempWriteData[3] = 0;
        
        if(index == NV_RPM_CRUISE_A)
        {
          nvRpmCruiseNewA = nvRpmCruiseA;
        }
        else if(index == NV_RPM_CRUISE_B)
        {
          nvRpmCruiseNewB = nvRpmCruiseB;
        }
        else if((index == NV_3P_LEVER_UP_LIMIT) || (index == NV_3P_LEVER_DOWN_LIMIT)) {
          lever_rate_calculate();
        }
        else if((index == NV_3P_POSITION_UP_LIMIT) || (index == NV_3P_POSITION_DOWN_LIMIT)) {
          position_rate_calculate();
        }
        
        fram_write(NV_CONTROL_START_ADDRESS + (index * DIAGNOSTIC_DATA_SIZE), tempWriteData);
        return TRUE;
      }
      else {
        return FALSE;
      }
    }
  }
  return FALSE;
}

uint8_t write_control_datas(uint16_t did, uint8_t *pData, uint16_t len)
{
  uint16_t temp;
  uint8_t i;
  uint8_t start;
  uint8_t end;
  
  if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_SHUTTLE_SETTING_DATA) {
    start = NV_SHUTTLE_PWM_FREQUENCY;
    end = NV_DIFF_AUTO_DISCONNECT_SPEED;
  }
  else if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_HITCH_SETTING_DATA) {
    start = NV_3P_LEVER_UP_LIMIT;
    end = NV_3P_STALL_STEP_DUTY;
  }
  else if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_BALANCE_SETTING_DATA) {
    start = NV_STROKE_RIGHT_LIMIT;
    end = NV_BALANCE_DEADBAND_LOW;
  }
  else if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_AUTO_BRAKE_SETTING_DATA) {
    start = NV_AD_LEFT_HIGH;
    end = NV_AD_NEUTRAL_RANGE;
  }
  else if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_PANEL_SETTING_DATA) {
    start = NV_PANEL_DATA;
    end = NV_SPEED_UNIT;
  }
  
  else {
    return FALSE;
  }
  
  if(len != (end - start))
  {
    return FALSE;
  }
   
  for(i = 0; i < len; i++)
  {
    temp = (pData[i * 2] << 8) + pData[i * 2 + 1];
    check_control_data(TRUE, start + i, temp);
    if((i % 5) == 0)
    {
        watchdog_trigger();
    }
  }
  return TRUE;
}

uint8_t get_control_datas(uint16_t did, uint8_t *pData, uint16_t *len)
{
  uint16_t temp;
  uint8_t i;
  uint8_t start;
  uint8_t end;
  
  if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_SHUTTLE_SETTING_DATA) {
    start = NV_SHUTTLE_PWM_FREQUENCY;
    end = NV_DIFF_AUTO_DISCONNECT_SPEED;
  }
  else if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_HITCH_SETTING_DATA) {
    start = NV_3P_LEVER_UP_LIMIT;
    end = NV_3P_STALL_STEP_DUTY;
  }
  else if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_BALANCE_SETTING_DATA) {
    start = NV_STROKE_RIGHT_LIMIT;
    end = NV_BALANCE_DEADBAND_LOW;
  }
  else if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_AUTO_BRAKE_SETTING_DATA) {
    start = NV_AD_LEFT_HIGH;
    end = NV_AD_NEUTRAL_RANGE;
  }
  else if(did == PUDS_SVC_PARAM_DI_M_BLOCK_OF_PANEL_SETTING_DATA) {
    start = NV_PANEL_DATA;
    end = NV_SPEED_UNIT;
  }
  else {
    return FALSE;
  }
  
  *len = ((end - start) * 2);
  
  for(i = start; i <= end; i++)
  {
    temp = *(controlSettingsData[i].value);
    pData[i * 2] = (uint8_t)(temp >> 8);
    pData[i * 2 + 1] = (uint8_t)(temp);
    if((i % 5) == 0)
    {
      watchdog_trigger();
    }
  }
  return TRUE;
}

uint8_t write_map_data_to_fram(uint8_t mapIndex, uint8_t mapDataIndex)
{
  uint8_t tempWriteData[4];
  
  tempWriteData[0] = mapDataInfo[mapIndex][mapDataIndex].x;
  tempWriteData[1] = mapDataInfo[mapIndex][mapDataIndex].x >> 8;
  tempWriteData[2] = mapDataInfo[mapIndex][mapDataIndex].y;
  tempWriteData[3] = mapDataInfo[mapIndex][mapDataIndex].y >> 8;
  fram_write(NV_MAP_POINT_START_ADDRESS + ((mapIndex * NUMBER_OF_MAP_POINT + mapDataIndex) * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  
  printf("mapDataIndex = %d\r\n", mapDataIndex);
  
  if(mapIndex < 20) {
    isUpdateHappenedMapData[mapIndex] = TRUE;
  }
  else if(mapIndex == 25) {
    isUpdateHappenedMapData[20] = TRUE;
  }
  else if(mapIndex == 26) {
    isUpdateHappenedMapData[21] = TRUE;
  }
  else if(mapIndex == 27) {
    isUpdateHappenedMapData[22] = TRUE;
  }
  else if(mapIndex == 28) {
    isUpdateHappenedMapData[23] = TRUE;
  }
  
  return TRUE;
}

uint8_t write_starter_map_number_to_fram(uint8_t mapNumberIndex)
{
  uint8_t tempWriteData[4];
  
  tempWriteData[0] = mapNumberInfo[mapNumberIndex].hl_map;
  tempWriteData[1] = mapNumberInfo[mapNumberIndex].hl_fill;
  tempWriteData[2] = mapNumberInfo[mapNumberIndex].cm_map;
  tempWriteData[3] = mapNumberInfo[mapNumberIndex].cm_fill;
  fram_write(NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS + (mapNumberIndex * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  return TRUE;
}

uint8_t write_transmission_map_number_to_fram(uint8_t mapNumberIndex)
{
  uint8_t tempWriteData[4];
  
  tempWriteData[0] = mapNumberShiftInfo[mapNumberIndex].hl_map;
  tempWriteData[1] = mapNumberShiftInfo[mapNumberIndex].hl_fill;
  tempWriteData[2] = mapNumberShiftInfo[mapNumberIndex].cm_map;
  tempWriteData[3] = mapNumberShiftInfo[mapNumberIndex].cm_fill;
  fram_write(NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS + (mapNumberIndex * DIAGNOSTIC_DATA_SIZE), tempWriteData);
  return TRUE;
}