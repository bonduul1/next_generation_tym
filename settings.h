#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include "stm32f1xx_hal.h"


/*----------------------------------------------------------------------------
 * Calculating memory
 * NV_CONTROL_START_ADDRESS = 4000 ==> 55 * 4
 * 64Kbit = 8KBytes
 * 8 * 1024 Bytes = 8192 Bytes =>
 * 32Bit data ==> 8192 Bytes / (read/write length = 4) = 2048 (32bit data)
 * 16Kbit = 2KBytes
 *
 *
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
#define NV_MAP_NUMBER_TRANSMISSION_START_ADDRESS        0                       // 0 + (24 * 4 ) = 0 + 96 = 96
#define NV_MAP_NUMBER_TRANSMISSION_SHIFT_ADDRESS        100                     // 100 + (48 * 4 ) = 100 + 192 = 292
#define NV_MAP_NUMBER_TRANSMISSION_TRAIN_ADDRESS        300                     // 300 + (32 * 4 ) = 300 + 128 = 428
#define NV_MAP_POINT_START_ADDRESS                      500                     // 500 + (100 * 8 * 4) = 3700                           // Maximum 100 map data
#define NV_CONTROL_START_ADDRESS                        4000                    // 4000 + (200 * 4) = 4800                              // Maximum 200 data
#define NV_START_ADDRESS_OF_UDS_STANDARD_DID            5000                    // 5000 + (200 * 4) = 5800                              // Maximum 200 data
#define NV_START_ADDRESS_OF_DTC_STATE_COUNTER           6000                    // 6000 + (200 * 4) = 6800                              // Maximum 200 data
#define NV_START_ADDRESS_OF_DTC_TIME                    7000                    // 7000 + (200 * 4) = 7800                              // Maximum 200 data


#define DIAGNOSTIC_DATA_SIZE                            4
/*----------------------------------------------------------------------------*/

/*--------------------------------------------------------- MAP number typedef and Variables Start ---------------------------------------------------*/
typedef union {
  uint32_t data;
  struct {
    uint8_t hl_map;
    uint8_t hl_fill;
    uint8_t cm_map;
    uint8_t cm_fill;
  } ;
} transmissionFillMapNumberState_t;

typedef union {
  uint32_t data;
  struct {
    uint8_t low_map;
    uint8_t low_fill;
    uint8_t high_map;                                                             // Updated on 2022.04.22
    uint8_t high_fill;                                                            // Updated on 2022.04.22
  } ;
} trainFillMapNumberState_t;

typedef union {
  uint32_t data;
  struct {
    uint16_t x;
    uint16_t y;
  } ;
} transmissionMapDataState_t;

#define NUMBER_OF_MAP_STAGE_NUMBER                      24                      // 4 state, off maps
#define NUMBER_OF_MAP_STAGE_SHIFT_NUMBER                48                      // 12 state shift maps
#define NUMBER_OF_MAP_STAGE_TRAIN_NUMBER                32                      // 4 state, C,L,M,H,High,Low (4 (C,L,M,H) * 4 (1,2,3,4) = 16*2(low mode, high mode)) --> High, Low map is included in structure (32 bit)
#define NUMBER_OF_MAP_NUMBER                            90                      // Changed on 2023.04.03
#define NUMBER_OF_MAP_POINT                             8
#define NUMBER_OF_MAP_USED_FOR_OTHER_PFUNCTION          30

#define FILL_NUMBER_TEMPERATURE_CURRENT_FIRST           1
#define FILL_NUMBER_TEMPERATURE_TIME_FIRST              2
#define FILL_NUMBER_TEMPERATURE_CURRENT_SECOND          3
#define FILL_NUMBER_TEMPERATURE_TIME_SECOND             4
#define MAP_NUMBER_FORWARD_SHUTTLE_COMPENSATION         5
#define MAP_NUMBER_BACKWARD_SHUTTLE_COMPENSATION        6
#define MAP_NUMBER_DRAG_ON                              7
#define MAP_NUMBER_DRAG_OFF                             8
#define MAP_NUMBER_PTO_ON                               9
#define MAP_NUMBER_PTO_OFF                              10
#define MAP_NUMBER_THREEP_UP                            11
#define MAP_NUMBER_THREEP_DOWN                          12
#define MAP_NUMBER_BALANCE_UP                           13
#define MAP_NUMBER_BALANCE_DOWN                         14
#define FILL_NUMBER_CLUTCH_FORWARD                      15                      // Forward fill clutch current & time
#define FILL_NUMBER_CLUTCH_BACKWARD                     16                      // Backward fill clutch current & time
#define MAP_NUMBER_CLUTCH_FORWARD_POSITION              17                      // Forward map clutch current & sensor position
#define MAP_NUMBER_CLUTCH_BACKWARD_POSITION             18                      // Backward map clutch current & sensor position
#define MAP_NUMBER_PTO_TEMPERATURE_DUTY                 19                      // Added on 2025.07.17

#define FILL_NUMBER_START_TRANSMISSION                  20                      // Changed on 2021.10.11
#define FILL_NUMBER_END_TRANSMISSION                    25
#define FILL_NUMBER_DEFAULT_TRANSMISSION                20

#define MAP_NUMBER_THREEP_DRAFT_UP                      26
#define MAP_NUMBER_THREEP_DRAFT_DOWN                    27
#define MAP_NUMBER_THREEP_DEPTH_UP                      28
#define MAP_NUMBER_THREEP_DEPTH_DOWN                    29

#define MAP_NUMBER_START_TRANSMISSION                   30                      // Changed on 2021.10.11
#define MAP_NUMBER_END_TRANSMISSION                     45
#define MAP_NUMBER_DEFAULT_TRANSMISSION                 30

#define MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION              46

extern transmissionFillMapNumberState_t                 mapNumberInfo[NUMBER_OF_MAP_STAGE_NUMBER];
extern transmissionFillMapNumberState_t                 mapNumberShiftInfo[NUMBER_OF_MAP_STAGE_SHIFT_NUMBER];
extern trainFillMapNumberState_t                        mapNumberTrainInfo[NUMBER_OF_MAP_STAGE_TRAIN_NUMBER];
extern transmissionMapDataState_t                       mapDataInfo[NUMBER_OF_MAP_NUMBER][NUMBER_OF_MAP_POINT];

/*--------------------------------------------------------- MAP number typedef and Variables End -----------------------------------------------------*/
extern int16_t *nvFillCurrentFirst;
extern int16_t *nvFillCurrentTemperatureFirst;
extern int16_t *nvFillTimeFirst;
extern int16_t *nvFillTimeTemperatureFirst;
  
extern int16_t *nvFillCurrentSecond;
extern int16_t *nvFillCurrentTemperatureSecond;
extern int16_t *nvFillTimeSecond;
extern int16_t *nvFillTimeTemperatureSecond;

extern int16_t *nvForwardShuttleCompensationCurrent;
extern int16_t *nvForwardShuttleCompensationTemp;
extern int16_t *nvBackwardShuttleCompensationCurrent;
extern int16_t *nvBackwardShuttleCompensationTemp;

extern int16_t *nvDragOnCurrent;
extern int16_t *nvDragOnTime;
extern int16_t *nvDragOffCurrent;
extern int16_t *nvDragOffTime;

extern int16_t *nvPtoOnCurrentPercent;
extern int16_t *nvPtoOnTime;
extern int16_t *nvPtoOffCurrentPercent;
extern int16_t *nvPtoOffTime;

extern int16_t *nvClutchMapCurrent;
extern int16_t *nvClutchMapTime;

extern int16_t *nvClutchForwardFillCurrent;
extern int16_t *nvClutchForwardFillTime;
extern int16_t *nvClutchBackwardFillCurrent;
extern int16_t *nvClutchBackwardFillTime;

extern int16_t *nvClutchForwardPositionCurrent;
extern int16_t *nvClutchForwardPositionSensor;
extern int16_t *nvClutchBackwardPositionCurrent;
extern int16_t *nvClutchBackwardPositionSensor;

extern int16_t *nvThreePUpCurrent;
extern int16_t *nvThreePUpPosition;
extern int16_t *nvThreePDownCurrent;
extern int16_t *nvThreePDownPosition;

extern int16_t *nvBalanceUpCurrent;
extern int16_t *nvBalanceUpPosition;
extern int16_t *nvBalanceDownCurrent;
extern int16_t *nvBalanceDownPosition;

extern int16_t *nvThreePDraftUpCurrent;
extern int16_t *nvThreePDraftUpPosition;
extern int16_t *nvThreePDraftDownCurrent;
extern int16_t *nvThreePDraftDownPosition;

extern int16_t *nvThreePDepthUpCurrent;
extern int16_t *nvThreePDepthUpPosition;
extern int16_t *nvThreePDepthDownCurrent;
extern int16_t *nvThreePDepthDownPosition;

extern int16_t *nvPTOTemperatureCompensationDuty;
extern int16_t *nvPTOTemperatureCompensationDutyTemperature;

extern int16_t  nvX[NUMBER_OF_MAP_USED_FOR_OTHER_PFUNCTION][NUMBER_OF_MAP_POINT];       // First 24 map data is copied into this two dimensional array
extern int16_t  nvY[NUMBER_OF_MAP_USED_FOR_OTHER_PFUNCTION][NUMBER_OF_MAP_POINT];       // First 24 map data is copied into this two dimensional array
extern uint8_t  isUpdateHappenedMapData[NUMBER_OF_MAP_USED_FOR_OTHER_PFUNCTION];        // If update is occured in the map data this flag should be TRUE
extern uint8_t  isUpdateHappenedMapNumber;                                              // If update is occured in the map number (transmission) this flag should be TRUE

extern int16_t  nvTransmissionFillTime[NUMBER_OF_MAP_POINT];                            // There is no need pointer
extern int16_t  nvTransmissionFillCurrent[NUMBER_OF_MAP_POINT];
extern int16_t  nvTransmissionMapTime[NUMBER_OF_MAP_POINT];
extern int16_t  nvTransmissionMapCurrent[NUMBER_OF_MAP_POINT];
extern int16_t  nvTransmissionClutchToShuttleTime[NUMBER_OF_MAP_POINT];
extern int16_t  nvTransmissionClutchToShuttleCurrent[NUMBER_OF_MAP_POINT];

/*--------------------------------------------------------- Control command typedef and Variables Start ----------------------------------------------*/
enum {
  NV_SHUTTLE_PWM_FREQUENCY = 0,
  NV_SHUTTLE_DITHER_FREQUENCY,
  NV_SHUTTLE_DITHER_CURRENT,
  NV_SHUTTLE_SENSITIVE_CURRENT_MAX,
  NV_SHUTTLE_SENSITIVE_CURRENT_MID,
  NV_SHUTTLE_SENSITIVE_CURRENT_MIN,
  NV_FORWARD_REVERSE_SWITCHING_SPEED,
  NV_FORWARD_REVERSE_SWITCHING_TIME,
  NV_CLUTCH_PEDAL_PUSH_TIME,
  NV_CLUTCH_PEDAL_FREE_TIME,
  NV_ENGINE_LOAD,
  NV_PTO_HIGHER_POSITION,
  NV_PTO_MEDIUM_POSITION,
  NV_PTO_LOWER_POSITION,
  NV_RPM_CRUISE_A,
  NV_DRAG_ON_TEMPERATURE,
  NV_DRAG_OFF_TEMPERATURE,
  NV_DRAG_AUTO_CONNECT_SPEED,
  NV_DRAG_AUTO_DISCONNECT_SPEED,
  NV_4WD_AUTO_CONNECT_SPEED,
  NV_4WD_AUTO_DISCONNECT_SPEED,
  NV_QT_AUTO_CONNECT_SPEED,
  NV_QT_AUTO_DISCONNECT_SPEED,
  NV_DIFF_AUTO_CONNECT_SPEED,
  NV_DIFF_AUTO_DISCONNECT_SPEED,
  NV_RPM_CRUISE_B,
  NV_STEERING_RIGHT,
  NV_STEERING_LEFT,
  NV_CLUTCH_GAP,
  NV_FORWARD_BASE_CURRENT,
  NV_REVERSE_BASE_CURRENT,
  NV_PTO_ON_WHEN_3P_DOWN,
  NV_3P_DOWN_WHEN_TURN_BACK_UP_AUTO_ON,
  
  NV_3P_LEVER_UP_LIMIT,
  NV_3P_LEVER_DOWN_LIMIT,
  NV_3P_POSITION_UP_LIMIT,
  NV_3P_POSITION_DOWN_LIMIT,
  NV_3P_MANUAL_UP_MIN_DUTY,
  NV_3P_MANUAL_DOWN_MIN_DUTY,
  NV_3P_MANUAL_UP_MAX_DUTY,
  NV_3P_MANUAL_DOWN_MAX_DUTY,
  NV_3P_MANUAL_UP_CYCLE,
  NV_3P_MANUAL_DOWN_CYCLE,
  NV_3P_AUTO_UP_MIN_DUTY,
  NV_3P_AUTO_DOWN_MIN_DUTY,
  NV_3P_AUTO_UP_MID_DUTY,
  NV_3P_AUTO_DOWN_MID_DUTY,
  NV_3P_AUTO_UP_MAX_DUTY,
  NV_3P_AUTO_DOWN_MAX_DUTY,
  NV_3P_SETTING_UP_DUTY,
  NV_3P_SETTING_DOWN_DUTY,
  NV_3P_UP_STEP_DUTY,
  NV_3P_DOWN_STEP_DUTY,
  NV_3P_QUICK_UP_STEP_DUTY,
  NV_3P_QUICK_DOWN_STEP_DUTY,
  NV_3P_QUICK_SPEED_DUTY,
  NV_3P_STALL_STEP_DUTY,
  
  NV_3P_DEPTH_1,
  NV_3P_DEPTH_2,
  NV_3P_DEPTH_3,
  NV_3P_DEPTH_4,

  NV_STROKE_RIGHT_LIMIT,
  NV_STROKE_LEFT_LIMIT,
  NV_STROKE_CENTER,
  NV_ROLLING_CENTER,
  NV_BALANCE_SAMPLING_TIME_HIGH,
  NV_BALANCE_SAMPLING_TIME_MID,
  NV_BALANCE_SAMPLING_TIME_LOW,
  NV_BALANCE_DEADBAND_HIGH,
  NV_BALANCE_DEADBAND_MID,
  NV_BALANCE_DEADBAND_LOW,

  NV_AD_LEFT_HIGH,
  NV_AD_LEFT_MID,
  NV_AD_LEFT_LOW,
  NV_AD_RIGHT_HIGH,
  NV_AD_RIGHT_MID,
  NV_AD_RIGHT_LOW,
  NV_AD_DUTY_HIGH,
  NV_AD_DUTY_MID,
  NV_AD_DUTY_LOW,
  NV_AD_AUTO_CONNECT_SPEED,
  NV_AD_AUTO_DISCONNECT_SPEED,
  NV_AD_ON_TIME,
  NV_AD_OFF_TIME,
  NV_AD_DEAD_BAND,
  NV_AD_PWM_DEAD_BAND,
  NV_AD_NEUTRAL_RANGE,
  
  NV_PANEL_DATA,
  NV_SPEED_UNIT,
  
  NUMBER_OF_CONTROL_PACKET
};

// NV Control data
  // Shuttle & drive control
extern uint16_t nvPwmFrequency;
extern uint16_t nvDitherFrequency;
extern uint16_t nvDitherCurrent;
extern uint16_t nvShuttleSensetiveUpLimit;
extern uint16_t nvShuttleSensetiveMiddleLimit;
extern uint16_t nvShuttleSensetiveDownLimit;
extern uint16_t nvForwardReverseSwitchingSpeed;
extern uint16_t nvForwardReverseSwitchingTime;
extern uint16_t nvClutchPedalPushTime;
extern uint16_t nvClutchPedalFreeTime;
extern uint16_t nvEngineLoad;
extern uint16_t nvPtoHigher3Pposition;
extern uint16_t nvPtoMedium3Pposition;
extern uint16_t nvPtoLower3Pposition;
extern uint16_t nvRpmCruiseA;
extern uint16_t nvDragOnTemperature;
extern uint16_t nvDragOffTemperature;
extern uint16_t nvDragAutoConnectSpeed;
extern uint16_t nvDragAutoDisconnectSpeed;
extern uint16_t nv4WdAutoConnectSpeed;
extern uint16_t nv4WdAutoDisconnectSpeed;
extern uint16_t nvQtAutoConnectSpeed;
extern uint16_t nvQtAutoDisconnectSpeed;
extern uint16_t nvDiffAutoConnectSpeed;
extern uint16_t nvDiffAutoDisconnectSpeed;
extern uint16_t nvRpmCruiseB;
extern uint16_t nvSteeringRight;
extern uint16_t nvSteeringLeft;
extern uint16_t nvClutchGap;
extern uint16_t nvForwardBaseCurrent;
extern uint16_t nvBackwardBaseCurrent;
extern uint16_t nvPtoOnWhenHitchDown;
extern uint16_t nvHitchDownWhenTurnBackUpOn;

  // Hitch control part
extern uint16_t nv3pLeverUpLimit;
extern uint16_t nv3pLeverDownLimit;
extern uint16_t nv3pPositionUpLimit;
extern uint16_t nv3pPositionDownLimit;
extern uint16_t nv3pManualUpMinDuty;
extern uint16_t nv3pManualDownMinDuty;
extern uint16_t nv3pManualUpMaxDuty;
extern uint16_t nv3pManualDownMaxDuty;
extern uint16_t nv3pManualUpCycle;
extern uint16_t nv3pManualDownCycle;
extern uint16_t nv3pAutoUpMinDuty;
extern uint16_t nv3pAutoDownMinDuty;
extern uint16_t nv3pAutoUpMidDuty;
extern uint16_t nv3pAutoDownMidDuty;
extern uint16_t nv3pAutoUpMaxDuty;
extern uint16_t nv3pAutoDownMaxDuty;
extern uint16_t nv3pSettingUpDuty;
extern uint16_t nv3pSettingDownDuty;
extern uint16_t nv3pUpStepDuty;
extern uint16_t nv3pDownStepDuty;
extern uint16_t nv3pQuickUpStepDuty;
extern uint16_t nv3pQuickDownStepDuty;
extern uint16_t nv3pQuickDownDuty;
extern uint16_t nv3pStallStepDuty;

extern uint16_t nvDepthSensorSamplingTime;
extern uint16_t nvDepthTime;
extern uint16_t nvDepthSwitchOutputTime;
extern uint16_t nvDepthDownLimit;

extern uint16_t nvStrokeRightLimit;
extern uint16_t nvStrokeLeftLimit;
extern uint16_t nvStrokeCenter;
extern uint16_t nvRollingCenter;
extern uint16_t nvBalanceSamplingTimeHigh;
extern uint16_t nvBalanceSamplingTimeMid;
extern uint16_t nvBalanceSamplingTimeLow;
extern uint16_t nvBalanceDeadbandHigh;
extern uint16_t nvBalanceDeadbandMid;
extern uint16_t nvBalanceDeadbandLow;

extern uint16_t nvAdLeftHigh;
extern uint16_t nvAdLeftMid;
extern uint16_t nvAdLeftLow;
extern uint16_t nvAdRightHigh;
extern uint16_t nvAdRightMid;
extern uint16_t nvAdRightLow;
extern uint16_t nvAdDutyHigh;
extern uint16_t nvAdDutyMid;
extern uint16_t nvAdDutyLow;
extern uint16_t nvAdAutoConnectSpeed;
extern uint16_t nvAdAutoDisconnectSpeed;
extern uint16_t nvAdOnTime;
extern uint16_t nvAdOffTime;
extern uint16_t nvAdDeadBand;
extern uint16_t nvAdPWMDeadBand;
extern uint16_t nvAdNeutralRange;

extern uint16_t nvPanelData;
extern uint16_t nvSpeedUnit;

typedef struct  {
   const uint16_t didValue;
   const uint16_t minValue;
   const uint16_t maxValue;
   const uint16_t defaultValue;
   uint16_t       *value;
} nvData_t;

/*--------------------------------------------------------- Control command typedef and Variables End -------------------------------------------------*/

/*----------------------------------------------------------------------------*/
void init_diagnostic_variables();

uint8_t save_nvThreePLeverUpLimit(uint8_t _isSave, uint16_t _nvUpLimit);
uint8_t save_nvThreePLeverDownLimit(uint8_t _isSave, uint16_t _nvDownLimit);

uint8_t save_nvThreePPositionLimit(uint8_t _isSave, uint16_t _nvUpLimit, uint16_t _nvDownLimit);
uint8_t save_nvBalanceStroke(uint8_t _isSave, uint16_t _nvBalance, uint16_t _nvStroke);
uint8_t save_nvRpmCruiseA(uint8_t _isSave, uint16_t _newRpmCruise);
uint8_t save_nvRpmCruiseB(uint8_t _isSave, uint16_t _newRpmCruise);

uint8_t save_nvPanelData(uint8_t _isSave, uint16_t _newValue);

uint16_t get_control_minimum_data(uint8_t index);
uint16_t get_control_maximum_data(uint8_t index);
uint16_t get_control_default_data(uint8_t index);

uint8_t get_control_data(uint16_t did, uint16_t *data);
uint8_t get_control_datas(uint16_t did, uint8_t *pData, uint16_t *len);
uint8_t write_control_data(uint16_t did, uint16_t data);
uint8_t write_control_datas(uint16_t did, uint8_t *pData, uint16_t len);
uint8_t check_control_data(uint8_t _isSave, uint8_t index, uint16_t data);

uint8_t write_map_data_to_fram(uint8_t mapIndex, uint8_t mapDataIndex);
uint8_t write_transmission_map_number_to_fram(uint8_t mapNumberIndex);
uint8_t write_starter_map_number_to_fram(uint8_t mapNumberIndex);

void set_default_setting(uint8_t isForce);
uint8_t clutch_setting();

#endif

