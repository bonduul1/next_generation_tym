
#include "shuttle.h"
#include "main.h"
#include "defines.h"
#include "can.h"
#include "input.h"
#include "output.h"
#include "sensors.h"
#include "algorithm.h"
#include "settings.h"

/*
 * In this algorithm, there is 4 mode we used
 *      1. SHUTTLE MODE CONTROL
 *              In this mode, we should control the output based on CLUTCH sensor data & clutch MAP data (Position and current data)
 *      2. SHUTTLE MODE FREE
 *              In this mode, we should control the output based on SHUTTLE MAP data (TIME and current data)
 *      3. SHUTTLE MODE OFF
 *              In this mode, the output is zero
 *      4. SHUTTLE MODE CHANGE
 *              In this mode, the transmission is shift is happened so the output is controlled by TRANSMISSION SHIFT MAP data (Time and current)
 *      5. SHUTTLE MODE IMPULSE
 *              In this mode, the fill current output is happened.
 *
 *
 *
 *
 *      In this algorithm, we use several flags for changing the mode and implement the corresponding calculation.
 *              1. ERROR Flag - Clutch (PEDAL) and shuttle (FORWARD, BACKWARD, and NEUTRAL)
 *                      a. If the shuttle sensor has an error (SHORT OR OPEN), then the forward and backward output is never controlled again (reset the device)
 *                      b. If the clutch sensor has an error, the output should immediately stopped.
 *
 *
 *
 */

#define USED_POINTS_IN_TEMPERATURE_FILL         8
#define USED_POINTS_IN_FILL                     5
#define USED_POINTS_IN_SHUTTLE                  8
#define USED_POINTS_IN_CLUTCH                   8
#define USED_POINTS_IN_COMPENSATION             8
/************************************** Local Variables of Mode  ************************/
uint8_t         shuttleMode;
uint8_t         shuttleModePrevious;

/************************************** Local Variables of Other ************************/
int16_t         finalCurrent;                                                   // The calculated final current is saved in here
float           temperatureFillCurrentPercent;                                  // The calculated FILL Temperature current percent is saved in here
float           temperatureFillTimePercent;                                     // The calculated FILL Temperature time percent is saved in here
int16_t         fillCurrent;                                                    // The calculated FILL current is saved in here
int16_t         clutchCurrent;                                                  // If clutch sensor is using, the calculated current is saved in here
int16_t         shuttleCurrent;                                                 // If clutch sensor is NOT using, the calculated current is saved in here

uint8_t         direction;
uint8_t         previousDirection;
uint8_t         previousDirectionFast;

uint16_t        clutchSensorValue;
uint16_t        previousClutchSensorValue;

uint16_t        timeFill;
uint16_t        timeShuttle;
uint16_t        timeClutch;

uint8_t         isUpdatedStartIndexOfShuttleCurrent;

#define TRACTOR_SHIFT_SPEED             30                                      // it means 3 km/h
uint8_t fillNumber;
uint8_t mapNumber;
uint8_t mapArrayIndex;
uint8_t isStartOrShift;

/************************************** Global Functions ********************************/
uint16_t        get_finalCurrent()    { return finalCurrent;    }

void shuttle_init()
{
  shuttleModePrevious = VAC_SHUTTLE_MODE_OFF;
  shuttleMode = VAC_SHUTTLE_MODE_OFF;
  timeFill = 0;
  timeShuttle = 0;
  timeClutch = 0;
  isUpdatedStartIndexOfShuttleCurrent = TRUE;
  flag.fillImplement = TRUE;
}

void update_direction() 
{
//Disabled on 2021.12.07  static uint8_t buzzerCheckWithSpeed = 0;
  static int16_t timerDelayBackward = 0;
  static int16_t timerDelayForward = 0;
  static uint8_t isShiftingStateNormal = 0;
  static uint16_t timerAutoStopShuttle = 0;
  static uint8_t isAutoStopBuzzerContinue = FALSE;
  
  flagOutputLamp.neutral = OFF;
  flagOutputLamp.backward = OFF;
  flagOutputLamp.forward = OFF;

  if(flagShuttle.forward == ON) {                                         // We have using VR for forward, backward and neutral state
    flagOutputLamp.forward = ON;
    if(timerDelayBackward != 0)
    {
      if(flagTimer.tenMs == TRUE)
      {
        timerDelayBackward -= 10;
        if(timerDelayBackward < 10) {
          timerDelayBackward = 0;
        }
      }
      direction = VAC_NEUTRAL;
      timerDelayForward = 0;
    }
    else {
      timerDelayForward = nvForwardReverseSwitchingTime;
      direction = VAC_FORWARD;
    }
  }
  else if(flagShuttle.backward == ON)
  {
    flagOutputLamp.backward = ON;
    if(timerDelayForward != 0)
    {
      if(flagTimer.tenMs == TRUE)
      {
        timerDelayForward -= 10;
        if(timerDelayForward < 0)
        {
          timerDelayForward = 0;
        }
      }
      direction = VAC_NEUTRAL;
      timerDelayBackward = 0;
    }
    else
    {
      timerDelayBackward = nvForwardReverseSwitchingTime;
      direction = VAC_BACKWARD;
    }
  }
  else
  {
//Disabled on 2021.12.07    buzzerCheckWithSpeed = 0;
    flagOutputLamp.neutral = ON;
    direction = VAC_NEUTRAL;
    flag.warningTransmission = FALSE;
    if(flagTimer.tenMs == TRUE)
    {
      if(timerDelayForward != 0)
      {
        timerDelayForward -= 10;
        if(timerDelayForward < 0)
        {
          timerDelayForward = 0;
        }
      }
      if(timerDelayBackward != 0)
      {
        timerDelayBackward -= 10;
        if(timerDelayBackward < 0)
        {
          timerDelayBackward = 0;
        }
      }
    }
  }
  
  if(flagInputStatus.transmit == ON) {
    direction = VAC_NEUTRAL;
  }
  
  if(flag.pedalPush == TRUE) {                                            // Added on 2022.02.04
    direction = VAC_NEUTRAL;
  }
  
  if(flagInputStatus.parkingBrake == TRUE) {                              // Added on 2022.09.22
    direction = VAC_NEUTRAL;
    if((flagShuttle.backward == ON) || (flagShuttle.forward == ON)) {
      flag.checkBuzzer = TRUE;
    }
  }
  
  if(canFlag.autoStop == TRUE)
  {
    if(((flagInputStatus.transmissionFour == ON) && (flagInputStatus.transmissionM == ON)) || 
       (flagInputStatus.transmissionH == ON) || (flagInputStatus.sideBrake == ON))
    {
      timerAutoStopShuttle += 2;
      if(timerAutoStopShuttle >= 500)
      {
        timerAutoStopShuttle = 500;
        flag.checkBuzzer = TRUE;
        canFlag.autoStop = FALSE;
      }

      if((flagShuttle.backward == ON) || (flagShuttle.forward == ON))
      {
        direction = VAC_NEUTRAL;
        if(flagInputStatus.sideBrake == OFF)
        {
          isAutoStopBuzzerContinue = TRUE;
        }
      }
    }
    else
    {
      timerAutoStopShuttle = 0;
    }
    
    if((flagInputStatus.sideBrake == OFF) && (flagInputStatus.brake == ON))
    {
      direction = VAC_NEUTRAL;
      isShiftingStateNormal = 1;        // Added on 2024.12.16
    }
  }
  else
  {
    if(isAutoStopBuzzerContinue == TRUE)
    {
      direction = VAC_NEUTRAL;
      flag.buzzerOn = TRUE;
      if(flagShuttle.neutral == TRUE)
      {
        isAutoStopBuzzerContinue = FALSE;
      }
    }
    timerAutoStopShuttle = 0;
//    armRestLedFlash1 &= ~VB1_AUTO_STOP_MODE;
  }
  
  // the following condition is added on 2022.10.05
  if(((flagInputStatus.transmissionThree == ON) || (flagInputStatus.transmissionFour == ON)) && (flagInputStatus.transmissionH == ON) && (isStartOrShift == 2)) {
    direction = VAC_NEUTRAL;
    if((flagShuttle.backward == ON) || (flagShuttle.forward == ON)) {
      flag.checkBuzzer = TRUE;
    }
  }
  
  // The following condition is added on 2022.01.07
  if((flagInputStatus.transmissionOne == OFF) && (flagInputStatus.transmissionTwo == OFF) && (flagInputStatus.transmissionThree == OFF) && (flagInputStatus.transmissionFour == OFF)) {
    direction = VAC_NEUTRAL;
    
    if((flag.pedalPush == TRUE) || (flagInputStatus.transmit == ON) || (flagShuttle.neutral == ON) ||
       ((canFlag.autoStop == TRUE) && (get_average_speed() < 2) && (flagInputStatus.brake == ON))) {
      isShiftingStateNormal = 1;
    }
    else {
      isShiftingStateNormal = 0;
    }
  }
  else {
    if(direction != VAC_NEUTRAL) {
      if(isShiftingStateNormal == 0) {
        // Error is occured
        flag.checkBuzzer = TRUE;                                                                                              // Changed on 2021.12.13
        direction = VAC_NEUTRAL;
// Disabled on 2022.05.23        flag.warningTransmission = TRUE;
      }
    }
    else {                                                                      // Added this case on 2021.01.11
      // direction is neutral,
      if((flag.pedalPush == TRUE) || (flagInputStatus.transmit == ON) || (flagShuttle.neutral == ON)) {
        isShiftingStateNormal = 1;
      }
    }
  }
   
  // if tractor speed is greateer than 7km/h and currently moving to forward or backward
  if(nvForwardReverseSwitchingSpeed <= get_average_speed()) {
    if(direction != VAC_NEUTRAL) {
      if(previousDirectionFast != direction)  {
        flag.checkBuzzer = TRUE;                                                                                              // Changed on 2021.12.13
        direction = VAC_NEUTRAL;
        flag.warningTransmission = TRUE;
      }
      else {
        previousDirectionFast = direction;
      }
    }
  }
  else {
    previousDirectionFast = direction;
  }

  if((direction == VAC_FORWARD) && (flag.warningTransmission == FALSE)) {
    flagOutputControl.forward = ON;
    flagOutputControl.backward = OFF;
  }
  else if((direction == VAC_BACKWARD) && (flag.warningTransmission == FALSE)) {
    flagOutputControl.forward = OFF;
    flagOutputControl.backward = ON;
  }
  else {
    flagOutputControl.forward = OFF;
    flagOutputControl.backward = OFF;
  }

  previousDirection = direction;
}

uint16_t timerClutchSafety = 10000;
uint16_t timerClutchSafetyReset = 10000;
uint16_t clutchSwitchingFast = 0;
int16_t waitingTime = 0;


void pedal_checking(uint8_t isTransmissionHappened) {
  static uint16_t timerClutchPush = 0;
  uint16_t freePosition;
  uint16_t pushPosition;
  pushPosition= (direction == VAC_BACKWARD) ? nvClutchBackwardPositionSensor[0]                                         // Not it is 10 bit
                                            : nvClutchForwardPositionSensor[0];
  freePosition = (direction == VAC_BACKWARD)? nvClutchBackwardPositionSensor[USED_POINTS_IN_CLUTCH - 1] 
                                            : nvClutchForwardPositionSensor[USED_POINTS_IN_CLUTCH - 1];
                                                
  clutchSensorValue = get_clutch_sensor();
   

  timerClutchSafety += 2;
  if(timerClutchSafety >= nvClutchPedalPushTime) {
    timerClutchSafety = nvClutchPedalPushTime;
  }
  
  timerClutchPush += 2;
  if(timerClutchPush >= 10000) {
    timerClutchPush = 10000;
  }
  
  if(clutchSensorValue >= (pushPosition + 8)) {                               // NOT pushed case
    
    if(flag.pedalPush == TRUE) {                                          // This case is added on 2021.11.28 by Park team leader
      if(timerClutchPush >= nvClutchPedalFreeTime) {
        flag.pedalPush = FALSE;
      }
    }
    
    if(clutchSwitchingFast == 2) {
      flag.fillImplement = TRUE;
      timerClutchSafety = 0;
    }
    clutchSwitchingFast = 1;
  }
  else if(clutchSensorValue < pushPosition) {
    clutchSwitchingFast = 0;
    if(timerClutchSafety >= nvClutchPedalPushTime) {
      clutchSwitchingFast = 2;
      flag.fillImplement = TRUE;
    }
    // transmission is happened (H, L, M, 1, 2, 3, 4) and shuttle direction is changed.
    if(isTransmissionHappened == ON) {
      flag.fillImplement = TRUE;
    }
    flag.pedalPush = TRUE;
  }
  
  if(flag.pedalPush == FALSE) {
    timerClutchPush = 0;
  }
  
  if(flag.pedalFree == TRUE) {
    if(clutchSensorValue <= freePosition - 8) {
      flag.pedalFree = FALSE;
    }
    flag.fillImplement = TRUE;
  }
  else {
    if(clutchSensorValue >= freePosition + 8) {
      flag.pedalFree = TRUE;
      flag.fillImplement = TRUE;
    }
  }
}

void update_mode()
{
  // Update mode
  if((direction == VAC_NEUTRAL) || (flag.pedalPush == TRUE)) {
    shuttleMode = VAC_SHUTTLE_MODE_OFF;
    timeShuttle = 0;
    timeClutch = 0;
    timeFill = 0;
    flag.fillOn = FALSE;
  }
  else {
    if((shuttleMode == VAC_SHUTTLE_MODE_OFF) && (flag.fillImplement == TRUE)) {
      flag.fillOn = TRUE;                                                 // The fill current should be calculated after shifting from this mode
      shuttleMode = VAC_SHUTTLE_MODE_FILL;
      flag.fillImplement = FALSE;
    }
    else if(flag.fillOn == FALSE) {
      if(flag.pedalFree == TRUE) {
        shuttleMode = VAC_SHUTTLE_MODE_FREE;
      }
      else {
        if(waitingTime == 0) {
          shuttleMode = VAC_SHUTTLE_MODE_CONTROL;
          isUpdatedStartIndexOfShuttleCurrent = FALSE;
        }  
        waitingTime--;
        if(waitingTime < 0) {
          waitingTime = 0;
        }
      }
    }
    else {
      // Fill implemtation is going on
    }
    /*
    if((shuttleModePrevious == VAC_SHUTTLE_MODE_FREE) && (shuttleMode == VAC_SHUTTLE_MODE_CONTROL)) {
      shuttleMode = VAC_SHUTTLE_MODE_CHANGE;
    }
    // How about clutch is NOT FULLY pushed and NOT FREE
    if(shuttleMode != VAC_SHUTTLE_MODE_CHANGE) {
      shuttleMode = VAC_SHUTTLE_MODE_CONTROL;
    }
    */
  }
}

uint16_t tempFillTime[USED_POINTS_IN_FILL]; 
uint16_t tempFillCurrent[USED_POINTS_IN_FILL];

  
// The fill section of forward and backward shuttle are controlled based on OIL TEMPERATURE of HYDRAULIC VALVE.
void calculate_fill_temperature(uint8_t isShouldUpdate)
{
  static uint16_t timerCalculateTemp = 0;
  
  uint16_t tempFillTimeDistance[USED_POINTS_IN_FILL - 1];
  
  uint8_t  whichFillTemperatureMap;
  uint16_t tempOilTemperature;
  int16_t  tempratureBefore, tempratureNext;
  int16_t  currentBefore, currentNext;
  int16_t  timeBefore, timeNext;
  uint8_t  i;

  if(isShouldUpdate == FALSE) {
    // It is updated every 500ms
    if(timerCalculateTemp < 500) { 
      if(flagTimer.hundredMs == TRUE)
      {
        timerCalculateTemp += 100;
      }
      return;                                                                   // We do NOT need to implement the following section
    }
  }
  timerCalculateTemp = 0;
  tempOilTemperature = get_oil_temperature();
  
  // Search first or second temperature map --> If the pedal push is true, then we search the map from CLUTCH FILL MAP. If not, use SHUTTLE MAP
  whichFillTemperatureMap = (flag.pedalFree == TRUE) ? nvTransmissionFillTime[0] :
        ((direction == VAC_BACKWARD) ? nvClutchBackwardFillTime[0] : nvClutchForwardFillTime[0]);
  if(whichFillTemperatureMap != 2){
    whichFillTemperatureMap = 1;
  }
  
  for(i = 0; i < USED_POINTS_IN_TEMPERATURE_FILL; i++) {
    tempratureNext = (whichFillTemperatureMap == 1) ? nvFillCurrentTemperatureFirst[i] : nvFillCurrentTemperatureSecond[i];
    if(tempOilTemperature <= tempratureNext) {
      if(i == 0) {
        temperatureFillCurrentPercent = (whichFillTemperatureMap == 1) ? nvFillCurrentFirst[i] : nvFillCurrentSecond[i];
      }
      else {  
        
        tempratureBefore = (whichFillTemperatureMap == 1) ? nvFillCurrentTemperatureFirst[i - 1] : nvFillCurrentTemperatureSecond[i - 1];
        
        currentBefore = (whichFillTemperatureMap == 1) ? nvFillCurrentFirst[i - 1] : nvFillCurrentSecond[i - 1];                                    // 8 variables
        currentNext = (whichFillTemperatureMap == 1) ? nvFillCurrentFirst[i] : nvFillCurrentSecond[i];
        
        temperatureFillCurrentPercent = (float)currentBefore + ((float)(currentNext - currentBefore) * (float)(tempOilTemperature - tempratureBefore ) / (float)(tempratureNext - tempratureBefore + 0.1));
      }
      break;                                                                    // Once the condition is happened, it should be break from for loop
    }
  }
  if(i == USED_POINTS_IN_TEMPERATURE_FILL) {
    temperatureFillCurrentPercent = (float)((whichFillTemperatureMap == 1) ? nvFillCurrentFirst[USED_POINTS_IN_TEMPERATURE_FILL - 1] : nvFillCurrentSecond[USED_POINTS_IN_TEMPERATURE_FILL - 1]);
  }
  temperatureFillCurrentPercent = temperatureFillCurrentPercent / 1000.0;       // The maximum value is 1000, if the value is 1000 it should be equal to 100. SCALE is 0.1
  if(temperatureFillCurrentPercent > 1.5) {
    temperatureFillCurrentPercent = 1.5;                                        // 150%
  }
  else if(temperatureFillCurrentPercent <= 0) {
    temperatureFillCurrentPercent = 0;                                          // 0%
  }
  
  for(i = 0; i < USED_POINTS_IN_TEMPERATURE_FILL; i++) {
    tempratureNext = (whichFillTemperatureMap == 1) ? nvFillTimeTemperatureFirst[i] : nvFillTimeTemperatureSecond[i];
    if(tempOilTemperature <= tempratureNext) {
      if(i == 0) {
        temperatureFillTimePercent = (whichFillTemperatureMap == 1) ? nvFillTimeFirst[i] : nvFillTimeSecond[i];
      }
      else {
        tempratureBefore = (whichFillTemperatureMap == 1) ? nvFillTimeTemperatureFirst[i - 1] : nvFillTimeTemperatureSecond[i - 1];
        
        timeBefore = (whichFillTemperatureMap == 1) ? nvFillTimeFirst[i - 1] : nvFillTimeSecond[i - 1];                                    // 8 variables
        timeNext = (whichFillTemperatureMap == 1) ? nvFillTimeFirst[i] : nvFillTimeSecond[i];
        temperatureFillTimePercent = (float)timeBefore + ((float)(timeNext - timeBefore) * (float)(tempOilTemperature - tempratureBefore) / (float)(tempratureNext - tempratureBefore + 0.1));
      }
      break;                                                                    // Once the condition is happened, it should be break from for loop
    }
  }
  if(i == USED_POINTS_IN_TEMPERATURE_FILL) {
    temperatureFillTimePercent = (float)((whichFillTemperatureMap == 1) ? nvFillTimeFirst[USED_POINTS_IN_TEMPERATURE_FILL - 1] : nvFillTimeSecond[USED_POINTS_IN_TEMPERATURE_FILL - 1]);
  }
  
  temperatureFillTimePercent = temperatureFillTimePercent / 1000.0;             // The maximum value is 1000, if the value is 1000 it should be equal to 100. SCALE is 0.1
  if(temperatureFillTimePercent > 1.5) {
    temperatureFillTimePercent = 1.5;
  }
  else if(temperatureFillTimePercent <= 0) {
    temperatureFillTimePercent = 0;
  }
  
  // The first loop we finding distance of the time and current
  for(i = 0; i < USED_POINTS_IN_FILL; i++) {
    if(i == 0) {
      tempFillTime[i] = 0;
      // 0 --> 0
    }
    else {
      tempFillTime[i] = (flag.pedalFree == TRUE) ? nvTransmissionFillTime[i] : ((direction == VAC_BACKWARD) ? nvClutchBackwardFillTime[i] : nvClutchForwardFillTime[i]);
      // 200, 400, 600, 800
      tempFillTimeDistance[i - 1] = tempFillTime[i] - tempFillTime[i - 1];
      // 200 - 0      200
      // 400 - 200    200
      // 600 - 400    200
      // 800 - 600    200
    }
    
    tempFillCurrent[i] = (flag.pedalFree == TRUE) ? nvTransmissionFillCurrent[i] : ((direction == VAC_BACKWARD) ? nvClutchBackwardFillCurrent[i] : nvClutchForwardFillCurrent[i]);
    if(i < 3) {
      tempFillCurrent[i] = (uint16_t)((float)tempFillCurrent[i] * temperatureFillCurrentPercent);                       // Three points are used in this calculation
    }
  }
  
  for(i = 0; i < USED_POINTS_IN_FILL; i++) {
    if(i == 0) {
      tempFillTime[i] = 0;
      // 0 --> 0
    }
    else if(i == 1) {
      tempFillTime[i] = (uint16_t)((float)tempFillTimeDistance[i - 1] * temperatureFillTimePercent);
      // 1 --> 200 * 0.4 = 80
    }
    else {
      tempFillTime[i] = (uint16_t)((float)tempFillTime[i - 1] + tempFillTimeDistance[i - 1]);
    }
  }
}

uint16_t temperatureFillTimeMsCAN;
uint16_t temperatureFillCurrentAmperCAN;

void calculate_fill(uint8_t isUpdateMap)
{
  uint8_t i;
  /*
  if(isUpdateMap == TRUE) {
    // The first loop we finding distance of the time and current
    for(i = 0; i < USED_POINTS_IN_FILL; i++) {
      if(i == 0) {
        tempFillTime[i] = 0;
        // 0 --> 0
      }
      else {
        tempFillTime[i] = (flag.pedalFree == TRUE) ? nvTransmissionFillTime[i] : ((direction == VAC_BACKWARD) ? nvClutchBackwardFillTime[i] : nvClutchForwardFillTime[i]);
        // 200, 400, 600, 800
        tempFillTimeDistance[i - 1] = tempFillTime[i] - tempFillTime[i - 1];
        // 200 - 0      200
        // 400 - 200    200
        // 600 - 400    200
        // 800 - 600    200
      }
      
      tempFillCurrent[i] = (flag.pedalFree == TRUE) ? nvTransmissionFillCurrent[i] : ((direction == VAC_BACKWARD) ? nvClutchBackwardFillCurrent[i] : nvClutchForwardFillCurrent[i]);
      if(i == 0) {
        tempFillCurrent[i] = (uint16_t)((float)tempFillCurrent[i] * temperatureFillCurrentPercent);
      }
    }
    
    for(i = 0; i < USED_POINTS_IN_FILL; i++) {
      if(i == 0) {
        tempFillTime[i] = 0;
        // 0 --> 0
      }
      else if(i == 1) {
        tempFillTime[i] = (uint16_t)((float)tempFillTimeDistance[i - 1] * temperatureFillTimePercent);
        // 1 --> 200 * 0.4 = 80
      }
      else {
        tempFillTime[i] = (uint16_t)((float)tempFillTime[i - 1] + tempFillTimeDistance[i - 1]);
      }
    }
  }
  */
  temperatureFillCurrentAmperCAN = tempFillCurrent[0];
  temperatureFillTimeMsCAN = tempFillTime[1];
  
  for(i = 1; i < USED_POINTS_IN_FILL; i++) {
    if(tempFillTime[i] > timeFill) {                                            // Time check
      fillCurrent = tempFillCurrent[i - 1];                                     // The last data is saved in here
      break;
    }
  }
  
  if(i == USED_POINTS_IN_FILL) {
    flag.fillOn = FALSE;
    timeFill = 0;
    if(flag.pedalFree == TRUE) {
      shuttleMode = VAC_SHUTTLE_MODE_FREE;
      shuttleCurrent = nvTransmissionMapCurrent[0];
      waitingTime = 50;                                                         // 100ms
    }
    else {
      clutchCurrent = (direction == VAC_BACKWARD) ? nvClutchBackwardPositionCurrent[0] : nvClutchForwardPositionCurrent[0];  
      shuttleMode = VAC_SHUTTLE_MODE_CONTROL;
      waitingTime = 50;                                                         // 100ms
    }
    fillCurrent = 0;
    isUpdatedStartIndexOfShuttleCurrent = TRUE;
    timeShuttle = 0;
  }
}

uint16_t tempShuttleCompensationTemp[USED_POINTS_IN_COMPENSATION];                   // 8 points are used in MAP section
uint16_t tempShuttleCompensationCurrent[USED_POINTS_IN_COMPENSATION];
float temperatureCompensationCurrent;

void calculate_shuttle_clutch_compensation(uint8_t isShouldUpdate)
{
  uint16_t tempOilTemperature;
  uint16_t tempratureBefore;
  uint16_t tempratureNext;
  uint16_t currentBefore;
  uint16_t currentNext;
  uint8_t  i;
  
  if(isShouldUpdate == TRUE) {
    // Update variables
    for(i = 0; i < USED_POINTS_IN_COMPENSATION; i++) {      
      tempShuttleCompensationTemp[i] = (direction == VAC_BACKWARD) ? nvBackwardShuttleCompensationTemp[i] : nvForwardShuttleCompensationTemp[i];
      tempShuttleCompensationCurrent[i] = (direction == VAC_BACKWARD) ? nvBackwardShuttleCompensationCurrent[i] : nvForwardShuttleCompensationCurrent[i];
    }
    
    // current temperature map
 
    tempOilTemperature = get_oil_temperature();
    
    for(i = 0; i < USED_POINTS_IN_COMPENSATION; i++) {
      tempratureNext = tempShuttleCompensationTemp[i];
      if(tempOilTemperature <= tempratureNext) {
        if(i == 0) {
          temperatureCompensationCurrent = tempShuttleCompensationCurrent[i];
        }
        else {
          tempratureBefore = tempShuttleCompensationTemp[i - 1];
          tempratureNext = tempShuttleCompensationTemp[i];
          currentBefore = tempShuttleCompensationCurrent[i - 1];
          currentNext = tempShuttleCompensationCurrent[i];
          temperatureCompensationCurrent = (float)currentBefore + ((float)(currentNext - currentBefore) * (float)(tempOilTemperature - tempratureBefore ) / (float)(tempratureNext - tempratureBefore));
        }
        break;                                                                    // Once the condition is happened, it should be break from for loop
      }
    }
    if(i == USED_POINTS_IN_COMPENSATION) {
      temperatureCompensationCurrent = tempShuttleCompensationCurrent[USED_POINTS_IN_COMPENSATION - 1];
    }
    
    temperatureCompensationCurrent = temperatureCompensationCurrent - SHUTTLE_COMPENSATION_OFFSET_CURRENT;      // offset is -200mA
  }
}

uint16_t tempShuttleTime[USED_POINTS_IN_SHUTTLE];                      // 5 points are used in MAP section
uint16_t tempShuttleCurrent[USED_POINTS_IN_SHUTTLE];
  
uint8_t followNewMap = FALSE;
  
void calculate_shuttle(uint8_t isShouldUpdate)
{
  float temp;
  uint8_t  i;
  
  if(isShouldUpdate == TRUE) {
    if(followNewMap == FALSE)
    {
      // Update variables
      for(i = 0; i < USED_POINTS_IN_SHUTTLE; i++) {
        if(i == 0) {
          tempShuttleTime[i] = 0;
        } 
        else {
          tempShuttleTime[i] = nvTransmissionMapTime[i];                            // Position
        }
        tempShuttleCurrent[i] = nvTransmissionMapCurrent[i];
      }
    }
    else
    {
      for(i = 0; i < USED_POINTS_IN_SHUTTLE; i++) {
        if(i == 0) {
          tempShuttleTime[i] = 0;
        } 
        else {
          tempShuttleTime[i] = nvTransmissionClutchToShuttleTime[i];                            // Position
        }
        tempShuttleCurrent[i] = nvTransmissionClutchToShuttleCurrent[i];
      }
    }
    
    calculate_shuttle_clutch_compensation(TRUE);
  }
  
  // Now in here we should find the output current
  if(isUpdatedStartIndexOfShuttleCurrent == FALSE) {
    isUpdatedStartIndexOfShuttleCurrent = TRUE;                                 // If mode is changed, then it should implement again
    
    if(nvTransmissionClutchToShuttleTime[0] == 0)                               // The previous map should be used
    {
      followNewMap = FALSE;
      for(i = 0; i < USED_POINTS_IN_SHUTTLE; i++) {
        if(i == 0) {
          tempShuttleTime[i] = 0;
        } 
        else {
          tempShuttleTime[i] = nvTransmissionMapTime[i];                            // Position
        }
        tempShuttleCurrent[i] = nvTransmissionMapCurrent[i];
      }
    }
    else                                                                        // New map should be used
    {
      followNewMap = TRUE;
      for(i = 0; i < USED_POINTS_IN_SHUTTLE; i++) {
        if(i == 0) {
          tempShuttleTime[i] = 0;
        } 
        else {
          tempShuttleTime[i] = nvTransmissionClutchToShuttleTime[i];                            // Position
        }
        tempShuttleCurrent[i] = nvTransmissionClutchToShuttleCurrent[i];
      }
    }
    
    calculate_shuttle_clutch_compensation(TRUE);
    
    for(i = 0; i < USED_POINTS_IN_SHUTTLE; i++) {        
      if(clutchCurrent <= tempShuttleCurrent[i]) {                              // "clutchCurrent" is saved last position current of clutch and fill
        if(i == 0) {
          timeShuttle = 0;
        }
        else {
          temp = (float)tempShuttleCurrent[i] - (float)tempShuttleCurrent[i - 1];
          temp = (((float)clutchCurrent - (float)tempShuttleCurrent[i - 1]) / temp);
          timeShuttle = (uint16_t)((((float)tempShuttleTime[i] - (float)tempShuttleTime[i - 1]) * temp) + (float)tempShuttleTime[i - 1]);
        }
        // In here we should calculate the time and 
        break;
      }
    }
    if(i == USED_POINTS_IN_SHUTTLE) {
      timeShuttle = tempShuttleTime[USED_POINTS_IN_SHUTTLE - 1];
    }
  }
  
  for(i = 0; i < USED_POINTS_IN_SHUTTLE; i++) {
    if(tempShuttleTime[i] > timeShuttle) {
      if(i == 0) {
        shuttleCurrent = tempShuttleCurrent[i];
        shuttleCurrent += temperatureCompensationCurrent;                             // This is added on 2022.01.24
      }
      else {
        temp  = ((float)(timeShuttle - tempShuttleTime[i - 1])) / ((float)(tempShuttleTime[i] - tempShuttleTime[i - 1] + 1));
        temp *= (float)(tempShuttleCurrent[i] - tempShuttleCurrent[i - 1]);
        if(temp < 0)
          temp = 0;
        shuttleCurrent = tempShuttleCurrent[i - 1] + (uint16_t)temp;
        shuttleCurrent += temperatureCompensationCurrent;                             // This is added on 2022.01.24
      }
      break;
    }
  }
  
  if(i == USED_POINTS_IN_SHUTTLE)
  {
    if(flagTimer.tenMs == TRUE)
    {
      if(shuttleCurrent < tempShuttleCurrent[USED_POINTS_IN_SHUTTLE - 1])
      {
        shuttleCurrent = tempShuttleCurrent[USED_POINTS_IN_SHUTTLE - 1];
      }
      else
      {
        shuttleCurrent += 10;                                                       // Increased by 10ms
      }
      if(shuttleCurrent >= VAC_SHUTTLE_MAX_CURRENT)
      {
        shuttleCurrent = VAC_SHUTTLE_MAX_CURRENT;
      }
    }
  }
  
  if(flagTimer.tenMs == TRUE)
  {
    timeShuttle += 10;                                                          // it is increased 10 by 10 every 10ms
    if(timeShuttle >= VAC_SHUTTLE_MAX_TIME)
    {
      timeShuttle = VAC_SHUTTLE_MAX_TIME;
    }
  }
}

void calculate_clutch()                                                         // Clutch controller
{  
  float temp;
  int16_t  tempClutchCurrent[USED_POINTS_IN_CLUTCH];                            // Current (Y-current, X-Sensor) map data
  int16_t  tempClutchPosition[USED_POINTS_IN_CLUTCH];                           // Sensor position (Y-current, X-Sensor) map data
  
  uint8_t  i;

  for(i = 0; i < USED_POINTS_IN_CLUTCH; i++) {
    
    tempClutchPosition[i] = (direction == VAC_BACKWARD) ? nvClutchBackwardPositionSensor[i] : nvClutchForwardPositionSensor[i];
    
    if(i == (USED_POINTS_IN_CLUTCH - 1)) {
      tempClutchCurrent[i] = (direction == VAC_BACKWARD) ? (nvClutchBackwardPositionCurrent[i] + (nvBackwardBaseCurrent - 200)) : 
                                                           (nvClutchForwardPositionCurrent[i] + (nvForwardBaseCurrent - 200));
    }
    else {
      tempClutchCurrent[i] = (direction == VAC_BACKWARD) ? (nvClutchBackwardPositionCurrent[i] + (nvBackwardBaseCurrent - 200)) : 
                                                           (nvClutchForwardPositionCurrent[i] + (nvForwardBaseCurrent - 200));
    }
    
    if(tempClutchCurrent[i] < 0) {
      tempClutchCurrent[i] = 0;
    }
  }
  
  calculate_shuttle_clutch_compensation(TRUE);
  
  // Update clutchSensorValue in here with moving average
  clutchSensorValue = get_clutch_sensor_average();                              // it is changed every 50ms
  
  if(clutchSensorValue > previousClutchSensorValue) {
    
    previousClutchSensorValue += 4;
  }
  else {
    previousClutchSensorValue = clutchSensorValue;
  }
  
  for(i = 0; i < (USED_POINTS_IN_CLUTCH - 1); i++) {
    if(previousClutchSensorValue <= tempClutchPosition[i + 1]) {
      temp  = (float)(tempClutchCurrent[i + 1] - tempClutchCurrent[i]);
      temp *= (float)(previousClutchSensorValue - tempClutchPosition[i]);               // The "clutchSensorValue" is replaced with previousClutchSensorValue on 2021.11.25
      temp /= (float)(tempClutchPosition[i + 1] - tempClutchPosition[i] + 1);           // 1 is added in order to remove the divider error of CPU
      clutchCurrent = tempClutchCurrent[i] + (uint16_t)temp;
      break;
    }
  }
  
  if(clutchCurrent < tempClutchCurrent[0]) {
    clutchCurrent = tempClutchCurrent[0];                                       // Added on 2021.11.25
  }
  
  if(i == (USED_POINTS_IN_CLUTCH - 1)) {
    clutchCurrent = tempClutchCurrent[i];
  }
  clutchCurrent = clutchCurrent + (int16_t)temperatureCompensationCurrent;
}

int16_t dialogueCurrent;

// Map number data is calculated in here
void shuttle_valve_out()
{
  uint8_t isDialogueUsed = FALSE;
  float localRunCurrent;
  if(shuttleMode == VAC_SHUTTLE_MODE_OFF) {
    localRunCurrent = 0;
  }
  else if(shuttleMode == VAC_SHUTTLE_MODE_FILL) {
    localRunCurrent = fillCurrent;
  }
  else if(shuttleMode == VAC_SHUTTLE_MODE_FREE) {
    if(isStartOrShift == 2) {                                                   // This dialogue is used only starter map 
      isDialogueUsed = TRUE;
    }
    localRunCurrent = shuttleCurrent;                                           // Calculated current
    // Below condition is added, because the base current is used for both starter and transmission map data.           ==> Changed on 2025.06.11
    if(direction != VAC_BACKWARD)
    {
      localRunCurrent += (nvForwardBaseCurrent - 200);
    }
    else
    {
      localRunCurrent += (nvBackwardBaseCurrent - 200);
    }
  }
  else if(shuttleMode == VAC_SHUTTLE_MODE_CONTROL) {
    localRunCurrent = clutchCurrent;                                            // Calculated current
  }
  else if (shuttleMode == VAC_SHUTTLE_MODE_CHANGE) {
    localRunCurrent = 0;                                                        // Calculated current
  }
  else {
    localRunCurrent = 0;
  }
  
  if(isDialogueUsed == TRUE) {
    // Disabled on 2021.11.03 localRunCurrent = (float)localRunCurrent * (1.0 +  (((((float)armSetVr.adSpeedRate - 127.0) / 127.0) * (float)(nvSensetiveSettingUpLimit)) / 100.0));
    // Enabled on 2021.11.03 --> Because of the settings as below
    // nvSensetiveSettingUpLimit          --> -10
    // nvSensetiveSettingMiddleLimit      --> 10
    // nvSensetiveSettingDownLimit        --> 20
    
    // 127 to 254   upLimit - middleLimit         = (20 - 10) = 10% increase max so 10 + (0 to 10) percent changes
    // 127          middleLimit                   = 10% increase
    // 0   to 127   middleLimit - DownLimit       = (10 - -10) = 20% decrease max so 10 - (0 to 20) percent changes
      
    // This is middle case
    dialogueCurrent = (int16_t)nvShuttleSensetiveMiddleLimit - (int16_t)SHUTTLE_DIALOG_OFFSET_CURRENT;
    
    if(canRxDial.shuttleDial > (AD_SPEED_MIDDLE + AD_SPEED_MIDDLE_GAP)) {
      // This is up limit
      dialogueCurrent = dialogueCurrent + 
        (int16_t)((((float)canRxDial.shuttleDial - AD_SPEED_MIDDLE) / AD_SPEED_MIDDLE) * (float)((int16_t)nvShuttleSensetiveUpLimit - (int16_t)nvShuttleSensetiveMiddleLimit));
    }
    else if(canRxDial.shuttleDial < (AD_SPEED_MIDDLE - AD_SPEED_MIDDLE_GAP)) {
      // This is down limit
      dialogueCurrent = dialogueCurrent +
        (int16_t)((((float)canRxDial.shuttleDial - AD_SPEED_MIDDLE) / AD_SPEED_MIDDLE) * (float)((int16_t)nvShuttleSensetiveMiddleLimit - (int16_t)nvShuttleSensetiveDownLimit));
    }
    localRunCurrent = localRunCurrent + dialogueCurrent;
  }

  finalCurrent = (localRunCurrent >= VAC_SHUTTLE_MAX_CURRENT) ? VAC_SHUTTLE_MAX_CURRENT :
                ((localRunCurrent <= VAC_SHUTTLE_MIN_CURRENT) ? 0 : (uint16_t)localRunCurrent);
                
  shuttleModePrevious = shuttleMode;                                            // After implementation, update previous mode
}

void shuttle_fill_update()
{
  
}

void shuttle(uint8_t isTransmissionHappened)
{
  static uint16_t timeChanging = 0;
  
  pedal_checking(isTransmissionHappened);
  update_direction();
  update_mode();
  
  calculate_fill_temperature(isTransmissionHappened);                           // During this off mode we calculate the Fill current based on temperature map and oil temperature
  
  if(shuttleMode == VAC_SHUTTLE_MODE_OFF) {
    
  }
  else if(shuttleMode == VAC_SHUTTLE_MODE_FILL) {                               // If shuttle mode is OFF, then the output should be zero we do not need other calculation in here
    calculate_fill(TRUE);                                                       // If we in the mode FREE or CONTROL, the fill current should be calculated
  }
  else if(shuttleMode == VAC_SHUTTLE_MODE_FREE) {
    calculate_shuttle(TRUE);
  }
  else if(shuttleMode == VAC_SHUTTLE_MODE_CONTROL) {
    calculate_clutch();                                                         // Clutch controller
  }
  else if(shuttleMode == VAC_SHUTTLE_MODE_CHANGE) {
    timeChanging += 2;
    if(timeChanging > 30) {                                                     // 30ms
      shuttleMode = VAC_SHUTTLE_MODE_CONTROL;
      timeChanging = 0;
    }
  }

  shuttle_valve_out();
}

uint8_t update_nv_data()
{
  static uint8_t prevPedalPush = FALSE;
  static uint8_t previousTransmissionState = 1;
  static uint8_t isFirst = TRUE;
  static uint8_t isUsedShiftMap = FALSE;
  // First measure states of the H, L, M, & 1, 2, 3, 4
  // If there is no difference between previous state and current state then we do nothing
  uint8_t updateMHL;
  uint8_t flagCheck = FALSE;
  uint8_t index = 0;
  uint8_t i, j;
  uint8_t shiftMapIndex = 6;

  
  // Fill temperature and current map is changed from diagnostic program OR this function is called first time
  for(i = 0; i < 24; i++) {                                                            // Until this map, we using constant map data
    if((isFirst == TRUE) || (isUpdateHappenedMapData[i] == TRUE)) {
      // The map numbers are clearly found in previous section, so now we should update the map data
      for(j = 0; j < 8; j++) {                                                  // There is totally eight points we are using for fill temperature
        if(i < FILL_NUMBER_START_TRANSMISSION) {
          nvX[i][j] = (int16_t)mapDataInfo[i][j].x;                         // The pointer is used so the actual data is updated
          nvY[i][j] = mapDataInfo[i][j].y;                                  // The pointer is used so the actual data is updated
        }
        else if(i == 20) {
          nvX[i][j] = (int16_t)mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][j].x;                         // The pointer is used so the actual data is updated
          nvY[i][j] = mapDataInfo[MAP_NUMBER_THREEP_DRAFT_UP - 1][j].y;                                  // The pointer is used so the actual data is updated
        }
        else if(i == 21) {
          nvX[i][j] = (int16_t)mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][j].x;                         // The pointer is used so the actual data is updated
          nvY[i][j] = mapDataInfo[MAP_NUMBER_THREEP_DRAFT_DOWN - 1][j].y;                                  // The pointer is used so the actual data is updated
        }
        else if(i == 22) {
          nvX[i][j] = (int16_t)mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][j].x;                         // The pointer is used so the actual data is updated
          nvY[i][j] = mapDataInfo[MAP_NUMBER_THREEP_DEPTH_UP - 1][j].y;                                  // The pointer is used so the actual data is updated
        }
        else if(i == 23) {
          nvX[i][j] = (int16_t)mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][j].x;                         // The pointer is used so the actual data is updated
          nvY[i][j] = mapDataInfo[MAP_NUMBER_THREEP_DEPTH_DOWN - 1][j].y;                                  // The pointer is used so the actual data is updated
        }
      }
      isUpdateHappenedMapData[i] = FALSE;
    }
  }
   /* Disabled on 2022.09.26
  //if((flagInputStatus.transmit == ON) || (flagInputStatus.clutch == ON)) {                          // it is moved from bottom.
  if((flagInputStatus.transmit == ON) || (flag.pedalPush == ON)) {                                      // Changed on 2022.09.26
    isUsedShiftMap = TRUE;
  }
  */
  
  isUsedShiftMap = TRUE;                // Enabled on 2022.09.26
  
  // Transmission map update
  // Even though there is nothing changed and it is first time configuration is happened, then update the variables
  if(isFirst == TRUE)                                                                           {    flagCheck = TRUE;  isFirst = FALSE;                                                                                }
  if(isUpdateHappenedMapNumber == TRUE)                                                         {    flagCheck = TRUE;  isUpdateHappenedMapNumber = FALSE;                                                              }
  if(flagInputStatus.transmissionH != flagInputStatusPrevious.transmissionH)        {    flagCheck = TRUE;  flagInputStatusPrevious.transmissionH = flagInputStatus.transmissionH;              }
  if(flagInputStatus.transmissionM != flagInputStatusPrevious.transmissionM)        {    flagCheck = TRUE;  flagInputStatusPrevious.transmissionM = flagInputStatus.transmissionM;              }
  if(flagInputStatus.transmissionL != flagInputStatusPrevious.transmissionL)        {    flagCheck = TRUE;  flagInputStatusPrevious.transmissionL = flagInputStatus.transmissionL;              }

  if(flagInputStatus.transmissionOne != flagInputStatusPrevious.transmissionOne)    {
    flagCheck = TRUE;
    flagInputStatusPrevious.transmissionOne = flagInputStatus.transmissionOne;
    
    if((isUsedShiftMap == TRUE) && (get_average_speed() > TRACTOR_SHIFT_SPEED))  {                // if it is greater than 3km/h, it should be use shift map --> this is added on 2021.12.22
      shiftMapIndex = 3;                                                                        // 2 to 1, 3 to 1, 4 to 1
    }
  }
  if(flagInputStatus.transmissionTwo != flagInputStatusPrevious.transmissionTwo)    {
    flagCheck = TRUE;
    flagInputStatusPrevious.transmissionTwo = flagInputStatus.transmissionTwo;
    
    if((isUsedShiftMap == TRUE) && (get_average_speed() > TRACTOR_SHIFT_SPEED))  {                  // if it is greater than 3km/h, it should be use shift map --> this is added on 2021.12.22
      if(previousTransmissionState == 1) {
        shiftMapIndex = 0;                                                                        // 1 to 2
      }
      else {
        shiftMapIndex = 4;                                                                        // 3 to 2, 4 to 2
      }
    }
  }
  if(flagInputStatus.transmissionThree != flagInputStatusPrevious.transmissionThree) {
    flagCheck = TRUE;  
    flagInputStatusPrevious.transmissionThree = flagInputStatus.transmissionThree;      
    
    if((isUsedShiftMap == TRUE) && (get_average_speed() > TRACTOR_SHIFT_SPEED))  {                  // if it is greater than 3km/h, it should be use shift map --> this is added on 2021.12.22
      if((previousTransmissionState == 1) || (previousTransmissionState == 2)) {
        shiftMapIndex = 1;                                                                        // 1 to 3, 2 to 3
      }
      else {
        shiftMapIndex = 5;                                                                        // 4 to 3
      }
    }
  }
  if(flagInputStatus.transmissionFour != flagInputStatusPrevious.transmissionFour)  {    
    flagCheck = TRUE;  
    flagInputStatusPrevious.transmissionFour = flagInputStatus.transmissionFour;
    if((isUsedShiftMap == TRUE) && (get_average_speed() > TRACTOR_SHIFT_SPEED))  {                  // if it is greater than 3km/h, it should be use shift map --> this is added on 2021.12.22
      shiftMapIndex = 2; 
    }
  }
  
  if(flagShuttle.backward != flagShuttlePrevious.backward)                          {    flagCheck = TRUE;  flagShuttlePrevious.backward = flagShuttle.backward;                                }
  if(flagShuttle.forward != flagShuttlePrevious.forward)                            {    flagCheck = TRUE;  flagShuttlePrevious.forward = flagShuttle.forward;                                  }

  // Added on 2025.08.04
  if(flag.pedalPush != prevPedalPush)                                               { flagCheck = TRUE; prevPedalPush = flag.pedalPush; }
  
  if(flagCheck == FALSE) {                                                       // There is no update is needed
    return FALSE;
  }
  
  if((isUsedShiftMap == TRUE) && (get_average_speed() > TRACTOR_SHIFT_SPEED))  {                  // if it is greater than 3km/h, it should be use shift map
    
    isUsedShiftMap = FALSE;                                                     // The shift map is not used next time
    
    // Shift map is used
    isStartOrShift = 1;
    if(flagInputStatus.transmissionH == ON) {                             // H state
      updateMHL = 1;
      index = (flagShuttle.backward == ON) ? 18 : 6;
     }
    else if(flagInputStatus.transmissionM == ON) {                        // M state
      updateMHL = 2;
      index = (flagShuttle.backward == ON) ? 12 : 0;
    }
    else{                                                                 // L state
      updateMHL = 1;
      index = (flagShuttle.backward == ON) ? 12 : 0;
    }
    
    if(flagInputStatus.transmissionOne == ON) {                           // 1 state
      previousTransmissionState = 1;
    }
    else if(flagInputStatus.transmissionTwo == ON) {                      // 2 state
      previousTransmissionState = 2;
    }
    else if(flagInputStatus.transmissionThree == ON) {                    // 3 state
      previousTransmissionState = 3;
    }
    else if(flagInputStatus.transmissionFour == ON) {                     // 4 state
      previousTransmissionState = 4;
    }
    
    if(shiftMapIndex > 5) {
      index += 0;
    }
    else {
      index += shiftMapIndex;
    }
    
    mapArrayIndex = index;
    
    if(updateMHL == 1) {
      fillNumber = mapNumberShiftInfo[index].hl_fill;                     // Read L or H fill number from the Non-Volatile memory
      mapNumber = mapNumberShiftInfo[index].hl_map;                       // Read L or H map number from the Non-Volatile memory
    }
    else {
      fillNumber = mapNumberShiftInfo[index].cm_fill;                     // Read M fill number from the Non-Volatile memory
      mapNumber = mapNumberShiftInfo[index].cm_map;                       // Read M map number from the Non-Volatile memory
    }
  }
  else {
    // Starter map is used
    isStartOrShift = 2;
    if(flagInputStatus.transmissionH == ON) {                             // H state
      updateMHL = 1;
      index = (flagShuttle.backward == ON) ? 18 : 6;
    }
    else if(flagInputStatus.transmissionM == ON) {                        // M state
      updateMHL = 2;
      index = (flagShuttle.backward == ON) ? 12 : 0;
    }
    else{                                                                       // L state
      updateMHL = 1;
      index = (flagShuttle.backward == ON) ? 12 : 0;
    }
    
    if(flagInputStatus.transmissionOne == ON) {                           // 1 state
      index += 0;
      previousTransmissionState = 1;
    }
    else if(flagInputStatus.transmissionTwo == ON) {                      // 2 state
      index += 1;
      previousTransmissionState = 2;
    }
    else if(flagInputStatus.transmissionThree == ON) {                    // 3 state
      index += 2;
      previousTransmissionState = 3;
    }
    else if(flagInputStatus.transmissionFour == ON) {                     // 4 state
      index += 3;
      previousTransmissionState = 4;
    }
    else {
      index += 0;                                                               // 1 state for test 2021.10.07
    }
    
    mapArrayIndex = index;
    
    if(updateMHL == 1) {
      fillNumber = mapNumberInfo[index].hl_fill;                          // Read L or H fill number from the Non-Volatile memory
      mapNumber = mapNumberInfo[index].hl_map;                            // Read L or H map number from the Non-Volatile memory
    }
    else {
      fillNumber = mapNumberInfo[index].cm_fill;                          // Read M fill number from the Non-Volatile memory
      mapNumber = mapNumberInfo[index].cm_map;                            // Read M map number from the Non-Volatile memory
    }
  }
  
  if((fillNumber > FILL_NUMBER_END_TRANSMISSION) || (fillNumber < FILL_NUMBER_START_TRANSMISSION)) {
    fillNumber = FILL_NUMBER_DEFAULT_TRANSMISSION;
  }
  if((mapNumber > MAP_NUMBER_END_TRANSMISSION) || (mapNumber < MAP_NUMBER_START_TRANSMISSION)) {
    mapNumber = MAP_NUMBER_DEFAULT_TRANSMISSION;
  }
  
  for(j = 0; j < 8; j++) {                                                      // There is totally eight points we are using for fill temperature
    nvTransmissionFillTime[j]   = mapDataInfo[fillNumber - 1][j].x;       // The X-Axis illustrates time
    nvTransmissionFillCurrent[j]= mapDataInfo[fillNumber - 1][j].y;       // The Y-Axis illustrates current
    nvTransmissionMapTime[j]    = mapDataInfo[mapNumber - 1][j].x;        // The X-Axis illustrates time
    nvTransmissionMapCurrent[j] = mapDataInfo[mapNumber - 1][j].y;        // The Y-Axis illustrates current
    
    nvTransmissionClutchToShuttleTime[j] = mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][j].x;        // The X-Axis illustrates time
    nvTransmissionClutchToShuttleCurrent[j] = mapDataInfo[MAP_CLUTCH_TO_SHUTTLE_TRANSMISSION - 1][j].y;        // The Y-Axis illustrates current

  }
  return TRUE;
}