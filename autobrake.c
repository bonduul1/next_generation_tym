#include "autobrake.h"
#include "can.h"
#include "input.h"
#include "output.h"
#include "sensors.h"
#include "main.h"
#include "algorithm.h"
#include "settings.h"
#include "autobrake.h"

uint16_t timerErrorMotorR = 0;
uint16_t timerErrorMotorL = 0;
  
uint16_t objectADPosition = 0;
uint16_t motorRightLimit = 0;
uint16_t motorPositionNow = 0;
uint16_t motorLeftLimit = 0;
uint16_t vaMotorCenter;
uint16_t vaMotorDeadband;
uint16_t vaMotorPwmBand;

uint16_t nvAdCenter;

void sub_autobreak_check_output_time_process()
{
  flag.autoBreakError = FALSE;
  
  if(flagOutputControl.adMotorRight == ON) {
    if(timerErrorMotorR >= VA_MOTOR_OUT_ERROR) { 
      flag.autoBreakError = TRUE;
      flagOutputControl.adMotorRight = OFF;
      timerErrorMotorR = VA_MOTOR_OUT_ERROR;
    }
  }
  else {
    timerErrorMotorR = 0;
  }

  if(flagOutputControl.adMotorLeft == ON) {
    if(timerErrorMotorL >= VA_MOTOR_OUT_ERROR) {
      flag.autoBreakError = TRUE;
      flagOutputControl.adMotorLeft = OFF;
      timerErrorMotorL = VA_MOTOR_OUT_ERROR;
    }
  }
  else {
    timerErrorMotorL = 0;
  }
}

void autobrake_process()
{
  static uint16_t timerAutoBreakPwm = 0;
  static uint16_t timerErrorMotorC = 0;
  static uint16_t timerAutoBreakLamp = 0;
  static uint16_t timerAutoBreakDelay = 0;
  static uint16_t timerAutoBreakRunDelay = 0;
  static uint8_t  adDirectionPrevious = 0;
  uint8_t         adDirection;
  uint8_t         flagCheck;
  uint16_t        timerAutoBreakOnOffDelay;
  
  flagOutputControl.adMotorRight = OFF;
  flagOutputControl.adMotorLeft = OFF;
  
//  flagOutputLamp.autoBreakFlash = OFF;
  
  if(flag.powerOn == FALSE)         
  {
    flag.autoBreakNeutral = FALSE;
    timerAutoBreakPwm = 0;
    timerErrorMotorC = 0;
    timerErrorMotorR = 0;
    timerErrorMotorL = 0;
    return;                                                                     // After settings are finished the following process should be run
  }

  if(flagTimer.tenMs == TRUE)
  {
    timerAutoBreakPwm += 10;
    timerAutoBreakLamp += 10;
    timerAutoBreakRunDelay += 10;
  }
   
  if(flagTimer.hundredMs == TRUE)
  {
    timerAutoBreakDelay += 100;
    timerErrorMotorR += 100;
    timerErrorMotorL += 100;
    timerErrorMotorC += 100;
  }
    
  motorPositionNow = get_motor_position();
  vaMotorCenter = nvAdCenter;
  vaMotorDeadband = nvAdDeadBand;
  vaMotorPwmBand = nvAdNeutralRange;
  
  if(canFlag.autoBrakeMode == 1) {
    motorRightLimit = nvAdRightLow;
    motorLeftLimit  = nvAdLeftLow;
  } 
  else if(canFlag.autoBrakeMode == 2) {
    motorRightLimit = nvAdRightMid;
    motorLeftLimit  = nvAdLeftMid;
  } 
  else if(canFlag.autoBrakeMode == 3) { 
    motorRightLimit = nvAdRightHigh;
    motorLeftLimit  = nvAdLeftHigh;
  }
  else {
    motorRightLimit = nvAdRightLow;
    motorLeftLimit  = nvAdLeftLow;
  }
  
  flagCheck = FALSE;

  if((flagShuttle.backward == ON) || (flagShuttle.neutral == ON)) {
    flagCheck = TRUE;
  }

  if((canFlag.autoBrakeMode == 0) || (canFlag.quickTurn == FALSE)) {
    flagCheck = TRUE;
  }
  
  if((flagCheck == TRUE) || (nvAdAutoDisconnectSpeed <= get_average_speed())) {
    if(flagCheck == TRUE) {
      timerAutoBreakLamp = 0;
      timerAutoBreakDelay = 9900;
    }
    else {
      timerAutoBreakDelay =0;
    }
    adDirection = VA_AUTOBREAK_DIR_NEUTRAL;
  }
  else {
    /* Check this condition
    if((flagInputStatus.steeringLeft == ON) && (nvAdAutoConnectSpeed > get_average_speed())) {              // If the current speed is lower than configured speed then it will be connected
      adDirection = VA_AUTOBREAK_DIR_LEFT;
    }
    else if((flagInputStatus.steeringRight == ON) && (nvAdAutoConnectSpeed > get_average_speed())) {        // If the current speed is lower than configured speed then it will be connected
      adDirection = VA_AUTOBREAK_DIR_RIGHT;
    }
    else {
      timerAutoBreakLamp = 0;
      adDirection = VA_AUTOBREAK_DIR_NEUTRAL;
    }
    */
  }
  
  if(adDirection == VA_AUTOBREAK_DIR_NEUTRAL) {
    timerAutoBreakOnOffDelay = nvAdOffTime;                                     // If the current direction is NEUTRAL, it assumes the AD is going to OFF
  } else {
    timerAutoBreakOnOffDelay = nvAdOnTime;                                      // If the current direction is NOT NEUTRAL, it assumes the AD is going to ON
  }
  
  if(adDirectionPrevious != adDirection) {
    if(timerAutoBreakRunDelay >= timerAutoBreakOnOffDelay) {
      adDirectionPrevious = adDirection;
      timerAutoBreakRunDelay = 0;
    }
    else {
      adDirection = adDirectionPrevious;
    }
  }    
  else {
    timerAutoBreakRunDelay = 0;
  }
  
  
  if(adDirection == VA_AUTOBREAK_DIR_NEUTRAL) {
    objectADPosition = vaMotorCenter;
    
    if(flag.autoBreakNeutral == TRUE) {
      sub_autobreak_check_output_time_process();
      return;
    }

    if(timerErrorMotorC >= VA_MOTOR_OUT_CENTER) {
      flag.autoBreakNeutral = TRUE;
      timerErrorMotorC = 0;
    }

    if((vaMotorCenter - vaMotorPwmBand) > motorPositionNow) {          // 512 - 143 = 369 lower than that case
      flagOutputControl.adMotorRight = ON;
      timerAutoBreakPwm = 0;
    }
    else if((vaMotorCenter - vaMotorDeadband) >= motorPositionNow) {    // 512 - 20 = 492
      if(timerAutoBreakPwm >= VA_PWM_CYCLE) {
        timerAutoBreakPwm = 0;
      }
      if(timerAutoBreakPwm >= VA_PWM_ON){
        flagOutputControl.adMotorRight = ON;
      }
    }
    else if((vaMotorCenter + vaMotorDeadband) >= motorPositionNow) {    // 512 + 20 = 532
      flagOutputControl.adMotorRight = OFF;
      flagOutputControl.adMotorLeft = OFF;
    }
    else if((vaMotorCenter + vaMotorPwmBand) >= motorPositionNow) {    // 512 + 143 = 655
      if(timerAutoBreakPwm >= VA_PWM_CYCLE) {
        timerAutoBreakPwm = 0;
      }
      if(timerAutoBreakPwm >= VA_PWM_ON) {
        flagOutputControl.adMotorLeft = ON;
      }
    }
    else {
      timerAutoBreakPwm = 0;
      flagOutputControl.adMotorLeft = ON;                                 // Or higher
    }
  }
  else {
    timerAutoBreakPwm = 0;
    timerErrorMotorC = 0;
    flag.autoBreakNeutral = FALSE;
    
    if(adDirection == VA_AUTOBREAK_DIR_RIGHT) {
      objectADPosition = motorRightLimit;
      if(motorRightLimit > motorPositionNow) {
        flagOutputControl.adMotorRight = ON;
      }
    }
    else {
      objectADPosition = motorLeftLimit;
      if(motorLeftLimit < motorPositionNow) {
        flagOutputControl.adMotorLeft = ON;
      }
    }
  }

//  flagOutputLamp.autoBreakFlash = ON;
  
  if(timerAutoBreakLamp >= VAC_WHEEL_LAMP_ON) {
  }
  else {
//    flagOutputLamp.autoBreakFlash = OFF;
  }

  if(timerAutoBreakLamp >= VAC_WHEEL_LAMP_CYCLE) {
    timerAutoBreakLamp = 0;
  }
}