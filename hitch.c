/*
 *
 * Lift lower limit 1.67V 
 * Lift higher limit 3.38V
 *
 */

#include "hitch.h"
#include "main.h"
#include "sensors.h"
#include "defines.h"
#include "can.h"
#include "input.h"
#include "output.h"
#include "settings.h"
#include "algorithm.h"

flagHitch_t flagHitch;

/************************************** Local Variables of Timer ************************/
uint16_t timerDepthDelay;
uint16_t timerDither;

/************************************** Local Variables of Other ************************/
int16_t stallCompUp;
int16_t stallCompDown;

uint16_t leverAngleForLeverControl;

uint16_t leverAngle;
uint16_t upLimitAngle;
uint16_t positionAngle;

uint16_t resultThreePDuty;

uint8_t manualOff = 0;                                                          // Added on 2021.11.29 for manual mode off condition is used this variable.

uint16_t threePDownDuty = 0;
uint16_t threePUpDuty = 0;

uint16_t get_threePDownDuty()   {       return threePDownDuty;  }
uint16_t get_threePUpDuty()     {       return threePUpDuty;    }

uint16_t leverUpLimit;
uint16_t leverDownLimit;
uint16_t positionUpLimit;
uint16_t positionDownLimit;

uint16_t leverSenseRate;
uint16_t positionSenseRate;

uint8_t changeDirection;

uint16_t updateThreePPositionSensor(uint16_t _rawData)
{
  uint16_t ret;
  if(_rawData >= positionUpLimit) {
    ret = 1023;
  }
  else if(_rawData <= positionDownLimit) {
    ret = 0;
  }
  else {
    ret = (uint16_t)(((float)(_rawData - positionDownLimit) * (float)100) / positionSenseRate);
  }
  
  if(ret > 1023) {
    ret = 1023;
  }
  return ret;
}

uint16_t updateLeverSensor(uint16_t _rawData)
{
  uint16_t ret;
  if(_rawData >= leverUpLimit) {
    ret = 1000;
  }
  else if(_rawData <= leverDownLimit) {
    ret = 0;
  }
  else {
    ret  = (uint16_t)(((float)(_rawData - leverDownLimit) * (float)100) / leverSenseRate);
  }
  if(ret > 1000) {
    ret = 1000;
  }
  return ret;
}

void lever_rate_calculate()
{
  uint32_t temp;
  leverUpLimit = nv3pLeverUpLimit - 2;                                          // 900 - 2 = 898
  leverDownLimit = nv3pLeverDownLimit + 2;                                      // 100 + 2 = 102
  
  temp                     = ((leverUpLimit - leverDownLimit) * 100) / 1000;
  leverSenseRate           = (uint16_t) temp;
}

void position_rate_calculate()
{ 
  uint32_t temp;
  positionUpLimit = nv3pPositionUpLimit - 2;
  positionDownLimit = nv3pPositionDownLimit + 2;
  
  temp                    = ((positionUpLimit - positionDownLimit) * 100) / 1023;
  positionSenseRate       = (uint16_t)temp;
}

void hitch_pwm_up_out(uint16_t _duty)
{
  static uint16_t dutyBefore = 10000;
  static uint16_t dutyCurrent = 0;
  uint16_t dutyObject;

  dutyObject = _duty;

  if((flag.liftDither == TRUE) && (dutyObject >= 25)) {
    if(timerDither <= 4) {
      dutyCurrent = 950;
    }
    else if(timerDither <= 16) {
      dutyCurrent = 200;
    }
    else if(timerDither <= 22) {
      dutyCurrent = 220;
    }
  }
  if(dutyBefore == dutyObject) {
    return;
  }
 
  if(dutyCurrent < dutyObject) {
    dutyCurrent += nv3pUpStepDuty;                                               // UP step
    if(dutyCurrent > dutyObject) {
      dutyCurrent = dutyObject;
    }
  }
  else if(dutyCurrent > dutyObject) {
    if(dutyCurrent > nv3pUpStepDuty) {
      dutyCurrent -= nv3pUpStepDuty;
    }
    else {
      dutyCurrent  = 0;
    }
  }
  if(dutyObject <= 25) {
    dutyCurrent = 0;
  }
  
  threePUpDuty = dutyCurrent;
  dutyBefore = dutyCurrent;
}

void hitch_pwm_down_out(uint16_t _duty)
{
  static uint16_t dutyBefore = 10000;
  static uint16_t dutyCurrent = 0;

  uint16_t dutyObject;

  dutyObject = _duty;

  if((flag.liftDither == TRUE) && (dutyObject > 25)) {
    if(timerDither <= 2) {
      dutyCurrent = 950;
    }
    else if(timerDither <= 9) {
      dutyCurrent = 170;
    }
    else {
      dutyCurrent = 190;
    }
  }

  if(dutyBefore == dutyObject) {
    return;
  }

  if(dutyCurrent < dutyObject) {
    dutyCurrent += nv3pDownStepDuty;
    if(dutyCurrent > dutyObject) {
      dutyCurrent = dutyObject;
    }
  }
  else if(dutyCurrent > dutyObject) {
    if(dutyCurrent > nv3pDownStepDuty) {
      dutyCurrent -= nv3pDownStepDuty;
    }
    else {
      dutyCurrent = 0;
    }
  }
  if(dutyObject <= 25) {
    dutyCurrent = 0;
  }
  
  threePDownDuty = dutyCurrent;
  dutyBefore = dutyCurrent;
}

void hitch_pwm_off_out()
{
  flagOutputControl.hitchUp = OFF;
  flagOutputControl.hitchDown = OFF;

  hitch_pwm_up_out(0);
  hitch_pwm_down_out(0);
}

void run_hitch_pwm(uint8_t _direction, uint16_t _outDuty)
{
  static uint8_t  directionBefore = 0;
  static uint16_t dutyBefore = 0;
  static uint16_t timerStopRun  = 0;
  static uint16_t timerOutRun = 0;
  static uint16_t timerShock = 0;
  
  uint16_t outDuty = _outDuty;
  uint8_t  direction = _direction;

  uint16_t calculateDuty;
  uint16_t dutyBuffer;

  timerDither += 2;                                                             // 2ms

  if(flagTimer.tenMs == TRUE)
  {
    timerOutRun += 10;
    timerShock += 10;
  }

  if(direction != directionBefore) {
    if(directionBefore != VAC_HITCH_DIRECTION_OFF) {
      if(directionBefore == VAC_HITCH_DIRECTION_DOWN) {
        dutyBuffer = 150;
      }
      else if(directionBefore == VAC_HITCH_DIRECTION_UP) {
        dutyBuffer = 250;
      }
      else {
        dutyBuffer = 150;
      }

      if(directionBefore > dutyBuffer) {
        dutyBuffer = dutyBefore / dutyBuffer;
      }
      else {
        dutyBuffer = 1;
      }

      if(dutyBefore > dutyBuffer) {
        outDuty = dutyBefore - dutyBuffer;
        if(outDuty <= nv3pManualDownMinDuty) {
          outDuty = nv3pManualDownMinDuty;
        }
      }
      direction = directionBefore;
      
      if(timerShock >= VAC_LIFT_DOWN_PWM_CHECK_TIME) {
        timerDither = 0;
        direction = VAC_HITCH_DIRECTION_OFF;
      }
      if(outDuty <= nv3pManualDownMinDuty + 10) {
        timerDither = 0;
        direction = VAC_HITCH_DIRECTION_OFF; 
        timerOutRun = 0; 
      }
    }
    else {
      timerDither = 0;
    }
  }
  else {
    timerShock = 0;
  }

  if((directionBefore == VAC_HITCH_DIRECTION_OFF) && (direction != VAC_HITCH_DIRECTION_OFF)) {
    if(timerStopRun < VAC_IMPULSE_CHECK_TIME) {
      direction = VAC_HITCH_DIRECTION_OFF;
    }
  }

  if(direction == VAC_HITCH_DIRECTION_OFF) {
    if(flagTimer.tenMs == TRUE)
    {
      timerStopRun += 10;
    }
    if(timerStopRun >= VAC_IMPULSE_CHECK_TIME) {
      timerStopRun = VAC_IMPULSE_CHECK_TIME + 1;
    }

    if(directionBefore != direction) {
      directionBefore = direction;
    }
    hitch_pwm_off_out();
    timerOutRun = 0;
    resultThreePDuty = 0;
    timerDither = 0;
    flag.liftDither = TRUE;
    return;
  }

  timerStopRun = 0;

  if(direction == VAC_HITCH_DIRECTION_UP) {
    if(flag.liftDither == TRUE) {
      if(timerDither >= VAC_IMPULSE_UP_TIME) {
        flag.liftDither = FALSE;
        timerDither = 0;
        timerOutRun = 0;
      }
    }
    else {
      timerDither = 0;
    }

    if(timerOutRun >= VAC_LIFT_UP_PWM_CHECK_TIME) {
      timerOutRun = VAC_LIFT_UP_PWM_CHECK_TIME;
    }
    else {
      if(flag.liftDither == FALSE) {
        calculateDuty = nv3pManualUpMinDuty * (uint16_t)timerOutRun / 10;       // Divided by 10
      }
      else {
        calculateDuty = 0;
      }
      calculateDuty += nv3pManualUpMinDuty;
      if(calculateDuty < outDuty) {
        outDuty = calculateDuty;
      }
    }

    flagOutputControl.hitchUp = ON;
    flagOutputControl.hitchDown = OFF;

    resultThreePDuty  = outDuty;
    resultThreePDuty += get_offset_battery();
    if(directionBefore != direction) {
      directionBefore = direction;
    }
    
    hitch_pwm_up_out(outDuty);
    hitch_pwm_down_out(0);
  }
  else if(direction == VAC_HITCH_DIRECTION_DOWN) {
    if(flag.liftDither == TRUE) {
      if(timerDither >= VAC_IMPULSE_DOWN_TIME) {
        flag.liftDither = FALSE;
        timerDither = 0;
        timerOutRun = 0;
      }
    }
    else {
      timerDither = 0;
    }

    if(timerOutRun >= VAC_LIFT_DOWN_PWM_CHECK_TIME) {
      timerOutRun = VAC_LIFT_DOWN_PWM_CHECK_TIME;
    }
    else {
      if(flag.liftDither == FALSE) {
        calculateDuty = nv3pManualDownMinDuty * (uint16_t)timerOutRun / 10;     // Divided by 10
      }
      else {
        calculateDuty = 0;
      }
      calculateDuty += nv3pManualDownMinDuty;
      if(calculateDuty < outDuty) {
        outDuty = calculateDuty;
      }
    }

    flagOutputControl.hitchUp = OFF;
    flagOutputControl.hitchDown = ON;
    
    resultThreePDuty  = outDuty;
    resultThreePDuty += get_offset_battery();
    if(directionBefore != direction) {
      directionBefore = direction;
    }
    
    hitch_pwm_down_out(outDuty);
    hitch_pwm_up_out(0);
  }
  dutyBefore = outDuty;
}



void lift_pwm_calculate(uint8_t _direction, uint16_t currentAngle, uint16_t targetAngle)
{
// Removed on 2025.07.17    static uint16_t  oneTouchDownLong = 0;
  static uint8_t   directionOld = 0;
  static uint8_t   directionPrev = 0;
  static uint16_t  beforeAngle = 0;
  static uint16_t  timerOutRun = 0;
  static uint16_t  timerDelayDown = 0;
  static uint16_t  timerDelayStall = 0;
  
  uint32_t         downSpeedRate;
  int16_t          differenceOfAngle;
  
  uint16_t         abError, duty, deadBand;
  uint8_t          direction, positionPoint;
  
  uint16_t         downSpeedPercentage;
  
  if(flagTimer.tenMs == TRUE)
  {
    timerDelayDown += 10;
  }

  if(_direction == VAC_HITCH_DIRECTION_OFF) {
    directionOld = VAC_HITCH_DIRECTION_OFF;
    directionPrev = VAC_HITCH_DIRECTION_OFF;
    run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
    return;
  }

  if(directionOld == VAC_HITCH_DIRECTION_OFF) {
    deadBand = nvThreePDownPosition[0] + VAC_LIFT_DEADBAND_ANGLE_2;
  }
  else {
    deadBand = nvThreePDownPosition[0];
  }

  differenceOfAngle = (int16_t)currentAngle - (int16_t)targetAngle;

  if(differenceOfAngle < 0) {
    abError = -1 * (int16_t)differenceOfAngle;
    direction = VAC_HITCH_DIRECTION_UP;
  }
  else {
    abError = differenceOfAngle;
    direction = VAC_HITCH_DIRECTION_DOWN;
  }

  if(changeDirection == FALSE)
  {
    if((directionPrev == VAC_HITCH_DIRECTION_UP) && (direction == VAC_HITCH_DIRECTION_DOWN))
    {
      direction = VAC_HITCH_DIRECTION_OFF;
      directionOld = VAC_HITCH_DIRECTION_OFF;
      run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
      return;
    }
    else if((directionPrev == VAC_HITCH_DIRECTION_DOWN) && (direction == VAC_HITCH_DIRECTION_UP))
    {
      direction = VAC_HITCH_DIRECTION_OFF;
      directionOld = VAC_HITCH_DIRECTION_OFF;
      run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
      return;
    }
  }
  else
  {
    directionPrev = direction;
    changeDirection = FALSE;
  }
  
  if(directionOld == VAC_HITCH_DIRECTION_OFF) {
    if(abError <= nvThreePUpPosition[1]) {
      flag.quickMove = TRUE;
    }
    else {
      flag.quickMove = FALSE;
    }
  }
   	
  positionPoint = 0;

  if (direction == VAC_HITCH_DIRECTION_UP) {
         if(abError <= deadBand)                        direction = VAC_HITCH_DIRECTION_OFF;
    else if(abError <= nvThreePUpPosition[1])           positionPoint = 1;
    else if(abError <= nvThreePUpPosition[2])           positionPoint = 2;
    else if(abError <= nvThreePUpPosition[3])           positionPoint = 3;
    else if(abError <= nvThreePUpPosition[4])           positionPoint = 4;
    else if(abError <= nvThreePUpPosition[5])           positionPoint = 5;
    else if(abError <= nvThreePUpPosition[6])           positionPoint = 6;
    else if(abError <= nvThreePUpPosition[7])           positionPoint = 7;
    else                                                positionPoint = 8;
  }
  else {
         if(abError <= deadBand)                        direction = VAC_HITCH_DIRECTION_OFF;
    else if(abError <= nvThreePDownPosition[1])         positionPoint = 1;
    else if(abError <= nvThreePDownPosition[2])         positionPoint = 2;
    else if(abError <= nvThreePDownPosition[3])         positionPoint = 3;
    else if(abError <= nvThreePDownPosition[4])         positionPoint = 4;
    else if(abError <= nvThreePDownPosition[5])         positionPoint = 5;
    else if(abError <= nvThreePDownPosition[6])         positionPoint = 6;
    else if(abError <= nvThreePDownPosition[7])         positionPoint = 7;
    else                                                positionPoint = 8;
  }

  if(positionPoint >= 2) {
    flag.quickMove = FALSE;
  }
  if(positionPoint <= 4) {
    timerOutRun = 0;
  }
  if(direction == VAC_HITCH_DIRECTION_UP) {
    switch(positionPoint) {
      case 0 :  duty = 0; break;
      case 1 :  duty = (uint32_t) (nvThreePUpCurrent[1] - nvThreePUpCurrent[0]) * (uint32_t)(abError - nvThreePUpPosition[0]) / (uint32_t)(nvThreePUpPosition[1] - nvThreePUpPosition[0]);
                duty += nvThreePUpCurrent[0];
                if(flag.quickMove == TRUE) {
                  duty += nv3pQuickUpStepDuty;
                }
                break;
      case 2 :  duty = (uint32_t)(nvThreePUpCurrent[2] - nvThreePUpCurrent[1]) * (uint32_t)(abError - nvThreePUpPosition[1]) / (uint32_t)(nvThreePUpPosition[2] - nvThreePUpPosition[1]);
                duty += nvThreePUpCurrent[1];
                break;
      case 3 :  duty = (uint32_t)(nvThreePUpCurrent[3] - nvThreePUpCurrent[2]) * (uint32_t)(abError - nvThreePUpPosition[2]) / (uint32_t)(nvThreePUpPosition[3] - nvThreePUpPosition[2]);
                duty += nvThreePUpCurrent[2];
                break;
      case 4 :  duty = (uint32_t)(nvThreePUpCurrent[4] - nvThreePUpCurrent[3]) * (uint32_t)(abError - nvThreePUpPosition[3]) / (uint32_t)(nvThreePUpPosition[4] - nvThreePUpPosition[3]);
                duty += nvThreePUpCurrent[3];
                break;
      case 5 :  duty = (uint32_t)(nvThreePUpCurrent[5] - nvThreePUpCurrent[4]) * (uint32_t)(abError - nvThreePUpPosition[4]) / (uint32_t)(nvThreePUpPosition[5] - nvThreePUpPosition[4]);
                duty += nvThreePUpCurrent[4];
                break;
      case 6 :  duty = (uint32_t)(nvThreePUpCurrent[6] - nvThreePUpCurrent[5]) * (uint32_t)(abError - nvThreePUpPosition[5]) / (uint32_t)(nvThreePUpPosition[6] - nvThreePUpPosition[5]);
                duty += nvThreePUpCurrent[5];
                break;
      case 7 :  duty = (uint32_t)(nvThreePUpCurrent[7] - nvThreePUpCurrent[6]) * (uint32_t)(abError - nvThreePUpPosition[6]) / (uint32_t)(nvThreePUpPosition[7] - nvThreePUpPosition[6]);
                duty += nvThreePUpCurrent[6];
                break;  
      default:  
                if(timerOutRun >= 750) {                                        // Enkhat changed 250 to 750 on 2021.12.10
                  timerOutRun = 750;
                }
                else { 
                  timerOutRun += 2;
                }
                
                duty = nvThreePUpCurrent[7] + timerOutRun;
                if(duty >= nv3pAutoUpMaxDuty) {
                  duty = nv3pAutoUpMaxDuty;                                  // 3P Maximum duty --> 
                }
                break;
    }
  }
  else if(direction == VAC_HITCH_DIRECTION_DOWN) {
    switch (  positionPoint ) {
      case 0 :  duty = 0; break;
      case 1 :  duty = (uint32_t)(nvThreePDownCurrent[1] - nvThreePDownCurrent[0]) * (uint32_t)(abError - nvThreePDownPosition[0]) / (uint32_t)(nvThreePDownPosition[1] - nvThreePDownPosition[0]);
                duty += nvThreePDownCurrent[0];
                if(flag.quickMove == TRUE) {
                  duty += nv3pQuickDownStepDuty;
                }
                break;
      case 2 :  duty = (uint32_t)(nvThreePDownCurrent[2] - nvThreePDownCurrent[1]) * (uint32_t)(abError - nvThreePDownPosition[1])/ (uint32_t)(nvThreePDownPosition[2] - nvThreePDownPosition[1]);
                duty += nvThreePDownCurrent[1];
                break;
      case 3 :  duty = (uint32_t)(nvThreePDownCurrent[3] - nvThreePDownCurrent[2]) * (uint32_t)(abError - nvThreePDownPosition[2])/ (uint32_t)(nvThreePDownPosition[3] - nvThreePDownPosition[2]);
                duty += nvThreePDownCurrent[2];
                break;
      case 4 :  duty = (uint32_t)(nvThreePDownCurrent[4] - nvThreePDownCurrent[3])   *(uint32_t)(abError - nvThreePDownPosition[3])/ (uint32_t)(nvThreePDownPosition[4] - nvThreePDownPosition[3]);
                duty += nvThreePDownCurrent[3];
                break;
      case 5 :  duty = (uint32_t)(nvThreePDownCurrent[5] - nvThreePDownCurrent[4])   *(uint32_t)(abError - nvThreePDownPosition[4])/ (uint32_t)(nvThreePDownPosition[5] - nvThreePDownPosition[4]);
                duty += nvThreePDownCurrent[4];
                break;
      case 6 :  duty = (uint32_t)(nvThreePDownCurrent[6] - nvThreePDownCurrent[5])   *(uint32_t)(abError - nvThreePDownPosition[5])/ (uint32_t)(nvThreePDownPosition[6] - nvThreePDownPosition[5]);
                duty += nvThreePDownCurrent[5];
                break;
      case 7 :  duty = (uint32_t)(nvThreePDownCurrent[7] - nvThreePDownCurrent[6])   *(uint32_t)(abError - nvThreePDownPosition[6])/ (uint32_t)(nvThreePDownPosition[7] - nvThreePDownPosition[6]);
                duty += nvThreePDownCurrent[6];
                break;
      default : 
                if(timerOutRun >= 750) {                                        // Enkhat changed 250 to 750 on 2021.12.10
                  timerOutRun = 750;
                }
                else { 
                  timerOutRun += 2;
                }
                duty = nvThreePDownCurrent[7] + timerOutRun;
                if(duty >= nv3pAutoDownMaxDuty) {
                  duty = nv3pAutoDownMaxDuty;
                }
                break;
    }
  }
  else {
    duty = 0;
    flag.oneDownRun = FALSE;
  }

  if(flagTimer.tenMs == TRUE)
  {
    timerDelayStall += 10;
    if(timerDelayStall >= 50)
    {
      timerDelayStall  = 0;
      if(positionPoint != 0) {
        differenceOfAngle = (int16_t)beforeAngle - (int16_t)currentAngle;
        if(differenceOfAngle < 0) {
          abError = -1 * (int16_t)differenceOfAngle; 
        }
        else {
          abError = differenceOfAngle; 
        }
      
        if(abError >= 10) {
          // // changed this part on 2025.06.17
          if(direction == VAC_HITCH_DIRECTION_UP)
          {
            if(stallCompUp >= 100) {
              stallCompUp -= 30;
            }
            else if(stallCompUp >= 10) {
              stallCompUp -= 10;
            }
            else {
              stallCompUp = 0;
            }
            
            if(stallCompUp < 0)
              stallCompUp = 0;
          }
          else
          {
            if(stallCompDown >= 100) {
              stallCompDown -= 30;
            }
            else if(stallCompDown >= 10) {
              stallCompDown -= 10;
            }
            else
            {
              stallCompDown = 0;
            }
            
            if(stallCompDown < 0)
              stallCompDown = 0;
          }
        }
        else if(abError <= 2)                                                     // Changed on 2025.06.17 from 1 to 2
        {
          // changed this part on 2025.06.17
          if(direction == VAC_HITCH_DIRECTION_UP) {
            stallCompUp += 3;//nv3pStallStepDuty;                                      // 6% ==> 0.6%, 3% --> 0.5%
            if(stallCompUp >= 200) {                                              // Max 20% changed on 2025.06.17
              stallCompUp   = 200;
            }
          }
          else {
            stallCompDown += 3;//nv3pStallStepDuty;                                    // 6% ==> 0.6%, 3% --> 0.5%
            if(stallCompDown >= 200) {                                            // Max 20% changed on 2025.06.17
              stallCompDown = 200;
            }
          }
        }
      }
      else { 
        stallCompUp = 0;
        stallCompDown = 0;
      }    
      beforeAngle =  currentAngle;
    }
  }
  /* Removed on 2025.07.17
  if((leverAngle <= 960) && (canRxArmRest.hitchDownButton == ON)) {
    oneTouchDownLong += 2;
  }
  else{
    oneTouchDownLong = 0;
  }
  
  if(oneTouchDownLong >= 500)                                                   //200ms
  {
    oneTouchDownLong = 500;
    if(((positionAngle - VAC_LIFT_DEADBAND_ANGLE_3) <= leverAngle) && ((positionAngle + VAC_LIFT_DEADBAND_ANGLE_3) >= leverAngle))
    {
      timerDelayDown = 0;
    }
    else {
      if(timerDelayDown >= 100) {                                               // 100 ms
        if(direction != VAC_HITCH_DIRECTION_DOWN) {
          duty = nv3pManualDownMaxDuty;
        }
        direction = VAC_HITCH_DIRECTION_DOWN;
        timerDelayDown = 550;
      }
    }
  }
  else {
    timerDelayDown = 0;
  }
  */
  
  if(direction == VAC_HITCH_DIRECTION_UP) {
    stallCompDown = 0;                                                          // Added on 2025.07.17
    duty += stallCompUp;
    if(duty >= 900) {
      duty = 900;
    }
  }
  else {
    stallCompUp = 0;                                                            // Added on 2025.07.17
    duty += stallCompDown;
    if(duty >= 900) {
      duty = 900;
    }
  } 
  directionOld = direction;

  if(direction != VAC_HITCH_DIRECTION_DOWN) {
    run_hitch_pwm(direction, duty);
    return;
  }
  
  downSpeedPercentage = canRxDial.hitchDownSpeed * 4;

  if(downSpeedPercentage <= VAC_DOWN_SPEED_CUT_OFF)
  {
    direction = VAC_HITCH_DIRECTION_OFF;
    duty = 0;
  }
  /* Removed the condition on 2025.07.17
  else if(timerDelayDown >= 100)
  {
    duty = nv3pQuickDownDuty;                                                   // The value is condifured by diagnostic
  }
  */
  else
  {
    if(VAC_DOWN_SPEED_MAX <= downSpeedPercentage)
    {
      downSpeedRate = nv3pAutoDownMaxDuty;
    }
    else if(VAC_LIFT_DOWN_SET_POS_MID <= downSpeedPercentage)                   // Over 50%
    {
      // (600 - 290) * (200 - 130) / (244 - 130) = 310 * 70 / 114 = 190
      // 290 + 190 = 480
      downSpeedRate  = ((uint16_t)(nv3pAutoDownMaxDuty - nv3pAutoDownMidDuty) * (uint32_t)(downSpeedPercentage - VAC_LIFT_DOWN_SET_POS_MID)); //
      downSpeedRate = downSpeedRate /(uint16_t)(VAC_DOWN_SPEED_MAX - VAC_LIFT_DOWN_SET_POS_MID);
      downSpeedRate += (uint16_t)nv3pAutoDownMidDuty;
      if(duty > downSpeedRate) {
        duty = (uint16_t)downSpeedRate;
      }
    }
    else                                                                        // Under 50%
    {
      // (290 - 220) * (120 - 10) / (130 - 10) = 70 * 110 / 120 = 64.16
      // 190 + 64 = 254
      downSpeedRate  = ((uint16_t)(nv3pAutoDownMidDuty - nvThreePDownCurrent[0]) * (uint32_t)(downSpeedPercentage - VAC_DOWN_SPEED_CUT_OFF));
      downSpeedRate = downSpeedRate / (uint16_t)(VAC_LIFT_DOWN_SET_POS_MID - VAC_DOWN_SPEED_CUT_OFF); //
      downSpeedRate += (uint16_t)nv3pAutoDownMinDuty;
      if(duty > downSpeedRate) {
        duty = (uint16_t)downSpeedRate;
      }
    }
  }
   
  if(flag.floating == TRUE) {
    if(currentAngle < VAC_LIFT_FLOATING) {
      if(duty >= nvThreePDownCurrent[2]) {
        duty = nvThreePDownCurrent[2];
      }
    }
  }
  
  run_hitch_pwm(direction, duty);
  return;
}

void run_one_up_process()
{
  flag.liftLock = FALSE;
  lift_pwm_calculate(VAC_HITCH_DIRECTION_RUN, positionAngle, upLimitAngle);
  return;
}

uint16_t objectAngle;
void run_depth_process()
{
  static uint16_t timerDownControl = 0;
  static uint16_t timerUpControl = 0;
  static uint16_t timerDownPosition = 0;
  int8_t i;
  uint8_t  direction;
  uint16_t differenceOfDepth;
  uint16_t duty, depthSet;
  uint16_t depthSensorData = get_threeP_depth_sensor(); 
  uint16_t downSpeedPercentage = canRxDial.hitchDownSpeed * 4;

  if(flagErrorSensors.threePDepth == TRUE) {                              // If depth sensor has an error the depth control is NOT working
    direction = VAC_HITCH_DIRECTION_OFF;
    duty = 0;
    objectAngle = 0;
    run_hitch_pwm(direction, duty);
    return;
  }
  
  if(flagTimer.hundredMs == TRUE)
  {
    timerDepthDelay += 100;
  }

  depthSet = (uint16_t)(VAC_DEPTH_UP_LIMIT - nvDepthDownLimit);
  depthSet = (uint16_t)(((float)canRxDial.subSetting * (float)depthSet) / 250.0);
  depthSet += nvDepthDownLimit;

  if(depthSet > VAC_DEPTH_UP_LIMIT) {
    depthSet = VAC_DEPTH_UP_LIMIT;
  }
  if(depthSet < nvDepthDownLimit) {
    depthSet = nvDepthDownLimit;
  }
  
  if(timerDepthDelay >= 2500) {
    flag.depthOnRun= TRUE;
    timerDepthDelay = 3000; 
  }
  
  if(VAC_DEPTH_SET_MAX >= depthSensorData) {    // sensor data lower than 1.05V = 53
    if(timerDownPosition < nvDepthTime) {                                       // The setting is configured by Diagnostic program
      timerDownPosition += 2;
    }
    else {
      // over 500ms
      flag.depthOnRun = FALSE;
    }
  }
  else {
    timerDownPosition = 0;
  }

  if(flag.depthOnRun == FALSE) {
    lift_pwm_calculate(VAC_HITCH_DIRECTION_RUN, positionAngle, leverAngle);
    return;
  }

  if(depthSet < (depthSensorData - nvThreePDepthUpPosition[0]))
  {
    timerDownControl = 0;
    if(timerUpControl < nvDepthSwitchOutputTime) {
      timerUpControl += 2;
      direction = VAC_HITCH_DIRECTION_OFF;
      duty = 0; 
    }
    else if(positionAngle >= VAC_LIFT_UP_MODE_LIMIT_ANGLE) {                        // changed on 2021.12.13
      direction = VAC_HITCH_DIRECTION_OFF;
      duty = 0; 
    }
    else {
      differenceOfDepth = (depthSensorData - nvThreePDepthUpPosition[0]) - depthSet;
      direction = VAC_HITCH_DIRECTION_UP;
      
      for(i = 7; i > 0; i--) {
        if(differenceOfDepth > nvThreePDepthUpPosition[i])
        {
          duty = nvThreePDepthUpCurrent[i];
          break;
        }
      }
      
      if(i == 0) {
        duty = nvThreePDepthUpCurrent[i];
      }
    }
    objectAngle = 0;

    run_hitch_pwm(direction, duty);
    return;
  }
  else if((positionAngle - nvThreePDownPosition[0]) >= leverAngle)
  {
    timerUpControl = 0;
    if(timerDownControl < nvDepthSwitchOutputTime) {
      timerDownControl += 2;
      direction = VAC_HITCH_DIRECTION_OFF;
      duty = 0;
      objectAngle = 0;
      run_hitch_pwm(direction, duty);
      return;
    }
    
    if(depthSet > depthSensorData + nvThreePDepthDownPosition[0])
    {
      if(depthSet > depthSensorData)
        differenceOfDepth = depthSet - depthSensorData;
      else
        differenceOfDepth = depthSensorData - depthSet;
    
      if(downSpeedPercentage >= VAC_DOWN_SPEED_CUT_OFF)
      {
        direction = VAC_HITCH_DIRECTION_DOWN;

        for(i = 7; i > 0; i--) {
          if(differenceOfDepth > nvThreePDepthDownPosition[i])
          {
            duty = nvThreePDepthDownCurrent[i];
            break;
          }
        }
        
        if(i == 0) {
          duty = nvThreePDepthDownCurrent[i];
        }
      }
      else {
        direction = VAC_HITCH_DIRECTION_OFF;
        duty = 0; 
      }
      objectAngle = 0;
      run_hitch_pwm(direction, duty);
      return;
    }
  }
  else {
    lift_pwm_calculate(VAC_HITCH_DIRECTION_RUN, positionAngle, leverAngle);
    return;
  }
  lift_pwm_calculate(VAC_HITCH_DIRECTION_OFF, 0, 0);
  return;
}

void run_draft_process()
{
  uint16_t draftSet;
  uint16_t duty;
  uint16_t draftSensorData = get_threeP_draft_sensor();
  int8_t i;
  
  draftSet = (uint16_t)(VAC_DRAFT_SENSOR_MAX - VAC_DRAFT_SENSOR_MIN);
  draftSet = (uint16_t)(((float)draftSet * (float)canRxDial.subSetting) / 250.0);
  draftSet += VAC_DRAFT_SENSOR_MIN;

  if(draftSet > VAC_DRAFT_SENSOR_MAX) {
    draftSet = VAC_DRAFT_SENSOR_MAX;
  }
  else if(draftSet < VAC_DRAFT_SENSOR_MIN) {
    draftSet = VAC_DRAFT_SENSOR_MIN;
  }
    
  if(draftSensorData > (draftSet + nvThreePDraftUpPosition[0])) {                                        // Higher than setted position
    if(positionAngle >= VAC_LIFT_UP_MODE_LIMIT_ANGLE) {
      run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
    }
    else {
      for(i = 7; i >= 0; i--) {
        if(draftSensorData > (draftSet + nvThreePDraftUpPosition[i])) {
          duty = nvThreePDraftUpCurrent[i];
          break;
        }
      }
      
      if(i == 0) {
        duty = nvThreePDraftUpCurrent[i];
      }
      
      run_hitch_pwm(VAC_HITCH_DIRECTION_UP, duty);
    }
    return;
  }
  else if(draftSensorData < VAC_DRAFT_SENSOR_MIN) {                                                        // Lower than draft lower position
    changeDirection = TRUE;
    lift_pwm_calculate(VAC_HITCH_DIRECTION_RUN, positionAngle, leverAngle);
    return;
  }
  else if(draftSensorData < (draftSet - nvThreePDraftDownPosition[0])) {                                  // Lower than setted position
    
    if(leverAngle > positionAngle) {
      changeDirection = TRUE;
      lift_pwm_calculate(VAC_HITCH_DIRECTION_RUN, positionAngle, leverAngle);
    }
    else {
      for(i = 7; i >= 0; i--) {
        if(draftSensorData < (draftSet - nvThreePDraftDownPosition[i]))
        {
          duty = nvThreePDraftDownCurrent[i];
          break;
        }
      }
      
      if(i == 0) {
        duty = nvThreePDraftDownCurrent[i];
      }
      run_hitch_pwm(VAC_HITCH_DIRECTION_DOWN, duty);
    }
    return;
  }

  lift_pwm_calculate(VAC_HITCH_DIRECTION_OFF, 0, 0);
  return;
}

void lift_control_process()
{
  static uint16_t manualDownDuty;
  static uint16_t manualUpDuty;
  static uint16_t timerManualDown = 0;
  static uint16_t timerManualUp = 0;
  static uint16_t timerSafety = 0;
  static uint16_t timerFloating = 0;
//  static uint8_t timerOutOff = 0; 

//  uint8_t chargeLiftOff; 
  static uint16_t  liftDepthSensorPrevious = 0;                                // changed the variable type from uint8_t to uint16_t
  static uint16_t  positionAngleBefore = 0;
  static uint16_t  leverAngleBefore = 0;
  
  static uint16_t  flagLeverAngle_Move =0;
  static uint16_t  timerLeverAngle_Move =0;
  static uint16_t  timerCheckLeverMove = 0;
  
  static uint16_t  timerTurnBackRpmCruiseAutoOff = 0;
  
  uint8_t flagCheck;
  uint8_t flagCheck2;

  //leverAngleForLeverControl = get_threeP_lever_sensor();
  leverAngleForLeverControl = get_threeP_lever_sensor_average();                // max 1000
    
  //leverAngle    = get_threeP_lever_sensor();
  leverAngle    = get_threeP_lever_sensor_average();                            // max 1000
  
  upLimitAngle  = canRxDial.upLimit * 4;                                        // max = 250 * 4 = 1000 ==> 100%
  
  positionAngle = get_threeP_position_sensor();                                 // it is 10 bit, changed on 2021.12.13
  
   
  if((positionAngle <= VAC_LIFT_FLOATING) && (leverAngle <= VAC_LIFT_FLOATING))
  {
    flag.floating = TRUE;
    timerFloating = 0;
  }
  else
  {
    if(leverAngle >= (VAC_LIFT_FLOATING + 8))
    {
      timerFloating += 2;
      if(timerFloating >= VAC_LIFT_FLOATING_TIME) {                             // 200ms
        // we need delay 0.1 seconds
        timerFloating = VAC_LIFT_FLOATING_TIME;
        flag.floating = FALSE;
      }
    }
  }
  
  if(positionAngle >= 1000) {                                                   // changed position angle from 1023 to 1000 on 2022.01.13 ¹Ú°­È£ ÆÀÀå´Ô
    positionAngle = 1000;
  }
  
  if(leverAngle >= VAC_LIFT_COUNT_MAX) {
    leverAngle = VAC_LIFT_COUNT_MAX;
  }
  if(upLimitAngle >= VAC_LIFT_COUNT_MAX) {
    upLimitAngle = VAC_LIFT_COUNT_MAX;
  }

  if(leverAngle > upLimitAngle) {                                               // The lowest value is save in leverAngle.
    leverAngle = upLimitAngle;
  }

  flagCheck = FALSE;
     
  if(flagHitch.settingMode != VAB_HITCH_SETTING_MODE_OFF)
  {
    flagCheck = TRUE;
  }
   
  if(flagCheck == TRUE) {
    flag.oneUpRun = FALSE;
    flag.oneDownRun = FALSE;
    flag.turnUpRun = FALSE;
    flag.backUpRun = FALSE;
    
    flag.turnDownRpmRun = FALSE;
    flag.backDownRpmRun = FALSE;
    
    flag.liftLock = TRUE;
    timerManualDown = 0;
    timerManualUp = 0;
    return;
  }

  // Turn up is ACTIVE && also one of the "steeringLeft" and "steeringRight" inputs are ACTIVE
  if(canFlag.turnUpHitch == TRUE) {                                                                           // Panel turn up Lamp output
    //if((flag.steeringLeft == ON) || (flag.steeringRight == ON))
    if(flag.isSteeringON == TRUE)
    {
      if(flag.oneUpRun == FALSE) {
        flag.turnUpRun = TRUE;
      }
    }
    else {      
      if(flag.turnUpRun == TRUE) {
        flag.oneUpRun = FALSE;
        flag.backUpRun = FALSE;
        if(nvHitchDownWhenTurnBackUpOn == OFF)                     // Added on 2025.07.17
        {
          flag.oneDownRun = FALSE;
          flag.liftLock = TRUE;
        }
        else
        {
          flag.oneDownRun = TRUE;
          changeDirection = TRUE;
        }
      }
      flag.turnUpRun = FALSE;
    }
  }
  else {
    flag.turnUpRun = FALSE;
  }
  
  if(canFlag.turnDownRpm == TRUE)
  {
    if(flag.isSteeringON == TRUE)
    {
      flag.turnDownRpmRun = TRUE;
    }
    else
    {
      flag.turnDownRpmRun = FALSE;
    }
  }
  else
  {
    flag.turnDownRpmRun = FALSE;
  }
  
  if(canFlag.backUpHitch == TRUE) {                                                  // Panel back up Lamp output
    if(flagShuttle.backward == ON) {                            // Also, back switch or shuttle is in backward direction
      if(flag.oneUpRun == OFF) {
        flag.backUpRun = ON;
      }
    }
    else 
    {
      if(flag.backUpRun == TRUE) {
        flag.oneUpRun = FALSE;
        flag.turnUpRun = FALSE; 
        if(nvHitchDownWhenTurnBackUpOn == OFF)                     // Added on 2025.07.17
        {
          flag.oneDownRun = FALSE;
          flag.liftLock = TRUE;
        }
        else
        {
          flag.oneDownRun = TRUE;
          changeDirection = TRUE;
        }
      }
      flag.backUpRun = FALSE;
    }
  }
  else {
    flag.backUpRun = FALSE;
  }
   
  if(canFlag.backDownRpm == TRUE)
  {
    if(flagShuttle.backward == ON)
    {
      flag.backDownRpmRun = TRUE;
    }
    else
    {
      flag.backDownRpmRun = FALSE;
    }
  }
  else
  {
    flag.backDownRpmRun = FALSE;
  }
  
  if((flag.turnDownRpmRun == TRUE) || (flag.backDownRpmRun == TRUE))
  {
    timerTurnBackRpmCruiseAutoOff = 0;
    flag.rpmCruiseAuto = TRUE;
    flag.rpmCruiseB = TRUE;
    flag.rpmCruiseA = FALSE;                    // Added on 2025.02.25
  }
  else
  {
    if(flag.rpmCruiseAuto == TRUE)
    {
      timerTurnBackRpmCruiseAutoOff += 2;
      if(timerTurnBackRpmCruiseAutoOff >= 2000)
      {
        flag.rpmCruiseAuto = FALSE;
        flag.rpmCruiseB = FALSE;
        flag.rpmCruiseA = FALSE;                  // Added on 2025.02.25
      }
    }
    else
    {
      timerTurnBackRpmCruiseAutoOff = 0;
    }
  }
  
// disabled on 2022.09.22  flag.buzzerOn = FALSE;
  
  if((canRxArmRest.hitchUpButton == CAN_SP_BUTTON_ON) && (canRxArmRestPrevious.hitchUpButton == CAN_SP_BUTTON_OFF))
  {
//    canRxArmRestPreviousApp.hitchUpButton  = canRxArmRest.hitchUpButton;
    flag.oneUpRun = TRUE;
    changeDirection = TRUE;
    flag.oneDownRun = FALSE;
    flag.turnUpRun = FALSE;
    flag.backUpRun = FALSE;     
    flag.liftLock = FALSE;
  }
  else if((canRxArmRest.hitchDownButton == CAN_SP_BUTTON_ON) && (canRxArmRestPrevious.hitchDownButton == CAN_SP_BUTTON_OFF))
  {
//    canRxArmRestPreviousApp.hitchDownButton = canRxArmRest.hitchDownButton;
    
    flagCheck = FALSE;
    flagCheck2 = FALSE;
    
    if((flag.oneUpRun == TRUE) || (flag.turnUpRun == TRUE) || (flag.backUpRun == TRUE)) {
      flagCheck = TRUE;
    }
    
    if(flagCheck == TRUE) {
      if((upLimitAngle - nvThreePUpPosition[0]) <= positionAngle) {                                    // Deadband -- changed on 2022.09.22 because it is clicked two times
        flagCheck2 = TRUE;
      }
    }
    else {
      flagCheck2 = TRUE;
    }
    
    if(flagCheck2 == TRUE) {
      changeDirection = TRUE;
      
      flag.oneDownRun = TRUE;
      flag.liftLock = FALSE;
      flag.ptoLiftLock = FALSE;
    }
    else  {
      flag.oneDownRun = FALSE;
      flag.liftLock = TRUE;
      flag.ptoLiftLock = TRUE;
    }  
    flag.oneUpRun = FALSE;
    flag.turnUpRun = FALSE; 
    flag.backUpRun = FALSE;  
  }
  else if((flagInputStatus.hitchManualUp == ON) || (flagInputStatus.hitchManualDown == ON) ||
          (canRxPillar.hitchUpButton == CAN_SP_BUTTON_ON) || (canRxPillar.hitchDownButton == CAN_SP_BUTTON_ON)) {                 // External buttons
    flag.oneUpRun = FALSE;
    flag.oneDownRun = FALSE;
    flag.turnUpRun = FALSE;
    flag.backUpRun = FALSE;
    flag.liftLock = TRUE;

    if(((flagInputStatus.hitchManualUp == ON) || (canRxPillar.hitchUpButton == CAN_SP_BUTTON_ON)) &&  
        (flagInputStatus.hitchManualDown == OFF) && (canRxPillar.hitchDownButton == CAN_SP_BUTTON_OFF)) {
      if(flagTimer.tenMs == TRUE)
      {
        timerManualUp += 10;            // Check again
        if(timerManualUp < nv3pManualUpCycle) {
          manualUpDuty = (uint16_t)((float)nv3pManualUpMinDuty + ((float)nv3pManualUpMaxDuty - (float)nv3pManualUpMinDuty) * ((float)timerManualUp) / (float)nv3pManualUpCycle);
        }
        else {
          timerManualUp = nv3pManualUpCycle;
          manualUpDuty = nv3pManualUpMaxDuty;
        }
        if(manualUpDuty > nv3pManualUpMaxDuty) {
          manualUpDuty = nv3pManualUpMaxDuty;
        }
      }
      
      run_hitch_pwm(VAC_HITCH_DIRECTION_UP, manualUpDuty);
      flagInputStatusPrevious.hitchManualUp = ON;
      return;
    }
    else if(((flagInputStatus.hitchManualDown == ON) || (canRxPillar.hitchDownButton == CAN_SP_BUTTON_ON)) && 
             (flagInputStatus.hitchManualUp == OFF) && (canRxPillar.hitchUpButton == CAN_SP_BUTTON_OFF)) {
      if(flagTimer.tenMs == TRUE)
      {
        timerManualDown += 10;                  // Check again
        if(timerManualDown < nv3pManualDownCycle) {
          manualDownDuty  = (uint16_t)((float)nv3pManualDownMinDuty + ((float)nv3pManualDownMaxDuty - (float)nv3pManualDownMinDuty) * ((float)timerManualDown) / (float)nv3pManualDownCycle);
        }
        else {
          timerManualDown = nv3pManualDownCycle;
          manualDownDuty = nv3pManualDownMaxDuty;
        }
        
        if(manualDownDuty > nv3pManualDownMaxDuty) {
          manualDownDuty = nv3pManualDownMaxDuty;
        }
      }
      run_hitch_pwm(VAC_HITCH_DIRECTION_DOWN, manualDownDuty);
      flagInputStatusPrevious.hitchManualDown = ON;
      return;
    }
  }
  timerManualDown = 0;
  timerManualUp = 0;
  /*
  if((flagInputStatusPrevious.threePUp == ON) || (flagInputStatusPrevious.threePDown == ON)) {              // Added on 2021.11.29
    // If 3P manual mode is implemented in previously, it should be turned off fast.
    flagInputStatusPrevious.threePUp = OFF;
    flagInputStatusPrevious.threePDown = OFF;
    manualOff = 1;
    run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
    //hitch_pwm_off_out();
    return;
  }
  manualOff = 0;
  */

  if(flag.armSetConnected == TRUE) {                                       // Check this flag again
    flag.oneUpRun = FALSE;
    flag.oneDownRun = FALSE;
    flag.turnUpRun = FALSE;
    flag.backUpRun = FALSE;
    flag.liftLock = TRUE;
    run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
    return;  
  }
  flagPrevious.liftLock = flag.liftLock;
    
  if(canFlag.hitchControlMode != canFlagPrevious.hitchControlMode) {	        // If working mode is changed
    if(positionAngle > positionAngleBefore + nvThreePDownPosition[0]) {
      flag.liftLock = TRUE;
    }
    else if(positionAngleBefore >= nvThreePDownPosition[0]) {
      if(positionAngle < positionAngleBefore - nvThreePDownPosition[0]) {
        flag.liftLock = TRUE;
      }
    }
  }
  positionAngleBefore = positionAngle;
  
  canFlagPrevious.hitchControlMode = canFlag.hitchControlMode;
  //
  // Lever Move check 
  //
  if(flagTimer.hundredMs == TRUE)
  {
    timerCheckLeverMove += 100;
    if(timerCheckLeverMove >= 200) {                                              // 200 ms
      timerCheckLeverMove = 0;
      if(((leverAngleBefore + 8) <= leverAngle) || ((leverAngleBefore - 8) >= leverAngle)) {
        timerLeverAngle_Move = 10; 
      }
      leverAngleBefore = leverAngle;

      if(timerLeverAngle_Move != 0) {
        timerLeverAngle_Move--;
        flagLeverAngle_Move = TRUE;
      }
      else {
        flagLeverAngle_Move = FALSE;
      }
    }
  }  
         
  if((flag.liftLock == TRUE) && (flagLeverAngle_Move == TRUE)) {
    if(((positionAngle - VAC_LIFT_DEADBAND_ANGLE_3) <= leverAngle) && ((positionAngle + VAC_LIFT_DEADBAND_ANGLE_3) >= leverAngle)){                         // Changed on 2021.12.15
      flag.liftLock = FALSE;
    } 
  }

  // Following things are added on 2021.12.17
  if(engineSpeed <= 400) {                                                       // If RPM is over 400, the hitch controller should be called. 2021.12.17
    flag.liftLock = TRUE;
    timerSafety = 0;
  }
  else {
    if(timerSafety <= 1500) {
      if(timerSafety <= 500) {
        flag.liftLock = TRUE;
      }
      if(flagTimer.hundredMs == TRUE)
      {
        timerSafety += 100;
      }
    }
  }
  
  if((flag.oneUpRun == FALSE) && (flag.oneDownRun == FALSE)) {
    if((flagPrevious.liftLock == TRUE) && (flag.liftLock == FALSE)) {
      if(timerSafety >= 1000)
        flag.checkBuzzer = TRUE;                  // This part is changed on 2021.12.17
    }
  }

  if((flag.turnUpRun == TRUE) || (flag.backUpRun == TRUE)) {
    flag.oneUpRun = TRUE;
    changeDirection = TRUE;
  }
 
  if(flag.liftLock == TRUE) {
    lift_pwm_calculate(VAC_HITCH_DIRECTION_OFF, 0, 0);
    return;
  }
  
  if(flag.oneUpRun == TRUE) {
    if(leverAngleForLeverControl >= 900) {
      flag.oneUpRun = FALSE;
      flag.turnUpRun = FALSE; 
      flag.backUpRun = FALSE;
      flag.leverDownRun = TRUE;
    }
    flag.floating = FALSE;
    flagPrevious.floating = FALSE;                                        // Added on 2021.12.14
    run_one_up_process();
    return;
  }
  
  if((flag.leverDownRun == TRUE) && (leverAngleForLeverControl <= 680)) {                  // Added this case on 2021.12.08
    flag.leverDownRun = FALSE;
    flag.leverUpRun = TRUE;
    flag.oneUpRun = FALSE;
    flag.turnUpRun = FALSE; 
    flag.backUpRun = FALSE;
    flag.oneDownRun = TRUE;
    
    changeDirection = TRUE;
  }
  else if((flag.leverUpRun == TRUE) && (leverAngleForLeverControl >= 900)) {
    flag.oneUpRun = FALSE;
    flag.turnUpRun = FALSE; 
    flag.backUpRun = FALSE;
    flag.oneDownRun = FALSE;
    flag.leverUpRun = FALSE;
    flag.leverDownRun = TRUE;
    lift_pwm_calculate(VAC_HITCH_DIRECTION_OFF, 0, 0);
    return;
  }
  
  if(flag.oneDownRun == TRUE) {
    if((canFlag.hitchControlMode == VAB_HITCH_DRAFT_MODE) || (canFlag.hitchControlMode == VAB_HITCH_DEPTH_MODE)) {
      if(flagTimer.hundredMs == TRUE)
      {
        timerDepthDelay += 100;
      }
      
      flagCheck = FALSE;
      if((liftDepthSensorPrevious + 15) < get_threeP_position_sensor()) {
        flagCheck = TRUE;
      }
      if(liftDepthSensorPrevious > 15) {
        if((liftDepthSensorPrevious - 15) > get_threeP_position_sensor()) {
          flagCheck = TRUE; 
        }
      }
      
      if(flagCheck == TRUE) {
         liftDepthSensorPrevious = get_threeP_position_sensor();
         
         if(canFlag.hitchControlMode == VAB_HITCH_DEPTH_MODE) {
          uint16_t depthSet;
          depthSet = (uint16_t)(VAC_DEPTH_UP_LIMIT - nvDepthDownLimit);
          depthSet = (uint16_t)(((float)canRxDial.subSetting * (float)depthSet) / 250.0);
          depthSet += nvDepthDownLimit;

          if(depthSet > VAC_DEPTH_UP_LIMIT) {
            depthSet = VAC_DEPTH_UP_LIMIT;
          }
          if(depthSet < nvDepthDownLimit) {
            depthSet = nvDepthDownLimit;
          }
          
          if((depthSet < (get_threeP_depth_sensor() - nvThreePDepthUpPosition[0])) || (flagErrorSensors.threePDepth == TRUE)) {   // Depth control will be run
            timerDepthDelay = 1500;
          }
          else {
            timerDepthDelay = 0;
          }
        }
        else {
          timerDepthDelay = 0;
        }
      }
      
      if(canFlag.hitchControlMode == VAB_HITCH_DEPTH_MODE) {
        if(timerDepthDelay >= 1500) {
          flag.depthOnRun= TRUE;
          timerDepthDelay = 3000;
          flag.oneDownRun = FALSE;
        }
      }
      else {
        flag.oneDownRun = FALSE;
      }
    }
    changeDirection = TRUE;
    lift_pwm_calculate(VAC_HITCH_DIRECTION_RUN, positionAngle, leverAngle);
    return;
  }
  
  flag.draftRun = FALSE;
  switch( canFlag.hitchControlMode ) {
    case VAB_HITCH_POSITION_MODE:
      flag.depthOnRun = FALSE;
      if(flag.floating == TRUE){
        run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
      }
      else {
        if(flagLeverAngle_Move == TRUE)
          changeDirection = TRUE;
        
        lift_pwm_calculate(VAC_HITCH_DIRECTION_RUN, positionAngle, leverAngle);
      }
      break;
    case VAB_HITCH_DRAFT_MODE:
      flag.depthOnRun = FALSE;
      flag.draftRun = TRUE;  
      run_draft_process();
      break;
    case VAB_HITCH_DEPTH_MODE:
      run_depth_process();   
      break;
    default:
      flag.depthOnRun = FALSE;
      lift_pwm_calculate(VAC_HITCH_DIRECTION_OFF, 0, 0);
      break;
  }
  return;
}

void setting_lift_position() 
{
  static uint16_t       timerPositionSetting = 0;
  
  static uint16_t       positionPrevious = 0;
  static uint16_t       positionAverage = 0;
  static uint16_t       positionHigh = 0;
  
  uint16_t              positionCurrent;
  uint8_t               flagCheck = FALSE;

  positionCurrent = get_rawThreePPositionSensor();                                  // The data range is between 0 and 1023 --> "204"-->888

  if(canRxArmRest.hitchDownButton == CAN_SP_BUTTON_ON) {
    if((get_control_maximum_data(NV_3P_POSITION_DOWN_LIMIT) >= positionCurrent) && (get_control_minimum_data(NV_3P_POSITION_DOWN_LIMIT) <= positionCurrent)) {                  // Changed the condition on 2022.01.03
      flagCheck = TRUE;
    }
  }
  else if(canRxArmRest.hitchUpButton == CAN_SP_BUTTON_ON) {
    if((get_control_maximum_data(NV_3P_POSITION_UP_LIMIT) >= positionCurrent) && (get_control_minimum_data(NV_3P_POSITION_UP_LIMIT) <= positionCurrent)) {                  // Changed the condition on 2022.01.03
      flagCheck = TRUE; 
    } 
  }
  else {
    timerPositionSetting = 0;
  }
  
  if((flagCheck == TRUE) && (flagTimer.hundredMs == TRUE))
  {
    timerPositionSetting += 100;
  }
  
  positionAverage = positionAverage + positionCurrent;
  positionAverage >>= 1;
  
  if(positionPrevious >= 10) {
    if(((positionPrevious - 10 ) > positionAverage) || ((positionPrevious + 10) < positionAverage)) {
      timerPositionSetting = 0;
      positionPrevious = positionAverage;
    }
  }

  if(canRxArmRest.hitchUpButton == CAN_SP_BUTTON_ON) {
    if(timerPositionSetting >= VAC_LIFT_SETTING_ON_TIME) {
      timerPositionSetting = VAC_LIFT_SETTING_ON_TIME;
      if((get_control_maximum_data(NV_3P_POSITION_UP_LIMIT) >= positionAverage) && (get_control_minimum_data(NV_3P_POSITION_UP_LIMIT) <= positionAverage)) {                  // Changed the condition on 2022.01.03
        run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
        flag.buzzerOn = FALSE;
        flagHitch.positionSettingSequence = 1;           
        positionHigh = positionAverage;
      }
    }
    else {
      flag.buzzerOn = TRUE;
      run_hitch_pwm(VAC_HITCH_DIRECTION_UP, nv3pSettingUpDuty);
    }
    return;
  }		

  if(canRxArmRest.hitchDownButton == CAN_SP_BUTTON_ON) { 
    if(flagHitch.positionSettingSequence == 1) {
      if(timerPositionSetting >= VAC_LIFT_SETTING_ON_TIME) {
        if((get_control_maximum_data(NV_3P_POSITION_DOWN_LIMIT) >= positionAverage) && (get_control_minimum_data(NV_3P_POSITION_DOWN_LIMIT) <= positionAverage)) {                  // Changed the condition on 2022.01.03
          run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
          flag.buzzerOn = FALSE;
          flagHitch.positionSettingSequence = 2;
          
          // Save the position setting
          flagCheck = save_nvThreePPositionLimit(TRUE, positionHigh, positionAverage);
          position_rate_calculate();
        }
      }
      else {
        flag.buzzerOn = TRUE;
        run_hitch_pwm(VAC_HITCH_DIRECTION_DOWN, nv3pSettingDownDuty);
      }
    }
    return;
  } 
 
  run_hitch_pwm(VAC_HITCH_DIRECTION_OFF, 0);
  flag.buzzerOn = FALSE;
  timerPositionSetting = 0;
}
                   
void hitch_setting_lever_process()
{
  static uint16_t timerLeverSetting =  0;
  static uint16_t timerLeverSettingCheck = 0;
  uint16_t       flagCheck;

  if(flagHitch.leverThreePSettingMode == 0)
  {
    return;
  }
  
  if(flagHitch.leverThreePSettingMode == 1)
  {
    if(flagTimer.hundredMs == TRUE)
    {
      timerLeverSetting += 100;
    }
    if(timerLeverSetting >= 6000) {                                                    // After 6 seconds
      flagHitch.leverThreePSettingMode = 0;
    }
    if((canRxArmRest.hitchDownButton == CAN_SP_BUTTON_ON) && (canRxArmRest.hitchUpButton == CAN_SP_BUTTON_ON)) {
      if(flagTimer.hundredMs == TRUE)
      {
        timerLeverSettingCheck += 100;
      }
      if(timerLeverSettingCheck >= 1000) {                                                    // After 1 seconds
        flagHitch.leverThreePSettingMode = 2;
        timerLeverSettingCheck = 0; 
        
        flag.checkBuzzer = TRUE;                                             // Added on 2022.02.11 based on request from TYM
      }
    }
    else {
      timerLeverSettingCheck = 0;
    }
    return;
  }
  
  if(flagTimer.hundredMs == TRUE)
  {
    timerLeverSettingCheck += 100;
  }
  if(timerLeverSettingCheck >= 15000)
  {
    flagHitch.leverThreePSettingMode = 0;
  }

  flagCheck = FALSE;
  if((canRxArmRest.hitchDownButton == CAN_SP_BUTTON_ON) && (canRxArmRestPrevious.hitchDownButton == CAN_SP_BUTTON_OFF)) {
    timerLeverSettingCheck = 0;
    // Down limit setting
    flagCheck = save_nvThreePLeverDownLimit(TRUE, get_rawThreePLeverSensor());                            // RAW ADC data should be saved

    if(flagCheck == TRUE) { 
      lever_rate_calculate();
      flag.checkBuzzer = TRUE;
    }
    else if(flagCheck == FALSE) {
      flag.ngBuzzer = TRUE;
    }
  }
    
  if((canRxArmRest.hitchUpButton == CAN_SP_BUTTON_ON) && (canRxArmRestPrevious.hitchUpButton == CAN_SP_BUTTON_OFF)) {
    timerLeverSettingCheck = 0;
    // Up limit setting
    flagCheck = save_nvThreePLeverUpLimit(TRUE, get_rawThreePLeverSensor());                          // RAW ADC data should be saved
    
    if(flagCheck == TRUE) {
      lever_rate_calculate();
      flag.checkBuzzer = TRUE;
    }
    else if(flagCheck == FALSE) {
      flag.ngBuzzer = TRUE;
    }
  }
}

void hitch_setting_process()
{
  static uint8_t   flagDepthDraftFinished = FALSE;
  static uint16_t  timerDepthDraftSetting = 0;
  static uint16_t  timerSetting = 0;
  static uint16_t  timerSettingOver = 0;

  if(flagTimer.hundredMs == TRUE)
  {
    timerSetting += 100;
    timerSettingOver += 100;
    if(flagDepthDraftFinished == TRUE)
    {
      timerDepthDraftSetting += 100;
    }
  }
  
  if(flagHitch.settingMode == VAB_HITCH_SETTING_MODE_OFF)
  {
    if(canRxFenderA.hitchModeButton == CAN_SP_BUTTON_OFF)
    {
      timerSetting = 0;
      flagHitch.settingMode = VAB_HITCH_SETTING_MODE_OFF;
      return;
    }
  }

  if(flagHitch.settingMode == VAB_HITCH_SETTING_MODE_OFF)
  {
    if(timerSetting >= VAB_LIFT_SETTING_CHECK_TIME)
    {
      flagHitch.settingMode = VAB_HITCH_SETTING_POSITION_MODE;
      flagHitch.positionSettingSequence = 0;
      timerSettingOver = 0;
      flag.checkBuzzer = TRUE;
    }
    return;
  }
  
  timerSetting = 0;
  
  if((canRxFenderA.hitchModeButton == CAN_SP_BUTTON_ON) || 
     (canRxArmRest.hitchUpButton == CAN_SP_BUTTON_ON) || 
     (canRxArmRest.hitchDownButton == CAN_SP_BUTTON_ON)) {
    timerSettingOver = 0;
  }
  
  if(timerSettingOver >= VAB_LIFT_SETTING_TIME_OVER)
  {
    flagHitch.settingMode = VAB_HITCH_SETTING_MODE_OFF;
    timerSettingOver = 0;
    flag.buzzerOn = FALSE;
    return;
  }

  if(flagHitch.settingMode == VAB_HITCH_SETTING_POSITION_MODE)
  {

    setting_lift_position();
     
    if(flagHitch.positionSettingSequence == 2)
    {
      flagHitch.settingMode = VAB_HITCH_SETTING_MODE_OFF;
      return;
    }
    return;
  }
 
  if(flagHitch.settingMode == VAB_HITCH_SETTING_DRAFT_MODE)
  {
    if((get_threeP_draft_sensor() < VAC_DRAFT_SET_MIN) || (get_threeP_draft_sensor() > VAC_DRAFT_SET_MAX)) {
      flag.buzzerOn = TRUE;
    }
    else {
      flag.buzzerOn = FALSE;
      if((flagInputStatus.hitchManualUp == ON) || (flagInputStatus.hitchManualDown == ON))
      {
        flagDepthDraftFinished = TRUE;
        // External buttons
        // implement save_nvDraftS1Value(TRUE, get_threeP_draft_sensor() + 3);
      }
      
      if(flagDepthDraftFinished == TRUE)
      {
        if(timerDepthDraftSetting < 2000)
        {
          flag.buzzerOn = TRUE;
        }
        else
        {
          flagHitch.settingMode = VAB_HITCH_SETTING_MODE_OFF;
          timerDepthDraftSetting = 0;
          flagDepthDraftFinished = FALSE;
        }
      }
    }
    return;
  }
  
  if(flagHitch.settingMode == VAB_HITCH_SETTING_DEPTH_MODE)
  {
    if((get_threeP_depth_sensor() <  VAC_DEPTH_SET_MIN) || (get_threeP_depth_sensor() >  VAC_DEPTH_SET_MAX)) {  // 0.7 and 1.0
      flag.buzzerOn = TRUE;
    }
    else {
      flag.buzzerOn = FALSE;
      if((flagInputStatus.hitchManualUp == ON) || (flagInputStatus.hitchManualDown == ON))
      {
        flagDepthDraftFinished = TRUE;
        // External buttons
        // implement save_nvDepthS1Value(TRUE, get_threeP_depth_sensor() + 5);  
      }
      
      if(flagDepthDraftFinished == TRUE)
      {
        if(timerDepthDraftSetting < 2000)
        {
          flag.buzzerOn = TRUE;
        }
        else
        {
          flagHitch.settingMode = VAB_HITCH_SETTING_MODE_OFF;
          timerDepthDraftSetting = 0;
          flagDepthDraftFinished = FALSE;
        }
      }
    }
    return;
  }
}

void lift_init()
{
  flagPrevious.liftLock = FALSE;                                          // This is solved the problem that check buzzer is ON when powered up.
  flag.liftLock = TRUE;
  flag.leverDownRun = FALSE;
  flag.leverUpRun = FALSE;
  flagPrevious.floating = FALSE;
    
  flagHitch.leverThreePSettingMode = 1;
  position_rate_calculate();
  lever_rate_calculate();
}