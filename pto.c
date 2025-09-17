#include "defines.h"
#include "pto.h"
#include "input.h"
#include "output.h"
#include "settings.h"
#include "main.h"
#include "can.h"
#include "algorithm.h"
#include "sensors.h"

#define USED_POINTS_IN_PTO_COMPENSATION             8
int16_t ptoCompensationDuty;

#define TIME_PTO_SWITCH_ON              500
#define TIME_PTO_SWITCH_OFF             500

#define TIME_PTO_LAMP_ON                500
#define TIME_PTO_LAMP_CYCLE             1000

#define TIME_PTO_ONETOUCH_UP_OFF        1000

uint8_t ptoSwitch = OFF;
uint8_t ptoMode;

uint16_t ptoDuty;

uint16_t timerPtoOn = 0;
uint16_t timerPtoOff = 0;
uint8_t  isPtoRunning = FALSE;
uint8_t  ptoOffMapEnabled = 0;
float    ptoPercent;
int16_t  ptoDeadBand = 0;
uint16_t position3P;

void pto_init()
{
  ptoSwitch = OFF;
  ptoMode = PTO_MODE_OFF;
  ptoDuty = 0;

  timerPtoOn = 0;
  timerPtoOff = 0;
  isPtoRunning = FALSE;
  ptoOffMapEnabled = 0;
  ptoDeadBand = 0;
}

void updatePtoSw()
{
  static uint16_t timerPtoSwitchOff = 0;
  static uint16_t timerPtoSwitchOn = 0;
  
  if(ptoSwitch == OFF)
  { 
    if((flagInputStatus.ptoManual == ON) && (flagInputStatus.ptoAuto == ON))
    {
      timerPtoSwitchOn = 0;
      timerPtoSwitchOff = 0;
      ptoSwitch = OFF;
      ptoMode = PTO_MODE_OFF;
    }
    else if((flagInputStatus.ptoManual == ON) || (flagInputStatus.ptoAuto == ON))
    {
      timerPtoSwitchOn += 2;
      if(timerPtoSwitchOn >= TIME_PTO_SWITCH_ON)
      {
        timerPtoSwitchOn = TIME_PTO_SWITCH_ON;
        timerPtoSwitchOff = 0;
        ptoSwitch = ON;
        if(flagInputStatus.ptoAuto == ON)
        {
          ptoMode = PTO_MODE_AUTO;
        } 
        else 
        {
          ptoMode = PTO_MODE_MANUAL;
        }
      }
    }
  }
  else
  {
    
    if((flagInputStatus.ptoManual == OFF) && (flagInputStatus.ptoAuto == OFF))
    {
      timerPtoSwitchOff += 2;
      if(timerPtoSwitchOff >= TIME_PTO_SWITCH_OFF)
      {
        timerPtoSwitchOff = TIME_PTO_SWITCH_OFF;
        
        ptoSwitch = OFF;
        timerPtoSwitchOn = 0;
        ptoMode = PTO_MODE_OFF;
      }
    }
  }
}

uint8_t get_ptoSwitch()
{
  uint8_t ret = FALSE;
  if((ptoSwitch == ON) || (flagOutputControl.pto == ON))
  {
    ret = TRUE;
  }
  return ret;
}

uint16_t get_ptoDuty()
{
  if(ptoDuty > PTO_DUTY_MAX)
  {
    ptoDuty = PTO_DUTY_MAX;
  }
  return ptoDuty;
}

void pto_off_process()
{
  uint8_t i;
  
  if(ptoOffMapEnabled == 0)
  {
    isPtoRunning = FALSE;
    flagOutputControl.pto = OFF;
    ptoDuty = 0;
    return; 
  }
  
  // PTO OFF should be controlled based on PTO OFF MAP
  // If above conditions are not implemented which means there is no error, the pto should be turned ON with PTO ON MAP
  for(i = 0; i < NUMBER_OF_PTO_OFF_POINTS; i++)
  {
    if(nvPtoOffTime[i] > timerPtoOff)
    {
      if(i == 0)
      {
        ptoPercent = nvPtoOffCurrentPercent[0];
      }
      else
      {
        ptoPercent = nvPtoOffCurrentPercent[i - 1] - ((((float)nvPtoOffCurrentPercent[i - 1] - (float)nvPtoOffCurrentPercent[i]) / ((float)nvPtoOffTime[i] - (float)nvPtoOffTime[i - 1]))
            * ((float)timerPtoOff - (float)nvPtoOffTime[i - 1]));
      }
      break;
    }
  }
  if(i == NUMBER_OF_PTO_OFF_POINTS)
  {
    ptoPercent = nvPtoOffCurrentPercent[NUMBER_OF_PTO_OFF_POINTS - 1];
    ptoOffMapEnabled = 0;
    timerPtoOff = nvPtoOffTime[NUMBER_OF_PTO_OFF_POINTS - 1];
  }
  if(ptoPercent < 0)
  {
    ptoPercent = 0;
  }

  if(ptoPercent == 0)
  {
    isPtoRunning = FALSE;
    flagOutputControl.pto = OFF;
  }
  else
  {
    flagOutputControl.pto = ON;                                         // PTO Valve on
  }
  ptoDuty = (uint16_t)ptoPercent;                                             // pto output maximum value is 1000 so we do not need scale it
}

void pto_on_process()
{
  uint8_t i;
  
  // If above conditions are not implemented which means there is no error, the pto should be turned ON with PTO ON MAP
  for(i = 0; i < NUMBER_OF_PTO_ON_POINTS; i++)
  {
    if(nvPtoOnTime[i] > timerPtoOn) {
      if(i == 0) {
        ptoPercent = nvPtoOnCurrentPercent[0];
      }
      else {
        ptoPercent = nvPtoOnCurrentPercent[i - 1] + ((((float)nvPtoOnCurrentPercent[i] - (float)nvPtoOnCurrentPercent[i - 1]) / ((float)nvPtoOnTime[i] - (float)nvPtoOnTime[i - 1]))
            * ((float)timerPtoOn - (float)nvPtoOnTime[i - 1]));
      }
      break;
    }
  }
  if(i == NUMBER_OF_PTO_ON_POINTS)
  {
    ptoPercent = nvPtoOnCurrentPercent[NUMBER_OF_PTO_ON_POINTS - 1];
    ptoOffMapEnabled = 1;
    timerPtoOn = nvPtoOnTime[NUMBER_OF_PTO_ON_POINTS - 1];
  }
  
  calculate_pto_compensation();
  ptoPercent += ptoCompensationDuty;
  
  if(ptoPercent < 0)
  {
    ptoPercent = 0;
  }
  ptoDeadBand = 0;
  flagOutputControl.pto = ON;                                             // PTO Valve on
  ptoDuty = (uint16_t)ptoPercent;                                               // pto output maximum value is 1000 so we do not need scale it
}

void pto_control_process()
{
  static uint16_t timerPtoUpDelay = 0;
  static uint16_t timerPtoFlashing = 0;
  static uint8_t  ptoSwitchPrevious = OFF;
  
  static uint8_t  isSafetyOff = FALSE;
  static uint16_t timerSafetyOff = PTO_SAFETY_CHECK_TIME;                       // After power up, we do NOT need check time for (PTO)
  
  uint8_t  flagCheck = FALSE;
  
  flagOutputLamp.pto = OFF;
  
  if(flag.isEUModel == TRUE)
  {
    if((canFlag.stationary == FALSE) && (isSafetyOff == FALSE)) {
      if(flagInputStatus.seat == FALSE)
      {
        if(timerSafetyOff >= PTO_SAFETY_CHECK_TIME) {
          isSafetyOff = TRUE;
        }
        else {
          timerSafetyOff += 2;                                                // it is increased by 10 every ten ms, so 3000 is equal to 3000 ms
        }
      }
      else {
        if(flagInputStatus.seat == TRUE)
        {
          timerSafetyOff = 0;
        }
      }
    }
    if(isSafetyOff == TRUE) {
      flagCheck = TRUE;
      if((ptoSwitchPrevious == OFF) && (ptoSwitch == ON) && (flagInputStatus.seat == TRUE))
      {
        isSafetyOff = FALSE;
        timerSafetyOff = 0;                                                       // we do not need to turn off here, becase the below function should handle this part "flagCheck is now TRUE"
      }
    }
    ptoSwitchPrevious = ptoSwitch;
  }
  
  if((ptoSwitch == OFF) || (flagCheck == TRUE))
  {
    timerPtoOff += 2;
    timerPtoFlashing += 2;
    if(isPtoRunning == TRUE) {
      ptoDeadBand = -20;                                                        // Changed on 2023.03.31
      pto_off_process();
    }
    timerPtoOn = 0;
    
    if((flagCheck == TRUE) && (ptoSwitch == ON))
    {
      if(timerPtoFlashing < TIME_PTO_LAMP_ON) {
        flagOutputLamp.pto = ON;
      }
      else if(timerPtoFlashing < TIME_PTO_LAMP_CYCLE) {
        flagOutputLamp.pto = OFF;
      }
      else {
        timerPtoFlashing = 0;
        flagOutputLamp.pto = ON;
      }
    }    
    return;
  }
  
  if(ptoMode == PTO_MODE_MANUAL)
  {
    isPtoRunning = TRUE;
    timerPtoOn += 2;
    pto_on_process();
    flagOutputLamp.pto = ON;
    timerPtoOff = 0;
    
    flag.ptoLiftLock = FALSE;
    
    return;
  }
  
  if(ptoMode != PTO_MODE_AUTO)
  {
    flagOutputControl.pto = OFF;
    ptoDuty = 0;
    timerPtoOn = 0;
    timerPtoOff = 0;
    return;
  }
  
  ptoSwitchPrevious = ptoSwitch;
  
  position3P = (uint16_t)((float)nvPtoLower3Pposition + ((float)nvPtoHigher3Pposition - (float)nvPtoLower3Pposition) * (float)canRxDial.ptoStopPosition / 250.0);
  
  if((canFlag.backUpHitch == TRUE) && (flagShuttle.backward == ON))
  {
    flagCheck = TRUE;
  }

  if((canFlag.turnUpHitch == TRUE) && (flag.isSteeringON == TRUE))//((flag.steeringRight == TRUE) || (flag.steeringLeft == TRUE)))
  {
    flagCheck = TRUE;
  }
  
  if((flag.oneUpRun == TRUE) || (flag.ptoLiftLock == TRUE)) {
    if(timerPtoUpDelay <= TIME_PTO_ONETOUCH_UP_OFF) {
      timerPtoUpDelay += 2;
    }
    else {
      flagCheck = TRUE;
    }
  }
  else {
    timerPtoUpDelay = 0;
  }

  if(nvPtoOnWhenHitchDown == ON)
  {
    if(flag.oneDownRun == FALSE)
    {
      if(get_threeP_position_sensor() >= (position3P + ptoDeadBand))
      {
        flagCheck = TRUE;
      }
    }
  }
  else {
    if(get_threeP_position_sensor() >= (position3P + ptoDeadBand))
    {
      flagCheck = TRUE;
    }
  }
  
  if(flagCheck == TRUE) {
    timerPtoOn = 0;
    timerPtoOff += 2;
    timerPtoFlashing += 2;
    
    if(isPtoRunning == TRUE) {
      ptoDeadBand = -20;                                                        // 2023.03.31
    }

    if(timerPtoFlashing < TIME_PTO_LAMP_ON) {
      flagOutputLamp.pto = ON;
    }
    else if(timerPtoFlashing < TIME_PTO_LAMP_CYCLE) {
      flagOutputLamp.pto = OFF;
    }
    else {
      timerPtoFlashing = 0;
      flagOutputLamp.pto = ON;
    }
    return;
  }
  
  timerPtoOff = 0;
  timerPtoOn += 2;
  isPtoRunning = TRUE;
  pto_on_process();
  
  flagOutputLamp.pto = ON;
}

void calculate_pto_compensation()
{
  uint16_t temperatureBefore;
  uint16_t temperatureNext;
  uint16_t dutyBefore;
  uint16_t dutyNext;
  uint8_t  i;
  
  uint16_t tempOilTemperature;
  
  tempOilTemperature = get_oil_temperature();
  
  for(i = 0; i < USED_POINTS_IN_PTO_COMPENSATION; i++) {
    temperatureNext = nvPTOTemperatureCompensationDutyTemperature[i];
    if(tempOilTemperature <= temperatureNext) {
      if(i == 0) {
        ptoCompensationDuty = nvPTOTemperatureCompensationDuty[i];
      }
      else {
        temperatureBefore = nvPTOTemperatureCompensationDutyTemperature[i - 1];
        dutyBefore = nvPTOTemperatureCompensationDuty[i - 1];
        dutyNext = nvPTOTemperatureCompensationDuty[i];
        ptoCompensationDuty = (int16_t)((float)dutyBefore + ((float)(dutyNext - dutyBefore) * (float)(tempOilTemperature - temperatureBefore) / (float)(temperatureNext - temperatureBefore + 0.1)));
      }
      break;                                                                    // Once the condition is happened, it should be break from for loop
    }
  }
  if(i == USED_POINTS_IN_PTO_COMPENSATION) {
    ptoCompensationDuty = nvPTOTemperatureCompensationDuty[USED_POINTS_IN_PTO_COMPENSATION - 1];
  }
  
  ptoCompensationDuty = ptoCompensationDuty - 500;
}