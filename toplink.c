#include "defines.h"
#include "toplink.h"
#include "input.h"
#include "output.h"
#include "main.h"
#include "can.h"
#include "algorithm.h"

void toplink_init()
{
}

/*
uint8_t unload_valve_control()
{
  static uint16_t timerUnloadValveOn = 0;
  static uint16_t timerUnloadValveOff = 0;
   
  if((flagOutputControl.topLinkUp == ON) || (flagOutputControl.topLinkDown == ON)) {
    timerUnloadValveOff = 0;
    timerUnloadValveOn += 2;
    if(timerUnloadValveOn >= UNLOAD_VALVE_ON_AND_OFF_TIME) {
      timerUnloadValveOn = UNLOAD_VALVE_ON_AND_OFF_TIME;
      flagOutputControl.unload = ON;
    }
  }

  if((flagOutputControl.topLinkUp == OFF) && (flagOutputControl.topLinkDown == OFF)) {
    flagOutputControl.unload = OFF;
    timerUnloadValveOn = 0;
    timerUnloadValveOff += 2;
    if(timerUnloadValveOff >= UNLOAD_VALVE_ON_AND_OFF_TIME) {
      timerUnloadValveOff = UNLOAD_VALVE_ON_AND_OFF_TIME;
    }
    else {
      flagOutputControl.topLinkUp = flagOutputControlPrevious.topLinkUp;
      flagOutputControl.topLinkDown = flagOutputControlPrevious.topLinkDown;
    }
  }
  
  flagOutputControlPrevious.topLinkUp = flagOutputControl.topLinkUp;
  flagOutputControlPrevious.topLinkDown = flagOutputControl.topLinkDown;
  return 1;
}
*/

void toplink_control_process()
{
  // External valve & unload valves control    
  if((flagInputStatus.topLinkDown == ON) && (flagInputStatus.topLinkUp == ON))
  {
    // ERROR: both up and down switches are ON, so up and down switches have been error --> May hardware error or two switches are pressed together
  }
  else {
    if((canRxPillar.topLinkDownButton == CAN_SP_BUTTON_ON) && (canRxPillar.topLinkUpButton == CAN_SP_BUTTON_ON)) {
      // ERROR: both up and down control bits are ON, so CAN error is occured for controlling the output
    }
    else {
      if((canRxPillar.topLinkUpButton == CAN_SP_BUTTON_ON) && (flagInputStatus.topLinkDown == ON)) {
        // ERROR : CAN UP valve ON and external DOWN switch is ON. Which means two opposite control is occured. Algorithm error.
      }
      else if((canRxPillar.topLinkDownButton == CAN_SP_BUTTON_ON) && (flagInputStatus.topLinkUp == ON)) {
        // ERROR : CAN DOWN valve ON and external UP switch is ON. Which means two opposite control is occured. Algorithm error.
      }
      else {
        // There is no error
        if((canRxPillar.topLinkUpButton == CAN_SP_BUTTON_ON) || (flagInputStatus.topLinkUp == ON)) {
          flagOutputControl.topLinkUp = ON;
        }
        else if((canRxPillar.topLinkDownButton == CAN_SP_BUTTON_ON) || (flagInputStatus.topLinkDown == ON)) {
          flagOutputControl.topLinkDown = ON;
        }
        else {
          // unexpected error is occured.
        }
      }
    }
  }
//  unload_valve_control();
}