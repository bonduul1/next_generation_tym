#include "defines.h"
#include "algorithm.h"
#include "input.h"
#include "can.h"
#include "output.h"
#include "main.h"
#include "sensors.h"
#include "settings.h"
#include "rpmController.h"
#include "hitch.h"
#include "shuttle.h"
#include "pto.h"
#include "balance.h"
#include "toplink.h"
#include "autobrake.h"

flag_t flag;                                                                    // The main algorithm is using these flags
flag_t flagPrevious;                                                            // The main algorithm is using these flags ( the previous flags )

/****************************** Model selection ******************************/

uint8_t check_model()
{
  static uint16_t timerModelSwitch = 0;
  static uint16_t counterModelSwitch = 0;
  
  timerModelSwitch += 2;
  if(timerModelSwitch >= 500)
  {
    timerModelSwitch = 501;
    if(counterModelSwitch > 450)
    {
      flag.isEUModel = TRUE;
    }
    else
    {
      flag.isEUModel = FALSE;
    }
    flag.powerOn = TRUE;
    return TRUE;
  }
  else
  {
    if(flagInputStatus.model == ON)
    {
      counterModelSwitch += 2;
    }
    else
    {
      counterModelSwitch = 0;
    }
  }
  return FALSE;
}

/****************************** Starter process ******************************/

void starter_control_process()
{
  static uint16_t starterTime = 0;

  if((get_ptoSwitch() == OFF) && (flagInputStatus.brake == ON) && 
    ((flag.isEUModel == FALSE) || ((flag.isEUModel == TRUE) && (flagInputStatus.seat == TRUE))))
  {
    if(starterTime < VAC_STARTER_ON_DELAY) {
      starterTime += 2;
    }
    else
    {
      flagOutputControl.starterOut = ON;
    }
  }
  else {
    starterTime = 0;
  }
}

/****************************** Buzzer process ******************************/
void buzzer_control_process()
{
  static uint16_t timerBuzzer = 0;
  uint16_t settingBuzzerOn, settingBuzzerCycle;
  
  
  if(flagShuttle.backward == ON)
  {
    flagOutputControl.backBuzzer = ON;
  }
  
  if(flagTimer.tenMs == TRUE)
  {
    timerBuzzer += 10;
  }
  flagOutputLamp.checkBuzzer = OFF;

  if(flag.buzzerOn == TRUE) {
    timerBuzzer = 0;
    flagOutputLamp.checkBuzzer = ON;
    return;
  }
  else if(flag.checkBuzzer == TRUE) {
    settingBuzzerOn = 500;
    settingBuzzerCycle = settingBuzzerOn + 500;
    flagOutputLamp.checkBuzzer = ON;
  }
  else if(flag.ngBuzzer == TRUE) {
    settingBuzzerOn = 200;
    settingBuzzerCycle = settingBuzzerOn + 200;
    flagOutputLamp.checkBuzzer = ON;
  }
  else if(flag.buzzerButton == TRUE) {                                          // Added on 2024.01.02 --> 150ms ON, 150ms OFF
    settingBuzzerOn = 150;
    settingBuzzerCycle = 300;
    flagOutputLamp.checkBuzzer = ON;
  }
  else {
    timerBuzzer = 0;
  }

  if(flagOutputLamp.checkBuzzer == ON)
  {
    if(flag.ngBuzzer == TRUE) {
      if(timerBuzzer <= 200) {
        flagOutputLamp.checkBuzzer = ON;
      }
      else if(timerBuzzer <= 400) {
        flagOutputLamp.checkBuzzer = OFF;
      }
      else if(timerBuzzer <= 600) {
        flagOutputLamp.checkBuzzer = ON;
      }
      else {
        flag.ngBuzzer = FALSE;
        flag.checkBuzzer = FALSE;
      }
      return;
    }

    if(timerBuzzer > settingBuzzerOn)
    {
      flagOutputLamp.checkBuzzer = OFF;
    } 
    if(timerBuzzer > settingBuzzerCycle)
    {
      timerBuzzer = 0;
      if(flag.checkBuzzer == TRUE) {
        flag.checkBuzzer = FALSE;
      }
      if(flag.buzzerButton == TRUE) {
        flag.buzzerButton = FALSE;
      }
    }
    return;
  }
  timerBuzzer = 0;
}

/****************************** Safety process ******************************/
void safety_control_process()
{
  static uint16_t timerSafety = 0;
  static uint16_t timerSafetyStart = TRUE;
  static uint8_t checkToggle = 0;
  
  if(flag.isEUModel == TRUE)
  {
    if((flagInputStatus.parkingBrake == OFF) && (flagInputStatus.seat == OFF)) {
      if(checkToggle == 0) {
        if(flagTimer.hundredMs == TRUE)
        {
          timerSafety += 100;
        }
        if(timerSafety >= 500)
        {
          timerSafety = 0;
          checkToggle = 1;
        }
      }
      else {
        if((timerSafetyStart == TRUE) && (flagTimer.hundredMs == TRUE))
        {
          timerSafety += 100;
        }
        if(timerSafety >= 10000)
        {
          timerSafetyStart = FALSE;
        }
        else
        {
          flag.buzzerOn = TRUE;
          if(timerSafety < 1000)      {   flag.parkingLamp = ON;  }
          else if(timerSafety < 2000) {   flag.parkingLamp = OFF; }
          else if(timerSafety < 3000) {   flag.parkingLamp = ON;  }
          else if(timerSafety < 4000) {   flag.parkingLamp = OFF; }
          else if(timerSafety < 5000) {   flag.parkingLamp = ON;  }
          else if(timerSafety < 6000) {   flag.parkingLamp = OFF; }
          else if(timerSafety < 7000) {   flag.parkingLamp = ON;  }
          else if(timerSafety < 8000) {   flag.parkingLamp = OFF; }
          else if(timerSafety < 9000) {   flag.parkingLamp = ON;  }
          else if(timerSafety < 10000){   flag.parkingLamp = OFF; }
        }
      }
    }
    else {
      checkToggle = 0;
      timerSafety = 0;
      timerSafetyStart = TRUE;
      if(flagInputStatus.parkingBrake == ON) {
        flag.parkingLamp = ON;
      }
    }
  }
  else
  {
    if(flagInputStatus.parkingBrake == ON) {
      flag.parkingLamp = ON;
    }
  }
}

/****************************** 4WD, QT process ******************************/
void drive_process()
{
  static uint16_t timerWheel = 0;
  static uint16_t timerQtRunDelay = 0;
  static uint16_t timerWheelDelay = 0;  
  
  static uint16_t timerQtDelay = 0;
//  static uint16_t timerDiffDelay = 0;
  static uint16_t timerBreakDelay = 0;      
    
  static uint16_t timerFourWDDisable = 0; 
  static uint16_t timerQtOnDelay = 0;
    
  uint8_t flagCheck;
     
  if(flag.powerOn == FALSE) {
    canFlag.quickTurn = FALSE;
    timerFourWDDisable = 0;
    return;
  }

  if(nvQtAutoDisconnectSpeed <= get_average_speed()) {                          // The current speed is greather than the configured speed --> Disconnect speed
    flag.qtDelay = TRUE;
    timerQtDelay = 0; 
  }
  //else if(nvQtAutoConnectSpeed > get_average_speed()) {                         // The current speed is lower than the configured speed --> Connect speed
  else {
    if(timerQtDelay >= 300)
    {
      flag.qtDelay = FALSE;
    }
    else if(flagTimer.tenMs == TRUE)
    {
      timerQtDelay += 10;
    }
  }
  
  if(canFlag.fourWD == VAC_4WD_MODE_OFF)
  {
    // 4WD & QT mode is OFF
    flagOutputControl.qt = OFF;
    timerWheel = 0;
    flagOutputLamp.fourWD = OFF;
    
    canTxIP1.quickTurnFlashing = OFF;
    
    timerQtRunDelay = 0;
    flagOutputControl.fourWD = OFF;
    flag.wheelSpeedLimit = OFF;
    
    canFlag.quickTurn = FALSE;
    
    if((flagInputStatus.brake == OFF) || (flagInputStatus.sideBrake == ON))
    {
      timerBreakDelay = 0;
      flagOutputControl.fourWD = OFF;
      return;
    }
    
    if(timerBreakDelay <= 100) {
      if(flagTimer.tenMs == ON) {
        timerBreakDelay += 10;
      }
      return;
    }
    flagOutputControl.fourWD = ON;
    return;
  }
  else if(canFlag.quickTurn == TRUE)
  {
    // QT mode on & 4WD manual mode
    flagOutputLamp.qt = ON;
    flagOutputLamp.fourWD = OFF;
    flagCheck = FALSE;

    //if((flag.steeringRight == TRUE) || (flag.steeringLeft == TRUE))
    if(flag.isSteeringON == TRUE)
    {
      if(flagOutputControl.backward == OFF)
      {
        if(timerQtOnDelay <= 100) {                                              // check the 10
          if(flagTimer.tenMs == TRUE) {
            timerQtOnDelay += 10;
          }
        }
        else {
          flagCheck = TRUE;                                                     // After 100ms
        }
      }
      else {
        timerQtOnDelay = 0;
      }
    }
    else {
      timerQtOnDelay = 0;
    }
       
    if(flagCheck == TRUE) {
      if(flag.qtDelay == TRUE) {
        flag.qtDisable = TRUE;
        canFlag.quickTurn = FALSE;
        flagCheck = FALSE;
      }
    }

    if(flagCheck == TRUE) {
      if(flagTimer.tenMs == TRUE)
      {
        timerQtRunDelay += 10;
      }
      if(timerQtRunDelay >= VAC_4WD_QT_DELAY) {
        timerQtRunDelay = VAC_4WD_QT_DELAY;
        timerFourWDDisable = VAC_4WD_QT_DELAY;
        flagOutputControl.qt = ON;
        canTxIP1.quickTurnFlashing = ON;
        flagOutputControl.fourWD = OFF;
        flagOutputLamp.fourWD = OFF;
//Enkhbat          flag.checkBuzzer = TRUE; 
        return;
      }
      flagOutputControl.fourWD = OFF;
      return;
    }
    timerQtRunDelay = 0;
  }
  else if(canFlag.fourWD == VAC_4WD_MANUAL_MODE)                                // Manual mode
  {
    // QT mode OFF & 4WD mode manual
    flagOutputControl.fourWD = ON;
    return;
  }
  
  // 4WD Auto MODE
  
  if(canFlag.quickTurn == OFF)
  {
    canFlag.quickTurn = FALSE;
    timerFourWDDisable = 0;
  }

  if(flagOutputLamp.qt == OFF) {
    flagOutputLamp.fourWD = ON;
  }
  flagOutputControl.qt = OFF;

  if(timerFourWDDisable != 0) {
    if(flagTimer.tenMs == TRUE) {
      timerFourWDDisable -= 10;
    }
    return;
  }
  else {
    flagOutputControl.fourWD = ON;
  }

  canTxIP1.quickTurnFlashing = OFF;

  if(nv4WdAutoDisconnectSpeed <= get_average_speed()) {                         // 4WD OFF Speed
    flag.wheelSpeedLimit = TRUE;
    timerWheelDelay = 0; 
  }
  else {
    if(flag.wheelSpeedLimit == TRUE) {
      if(nv4WdAutoConnectSpeed >= get_average_speed()) {                        // 4WD ON Speed
        if(flagTimer.tenMs == TRUE) {
          timerWheelDelay += 10;
        }
        if(timerWheelDelay >= 200) {
          flag.wheelSpeedLimit = FALSE;
        }
      }
      else {
        timerWheelDelay = 0;
      }
    }
  }

  if(flagTimer.tenMs == TRUE) {
    timerWheel += 10;
  }

  if(flag.wheelSpeedLimit == TRUE) {
    flagOutputControl.fourWD = OFF;
    flagOutputControl.qt = OFF;

    if(flagOutputLamp.fourWD == ON) {  
      if(timerWheel >= VAC_WHEEL_LAMP_ON) {
        flagOutputLamp.fourWD = OFF;
      }
      else {
        timerWheel = 0;
      }
    }
  }
  else {
    timerWheel =0;
  }

  if(timerWheel >= VAC_WHEEL_LAMP_CYCLE)
  {
    timerWheel = 0;
  }
  return;
}

/****************************** Reading light control process ******************************/
void reading_light_control_process()
{
  
}

/****************************** Regen control process ******************************/
void regen_control_process()
{
  /* Disabled on 2025.07.30
  if(canFlag.regen == CAN_SP_LED_ON)
  {
    flagOutputControl.regen = ON;
  }
  else
  {
    flagOutputControl.regen = OFF;
  }
  */
}

/****************************** Screen transition control process ******************************/
void screen_transition_control_process()
{
  
}


/****************************** Control init ******************************/
void control_init()
{
  uint8_t i;
  
  for(i = 0; i < 8; i++)
  {
    flagErrorSensors.data[i] = 0;                                                 // Added on 2021.12.06
    flagErrorSensorsHigh.data[i] = 0;
    flagErrorSensorsLow.data[i] = 0;
  }
  
  flag.armSetConnected = FALSE;                                           // No needed
  flag.checkBuzzer = FALSE;
  flag.buzzerOn = FALSE;
  flag.ngBuzzer = FALSE;
  flag.powerOn = FALSE;
  flag.parkStop = FALSE;
  flag.engineRun = FALSE;
  flag.rpmCruiseA = FALSE;
  flag.rpmCruiseB = FALSE;
  flag.isSteeringON = FALSE;
    
  init_diagnostic_variables();
  
  shuttle_init();
  
  lift_init();
  balance_init();
  
  pto_init();
    
  nvRpmCruiseNewA = nvRpmCruiseA;
  nvRpmCruiseNewB = nvRpmCruiseB;
  
// Added on 2025.02.25  steeringOffset = (((float)nvSteeringRight - (float)nvSteeringLeft) / 2) - 512;
  
  // update panel data
  for(i = 0; i < 10; i++)
  {
    canFlag.data[i] = 0;
  }
  
  // Stationary off
  // reading lamp off
  // auto stop off
  // 3P position mode
  // balance manual mode
  // balance normal
  // QT mode OFF
  canFlag.stationary = FALSE;
  canFlag.readingLight = FALSE;
  canFlag.autoStop = FALSE;
  canFlag.hitchControlMode = VAB_HITCH_POSITION_MODE;
  canFlag.balanceControlMode = VAB_BALANCE_MANUAL_MODE;
  canFlag.balanceSensitivityMode = VAB_BALANCE_NORMAL_MODE;
  canFlag.quickTurn = FALSE;  
  // Turn up hitch
  // Back up hitch
  // Turn down RPM
  // Back down RPM
  // 4WD manual
  // 4WD auto
  
  if((nvPanelData & BIT_NV_TURN_UP_MODE) == BIT_NV_TURN_UP_MODE)        canFlag.turnUpHitch = TRUE;
  else                                                                  canFlag.turnUpHitch = FALSE;
  
  if((nvPanelData & BIT_NV_BACK_UP_MODE) == BIT_NV_BACK_UP_MODE)        canFlag.backUpHitch = TRUE;
  else                                                                  canFlag.backUpHitch = FALSE;
  
  if((nvPanelData & BIT_NV_TURN_DOWN_MODE) == BIT_NV_TURN_DOWN_MODE)    canFlag.turnDownRpm = TRUE;
  else                                                                  canFlag.turnDownRpm = FALSE;
  
  if((nvPanelData & BIT_NV_BACK_DOWN_MODE) == BIT_NV_BACK_DOWN_MODE)    canFlag.backDownRpm = TRUE;
  else                                                                  canFlag.backDownRpm = FALSE;
  
  if((nvPanelData & BIT_NV_4WD_MANUAL_MODE) == BIT_NV_4WD_MANUAL_MODE)  canFlag.fourWD = VAC_4WD_MANUAL_MODE;
  else if((nvPanelData & BIT_NV_4WD_AUTO_MODE) == BIT_NV_4WD_AUTO_MODE) canFlag.fourWD = VAC_4WD_AUTO_MODE;
  else                                                                  canFlag.fourWD = VAC_4WD_MODE_OFF;
}

/****************************** Control previous process ******************************/

uint8_t control_previous_process()
{
  output_clear();                                                               // clear output flags
  analog_sensors();                                                             // Update ADC value if DMA is finished, then calculate the sensor value based on RAW ADC data      
  digital_sensors();                                                            // Read digital switches, buttons, and sensors, and then update flags  
  updatePtoSw();
  can_receive_process();                                                        // If new data is received, then CAN buffers are updated
  
  starter_control_process();
  
  return check_model();
}

/****************************** Control process ******************************/

void control_process()
{
  output_clear();                                                               // clear output flags
  
  analog_sensors();                                                             // Update ADC value if DMA is finished, then calculate the sensor value based on RAW ADC data      
  digital_sensors();                                                            // Read digital switches, buttons, and sensors, and then update flags  
  
  updatePtoSw();
  
  can_receive_process();                                                        // If new data is received, then CAN buffers are updated
  check_sensors_error();                                                        // Check sensor errors by using thier threshold value

  pto_control_process();                                                        // PTO switch is checked in here so it should be implemented first.
  starter_control_process();
  rpm_control_process();

  drive_process();
  
  shuttle(update_nv_data());

// Disabled on 2025.02.20  autobrake_process();

  balance_setting();
  balance_process();
  toplink_control_process();
  
  unload_valve_control();
  
  hitch_setting_lever_process();
  hitch_setting_process();
  
  lift_control_process();
  
  safety_control_process();
  
  reading_light_control_process();
  regen_control_process();
  screen_transition_control_process();
  
  buzzer_control_process();
  output_controller();
}

