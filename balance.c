#include "balance.h"
#include "can.h"
#include "input.h"
#include "output.h"
#include "sensors.h"
#include "main.h"
#include "algorithm.h"
#include "settings.h"

flagBalance_t flagBalance;

float targetStrokePosition;

float slopeRollCenter;
float rollingResult;
float strokeResult;
float balanceDeadBand;

uint16_t rollingChangeTime;
uint16_t rollingSampling;

uint16_t timerFlat;
uint16_t timerSlope;
uint16_t timerRollingChange;

uint16_t timerBalanceSetting = 0;

float balance_vr()
{
  static uint16_t balanceSettingPrevious;
  uint16_t balanceSetting = canRxDial.balanceSetting * 4;                       /* 250 * 4 = 1000 */
  float calculatedBalance;
  float calculation;
   
  if(((balanceSettingPrevious + 8) <= balanceSetting) || ((balanceSettingPrevious - 8) >= balanceSetting))
  {
    balanceSettingPrevious = balanceSetting;
    flagBalance.volumeMoved = TRUE;
  }
  
  calculatedBalance = (float)(balanceSetting) * 620 / 1000 + 200;
  if(calculatedBalance > 820 )    calculatedBalance = 820; 
  if(calculatedBalance < 200 )     calculatedBalance = 200; 
  
  if((VA_SET_CENTER + VAC_EFFECTIVE_RANGE) < calculatedBalance)
  {
    calculation = calculatedBalance - (VA_SET_CENTER + VAC_EFFECTIVE_RANGE);
  }
  else if((VA_SET_CENTER - VAC_EFFECTIVE_RANGE) > calculatedBalance)
  {
    calculation = calculatedBalance - (VA_SET_CENTER - VAC_EFFECTIVE_RANGE);
  }
  else {
    calculation = 0;
  }
  
  return calculation;
}
//nvBalanceUpCurrent
void balance_fuzzy(uint8_t init)
{
  static uint8_t balanceStatus = 0;
  static uint16_t timerOutUp = 0;
  static uint16_t timerOutDown = 0;
  
  uint8_t balanceStatusPrevious, runMode; 
  
  if(init == 0)
  {
    flagBalance.volumeMoved = FALSE;
    balanceStatus = 0;
    timerOutUp = 0;
    timerOutDown = 0;
  } 
  balanceStatusPrevious = balanceStatus;
  
  if(strokeResult + balanceDeadBand < targetStrokePosition) {
    balanceStatus = VAB_DOWN_MODE;
  }
  else if(strokeResult - balanceDeadBand > targetStrokePosition) {
    balanceStatus = VAB_UP_MODE;
  }  
  else {
    if(flagBalance.volumeMoved == TRUE)
    {
      if(strokeResult < targetStrokePosition)            balanceStatus = VAB_DOWN_MODE;  
      else if(strokeResult > targetStrokePosition)       balanceStatus = VAB_UP_MODE;  
    }
  }

  flagBalance.volumeMoved = FALSE;
  runMode = VAB_STOP_MODE;

  if(balanceStatusPrevious == VAB_UP_MODE) {  
    if(balanceStatus == VAB_UP_MODE ) {
      if(strokeResult <= targetStrokePosition + VAC_STOP_DEADBAND_DOWN)  runMode = VAB_STOP_MODE;
      else                                                                      runMode = VAB_UP_MODE;
    }  
  }
  else if(balanceStatusPrevious == VAB_DOWN_MODE) {
    if(balanceStatus == VAB_DOWN_MODE) {
      if(strokeResult >= targetStrokePosition - VAC_STOP_DEADBAND_UP)    runMode = VAB_STOP_MODE;
      else                                                                      runMode = VAB_DOWN_MODE;
    }
  }
  else {
    if(balanceStatus == VAB_UP_MODE )                           runMode = VAB_UP_MODE;
    else if(balanceStatus == VAB_DOWN_MODE )                    runMode = VAB_DOWN_MODE;
    else                                                        runMode = VAB_STOP_MODE;
  }

    
  if(runMode == VAB_UP_MODE)
  {
    timerOutDown = 0;
    balanceStatus = VAB_UP_MODE;
    
    if(flagTimer.tenMs == TRUE)
    {
      timerOutUp += 10;
    }
    
    if(timerOutUp >= VA_BALANCE_UP_DELAY)
    {
       timerOutUp = VA_BALANCE_UP_DELAY;
       flagOutputControl.balanceDown = ON;
    }
        
    if(strokeResult < nvStrokeLeftLimit) {
      flagOutputControl.balanceDown = OFF;
    }
    return;
  }
  else if(runMode == VAB_DOWN_MODE)
  {
    timerOutUp = 0;
    balanceStatus = VAB_DOWN_MODE;
        
    if(flagTimer.tenMs == TRUE)
    {
      timerOutDown += 10;
    }
    
    if(timerOutDown >= VA_BALANCE_DOWN_DELAY) {
      timerOutDown= VA_BALANCE_DOWN_DELAY;
      flagOutputControl.balanceUp = ON;
    }
    
    if(strokeResult > nvStrokeRightLimit) {
      flagOutputControl.balanceUp = OFF;
    }
    return;
  }
  
  balanceStatus = VAB_STOP_MODE; 
  timerOutUp = 0;
  timerOutDown = 0;
  flagOutputControl.balanceUp = OFF;
  flagOutputControl.balanceDown = OFF;
  return;
}

void calculate_slope()
{
  float calculation;
  
  calculation = balance_vr() + (slopeRollCenter - rollingResult) * VAC_ROLL_CENTER_TO_STROKE + VA_SET_CENTER;

  if(calculation > nvStrokeRightLimit)          targetStrokePosition = nvStrokeRightLimit;
  else if(calculation < nvStrokeLeftLimit)      targetStrokePosition = nvStrokeLeftLimit;
  else                                          targetStrokePosition = calculation;
}

void calculate_flat()
{ 
  float calculation;
  
  calculation = balance_vr() + (VA_SET_CENTER - rollingResult) * VAC_ROLL_CENTER_TO_STROKE + VA_SET_CENTER ;
  
  if(calculation > nvStrokeRightLimit)          targetStrokePosition = nvStrokeRightLimit;
  else if(calculation < nvStrokeLeftLimit)      targetStrokePosition = nvStrokeLeftLimit;
  else                                          targetStrokePosition = calculation;
}

void auto_flat()
{
  if(flagBalance.autoFlat == FALSE)
  {
    if(timerFlat >= VA_BALANCE_DELAY)
    {
      flagBalance.autoFlat = TRUE;
      timerFlat = 0;
      balance_fuzzy(0);
    }
    return;
  }
  timerFlat = 0;
 
  calculate_flat();
  balance_fuzzy(1);                                                             // Added on 2022.01.25

  return;
}

void auto_slope()
{
  if(flagBalance.autoSlope == FALSE)
  {
    if(timerSlope >= VA_BALANCE_DELAY)
    {
      flagBalance.autoSlope = TRUE;
      timerSlope = 0;
      balance_fuzzy(0);
    }
    return;
  }
  timerSlope = 0;
  
  calculate_slope();
  balance_fuzzy(1);                                                             // Added on 2022.01.25

  return;
}

void auto_pall()
{
  if((strokeResult <= VA_SET_CENTER + 12) && (strokeResult >= VA_SET_CENTER - 12))
  {
    flagOutputControl.balanceUp = OFF;
    flagOutputControl.balanceDown = OFF;
    flagBalance.ptoStopped = FALSE;
  }
  else
  {
    if(strokeResult < VA_SET_CENTER)
    {
      flagOutputControl.balanceUp = ON;
      flagOutputControl.balanceDown = OFF;
    }
    if(strokeResult > VA_SET_CENTER)
    {
      flagOutputControl.balanceUp = OFF;
      flagOutputControl.balanceDown = ON;
    }
  }
}

void balance_output_limit()
{
  static uint16_t timerOutUpLimit = 0;
  static uint16_t timerOutDownLimit = 0;
 
  if(flagOutputControl.balanceUp == ON)
  {
    
    if(timerOutUpLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
    {
      flagOutputControl.balanceUp = OFF;
      timerOutUpLimit = VAC_BALANCE_OUTPUT_LIMIT_TIME + 300;
    }
    else 
    {
      if(flagTimer.hundredMs == TRUE)
      {
        timerOutUpLimit += 100;
      }
    }
    
    if(timerOutDownLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
    {
      if(flagTimer.hundredMs == TRUE)
      {
        timerOutDownLimit -= 100;
      }
    }
    else
    {
      timerOutDownLimit = 0;
    }
    
    return;
  }
  
  if(flagOutputControl.balanceDown == ON)
  {
    if(timerOutDownLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
    {
      flagOutputControl.balanceDown = OFF;
      timerOutDownLimit = VAC_BALANCE_OUTPUT_LIMIT_TIME + 300;
    }
    else
    {
      if(flagTimer.hundredMs == TRUE)
      {
        timerOutDownLimit += 100;
      }
    }
    
    if(timerOutUpLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
    {
      if(flagTimer.hundredMs == TRUE)
      {
        timerOutUpLimit -= 100;
      }
    }
    else
    {
      timerOutUpLimit = 0;
    }
    return;
  }
  
  if(timerOutUpLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
  {
    if(flagTimer.hundredMs == TRUE)
    {
      timerOutUpLimit -= 100;
    }
  }
  else
  {
    timerOutUpLimit = 0;
  }
  
  if(timerOutDownLimit >= VAC_BALANCE_OUTPUT_LIMIT_TIME)
  {
    if(flagTimer.hundredMs == TRUE)
    {
      timerOutDownLimit -= 100;
    }
  }
  else
  {
    timerOutDownLimit = 0;
  }
}

void balance_process()
{
  static uint8_t flagSlopeMode = 0;
  static uint16_t timerChangeFlagSlopeMode = 0;

  uint8_t isManualButtonPressed = FALSE;
  uint16_t liftStop;
  float calculation;
  
  if(flagTimer.tenMs == TRUE)
  {
    timerSlope += 10;   
    timerFlat += 10;
    timerRollingChange += 10;
  }

  if(canFlag.balanceSensitivityMode == VAB_BALANCE_FAST_MODE)
  {
    balanceDeadBand = (float)nvBalanceDeadbandLow;
    rollingChangeTime = nvBalanceSamplingTimeLow;
  }
  else if(canFlag.balanceSensitivityMode == VAB_BALANCE_NORMAL_MODE)
  {
    balanceDeadBand = (float)nvBalanceDeadbandMid;
    rollingChangeTime = nvBalanceSamplingTimeMid; 
  }
  else if(canFlag.balanceSensitivityMode == VAB_BALANCE_SLOW_MODE)
  {
    balanceDeadBand = (float)nvBalanceDeadbandHigh;
    rollingChangeTime = nvBalanceSamplingTimeHigh;
  }
  else
  {
    balanceDeadBand = (float)nvBalanceDeadbandMid;
    rollingChangeTime = nvBalanceSamplingTimeMid; 
  }
  
  calculation = get_stroke_sensor() - nvStrokeCenter + VA_SET_CENTER;
  if(calculation < 0)           calculation = 0;
  else if(calculation > 1023)   calculation = 1023; 
  strokeResult = calculation;

  if(timerRollingChange > rollingChangeTime)
  {
    timerRollingChange = 0;
    rollingSampling = (rollingSampling/2) + (get_balance_sensor()/2);

    calculation = rollingSampling - nvRollingCenter + VA_SET_CENTER;
    if(calculation < 0)           calculation = 0;
    else if(calculation > 1023)   calculation = 1023; 
    rollingResult = calculation;
  }
  /*
  if(flagTimer.oneSecond == TRUE)
  {
    printf("strokeResult = %.0f, rollingResult = %.0f, targetStrokePosition = %.0f \r\n", 
           strokeResult, rollingResult, targetStrokePosition);
  }
  */
  flagBalance.balanceMode = VAB_BALANCE_MANUAL_MODE;
  if(flagBalance.settingMode == TRUE)
  {
    flagBalance.balanceMode = VAB_BALANCE_MANUAL_MODE;
    slopeRollCenter = rollingResult;
  }
  else if(canFlag.balanceControlMode == VAB_BALANCE_FLAT_MODE) {
    flagBalance.balanceMode = VAB_BALANCE_FLAT_MODE;
    slopeRollCenter = rollingResult;
  }
  else if(canFlag.balanceControlMode == VAB_BALANCE_SLOPE_MODE) {
    flagBalance.balanceMode = VAB_BALANCE_SLOPE_MODE;
  }  
  else {
    flagBalance.balanceMode = VAB_BALANCE_MANUAL_MODE;
    slopeRollCenter = rollingResult;
  }

  // This content is added on 2022.06.15 - Start
  if(flagBalance.balanceMode == VAB_BALANCE_SLOPE_MODE)
  {
    if(flagSlopeMode != 2)
    {
      //if((flag.steeringRight == TRUE) || (flag.steeringLeft == TRUE))
      if(flag.isSteeringON == TRUE)
      {
        flagSlopeMode = 1;
      }
      else
      {
        // Steering OFF
        if(flagSlopeMode == 1)
        {
          flagSlopeMode = 2;
          timerChangeFlagSlopeMode = 0;
        }
        else
        {
          if((flag.backUpRun == TRUE) || (flag.turnUpRun == TRUE) || (flag.oneUpRun == TRUE))
          {
            flagSlopeMode = 3;
          }
          else if(flag.oneDownRun == TRUE)
          {
            if(flagSlopeMode == 3)
            {
              flagSlopeMode = 2;
              timerChangeFlagSlopeMode = 0;
            }
          }
        }
      }
    }
    
    if(flagSlopeMode == 2)
    {
      if(flagTimer.tenMs == TRUE)
      {
        timerChangeFlagSlopeMode += 10;
      }
      if(timerChangeFlagSlopeMode >= 1000)
      {
        slopeRollCenter = rollingResult;
        timerChangeFlagSlopeMode = 0;
      }
    }
  }

  liftStop = (uint16_t)((float)nvPtoLower3Pposition + 
                       ((float)nvPtoHigher3Pposition - (float)nvPtoLower3Pposition) * (float)canRxDial.ptoStopPosition / 250.0);
  
  if(get_threeP_position_sensor() > liftStop)
  {
    flagBalance.ptoStopped = TRUE;
    flagBalance.balanceMode = VAB_BALANCE_MANUAL_MODE;
    
    slopeRollCenter = rollingResult;
  }
  else
  {
    flagBalance.ptoStopped = FALSE;
  }

  if((canRxPillar.balanceUpButton == CAN_SP_BUTTON_ON) || (flagInputStatus.balanceUp == ON))
  {
    timerBalanceSetting = 0;
    
    isManualButtonPressed = TRUE;
    flagOutputControl.balanceUp = ON;
    if((canRxPillar.balanceDownButton == CAN_SP_BUTTON_ON) || (flagInputStatus.balanceDown == ON))
    {
      flagOutputControl.balanceUp = OFF;
      flagOutputControl.balanceDown = OFF;
    }
  }
  else if((canRxPillar.balanceDownButton == CAN_SP_BUTTON_ON) || (flagInputStatus.balanceDown == ON))
  {
    timerBalanceSetting = 0;
    
    isManualButtonPressed = TRUE;
    flagOutputControl.balanceDown = ON;
    if((canRxPillar.balanceUpButton == CAN_SP_BUTTON_ON) || (flagInputStatus.balanceUp == ON))
    {
      flagOutputControl.balanceUp = OFF;
      flagOutputControl.balanceDown = OFF;
    }
  }
  else {
    isManualButtonPressed = FALSE;
  }

  if(isManualButtonPressed == TRUE)
  {
    // The button input is 
    flagBalance.ptoStopped = FALSE;
    flagBalance.autoFlat = FALSE;
    flagBalance.autoSlope = FALSE;
    timerFlat = 0;
    timerSlope = 0;
    balance_output_limit();
    return;
  }
  
  if(flagBalance.ptoStopped == TRUE)
  {
    auto_pall();
    balance_output_limit();
    return;
  }
  else if(flagBalance.balanceMode == VAB_BALANCE_FLAT_MODE)
  {
    // Flat mode
    flagBalance.autoSlope = FALSE;
    timerSlope = 0;
    auto_flat();
    balance_output_limit();
    return;
  }
  else if(flagBalance.balanceMode == VAB_BALANCE_SLOPE_MODE)
  {
    // Slope mode
    flagBalance.autoFlat = FALSE;
    timerFlat = 0;
    auto_slope();
    balance_output_limit();
    return;
  }  
  
  flagBalance.autoFlat = FALSE;
  flagBalance.autoSlope = FALSE;
  timerSlope = 0;
  timerFlat = 0;
  balance_output_limit();
  return;
}


void balance_setting(void)
{
  static uint16_t timerSwOn = 0;
 
  if(flagTimer.tenMs == TRUE)
  {
    timerSwOn += 10;
  }
  if(flagTimer.hundredMs == TRUE)
  {
    timerBalanceSetting += 100;
  }

  if(flagBalance.settingMode == FALSE)
  {
    if(canRxFenderB.balanceModeButton == CAN_SP_BUTTON_ON)
    {
      if(timerBalanceSetting >= VAC_BALANCE_SETTING_TIME)
      {
        flagBalance.settingMode = TRUE;
        flagBalance.settingStroke = TRUE;
        flagBalance.settingRoll = FALSE;
        
        flagBalance.settingSequence = 0;
        
        timerBalanceSetting = 0;
        flag.checkBuzzer = TRUE;
      }
      return;
    }
    timerBalanceSetting = 0;
    return;
  }
 
  if(timerBalanceSetting >= VAC_BALANCE_SETTING_OVER_TIME)
  {
    timerBalanceSetting = 0;
    flagBalance.settingMode = FALSE;  
    flag.ngBuzzer = TRUE;

// Not saving in here    save_nvBalanceStroke(TRUE, VA_SET_CENTER, VA_SET_CENTER);
   
    return;
  }
 
  if(canRxFenderB.balanceModeButton == CAN_SP_BUTTON_ON)
  {
    timerBalanceSetting = 0;
    flag.checkBuzzer = TRUE;

    if((timerSwOn >= 100) && (timerSwOn <= 120)) {
      timerSwOn = 130;
      if(flagBalance.settingRoll == TRUE)
      {
        flagBalance.settingStroke = TRUE;
        flagBalance.settingRoll = FALSE;
      }
      else
      {
        flagBalance.settingStroke = FALSE;
        flagBalance.settingRoll = TRUE;
      }
      flagBalance.settingSequence = 0;
    }
  }
  else {
    timerSwOn = 0;
  }
  
  if(flagBalance.settingRoll == TRUE)
  {
    flagBalance.settingSequence = 1;
    if((rollingSampling >= (uint16_t)(VA_SET_CENTER - VA_SET_GAP)) && 
       (rollingSampling <= (uint16_t)(VA_SET_CENTER + VA_SET_GAP)))
    {
      flagBalance.settingSequence = 2;
       
      if((canRxFenderBPrevious.balanceSensitiveButton == CAN_SP_BUTTON_OFF) && (canRxFenderB.balanceSensitiveButton == CAN_SP_BUTTON_ON))
      {
        flag.checkBuzzer = TRUE;
        flagBalance.settingSequence = 3;
        save_nvBalanceStroke(TRUE, rollingSampling, nvStrokeCenter);
      }
      return;
    }
    return;
  }
  
  if(flagBalance.settingStroke == TRUE)
  {
    flagBalance.settingSequence = 1;
    
    if((get_stroke_sensor() >= (uint16_t)(VA_SET_CENTER - VA_SET_GAP)) && 
       (get_stroke_sensor() <= (uint16_t)(VA_SET_CENTER + VA_SET_GAP)))
    {
      flagBalance.settingSequence = 2;

      if((canRxFenderBPrevious.balanceSensitiveButton == CAN_SP_BUTTON_OFF) && (canRxFenderB.balanceSensitiveButton == CAN_SP_BUTTON_ON))
      {
        flag.checkBuzzer = TRUE;
        flagBalance.settingSequence = 3;
        
        save_nvBalanceStroke(TRUE, nvRollingCenter, get_stroke_sensor());
      }
      return;
    }
    return;
  }
}

void balance_init()
{ 
  flagBalance.settingMode = FALSE;
  flagBalance.settingRoll = FALSE;
  flagBalance.settingStroke = FALSE;
  
  strokeResult = get_stroke_sensor();
}

uint8_t unload_valve_control()
{
  static uint16_t timerUnloadValveOn = UNLOAD_VALVE_ON_AND_OFF_TIME;
  static uint16_t timerUnloadValveOff = UNLOAD_VALVE_ON_AND_OFF_TIME;
  
  flagOutputControl.unload = OFF;                                           // clear output
  
  if((flagOutputControl.balanceUp == ON) || (flagOutputControl.balanceDown == ON) ||
     (flagOutputControl.topLinkUp == ON) || (flagOutputControl.topLinkDown == ON)) {
    timerUnloadValveOff = 0;
    timerUnloadValveOn += 2;                                                       // Every 2ms, this variable is added
    if(timerUnloadValveOn >= UNLOAD_VALVE_ON_AND_OFF_TIME) {                                               // 2ms * 3 = 6 ms
      timerUnloadValveOn = UNLOAD_VALVE_ON_AND_OFF_TIME;
      flagOutputControl.unload = ON;
    }
  }

  if((flagOutputControl.balanceUp == OFF) && (flagOutputControl.balanceDown == OFF) &&
     (flagOutputControl.topLinkUp == OFF) && (flagOutputControl.topLinkDown == OFF)) {
    flagOutputControl.unload = OFF;
    timerUnloadValveOn = 0;
    timerUnloadValveOff += 2;                                                       // Every 2ms, this variable is added
    if(timerUnloadValveOff >= UNLOAD_VALVE_ON_AND_OFF_TIME) {                                              // 2ms * 3 = 6 ms
      timerUnloadValveOff = UNLOAD_VALVE_ON_AND_OFF_TIME;
    }
    else {
      flagOutputControl.balanceDown = flagOutputControlPrevious.balanceDown;
      flagOutputControl.balanceUp = flagOutputControlPrevious.balanceUp;
      flagOutputControl.topLinkUp = flagOutputControlPrevious.topLinkUp;
      flagOutputControl.topLinkDown = flagOutputControlPrevious.topLinkDown;
    }
  }
  
  flagOutputControlPrevious.balanceDown = flagOutputControl.balanceDown;
  flagOutputControlPrevious.balanceUp = flagOutputControl.balanceUp;
  flagOutputControlPrevious.topLinkUp = flagOutputControl.topLinkUp;
  flagOutputControlPrevious.topLinkDown = flagOutputControl.topLinkDown;
  flagOutputControlPrevious.unload = flagOutputControl.unload;
  return 1;
}

uint16_t balanceUpCurrent;
uint16_t balanceDownCurrent;

uint16_t get_balanceUpCurrent()
{
  return balanceUpCurrent;
}

uint16_t get_balanceDownCurrent()
{
  return balanceDownCurrent;
}
