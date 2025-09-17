/**
  ******************************************************************************
  * @Author         : Enkhbat Batbayar
  * @brief          : TSC1 -- Torque/Speed Control ECU
  ******************************************************************************
  * @TSC1 CAN protocol
  *
  *     Byte-0
  *             overrideControlModePriority
  *                     00 --> Override disabled
  *                     01 --> Speed Control
  *                     10 --> Torque Control
  *                     11 --> Speed/Torque Limit Control
  *             engineRequestedSpeedControlConditions
  *                     00 --> Transient Optimized, driveline disengaged
  *                     01 --> Stability Optimized, driveline disengaged
  *                     10 --> Stability Optimized, driveline condition 1
  *                     11 --> Stability Optimized, driveline condition 2
  *             engineOverrideControlMode
  *                     00 --> Highest priority
  *                     01 --> High priority
  *                     10 --> Medium priority
  *                     11 --> Low priority
  *     Byte-1,2
  *             RESULT_RPM
  *                     Length          --> 16 bit
  *                     Data Format     --> Intel
  *                     Offset value    --> 0
  *                     Resolution      --> 0.125 rpm/bit
  *                     Data Range      --> 0 to 8031.875 rpm
  *     Byte-3
  *             RESULT_TORQUE
  *                     Length          --> 8 bit
  *                     Data Format     --> Intel
  *                     Offset value    --> -125
  *                     Resolution      --> 1 %/bit
  *                     Data Range      --> -125 to 125%
  *                     Operating Range --> 0 to 125%
  *     Byte-4 to 7
  *             Not used fill 0xFF
  *
  ******************************************************************************
  */
/**
  ******************************************************************************
  *
  *     2021.06.15
  *             1. The main structure of the algorithm and program are implemented
  *             2. Implement TSC1 protocol.
  *
  ******************************************************************************
  */
#include "rpmController.h"
#include "input.h"
#include "output.h"
#include "can.h"
#include "algorithm.h"
#include "main.h"
#include "settings.h"
#include "adc.h"
#include "sensors.h"

/* --------------------------------------------------------- CAN definitions -------------------------------------------------------*/

#define TSC1_OVERRIDE_DISABLED          0x00
#define TSC1_SPEED_CONTROL              0x01
#define TSC1_TORQUE_CONTROL             0x02
#define TSC1_TORQUE_SPEED_LIMIT         0x03

#define TSC1_NOT_AVAILABLE              0x00                                    // Changed on 2022.04.12

#define TSC1_HIGHEST_PRIORITY           0x00
#define TSC1_HIGH_PRIORITY              0x01
#define TSC1_MEDIUM_PRIORITY            0x02
#define TSC1_LOW_PRIORITY               0x03

#define TSC1_NOT_USED                   0x03                                    // Added on 2022.04.12

#define TSC1_SPEED_ERROR_INDICATOR      0xFE00
#define TSC1_SPEED_NOT_AVAILABLE        0xFF00

#define TSC1_TORQUE_ERROR_INDICATOR     0xFE
#define TSC1_TORQUE_NOT_AVAILABLE       0xFF

#define TSC1_SPEED_OFFSET               0
#define TSC1_TORQUE_OFFSET              125

/* --------------------------------------------------------- Engine RPM definitions ------------------------------------------------*/

/* --------------------------------------------------------- Engine RPM variables --------------------------------------------------*/
rpmControllerFlag_t     rpmControllerFlag;
uint8_t                 overrideControlModePriority;
uint8_t                 engineRequestedSpeedControlConditions;
uint8_t                 engineOverrideControlMode;
uint8_t                 resultTorque;
uint16_t                resultRpm;

uint16_t                nvRpmCruiseNewA;
uint16_t                nvRpmCruiseNewB;

void rpm_control_process()
{
  static uint8_t  isSavingProcessHappenedA = FALSE;
  static uint8_t  isSavingProcessHappenedB = FALSE;
  static uint16_t timerRpmCruiseA = 0;
  static uint16_t timerRpmCruiseB = 0;
  
  uint16_t footValue, handValue;
  float    footRpm, handRpm;
  uint16_t calculatedRpm = TSC1_SPEED_ERROR_INDICATOR;
  
  if((canRxArmRest.rpmCruiseAButton == CAN_SP_BUTTON_OFF) && (canRxArmRestPreviousApp.rpmCruiseAButton == CAN_SP_BUTTON_ON))
  {             // push button clicked
    if(isSavingProcessHappenedA == FALSE) {
      flag.rpmCruiseA = (flag.rpmCruiseA == TRUE) ? FALSE : TRUE;
    }
    isSavingProcessHappenedA = FALSE;
    timerRpmCruiseA = 0;
    
    if(flag.rpmCruiseA == TRUE)
    {
      flag.rpmCruiseB = FALSE;
      isSavingProcessHappenedB = FALSE;
    }
  }
    
  if((canRxArmRest.rpmCruiseBButton == CAN_SP_BUTTON_OFF) && (canRxArmRestPreviousApp.rpmCruiseBButton == CAN_SP_BUTTON_ON))
  {
    if(isSavingProcessHappenedB == FALSE)
    {
      flag.rpmCruiseB = (flag.rpmCruiseB == TRUE) ? FALSE : TRUE;
    }
    isSavingProcessHappenedB = FALSE;
    timerRpmCruiseB = 0;
    
    if(flag.rpmCruiseB == TRUE)
    {
      flag.rpmCruiseA = FALSE;
      isSavingProcessHappenedA = FALSE;
    }
  }  
  
  if((flag.rpmCruiseA == TRUE) || (flag.rpmCruiseB == TRUE))
  {
    if((flagInputStatus.brake == ON) && (flagInputStatus.sideBrake == OFF))
    {
      flag.rpmCruiseA = FALSE;
      flag.rpmCruiseB = FALSE;
    }
  }
  
  canRxArmRestPreviousApp.rpmCruiseAButton = canRxArmRest.rpmCruiseAButton;
  canRxArmRestPreviousApp.rpmCruiseBButton = canRxArmRest.rpmCruiseBButton;
  
//  armSetLamp.cruiseCheck = flag.rpmCruise;
//  flagInputStatusPrevious.rpmCruise = flagInputStatus.rpmCruise;                            // update previous value
// Disabled for STAGE-V  flagOutputControl.adPro = OFF;
  
  if(flag.rpmCruiseA == TRUE)
  {
    if((canRxArmRest.rpmCruiseAButton == CAN_SP_BUTTON_ON) && (canRxArmRestPreviousApp.rpmCruiseAButton == CAN_SP_BUTTON_ON))
    {
      if(flagTimer.tenMs == TRUE)
      {
        timerRpmCruiseA += 10;
      }
      if(timerRpmCruiseA > VAC_RPM_CRUISE_SAVE_TIME)
      {
        rpmControllerFlag.rpmCruiseSwLongPressedA = ON;
      }
    }
  
    rpmControllerFlag.rpmCruiseUsedA = ON;
    rpmControllerFlag.handSensorError = OFF;
    rpmControllerFlag.handSensorUsed  = OFF;
    rpmControllerFlag.footSensorError = OFF;
    rpmControllerFlag.footSensorUsed  = OFF;
    
    if((canRxArmRest.rpmDecreaseButton == CAN_SP_BUTTON_OFF) && (canRxArmRestPreviousApp.rpmDecreaseButton == CAN_SP_BUTTON_ON))
    {
      nvRpmCruiseNewA -= 50;
    }
    else if((canRxArmRest.rpmIncreaseButton == CAN_SP_BUTTON_OFF) && (canRxArmRestPreviousApp.rpmIncreaseButton == CAN_SP_BUTTON_ON))
    {
      nvRpmCruiseNewA += 50;
    }
    canRxArmRestPreviousApp.rpmDecreaseButton = canRxArmRest.rpmDecreaseButton;
    canRxArmRestPreviousApp.rpmIncreaseButton = canRxArmRest.rpmIncreaseButton;
    
    if(nvRpmCruiseNewA < get_control_minimum_data(NV_RPM_CRUISE_A))
      nvRpmCruiseNewA = get_control_minimum_data(NV_RPM_CRUISE_A);
    else if(nvRpmCruiseNewA > get_control_maximum_data(NV_RPM_CRUISE_A))
      nvRpmCruiseNewA = get_control_maximum_data(NV_RPM_CRUISE_A);
    
    resultRpm = (uint16_t)((float)(nvRpmCruiseNewA) / VAC_RPM_TO_BIT);                                   // update rpm
    
    if(rpmControllerFlag.rpmCruiseSwLongPressedA == ON)
    {
      rpmControllerFlag.rpmCruiseSwLongPressedA = OFF;
      save_nvRpmCruiseA(TRUE, nvRpmCruiseNewA);
      flag.checkBuzzer = ON;
      timerRpmCruiseA = 0;
      isSavingProcessHappenedA = TRUE;
    }
    return;                                                                     // Exit from here
  }
  else if(flag.rpmCruiseB == TRUE)
  {
    if((canRxArmRest.rpmCruiseBButton == CAN_SP_BUTTON_ON) && (canRxArmRestPreviousApp.rpmCruiseBButton == CAN_SP_BUTTON_ON))
    {
      if(flagTimer.tenMs == TRUE)
      {
        timerRpmCruiseB += 10;
      }
      if(timerRpmCruiseB > VAC_RPM_CRUISE_SAVE_TIME)
      {
        rpmControllerFlag.rpmCruiseSwLongPressedB = ON;
      }
    }
  
    rpmControllerFlag.rpmCruiseUsedB = ON;
    rpmControllerFlag.handSensorError = OFF;
    rpmControllerFlag.handSensorUsed  = OFF;
    rpmControllerFlag.footSensorError = OFF;
    rpmControllerFlag.footSensorUsed  = OFF;
    
    if((canRxArmRest.rpmDecreaseButton == CAN_SP_BUTTON_OFF) && (canRxArmRestPreviousApp.rpmDecreaseButton == CAN_SP_BUTTON_ON))
    {
      nvRpmCruiseNewB -= 50;
    }
    else if((canRxArmRest.rpmIncreaseButton == CAN_SP_BUTTON_OFF) && (canRxArmRestPreviousApp.rpmIncreaseButton == CAN_SP_BUTTON_ON))
    {
      nvRpmCruiseNewB += 50;
    }
    canRxArmRestPreviousApp.rpmDecreaseButton = canRxArmRest.rpmDecreaseButton;
    canRxArmRestPreviousApp.rpmIncreaseButton = canRxArmRest.rpmIncreaseButton;
    
    if(nvRpmCruiseNewB < get_control_minimum_data(NV_RPM_CRUISE_B))
      nvRpmCruiseNewB = get_control_minimum_data(NV_RPM_CRUISE_B);
    else if(nvRpmCruiseNewB > get_control_maximum_data(NV_RPM_CRUISE_B))
      nvRpmCruiseNewB = get_control_maximum_data(NV_RPM_CRUISE_B);
    
    resultRpm = (uint16_t)((float)(nvRpmCruiseNewB) / VAC_RPM_TO_BIT);                                   // update rpm
    
    if(rpmControllerFlag.rpmCruiseSwLongPressedB == ON)
    {
      rpmControllerFlag.rpmCruiseSwLongPressedB = OFF;
      save_nvRpmCruiseB(TRUE, nvRpmCruiseNewB);
      flag.checkBuzzer = ON;
      timerRpmCruiseB = 0;
      isSavingProcessHappenedB = TRUE;
    }
    return;                                                                     // Exit from here
  }
  
  rpmControllerFlag.footIvs = flagInputStatus.footPedal;
  
  footValue = get_foot_accelerator();
  handValue = get_hand_accelerator();
  
       if(handValue <= VAC_HAND_ACCEL_MIN)       handRpm = VAC_RPM_MIN;
  else if(handValue >= VAC_HAND_ACCEL_MAX)       handRpm = VAC_RPM_MAX;
  else {
    handRpm = (((float)handValue - VAC_HAND_ACCEL_MIN) * VAC_HAND_RPM_CAL) + VAC_RPM_MIN;
  }

       if(footValue <= VAC_FOOT_ACCEL_MIN)       footRpm = VAC_RPM_MIN;
  else if(footValue >= VAC_FOOT_ACCEL_MAX)       footRpm = VAC_RPM_MAX;
  else {
    footRpm = (((float)footValue - VAC_FOOT_ACCEL_MIN) * VAC_FOOT_RPM_CAL) + VAC_RPM_MIN;
  }
  
  if((flagErrorSensors.footPedal) && (flagErrorSensors.handPedal))
  {
    calculatedRpm = TSC1_SPEED_ERROR_INDICATOR;                                 // Error is occured, please check the hand and the foot sensors
    rpmControllerFlag.handSensorError = ON;
    rpmControllerFlag.handSensorUsed  = OFF;
    rpmControllerFlag.footSensorError = ON;
    rpmControllerFlag.footSensorUsed  = OFF;
  }
  else if(flagErrorSensors.footPedal)
  {
    rpmControllerFlag.handSensorError = OFF;
    rpmControllerFlag.handSensorUsed  = ON;
    rpmControllerFlag.footSensorError = ON;
    rpmControllerFlag.footSensorUsed  = OFF;

           if(handRpm > VAC_RPM_MAX)      calculatedRpm = (uint16_t)VAC_RPM_MAX;
      else if(handRpm < VAC_RPM_MIN)      calculatedRpm = (uint16_t)VAC_RPM_MIN;
      else                                calculatedRpm = (uint16_t)handRpm;
  }
  else if(flagErrorSensors.handPedal)
  {
    rpmControllerFlag.handSensorError = ON;
    rpmControllerFlag.handSensorUsed  = OFF;
    rpmControllerFlag.footSensorError = OFF;
    rpmControllerFlag.footSensorUsed  = ON;
    
           if(footRpm > VAC_RPM_MAX)      calculatedRpm = (uint16_t)VAC_RPM_MAX;
      else if(footRpm < VAC_RPM_MIN)      calculatedRpm = (uint16_t)VAC_RPM_MIN;
      else                                calculatedRpm = (uint16_t)footRpm;
  }
  else{

      rpmControllerFlag.handSensorError = OFF;
      rpmControllerFlag.footSensorError = OFF;
      if(handRpm > footRpm)
      {
        calculatedRpm = (uint16_t)handRpm;
        rpmControllerFlag.handSensorUsed  = ON;
        rpmControllerFlag.footSensorUsed  = OFF;
      }
      else{
        calculatedRpm = (uint16_t)footRpm;
        rpmControllerFlag.handSensorUsed  = OFF;
        rpmControllerFlag.footSensorUsed  = ON;
      }
  }

  if((flagErrorSensors.footPedal == FALSE) || (flagErrorSensors.handPedal == FALSE)){
         if(calculatedRpm > VAC_RPM_MAX)        calculatedRpm = (uint16_t)VAC_RPM_MAX;
    else if(calculatedRpm < VAC_RPM_MIN)        calculatedRpm = (uint16_t)VAC_RPM_MIN;
  }

  resultRpm = calculatedRpm;
}
    
void rpm_can_packet(uint8_t *data)
{
  static uint8_t checkSumCounter = 0;
  static uint8_t checkSum = 0;
  static uint8_t checkSumResult = 0;
  uint8_t i;
  
  overrideControlModePriority = TSC1_HIGHEST_PRIORITY;                          // Highest priority
  engineRequestedSpeedControlConditions = TSC1_NOT_AVAILABLE;                   // NOT available
  engineOverrideControlMode = TSC1_SPEED_CONTROL;                               // SPEED Control
  
  resultTorque = 225;
  
  data[0] = engineOverrideControlMode | (engineRequestedSpeedControlConditions << 2) | (overrideControlModePriority << 4) | (TSC1_NOT_USED << 6);              // 0xC0 is NOT used bits
  data[1] = (uint8_t)(resultRpm);
  data[2] = (uint8_t)(resultRpm >> 8);
  data[3] = (uint8_t)(resultTorque);
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  
  // Calculating checksum
#define TSC1_ID_CHECKSUM	0x0C + 0x00 + 0x00 + 0x03	        // TSC1 ID =  0C 00 00 00 17 총 합계 
  
  checkSumCounter++;
  if(checkSumCounter >= 8) {
    checkSumCounter = 0; 	                                                // Messaage Counter 0 to 7 and then wraps ..
  }
  
  checkSum = 0;
  for(i = 0; i < 7; i++) {
    checkSum += data[i];
  }
  checkSum += (checkSumCounter & 0x0F);
  checkSum += TSC1_ID_CHECKSUM;
  
  checkSumResult =  (checkSum >> 6) & 0x03;
  checkSumResult =   checkSumResult + (checkSum >> 3);	                        // 2018.08.20. Shift 숫자가 잘못되어 수정 6->3으로
  checkSumResult =  (checkSumResult +  checkSum) & 0x07;
	  
  checkSumResult =  checkSumResult << 4;		                        // SHIFT 4 
  checkSumResult =  checkSumResult & 0xF0;
  checkSumResult += checkSumCounter;			                        // COUNIT값 OR 처리.. 
  
  data[7] = checkSumResult;
}