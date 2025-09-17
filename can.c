#include "can.h"
#include "main.h"
#include "defines.h"
#include "output.h"
#include "input.h"
#include "rpmController.h"
#include "sensors.h"
#include "settings.h"
#include "algorithm.h"
#include "balance.h"
#include "hitch.h"

#include "vn5025.h"
#include "bts7006.h"
#include "pto.h"

/* --------------------------------------------------------- STM32 CAN variables ---------------------------------------------------*/
CAN_HandleTypeDef               hcan1;
CAN_HandleTypeDef               hcan2;

/* --------------------------------------------------------- CAN variables ---------------------------------------------------------*/
CAN_TxHeaderTypeDef             canOneTxHeader;
CAN_RxHeaderTypeDef             canOneRxHeader;
uint8_t                         canOneTxData[8];
uint8_t                         canOneRxData[8];
uint32_t                        canOneTxMailbox;

CAN_TxHeaderTypeDef             canTwoTxHeader;
CAN_RxHeaderTypeDef             canTwoRxHeader;
uint8_t                         canTwoTxData[8];
uint8_t                         canTwoRxData[8];
uint32_t                        canTwoTxMailbox;

uint16_t                        canErrorCounter[2];

uint16_t                        engineSpeed;

canRxEEC2_t                     canRxEEC2;
canRxEEC2_t                     canRxEEC2Temp;
canRxEEC2_t                     canRxEEC2Previous;

canRxEEC1_t                     canRxEEC1;
canRxEEC1_t                     canRxEEC1Temp;
canRxEEC1_t                     canRxEEC1Previous;

canTxTSC1_t                     canTxTSC1;
canTxSKC_t                      canTxSKC;
canTxIP1_t                      canTxIP1;
canTxIP2_t                      canTxIP2;

canRxACU_t                      canRxACU;
canRxACU_t                      canRxACUTemp;
canRxACU_t                      canRxACUPrevious;
canTxACU_t                      canTxACU;

canTxPA_t                       canTxPA;
canTxEAT_t                      canTxEAT;

canRxTBOX1_t                    canRxTBOX1;
canRxTBOX1_t                    canRxTBOX1Temp;
canRxTBOX1_t                    canRxTBOX1Previous;

canRxTBOX2_t                    canRxTBOX2;
canRxTBOX2_t                    canRxTBOX2Temp;
canRxTBOX2_t                    canRxTBOX2Previous;

canRxCPG1_t                     canRxCPG1;
canRxCPG1_t                     canRxCPG1Temp;
canRxCPG1_t                     canRxCPG1Previous;

canRxCPG2_t                     canRxCPG2;
canRxCPG2_t                     canRxCPG2Temp;
canRxCPG2_t                     canRxCPG2Previous;

canRxCPG3_t                     canRxCPG3;
canRxCPG3_t                     canRxCPG3Temp;
canRxCPG3_t                     canRxCPG3Previous;

canRxArmRest_t                  canRxArmRest;
canRxArmRest_t                  canRxArmRestTemp;
canRxArmRest_t                  canRxArmRestPrevious;
canRxArmRest_t                  canRxArmRestPreviousApp;

canRxDash_t                     canRxDash;
canRxDash_t                     canRxDashTemp;
canRxDash_t                     canRxDashPrevious;
canTxDashSetting_t              canTxDashSetting;
canTxDash_t                     canTxDash;

canRxPillar_t                   canRxPillar;
canRxPillar_t                   canRxPillarTemp;
canRxPillar_t                   canRxPillarPrevious;
canTxPillarSetting_t            canTxPillarSetting;
canTxPillar_t                   canTxPillar;

canRxFenderA_t                  canRxFenderA;
canRxFenderA_t                  canRxFenderATemp;
canRxFenderA_t                  canRxFenderAPrevious;
canTxFenderASetting_t           canTxFenderASetting;
canTxFenderA_t                  canTxFenderA;

canRxFenderB_t                  canRxFenderB;
canRxFenderB_t                  canRxFenderBTemp;
canRxFenderB_t                  canRxFenderBPrevious;
canTxFenderBSetting_t           canTxFenderBSetting;
canTxFenderB_t                  canTxFenderB;

canRxDial_t                     canRxDial;
canRxDial_t                     canRxDialTemp;
canRxDial_t                     canRxDialPrevious;

canRxAngle_t                    canRxAngle;
canRxAngle_t                    canRxAngleTemp;
canRxAngle_t                    canRxAnglePrevious;

canRxGyro_t                     canRxGyro;
canRxGyro_t                     canRxGyroTemp;
canRxGyro_t                     canRxGyroPrevious;

canFlag_t                       canFlag;
canFlag_t                       canFlagPrevious;
/* --------------------------------------------------------- Local Functions ----------------------------------------------------*/
void can_filter_Init(uint8_t filter, uint32_t id, uint32_t mask, uint8_t std);
void can_interrupt_enable_one();
void can_interrupt_enable_two();

void init_can_variables()
{
  
}

void can_receive_process()
{
  uint8_t i;
  static uint16_t timerNewNvPanelData = 0;
  uint16_t newNvPanelData = 0;
  
  // CAN1 messages are handled
  if(canRxEEC2Temp.flag == TRUE)
  {
    canRxEEC2Temp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxEEC2Previous.data[i] = canRxEEC2.data[i];
      canRxEEC2.data[i] = canRxEEC2Temp.data[i];
    }
  }
  
  if(canRxEEC1Temp.flag == TRUE)
  {
    canRxEEC1Temp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxEEC1Previous.data[i] = canRxEEC1.data[i];
      canRxEEC1.data[i] = canRxEEC1Temp.data[i];
    }
    
    engineSpeed = ((canRxEEC1.engineSpeedMSB << 8) + canRxEEC1.engineSpeedLSB) / 8;
  }
  
  if(canRxACUTemp.flag == TRUE)
  {
    canRxACUTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxACUPrevious.data[i] = canRxACU.data[i];
      canRxACU.data[i] = canRxACUTemp.data[i];
    }
  }
  
  if(canRxTBOX1Temp.flag == TRUE)
  {
    canRxTBOX1Temp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxTBOX1Previous.data[i] = canRxTBOX1.data[i];
      canRxTBOX1.data[i] = canRxTBOX1Temp.data[i];
    }
  }
  
  if(canRxTBOX2Temp.flag == TRUE)
  {
    canRxTBOX2Temp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxTBOX2Previous.data[i] = canRxTBOX2.data[i];
      canRxTBOX2.data[i] = canRxTBOX2Temp.data[i];
    }
  }
  
  if(canRxCPG1Temp.flag == TRUE)
  {
    canRxCPG1Temp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxCPG1Previous.data[i] = canRxCPG1.data[i];
      canRxCPG1.data[i] = canRxCPG1Temp.data[i];
    }
  }
  
  if(canRxCPG2Temp.flag == TRUE)
  {
    canRxCPG2Temp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxCPG2Previous.data[i] = canRxCPG2.data[i];
      canRxCPG2.data[i] = canRxCPG2Temp.data[i];
    }
  }
  
  if(canRxCPG3Temp.flag == TRUE)
  {
    canRxCPG3Temp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxCPG3Previous.data[i] = canRxCPG3.data[i];
      canRxCPG3.data[i] = canRxCPG3Temp.data[i];
    }
  }
  
  // CAN2 messages are handled
  if(canRxArmRestTemp.flag == TRUE)
  {
    canRxArmRestTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxArmRestPrevious.data[i] = canRxArmRest.data[i];
      canRxArmRest.data[i] = canRxArmRestTemp.data[i];
    }
  }
  
  if(canRxDashTemp.flag == TRUE)
  {
    canRxDashTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxDashPrevious.data[i] = canRxDash.data[i];
      canRxDash.data[i] = canRxDashTemp.data[i];
    }
    
    // Dash Button check
    if((canRxDash.fourWDButton == CAN_SP_BUTTON_ON) && (canRxDashPrevious.fourWDButton == CAN_SP_BUTTON_OFF))
    {
      if(canFlag.quickTurn == FALSE)
      {
        canFlag.saveLastData = TRUE;
        timerNewNvPanelData = 0;
        
        flag.buzzerButton = TRUE;
        if(canFlag.fourWD == VAC_4WD_MODE_OFF)
        {
          canFlag.fourWD = VAC_4WD_AUTO_MODE;             // Auto
        }
        else if(canFlag.fourWD == VAC_4WD_AUTO_MODE)
        {
          canFlag.fourWD = VAC_4WD_MANUAL_MODE;           // Manual
        }
        else
        {
          canFlag.fourWD = VAC_4WD_MODE_OFF;              // OFF
        }
      }
    }
      
    if((canRxDash.quickTurnButton == CAN_SP_BUTTON_ON) && (canRxDashPrevious.quickTurnButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(canFlag.quickTurn == FALSE)
      {
        canFlag.quickTurn = TRUE;                       // ON
        canFlag.fourWDPrevious = canFlag.fourWD;
        canFlag.fourWD = VAC_4WD_MANUAL_MODE;           // Manual
      }
      else
      {
        canFlag.quickTurn = FALSE;                      // OFF
        canFlag.fourWD = canFlag.fourWDPrevious;
      }
    }

    if(canRxDash.regenButton == ON)
    {
      canFlag.regen = TRUE;
      if(canRxDashPrevious.regenButton == CAN_SP_BUTTON_OFF)
      {
        flag.buzzerButton = TRUE;
      }
    }
    else
    {
      canFlag.regen = FALSE;
    }
    
    if((canRxDash.screenTransitionButton == CAN_SP_BUTTON_ON) && (canRxDashPrevious.screenTransitionButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(canFlag.screenTransaction == FALSE)
      {
        canFlag.screenTransaction = TRUE;
      }
      else
      {
        canFlag.screenTransaction = FALSE;
      }
    }
  }
  
  if(canRxPillarTemp.flag == TRUE)
  {
    canRxPillarTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxPillarPrevious.data[i] = canRxPillar.data[i];
      canRxPillar.data[i] = canRxPillarTemp.data[i];
    }
    
    // Pillar Button check
    if(canRxPillar.hitchUpButton == CAN_SP_BUTTON_ON)
    {
      canFlag.hitchUpPilar = TRUE;
      
      if(canRxPillarPrevious.hitchUpButton == CAN_SP_BUTTON_OFF)
      {
        flag.buzzerButton = TRUE;  
      }
    }
    else
    {
      canFlag.hitchUpPilar = FALSE;  
    }
        
    if(canRxPillar.hitchDownButton == CAN_SP_BUTTON_ON)
    {
      canFlag.hitchDownPilar = TRUE;
      if(canRxPillarPrevious.hitchDownButton == CAN_SP_BUTTON_OFF)
      {
        flag.buzzerButton = TRUE;
      }
    }
    else
    {
      canFlag.hitchDownPilar = FALSE;  
    }

    if(canRxPillar.topLinkUpButton == CAN_SP_BUTTON_ON)
    {
      canFlag.toplinkUpPilar = TRUE;
      if(canRxPillarPrevious.topLinkUpButton == CAN_SP_BUTTON_OFF)
      {
        flag.buzzerButton = TRUE;
      }
    }
    else
    {
      canFlag.toplinkUpPilar = FALSE;  
    }

    if(canRxPillar.topLinkDownButton == CAN_SP_BUTTON_ON)
    {
      canFlag.toplinkDownPilar = TRUE;
      if(canRxPillarPrevious.topLinkDownButton == CAN_SP_BUTTON_OFF)
      {
        flag.buzzerButton = TRUE;
      }
    }
    else
    {
      canFlag.toplinkDownPilar = FALSE;  
    }

    if(canRxPillar.balanceUpButton == CAN_SP_BUTTON_ON)
    {
      canFlag.balanceUpPillar = TRUE;
      if(canRxPillarPrevious.balanceUpButton == CAN_SP_BUTTON_OFF)
      {
        flag.buzzerButton = TRUE;
      }
    }
    else
    {
      canFlag.balanceUpPillar = FALSE;  
    }
      
    if(canRxPillar.balanceDownButton == CAN_SP_BUTTON_ON)
    {
      canFlag.balanceDownPillar = TRUE;
      if(canRxPillarPrevious.balanceDownButton == CAN_SP_BUTTON_OFF)
      {
        flag.buzzerButton = TRUE;
      }
    }
    else
    {
      canFlag.balanceDownPillar = FALSE;  
    }
  }
  
  if(canRxFenderATemp.flag == TRUE)
  {
    canRxFenderATemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxFenderAPrevious.data[i] = canRxFenderA.data[i];
      canRxFenderA.data[i] = canRxFenderATemp.data[i];
    }
    
    // Fender A Button check
    if((canRxFenderA.upHitchButton == CAN_SP_BUTTON_ON) && (canRxFenderAPrevious.upHitchButton == CAN_SP_BUTTON_OFF))
    {
      canFlag.saveLastData = TRUE;
      timerNewNvPanelData = 0;
      
      flag.buzzerButton = TRUE;
      if((canFlag.turnUpHitch == FALSE) && (canFlag.backUpHitch == FALSE))
      {
        // Turn up ON
        canFlag.turnUpHitch = TRUE;
        canFlag.backUpHitch = FALSE;
      }
      else if((canFlag.turnUpHitch == TRUE) && (canFlag.backUpHitch == FALSE))
      {
        // Back up ON
        canFlag.turnUpHitch = FALSE;
        canFlag.backUpHitch = TRUE;
      }
      else if((canFlag.turnUpHitch == FALSE) && (canFlag.backUpHitch == TRUE))
      {
        // Both ON
        canFlag.turnUpHitch = TRUE;
        canFlag.backUpHitch = TRUE;
      }
      else
      {
        // Both OFF
        canFlag.turnUpHitch = FALSE;
        canFlag.backUpHitch = FALSE;
      }
    }
    
    if((canRxFenderA.downRpmButton == CAN_SP_BUTTON_ON) && (canRxFenderAPrevious.downRpmButton == CAN_SP_BUTTON_OFF))
    {
      canFlag.saveLastData = TRUE;
      timerNewNvPanelData = 0;
      
      flag.buzzerButton = TRUE;
      if((canFlag.turnDownRpm == FALSE) && (canFlag.backDownRpm == FALSE))
      {
        // Turn down ON
        canFlag.turnDownRpm = TRUE;
        canFlag.backDownRpm = FALSE;
      }
      else if((canFlag.turnDownRpm == TRUE) && (canFlag.backDownRpm == FALSE))
      {
        // Back down ON
        canFlag.turnDownRpm = FALSE;
        canFlag.backDownRpm = TRUE;
      }
      else if((canFlag.turnDownRpm == FALSE) && (canFlag.backDownRpm == TRUE))
      {
        // Both ON
        canFlag.turnDownRpm = TRUE;
        canFlag.backDownRpm = TRUE;
      }
      else
      {
        // Both OFF
        canFlag.turnDownRpm = FALSE;
        canFlag.backDownRpm = FALSE;
      }
    }
      
    if((canRxFenderA.readingButton == CAN_SP_BUTTON_ON) && (canRxFenderAPrevious.readingButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(canFlag.readingLight == FALSE)
      {
        canFlag.readingLight = TRUE;
      }
      else
      {
        canFlag.readingLight = FALSE;
      }
    }

    if((canRxFenderA.stationaryButton == CAN_SP_BUTTON_ON) && (canRxFenderAPrevious.stationaryButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(canFlag.stationary == FALSE)
      {
        canFlag.stationary = TRUE;
      }
      else
      {
        canFlag.stationary = FALSE;
      }
    }
      
    if((canRxFenderA.hitchModeButton == CAN_SP_BUTTON_ON) && (canRxFenderAPrevious.hitchModeButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(flagHitch.settingMode == VAB_HITCH_SETTING_MODE_OFF)
      {
        if(canFlag.hitchControlMode == VAB_HITCH_POSITION_MODE)
        {
          canFlag.hitchControlMode = VAB_HITCH_DRAFT_MODE;                        // Draft mode
        }
        else if(canFlag.hitchControlMode == VAB_HITCH_DRAFT_MODE)
        {
          canFlag.hitchControlMode = VAB_HITCH_DEPTH_MODE;                        // Depth mode
        }
        else
        {
          canFlag.hitchControlMode = VAB_HITCH_POSITION_MODE;                     // Position mode
        }
      }
      else
      {
        if(flagHitch.settingMode == VAB_HITCH_SETTING_POSITION_MODE)
        {
          flagHitch.settingMode = VAB_HITCH_SETTING_DRAFT_MODE;
        }
        else if(flagHitch.settingMode == VAB_HITCH_SETTING_DRAFT_MODE)
        {
          flagHitch.settingMode = VAB_HITCH_SETTING_DEPTH_MODE;
        }
        else
        {
          flagHitch.settingMode = VAB_HITCH_SETTING_POSITION_MODE;
          flagHitch.positionSettingSequence = 0;
        }
      }
    }
        
    if((canRxFenderA.autoStopButton == CAN_SP_BUTTON_ON) && (canRxFenderAPrevious.autoStopButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(canFlag.autoStop == FALSE)
      {
        canFlag.autoStop = TRUE;
      }
      else
      {
        canFlag.autoStop = FALSE;
      }
    }
  }
  
  if(canRxFenderBTemp.flag == TRUE)
  {
    canRxFenderBTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxFenderBPrevious.data[i] = canRxFenderB.data[i];
      canRxFenderB.data[i] = canRxFenderBTemp.data[i];
    }
    
    // Fender B Button check
    if((canRxFenderB.powerAssistButton == CAN_SP_BUTTON_ON) && (canRxFenderBPrevious.powerAssistButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(canFlag.powerAssist == FALSE)
      {
        canFlag.powerAssist = TRUE;
      }
      else
      {
        canFlag.powerAssist = FALSE;  
      }
    }
    
    if((canRxFenderB.balanceModeButton == CAN_SP_BUTTON_ON) && (canRxFenderBPrevious.balanceModeButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(canFlag.balanceControlMode == VAB_BALANCE_MANUAL_MODE)
      {
        canFlag.balanceControlMode = VAB_BALANCE_FLAT_MODE;                     // Balance Flat mode
      }
      else if(canFlag.balanceControlMode == VAB_BALANCE_FLAT_MODE)
      {
        canFlag.balanceControlMode = VAB_BALANCE_SLOPE_MODE;                    // Balance Slope mode
      }
      else
      {
        canFlag.balanceControlMode = VAB_BALANCE_MANUAL_MODE;                   // Balance Manual mode
      }
    }
        
    if((canRxFenderB.balanceSensitiveButton == CAN_SP_BUTTON_ON) && (canRxFenderBPrevious.balanceSensitiveButton == CAN_SP_BUTTON_OFF))
    {
      flag.buzzerButton = TRUE;
      if(canFlag.balanceSensitivityMode == VAB_BALANCE_FAST_MODE)
      {
        canFlag.balanceSensitivityMode = VAB_BALANCE_NORMAL_MODE;               // Balance Normal mode
      }
      else if(canFlag.balanceSensitivityMode == VAB_BALANCE_NORMAL_MODE)
      {
        canFlag.balanceSensitivityMode = VAB_BALANCE_SLOW_MODE;                 // Balance Slow mode
      }
      else
      {
        canFlag.balanceSensitivityMode = VAB_BALANCE_FAST_MODE;                 // Balance Fast mode
      }
    }  
  }
  
  if(canRxDialTemp.flag == TRUE)
  {
    canRxDialTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxDialPrevious.data[i] = canRxDial.data[i];
      canRxDial.data[i] = canRxDialTemp.data[i];
    }
  }
  
  if(canRxAngleTemp.flag == TRUE)
  {
    canRxAngleTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxAnglePrevious.data[i] = canRxAngle.data[i];
      canRxAngle.data[i] = canRxAngleTemp.data[i];
    }
  }
  
  if(canRxGyroTemp.flag == TRUE)
  {
    canRxGyroTemp.flag = FALSE;
    for(i = 0; i < 8; i++)
    {
      canRxGyroPrevious.data[i] = canRxGyro.data[i];
      canRxGyro.data[i] = canRxGyroTemp.data[i];
    }
  }
  
  // save process
  if(canFlag.saveLastData == TRUE)
  {
    timerNewNvPanelData += 2;
    
    if(timerNewNvPanelData >= 3000)
    {
      timerNewNvPanelData = 0;
      canFlag.saveLastData = FALSE;
      
      newNvPanelData = 0;
      
      if(canFlag.turnUpHitch == TRUE)                     newNvPanelData |= BIT_NV_TURN_UP_MODE;
      if(canFlag.backUpHitch == TRUE)                     newNvPanelData |= BIT_NV_BACK_UP_MODE;
      if(canFlag.turnDownRpm == TRUE)                     newNvPanelData |= BIT_NV_TURN_DOWN_MODE;
      if(canFlag.backDownRpm == TRUE)                     newNvPanelData |= BIT_NV_BACK_DOWN_MODE;
      
      if(canFlag.fourWD == VAC_4WD_MANUAL_MODE)           newNvPanelData |= BIT_NV_4WD_MANUAL_MODE;
      else if(canFlag.fourWD == VAC_4WD_AUTO_MODE)        newNvPanelData |= BIT_NV_4WD_AUTO_MODE;
      
      save_nvPanelData(TRUE, newNvPanelData);
    }
  }
}

void can_data_handler_2(uint32_t ID, uint8_t *rxData)
{
  uint8_t i;
  
  switch(ID)
  {
    case CAN2_RX_ARMREST:
      canRxArmRestTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxArmRestTemp.data[i] = rxData[i];
      break;
      
    case CAN2_RX_DASH:
      canRxDashTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxDashTemp.data[i] = rxData[i];
      break;
      
    case CAN2_RX_PILLAR:
      canRxPillarTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxPillarTemp.data[i] = rxData[i];
      break;
    
    case CAN2_RX_FENDER_A:
      canRxFenderATemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxFenderATemp.data[i] = rxData[i];
      break;
          
    case CAN2_RX_FENDER_B:
      canRxFenderBTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxFenderBTemp.data[i] = rxData[i];
      break;
      
    case CAN2_RX_DIAL:
      canRxDialTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxDialTemp.data[i] = rxData[i];
      break;
          
    case CAN2_RX_ANGLE:
      canRxAngleTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxAngleTemp.data[i] = rxData[i];
      break;
          
    case CAN2_RX_GYRO:
      canRxGyroTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxGyroTemp.data[i] = rxData[i];
      break;
      
    default:
      break;
  }
}

void can_data_handler(uint32_t ID, uint8_t *rxData)
{
  uint8_t i;
  
  switch(ID)
  {
    case CAN1_RX_ECU_J1939_EEC2:
      canRxEEC2Temp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxEEC2Temp.data[i] = rxData[i];
      break;
      
    case CAN1_RX_ECU_J1939_EEC1:
      canRxEEC1Temp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxEEC1Temp.data[i] = rxData[i];
      break;
      
    case CAN1_RX_ACU:
      canRxACUTemp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxACUTemp.data[i] = rxData[i];
      break;
      
    case CAN1_RX_TBOX1:
      canRxTBOX1Temp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxTBOX1Temp.data[i] = rxData[i];
      break;
      
    case CAN1_RX_TBOX2:
      canRxTBOX2Temp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxTBOX2Temp.data[i] = rxData[i];
      break;
      
    case CAN1_RX_CPG1:
      canRxCPG1Temp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxCPG1Temp.data[i] = rxData[i];
      break;

    case CAN1_RX_CPG2:
      canRxCPG2Temp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxCPG2Temp.data[i] = rxData[i];
      break;

    case CAN1_RX_CPG3:
      canRxCPG3Temp.flag = TRUE;
      for(i = 0; i < 8; i++)
        canRxCPG3Temp.data[i] = rxData[i];
      break;
      
    default:
      break;
  }
}

// CAN1 transmissions
uint8_t can_transmit_tsc1(uint8_t *data)
{
  rpm_can_packet(data);
  return TRUE;
}
uint8_t can_transmit_ip1(uint8_t *data)
{
  static uint16_t timerFlashPosition = 0;
  static uint16_t timerFlashHitchUpDown = 0;
  
  uint8_t i;
  
  canTxIP1.pto = flagOutputLamp.pto;
  canTxIP1.fourWD = flagOutputControl.fourWD;
  canTxIP1.forward = flagShuttle.forward;
    
  canTxIP1.quickTurn = OFF;
  if(canTxIP1.quickTurnFlashing == OFF)
  {
    if(canFlag.quickTurn == TRUE) {
      canTxIP1.quickTurn = ON;
    }
  }
  
  canTxIP1.turnUp = canFlag.turnUpHitch;
  canTxIP1.backUp = canFlag.backUpHitch;
  canTxIP1.startCheck = 0;//flagOutputControl.starterOut;
  canTxIP1.res1 = 0;
  
  canTxIP1.reverse = flagShuttle.backward;
  
  if(flagHitch.leverThreePSettingMode >= 2) {
    timerFlashPosition += 50;
    if(timerFlashPosition > 300)
    {
      canTxIP1.hitchUp = ON;
      canTxIP1.hitchDown = ON;
    }
    else if(timerFlashPosition > 500)
    {
      timerFlashPosition = 0;
    }
  }
  else {
    if(flagOutputControl.hitchUp == ON) {
      canTxIP1.hitchUp = ON;
    }else{
      canTxIP1.hitchUp = OFF;
    }
    if(flagOutputControl.hitchDown == ON) {
      canTxIP1.hitchDown = ON;
    }else {
      canTxIP1.hitchDown = OFF;
    }
  }
  
  canTxIP1.res2 = 0;
  
  if(canFlag.hitchControlMode == VAB_HITCH_DRAFT_MODE)
  {
    canTxIP1.draft = ON;
    canTxIP1.depth = OFF;
  }
  else if(canFlag.hitchControlMode == VAB_HITCH_DEPTH_MODE)
  {
    canTxIP1.draft = OFF;
    canTxIP1.depth = ON;
  }
  else
  {
    canTxIP1.draft = OFF;
    canTxIP1.depth = OFF;
  } 
  
  canTxIP1.res3 = 0;
  canTxIP1.res4 = 0;
  canTxIP1.speedCheck = 0;
  
  if((flag.rpmCruiseA == TRUE) || (flag.rpmCruiseB == TRUE))
  {
    if((canTxIP1.hitchUp == OFF) && (canTxIP1.hitchDown == OFF))
    {
      canTxIP1.rpmCruise = ON;
    }
  }
  else
  {
    canTxIP1.rpmCruise = OFF;
  }
  
  if(flagErrorSensors.threePPosition == TRUE)
  {
    canTxIP1.sensorCheck = TRUE;
  }
  else
  {
    canTxIP1.sensorCheck = FALSE;
  }
  
  canTxIP1.hitchUpFlashing = OFF;
  canTxIP1.hitchDownFlashing = OFF;
  /*
  if(flagHitch.settingMode != VAB_HITCH_SETTING_MODE_OFF)
  {
    canTxIP1.draft = OFF;
    canTxIP1.depth = OFF;

    if(flagOutputControl.hitchUp == ON){
      canTxIP1.hitchUpFlashing = ON;
      canTxIP1.hitchDownFlashing = OFF;
    }
    else if(flagOutputControl.hitchDown == ON){
      canTxIP1.hitchUpFlashing = OFF;
      canTxIP1.hitchDownFlashing = ON;
    }
    else if(flag.liftLock == TRUE) {     
      canTxIP1.hitchUpFlashing = ON;
      canTxIP1.hitchDownFlashing = ON;
    }
  }
  else {
    if(flag.liftLock == TRUE) {
      canTxIP1.hitchUpFlashing = ON;
      canTxIP1.hitchDownFlashing = ON;
    }
  }
  */
  // The below part will be changed after the display program is changed
  if(flagHitch.settingMode != VAB_HITCH_SETTING_MODE_OFF)
  {
    canTxIP1.draft = OFF;
    canTxIP1.depth = OFF;

    if(flagOutputControl.hitchUp == ON){
      canTxIP1.hitchUp = ON;
      canTxIP1.hitchDown = OFF;
    }
    else if(flagOutputControl.hitchDown == ON){
      canTxIP1.hitchUp = OFF;
      canTxIP1.hitchDown = ON;
    }
    else if(flag.liftLock == TRUE) {     
      timerFlashHitchUpDown += 50;
      if(timerFlashHitchUpDown < 1000)
      {
        canTxIP1.hitchUp = ON;
        canTxIP1.hitchDown = OFF;
      }
      else if(timerFlashHitchUpDown < 2000)
      {
        canTxIP1.hitchUp = OFF;
        canTxIP1.hitchDown = ON;
      }
      else
      {
        timerFlashHitchUpDown = 0;
      }
    }
  }
  else {
    if(flag.liftLock == TRUE) {
      
      if((canTxIP1.hitchUp == OFF) && (canTxIP1.hitchDown == OFF))
      {
        if(canTxIP1.rpmCruise == ON)
        {
          timerFlashHitchUpDown += 50;
          if(timerFlashHitchUpDown < 1000)
          {
            canTxIP1.hitchUp = ON;
            canTxIP1.hitchDown = OFF;
          }
          else if(timerFlashHitchUpDown < 2000)
          {
            canTxIP1.hitchUp = OFF;
            canTxIP1.hitchDown = ON;
          }
          else if(timerFlashHitchUpDown < 3000)
          {
            canTxIP1.hitchUp = OFF;
            canTxIP1.hitchDown = OFF;
          }
          else
          {
            timerFlashHitchUpDown = 0;
          }
        }
        else
        {
          timerFlashHitchUpDown += 50;
          if(timerFlashHitchUpDown < 1000)
          {
            canTxIP1.hitchUp = ON;
            canTxIP1.hitchDown = OFF;
          }
          else if(timerFlashHitchUpDown < 2000)
          {
            canTxIP1.hitchUp = OFF;
            canTxIP1.hitchDown = ON;
          }
          else
          {
            timerFlashHitchUpDown = 0;
          }
        }
      }
    }
  }
  
  canTxIP1.res5 = 0;
  
  canTxIP1.depthFlashing = OFF;
  canTxIP1.res6 = 0;
  canTxIP1.speedUnit = nvSpeedUnit;
  canTxIP1.dtc = 0;
    
  if(flagInputStatus.transmissionL == ON) {
    canTxIP1.subTransmission = 0;
  }
  else if(flagInputStatus.transmissionM == ON) {
    canTxIP1.subTransmission = 1;
  }
  else if(flagInputStatus.transmissionH == ON) {
    canTxIP1.subTransmission = 2;
  }
  else {
    canTxIP1.subTransmission = 3;
  }
  
  if(flagInputStatus.transmissionOne == ON) {
    canTxIP1.mainTransmission = 1;
  }
  else if(flagInputStatus.transmissionTwo == ON) {
    canTxIP1.mainTransmission = 2;
  }
  else if(flagInputStatus.transmissionThree == ON) {
    canTxIP1.mainTransmission = 3;
  }
  else if(flagInputStatus.transmissionFour == ON) {
    canTxIP1.mainTransmission = 4;
  }
  else {
    canTxIP1.mainTransmission = 0;
  }
  
  canTxIP1.res7 = 0;
  canTxIP1.yususu = canFlag.screenTransaction;
    
  canTxIP1.temperature = get_oil_temperature();                                 // 0 means -25
  canTxIP1.res9 = 0;
  canTxIP1.res10 = 0;
  
  for(i = 0; i < 8; i++) {
    data[i] = canTxIP1.data[i];
  }
  return TRUE;
}

uint8_t can_transmit_ip2(uint8_t *data)
{
  uint8_t i;
  
  if(flagShuttle.forward == TRUE)
  {
    canTxIP2.fnr = 0;
  }
  else if(flagShuttle.backward == TRUE)
  {
    canTxIP2.fnr = 2;
  }
  else
  {
    canTxIP2.fnr = 1;
  }
  
  canTxIP2.fourWD = flagOutputControl.fourWD;
  
  if(canFlag.hitchControlMode == VAB_HITCH_POSITION_MODE)                       // Hitch position mode
  {
    canTxIP2.hitchMode = 0;
  }
  else if(canFlag.hitchControlMode == VAB_HITCH_DRAFT_MODE)                     // Hitch draft mode
  {
    canTxIP2.hitchMode = 1;
  }
  else if(canFlag.hitchControlMode == VAB_HITCH_DEPTH_MODE)                     // Hitch depth mode
  {
    canTxIP2.hitchMode = 4;
  }
  else                                                                          // Hitch position mode
  {
    canTxIP2.hitchMode = 0;
  }
  
  canTxIP2.startCheck = 0;//flagOutputControl.starterOut;

  if(flag.rpmCruiseA == TRUE)
  {
    canTxIP2.tsc1Mode = 1;
  }
  else if(flag.rpmCruiseB == TRUE)
  {
    canTxIP2.tsc1Mode = 2;
  }
  else
  {
    canTxIP2.tsc1Mode = 0;
  }
//  canTxIP2.tsc1Mode = 0;
  
  canTxIP2.res2 = 0;
  canTxIP2.res3 = 0;
  canTxIP2.res4 = 0;
  canTxIP2.res5 = 0;
  canTxIP2.res6 = 0;
  canTxIP2.res7 = 0;
  canTxIP2.res8 = 0;
    
  for(i = 0; i < 8; i++) {
    data[i] = canTxIP2.data[i];
  }
  return TRUE;
}

uint8_t can_transmit_skc(uint8_t *data)
{
  uint8_t i;
  
  canTxSKC.temperature = get_oil_temperature() + 15;
  canTxSKC.brakeSwitch = flagInputStatus.brake;
  canTxSKC.ptoSwitch = ptoSwitch;

  canTxSKC.parkingBrakeSwitch = flagInputStatus.parkingBrake;
  canTxSKC.clutchSwitch = 0;
  canTxSKC.seatSwitch = flagInputStatus.seat;
  canTxSKC.modelSwitch = flag.isEUModel;
  canTxSKC.readingLamp = canFlag.readingLight;
  
  if(flagShuttle.forward == TRUE)
  {
    canTxSKC.fnr = 0;
  }
  else if(flagShuttle.backward == TRUE)
  {
    canTxSKC.fnr = 2;
  }
  else
  {
    canTxSKC.fnr = 1;
  }
  
  for(i = 0; i < 8; i++) {
    data[i] = canTxSKC.data[i];
  }
  return TRUE;
}

uint8_t can_transmit_acu(uint8_t *data)
{
  uint8_t i;
  
  canTxACU.res1 = 0;
  canTxACU.res2 = 0;
  canTxACU.res3 = 0;
  canTxACU.res4 = 0;
  canTxACU.res5 = 0;
  canTxACU.res6 = 0;
  canTxACU.res7 = 0;
  canTxACU.res8 = 0;
  
  for(i = 0; i < 8; i++) {
    data[i] = canTxACU.data[i];
  }
  return TRUE;
}

uint8_t can_transmit_power_assist(uint8_t *data)
{
  uint8_t i;
  
  canTxPA.engineTorqueMapSelector = 0;
  canTxPA.res1 = 0x0;
  
  if(canFlag.powerAssist == TRUE)
    canTxPA.droopSelector = 2;
  else
    canTxPA.droopSelector = 1;
  
  canTxPA.res2 = 0x0;
  
  canTxPA.res3 = 0x0;
  canTxPA.res4 = 0x0;
  canTxPA.res5 = 0x0;
  canTxPA.res6 = 0x0;
  canTxPA.res7 = 0x0;
  canTxPA.res8 = 0x0;
  
  for(i = 0; i < 8; i++) {
    data[i] = canTxPA.data[i];
  }
  return TRUE;
}

uint8_t can_transmit_eat(uint8_t *data)
{
  static uint8_t checkSumCounter = 0;
  static uint8_t checkSum = 0;
  static uint8_t checkSumResult = 0;

  uint8_t i;
  
  canTxEAT.regenerationStop = 0;
  
  if(canFlag.regen == TRUE)                // REGEN mode on
    canTxEAT.regenerationRelease = 1;
  else
    canTxEAT.regenerationRelease = 0;
  
  canTxEAT.res1 = 0xF;
  
  canTxEAT.startEOLRoutine = 0;
  canTxEAT.EOLRoutine = 0;
  
  canTxEAT.res2 = 0x0;
  canTxEAT.inhibitHeatMode = 0;
  canTxEAT.stationarySwitch = 0;
  canTxEAT.EATOverrideSwitch = 0;
  
  canTxEAT.res3 = 0xFF;
  canTxEAT.res4 = 0xFF;
  canTxEAT.res5 = 0xFF;
  
// Calculating checksum
#define EAT_ID_CHECKSUM	0x18 + 0xFF + 0x44 + 0x03
  
  checkSumCounter++;
  if(checkSumCounter >= 8) {
    checkSumCounter = 0; 	                                                // Messaage Counter 0 to 7 and then wraps ..
  }
  
  checkSum = 0;
  for(i = 0; i < 7; i++) {
    checkSum += canTxEAT.data[i];
  }
  checkSum += (checkSumCounter & 0x0F);
  checkSum += EAT_ID_CHECKSUM;
  
  checkSumResult =  (checkSum >> 6) & 0x03;
  checkSumResult =   checkSumResult + (checkSum >> 3);	                        // 2018.08.20. Shift 숫자가 잘못되어 수정 6->3으로
  checkSumResult =  (checkSumResult +  checkSum) & 0x07;
	  
  checkSumResult =  checkSumResult << 4;		                        // SHIFT 4 
  checkSumResult =  checkSumResult & 0xF0;
  checkSumResult += checkSumCounter;			                        // COUNIT값 OR 처리.. 
  
  canTxEAT.data[7] = checkSumResult;
  
  for(i = 0; i < 8; i++) {
    data[i] = canTxEAT.data[i];
  }
  return TRUE;
}

// CAN2 transmissions
uint8_t can_transmit_dash_setting()
{
  if(canTxDashSetting.flag == FALSE)
    return FALSE;
  
  canTxDashSetting.flag = FALSE;
  
  return can_transmit(CAN_CHANNEL_2, CAN2_TX_DASH_SETTING, canTxDashSetting.data);
}

uint8_t can_transmit_dash()
{
  if(canTxDash.flag == FALSE)
    return FALSE;
  
  canTxDash.flag = FALSE;
  
  if(canFlag.fourWD == VAC_4WD_MODE_OFF)                // FourWD mode off
  {
    canTxDash.fourWDAuto = CAN_SP_LED_OFF;
    canTxDash.fourWDManual = CAN_SP_LED_OFF;
  }
  else if(canFlag.fourWD == VAC_4WD_MANUAL_MODE)        // FourWD mode is manual
  {
    canTxDash.fourWDAuto = CAN_SP_LED_OFF;
    canTxDash.fourWDManual = CAN_SP_LED_ON;
  }
  else if(canFlag.fourWD == VAC_4WD_AUTO_MODE)          // FourWD mode is auto
  {
    canTxDash.fourWDAuto = CAN_SP_LED_ON;
    canTxDash.fourWDManual = CAN_SP_LED_OFF;
  }
  else                                                  // FourWD mode off
  {
    canTxDash.fourWDAuto = CAN_SP_LED_OFF;
    canTxDash.fourWDManual = CAN_SP_LED_OFF;
  }
  
  if(canFlag.quickTurn == FALSE)                // QT mode off
  {
    canTxDash.quickTurn = CAN_SP_LED_OFF;
  }
  else if(canFlag.quickTurn == TRUE)            // QT mode on
  {
    canTxDash.quickTurn = CAN_SP_LED_ON;
  }
  
  if(canFlag.regen == FALSE)                    // REGEN mode off
  {
    canTxDash.regen = CAN_SP_LED_OFF;
  }
  else if(canFlag.regen == TRUE)                // REGEN mode on
  {
    canTxDash.regen = CAN_SP_LED_ON;
  }

  if(canFlag.screenTransaction == FALSE)        // Screen transition mode off
  {
    canTxDash.screenTransition = CAN_SP_LED_OFF;
  }
  else if(canFlag.screenTransaction == TRUE)    // Screen transition mode on
  {
    canTxDash.screenTransition = CAN_SP_LED_ON;
  }
    
  return can_transmit(CAN_CHANNEL_2, CAN2_TX_DASH, canTxDash.data);
}

uint8_t can_transmit_pillar_setting()
{
  if(canTxPillarSetting.flag == FALSE)
    return FALSE;
  
  canTxPillarSetting.flag = FALSE;

  return can_transmit(CAN_CHANNEL_2, CAN2_TX_PILLAR_SETTING, canTxPillarSetting.data);
}

uint8_t can_transmit_pillar()
{
  if(canTxPillar.flag == FALSE)
    return FALSE;
  
  canTxPillar.flag = FALSE;

  if(canFlag.hitchUpPilar == TRUE)      canTxPillar.hitchUp = CAN_SP_LED_ON;
  else                                  canTxPillar.hitchUp = CAN_SP_LED_OFF;
  
  if(canFlag.hitchDownPilar == TRUE)    canTxPillar.hitchDown = CAN_SP_LED_ON;
  else                                  canTxPillar.hitchDown = CAN_SP_LED_OFF;
  
  if(canFlag.balanceUpPillar == TRUE)   canTxPillar.balanceUp = CAN_SP_LED_ON;
  else                                  canTxPillar.balanceUp = CAN_SP_LED_OFF;
  
  if(canFlag.balanceDownPillar == TRUE) canTxPillar.balanceDown = CAN_SP_LED_ON;
  else                                  canTxPillar.balanceDown = CAN_SP_LED_OFF;
  
  if(canFlag.toplinkUpPilar == TRUE)    canTxPillar.topLinkUp = CAN_SP_LED_ON;
  else                                  canTxPillar.topLinkUp = CAN_SP_LED_OFF;
  
  if(canFlag.toplinkDownPilar == TRUE)  canTxPillar.topLinkDown = CAN_SP_LED_ON;
  else                                  canTxPillar.topLinkDown = CAN_SP_LED_OFF;
  
  return can_transmit(CAN_CHANNEL_2, CAN2_TX_PILLAR, canTxPillar.data);
}

uint8_t can_transmit_fenderA_setting()
{
  if(canTxFenderASetting.flag == FALSE)
    return FALSE;
  
  canTxFenderASetting.flag = FALSE;
  
  return can_transmit(CAN_CHANNEL_2, CAN2_TX_FENDER_A_SETTING, canTxFenderASetting.data);
}

uint8_t can_transmit_fenderA()
{
  if(canTxFenderA.flag == FALSE)
    return FALSE;
  
  canTxFenderA.flag = FALSE;

  if(canFlag.turnUpHitch == TRUE)       canTxFenderA.turnUpHitch = CAN_SP_LED_ON;
  else                                  canTxFenderA.turnUpHitch = CAN_SP_LED_OFF;
  
  if(canFlag.backUpHitch == TRUE)       canTxFenderA.backUpHitch = CAN_SP_LED_ON;
  else                                  canTxFenderA.backUpHitch = CAN_SP_LED_OFF;
  
  if(canFlag.turnDownRpm == TRUE)       canTxFenderA.turnDownRpm = CAN_SP_LED_ON;
  else                                  canTxFenderA.turnDownRpm = CAN_SP_LED_OFF;
  
  if(canFlag.backDownRpm == TRUE)       canTxFenderA.backDownRpm = CAN_SP_LED_ON;
  else                                  canTxFenderA.backDownRpm = CAN_SP_LED_OFF;
  
  if(canFlag.readingLight == TRUE)      canTxFenderA.reading = CAN_SP_LED_ON;
  else                                  canTxFenderA.reading = CAN_SP_LED_OFF;
  
  if(canFlag.stationary == TRUE)        canTxFenderA.stationary = CAN_SP_LED_ON;
  else                                  canTxFenderA.stationary = CAN_SP_LED_OFF;
  
  if(flagHitch.settingMode != VAB_HITCH_SETTING_MODE_OFF)
  {
    if(flagHitch.settingMode == VAB_HITCH_SETTING_POSITION_MODE)                // Hitch setting position mode
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_BLINK;
      canTxFenderA.hitchDraft = CAN_SP_LED_OFF;
      canTxFenderA.hitchDepth = CAN_SP_LED_OFF;
    }
    else if(flagHitch.settingMode == VAB_HITCH_SETTING_DRAFT_MODE)              // Hitch setting draft mode
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_OFF;
      canTxFenderA.hitchDraft = CAN_SP_LED_BLINK;
      canTxFenderA.hitchDepth = CAN_SP_LED_OFF;
    }
    else if(flagHitch.settingMode == VAB_HITCH_SETTING_DEPTH_MODE)              // Hitch setting depth mode
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_OFF;
      canTxFenderA.hitchDraft = CAN_SP_LED_OFF;
      canTxFenderA.hitchDepth = CAN_SP_LED_BLINK;
    }
    else                                                                        // Hitch setting mode off
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_OFF;
      canTxFenderA.hitchDraft = CAN_SP_LED_OFF;
      canTxFenderA.hitchDepth = CAN_SP_LED_OFF;
    }
  }
  else
  {
    // Normal mode
    if(canFlag.hitchControlMode == VAB_HITCH_POSITION_MODE)                       // Hitch position mode
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_ON;
      canTxFenderA.hitchDraft = CAN_SP_LED_OFF;
      canTxFenderA.hitchDepth = CAN_SP_LED_OFF;
    }
    else if(canFlag.hitchControlMode == VAB_HITCH_DRAFT_MODE)                     // Hitch draft mode
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_OFF;
      canTxFenderA.hitchDraft = CAN_SP_LED_ON;
      canTxFenderA.hitchDepth = CAN_SP_LED_OFF;
    }
    else if(canFlag.hitchControlMode == VAB_HITCH_DEPTH_MODE)                     // Hitch depth mode
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_OFF;
      canTxFenderA.hitchDraft = CAN_SP_LED_OFF;
      canTxFenderA.hitchDepth = CAN_SP_LED_ON;
    }
    else                                                                          // Hitch position mode
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_ON;
      canTxFenderA.hitchDraft = CAN_SP_LED_OFF;
      canTxFenderA.hitchDepth = CAN_SP_LED_OFF;
    }

    if(flagErrorSensors.threePPosition == ON)
    {
      canTxFenderA.hitchPosition = CAN_SP_LED_BLINK;
    }
    
    if(flagErrorSensors.threePDraft == ON)
    {
      canTxFenderA.hitchDraft = CAN_SP_LED_BLINK;
    }
    
    if(flagErrorSensors.threePDepth == ON)
    {
      canTxFenderA.hitchDepth = CAN_SP_LED_BLINK;
    }
  }
  
  
  
  if(canFlag.autoStop == TRUE)          canTxFenderA.autoStop = CAN_SP_LED_ON;
  else                                  canTxFenderA.autoStop = CAN_SP_LED_OFF;
  
  return can_transmit(CAN_CHANNEL_2, CAN2_TX_FENDER_A, canTxFenderA.data);
}

uint8_t can_transmit_fenderB_setting()
{

  if(canTxFenderBSetting.flag == FALSE)
    return FALSE;
  
  canTxFenderBSetting.flag = FALSE;

  return can_transmit(CAN_CHANNEL_2, CAN2_TX_FENDER_B_SETTING, canTxFenderBSetting.data);
}

uint8_t can_transmit_fenderB()
{
  if(canTxFenderB.flag == FALSE)
    return FALSE;
  
  canTxFenderB.flag = FALSE;
  
  if(canFlag.powerAssist == TRUE)       canTxFenderB.powerAssist = CAN_SP_LED_ON;
  else                                  canTxFenderB.powerAssist = CAN_SP_LED_OFF;
    
  if(flagBalance.settingMode == FALSE)
  {
    if(canFlag.balanceControlMode == VAB_BALANCE_MANUAL_MODE)                   // Balance manual mode
    {
      canTxFenderB.balanceManual = CAN_SP_LED_ON;
      canTxFenderB.balanceFlat = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlope = CAN_SP_LED_OFF;
    }
    else if(canFlag.balanceControlMode == VAB_BALANCE_FLAT_MODE)                // Balance flat mode
    {
      canTxFenderB.balanceManual = CAN_SP_LED_OFF;
      canTxFenderB.balanceFlat = CAN_SP_LED_ON;
      canTxFenderB.balanceSlope = CAN_SP_LED_OFF;
    }
    else if(canFlag.balanceControlMode == VAB_BALANCE_SLOPE_MODE)               // Balance slope mode
    {
      canTxFenderB.balanceManual = CAN_SP_LED_OFF;
      canTxFenderB.balanceFlat = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlope = CAN_SP_LED_ON;
    }
    else                                                                        // Balance manual mode
    {
      canTxFenderB.balanceManual = CAN_SP_LED_ON;
      canTxFenderB.balanceFlat = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlope = CAN_SP_LED_OFF;
    }
    
    if(canFlag.balanceSensitivityMode == VAB_BALANCE_FAST_MODE)                 // Balance fast mode
    {
      canTxFenderB.balanceFast = CAN_SP_LED_ON;
      canTxFenderB.balanceNormal = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlow = CAN_SP_LED_OFF;
    }
    else if(canFlag.balanceSensitivityMode == VAB_BALANCE_NORMAL_MODE)          // Balance normal mode
    {
      canTxFenderB.balanceFast = CAN_SP_LED_OFF;
      canTxFenderB.balanceNormal = CAN_SP_LED_ON;
      canTxFenderB.balanceSlow = CAN_SP_LED_OFF;
    }
    else if(canFlag.balanceSensitivityMode == VAB_BALANCE_SLOW_MODE)            // Balance slow mode
    {
      canTxFenderB.balanceFast = CAN_SP_LED_OFF;
      canTxFenderB.balanceNormal = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlow = CAN_SP_LED_ON;
    }
    else                                                                        // Balance normal mode
    {
      canTxFenderB.balanceFast = CAN_SP_LED_OFF;
      canTxFenderB.balanceNormal = CAN_SP_LED_ON;
      canTxFenderB.balanceSlow = CAN_SP_LED_OFF;
    }
  }
  else
  {
    if(flagBalance.settingStroke == TRUE)
    {
      canTxFenderB.balanceManual = CAN_SP_LED_BLINK;
      canTxFenderB.balanceFlat = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlope = CAN_SP_LED_OFF;
    }
    else if(flagBalance.settingRoll == TRUE)
    {
      canTxFenderB.balanceManual = CAN_SP_LED_OFF;
      canTxFenderB.balanceFlat = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlope = CAN_SP_LED_BLINK;
    }
    
    if(flagBalance.settingSequence == 0)
    {
      canTxFenderB.balanceFast = CAN_SP_LED_OFF;
      canTxFenderB.balanceNormal = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlow = CAN_SP_LED_OFF;
    }
    else if(flagBalance.settingSequence == 1)
    {
      canTxFenderB.balanceFast = CAN_SP_LED_ON;
      canTxFenderB.balanceNormal = CAN_SP_LED_ON;
      canTxFenderB.balanceSlow = CAN_SP_LED_ON;
    }
    else if(flagBalance.settingSequence == 2)
    {
      canTxFenderB.balanceFast = CAN_SP_LED_BLINK;
      canTxFenderB.balanceNormal = CAN_SP_LED_OFF;
      canTxFenderB.balanceSlow = CAN_SP_LED_BLINK;
    }
    else if(flagBalance.settingSequence == 3)
    {
      canTxFenderB.balanceFast = CAN_SP_LED_BLINK;
      canTxFenderB.balanceNormal = CAN_SP_LED_ON;
      canTxFenderB.balanceSlow = CAN_SP_LED_BLINK;
      
      flagBalance.settingMode = FALSE;
      canFlag.balanceControlMode = VAB_BALANCE_MANUAL_MODE;
      canFlag.balanceSensitivityMode = VAB_BALANCE_NORMAL_MODE;
    }
  }
  
  return can_transmit(CAN_CHANNEL_2, CAN2_TX_FENDER_B, canTxFenderB.data);
}

void can_transmit_process_2()
{
  if(flagTimer.hundredMs == TRUE)
  {
//    canTxDashSetting.flag = TRUE;
    canTxDash.flag = TRUE;
//    canTxPillarSetting.flag = TRUE;
    canTxPillar.flag = TRUE;
//    canTxFenderASetting.flag = TRUE;
    canTxFenderA.flag = TRUE;
//    canTxFenderBSetting.flag = TRUE;
    canTxFenderB.flag = TRUE;
  }
  
  if(can_transmit_dash_setting() == TRUE)
    return;
    
  if(can_transmit_dash() == TRUE)
    return;
      
  if(can_transmit_pillar_setting() == TRUE)
    return;

  if(can_transmit_pillar() == TRUE)
    return;

  if(can_transmit_fenderA_setting() == TRUE)
    return;
  
  if(can_transmit_fenderA() == TRUE)
    return;
  
  if(can_transmit_fenderB_setting() == TRUE)
    return;
  
  if(can_transmit_fenderB() == TRUE)
    return;
}

uint8_t can_transmit(uint8_t channel, uint32_t id, uint8_t _data[])
{
  if(channel == CAN_CHANNEL_1)
  {
    canOneTxHeader.IDE = CAN_ID_EXT;
    canOneTxHeader.RTR = CAN_RTR_DATA;
    canOneTxHeader.ExtId = id;
    canOneTxHeader.DLC = 8;
    
    canOneTxData[0] = _data[0];
    canOneTxData[1] = _data[1];
    canOneTxData[2] = _data[2];
    canOneTxData[3] = _data[3];
    canOneTxData[4] = _data[4];
    canOneTxData[5] = _data[5];
    canOneTxData[6] = _data[6];
    canOneTxData[7] = _data[7];
    
    canOneTxHeader.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &canOneTxHeader, canOneTxData, &canOneTxMailbox) != HAL_OK)
    {
      canErrorCounter[0]++;
      if(canErrorCounter[0] >= 20) {
        HAL_CAN_DeInit(&hcan1);
        HAL_CAN_DeInit(&hcan2);
        can_init(CAN_CHANNEL_1);
        can_init(CAN_CHANNEL_2);
        canErrorCounter[0] = 0;
        canErrorCounter[1] = 0;
        
//        printf("CAN 1, 2 error is occurred \r\n");
      }
      return FALSE;
    }
    else {
      if(canErrorCounter[0] > 0) {
        canErrorCounter[0]--;  
      }
    }
  }
  else if(channel == CAN_CHANNEL_2)
  {
    canTwoTxHeader.IDE = CAN_ID_EXT;
    canTwoTxHeader.RTR = CAN_RTR_DATA;
    canTwoTxHeader.ExtId = id;
    canTwoTxHeader.DLC = 8;
    
    canTwoTxData[0] = _data[0];
    canTwoTxData[1] = _data[1];
    canTwoTxData[2] = _data[2];
    canTwoTxData[3] = _data[3];
    canTwoTxData[4] = _data[4];
    canTwoTxData[5] = _data[5];
    canTwoTxData[6] = _data[6];
    canTwoTxData[7] = _data[7];
    
    canTwoTxHeader.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan2, &canTwoTxHeader, canTwoTxData, &canTwoTxMailbox) != HAL_OK)
    {
      canErrorCounter[1]++;
      if(canErrorCounter[1] >= 20) {
        HAL_CAN_DeInit(&hcan2);
        can_init(CAN_CHANNEL_2);
        canErrorCounter[1] = 0;
        
//        printf("CAN 2 error is occurred \r\n");
      }
      return FALSE;
    }
    else {
      if(canErrorCounter[1] > 0) {
        canErrorCounter[1]--;  
      }
    }
  }
  return TRUE;
  // If no channel selected then no data is transmitted to CAN 
}

/* CAN Interrupt function */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance == CAN2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canTwoRxHeader, canTwoRxData) == HAL_OK)
    {
      can_data_handler_2(canTwoRxHeader.ExtId, canTwoRxData);
    }
    canTwoRxHeader.DLC = 0;
  }
}

void can_reinit()
{
  HAL_CAN_DeInit(&hcan1);
  HAL_CAN_DeInit(&hcan2);
  can_init(CAN_CHANNEL_1);
  can_init(CAN_CHANNEL_2);
  canErrorCounter[0] = 0;
  canErrorCounter[1] = 0;
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void can_init(uint8_t channel)
{
  if(channel == CAN_CHANNEL_1)
  {
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 9;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;                                       // Changed on 2022.03.02
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
      Error_Handler();
    }
    // Filter CAN ID is updated on 2021.12.28 for mass product
    can_filter_Init(0, CAN1_UDS_PHYSICAL_REQUEST, CAN1_UDS_PHYSICAL_REQUEST, 0);
    can_filter_Init(1, CAN1_UDS_FUNCTIONAL_REQUEST, CAN1_UDS_FUNCTIONAL_REQUEST, 0);
    can_filter_Init(2, CAN1_RX_ECU_J1939_EEC2, CAN1_RX_ECU_J1939_EEC2, 0);  
    can_filter_Init(3, CAN1_RX_ECU_J1939_EEC1, CAN1_RX_ECU_J1939_EEC1, 0);
    can_filter_Init(4, CAN1_RX_TBOX1, CAN1_RX_TBOX1, 0);
    can_filter_Init(5, CAN1_RX_TBOX2, CAN1_RX_TBOX2, 0);
    can_filter_Init(6, CAN1_RX_ACU, CAN1_RX_ACU, 0);
    can_filter_Init(7, CAN1_RX_CPG1, CAN1_RX_CPG1, 0);
    can_filter_Init(8, CAN1_RX_CPG2, CAN1_RX_CPG2, 0);
    can_filter_Init(9, CAN1_RX_CPG3, CAN1_RX_CPG3, 0);
    
    if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
      Error_Handler();
    }

    if(HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      Error_Handler();
    }
    can_interrupt_enable_one();
    
    canErrorCounter[0] = 0;
  }
  else if(channel == CAN_CHANNEL_2)
  {
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 9;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan2.Init.TimeTriggeredMode = DISABLE;
    hcan2.Init.AutoBusOff = DISABLE;
    hcan2.Init.AutoWakeUp = DISABLE;
    hcan2.Init.AutoRetransmission = ENABLE;                                       // Changed on 2022.03.02
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
      Error_Handler();
    }
    can_filter_Init(14, CAN2_RX_ARMREST, CAN2_RX_ARMREST, 0);
    can_filter_Init(15, CAN2_RX_DASH, CAN2_RX_DASH, 0);
    can_filter_Init(16, CAN2_RX_PILLAR, CAN2_RX_PILLAR, 0);
    can_filter_Init(17, CAN2_RX_FENDER_A, CAN2_RX_FENDER_A, 0);
    can_filter_Init(18, CAN2_RX_FENDER_B, CAN2_RX_FENDER_B, 0);
    can_filter_Init(19, CAN2_RX_DIAL, CAN2_RX_DIAL, 0);
    can_filter_Init(20, CAN2_RX_ANGLE, CAN2_RX_ANGLE, 0);
    can_filter_Init(21, CAN2_RX_GYRO, CAN2_RX_GYRO, 0);
    
    if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
      Error_Handler();
    }

    if(HAL_CAN_Start(&hcan2) != HAL_OK)
    {
      Error_Handler();
    }
    can_interrupt_enable_two();
    
    canErrorCounter[1] = 0;
  }
}

/* CAN FILTER init function*/
void can_filter_Init(uint8_t filter, uint32_t id, uint32_t mask, uint8_t std)
{
  CAN_FilterTypeDef  sFilterConfig;

  if(filter > 27){
    filter = 27;
  }
  
  sFilterConfig.FilterBank = filter;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
    
//  id = (id << 3) | (1 << 2);                                    // IDE should be 1        
//  mask = (mask << 3) | (1 << 2);                                // IDE should be 1
  
  mask = 0;
  if(std == 1) {
    id = (id << 3);
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterIdHigh = id;
  }
  else {
    id = (id << 3) | (1 << 2);
    sFilterConfig.FilterIdLow = id;
    sFilterConfig.FilterIdHigh = id >> 16;
  }
  sFilterConfig.FilterMaskIdLow = id & mask;
  sFilterConfig.FilterMaskIdHigh = (id & mask) >> 16;

  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}


static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

/**
* @brief CAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_3();

    /* CAN1 interrupt Init */
    
  /* USER CODE BEGIN CAN1_MspInit 1 */
  
  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN2_ENABLE();

    /* CAN2 interrupt Init */
    
  /* USER CODE BEGIN CAN2_MspInit 1 */
  
  /* USER CODE END CAN2_MspInit 1 */
  }
}

void can_interrupt_enable_one()
{
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void can_interrupt_enable_two()
{
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
}

/**
* @brief CAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* CAN2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }

}

/*
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}
*/

void CAN2_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}