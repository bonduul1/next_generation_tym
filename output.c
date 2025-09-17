#include "output.h"
#include "main.h"
#include "tle92464.h"
#include "bts7006.h"
#include "vn5025.h"
#include "settings.h"

#include "shuttle.h"
#include "can.h"
#include "sensors.h"
#include "pto.h"
#include "algorithm.h"
#include "input.h"
#include "timer.h"
#include "usart.h"
#include "hitch.h"
#include "balance.h"

flagOutput_t flagOutputControlPrevious;

flagOutput_t flagOutputControl;
flagOutput_t flagOutputStatus;
flagOutput_t flagOutputLamp;
flagOutput_t flagOutputValve;

uint8_t tle92464_init;
uint8_t tle92464_icFirstInit;
uint8_t tle92464_icSecondInit;

void adMotorLeftOn();
void adMotorLeftOff();

void adMotorRightOn();
void adMotorRightOff();

void reserved2On();
void reserved2Off();

void fourWDOn();
void fourWDOff();

void unloadOn();
void unloadOff();

void qtOn();
void qtOff();

void backBuzzerOn();
void backBuzzerOff();

void checkBuzzerOn();
void checkBuzzerOff();

void starterOn();
void starterOff();

void ptoOn(uint16_t duty);
void ptoOff();

void hitchDownOn(uint16_t duty);
void hitchDownOff();

void hitchUpOn(uint16_t duty);                                                 // The output is using hardware PWM
void hitchUpOff();

void forwardOn(uint16_t outCurrent);                                            // control output current
void forwardOff();                                                              // The output current is 0, so we do NOT need an argument

void backwardOn(uint16_t outCurrent);
void backwardOff();

void reserved3On(uint16_t outCurrent);
void reserved3Off();

void topLinkUpOn(uint16_t outCurrent);
void topLinkUpOff();

void topLinkDownOn(uint16_t outCurrent);
void topLinkDownOff();

void balanceUpOn(uint16_t outCurrent);
void balanceUpOff();

void balanceDownOn(uint16_t outCurrent);
void balanceDownOff();

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void gpio_output_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  
  HAL_GPIO_WritePin(FOC_RESERVED_GPIO_Port,  FOC_RESERVED_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_REGEN_GPIO_Port,  FOC_REGEN_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_BALANCE_UP_GPIO_Port,  FOC_BALANCE_UP_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_BALANCE_DOWN_GPIO_Port,  FOC_BALANCE_DOWN_Pin,  GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin(FOC_SOL4_GPIO_Port,  FOC_SOL4_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_SOL5_GPIO_Port,  FOC_SOL5_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_SOL6_GPIO_Port,  FOC_SOL6_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_SOL7_GPIO_Port,  FOC_SOL7_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_SOL8_GPIO_Port,  FOC_SOL8_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_LAMP_GPIO_Port,  FOC_LAMP_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_START_GPIO_Port, FOC_START_Pin, GPIO_PIN_SET);          // changed on 2022.02.03

  /*Configure GPIO pins : FOC_RESERVED_Pin */
  GPIO_InitStruct.Pin = FOC_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_RESERVED_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FOC_REGEN_Pin */
  GPIO_InitStruct.Pin = FOC_REGEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_REGEN_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FOC_BALANCE_UP_Pin */
  GPIO_InitStruct.Pin = FOC_BALANCE_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_BALANCE_UP_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FOC_BALANCE_DOWN_Pin */
  GPIO_InitStruct.Pin = FOC_BALANCE_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_BALANCE_DOWN_GPIO_Port, &GPIO_InitStruct);
  
  
  /*Configure GPIO pins : FOC_SOL4_Pin */
  GPIO_InitStruct.Pin = FOC_SOL4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_SOL4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FOC_SOL5_Pin */
  GPIO_InitStruct.Pin = FOC_SOL5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_SOL5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FOC_SOL6_Pin */
  GPIO_InitStruct.Pin = FOC_SOL6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_SOL6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FOC_SOL7_Pin */
  GPIO_InitStruct.Pin = FOC_SOL7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_SOL7_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FOC_SOL8_Pin */
  GPIO_InitStruct.Pin = FOC_SOL8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_SOL8_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FOC_LAMP_Pin */
  GPIO_InitStruct.Pin = FOC_LAMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_LAMP_GPIO_Port, &GPIO_InitStruct); 
  
  /*Configure GPIO pins : FOC_START_Pin */
  GPIO_InitStruct.Pin = FOC_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_START_GPIO_Port, &GPIO_InitStruct); 
  
  flagOutputControlPrevious.data = 0;                                           // Added on 2021.12.17
  flagOutputControl.data = 0;                                                   // Added on 2021.12.17

  vn5025_init();
  bts7006_init();
  tle9246x_init(TLE9246x_ALL_IC);

  starterOff();
  fourWDOff();
  unloadOff();
  qtOff();
  backBuzzerOff();
  checkBuzzerOff();
  
  tle92464_init = 0;
  tle92464_icFirstInit = 0;
  tle92464_icSecondInit = 0;
}

uint8_t output_init()
{
  uint8_t ret = FALSE;
  uint16_t data[3] = { nvDitherFrequency, nvDitherCurrent, nvPwmFrequency };      
  
  if(tle92464_icFirstInit == 0){
    tle9246x_disable_output();
    tle92464_icFirstInit = (tle9246x_handler(TLE9246x_FIRST_IC) == TRUE) ? 1 : 0;
  }
  else if(tle92464_icFirstInit == 1) {
    // Now we should configure dither frequency
    tle92464_icFirstInit = (tle9246x_set_settings(TLE9246x_FIRST_IC, data) == TRUE) ? 2 : 1;
  }
  else if(tle92464_icFirstInit == 2) {
    tle9246x_set_current(TLE9246x_CHANNEL_FORWARD, 0);
    tle9246x_set_current(TLE9246x_CHANNEL_BACKWARD, 0);
    tle9246x_set_current(TLE9246x_CHANNEL_REGEN, 0);
    tle9246x_set_current(TLE9246x_CHANNEL_RESERVED3, 0);
    
    tle92464_icFirstInit = 3;
  }
  
  if(tle92464_icFirstInit == 3) {
    if(tle92464_icSecondInit == 0){
      tle92464_icSecondInit = (tle9246x_handler(TLE9246x_SECOND_IC) == TRUE) ? 1 : 0;
    }
    else if(tle92464_icSecondInit == 1) {
      // Now we should configure dither frequency
      tle92464_icSecondInit = (tle9246x_set_settings(TLE9246x_SECOND_IC, data) == TRUE) ? 2 : 1;
    }
    else if(tle92464_icSecondInit == 2) {
      tle9246x_set_current(TLE9246x_CHANNEL_TOPLINK_UP, 0);
      tle9246x_set_current(TLE9246x_CHANNEL_TOPLINK_DOWN, 0);
      tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_UP, 0);
      tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_DOWN, 0);
      tle92464_icSecondInit = 3;
    }
  }
  
  if((tle92464_icFirstInit == 3) && (tle92464_icSecondInit == 3)) {
    ret = TRUE;
    tle9246x_enable_output();
    tle92464_init = 1;
  }
  
  return ret;
}

void write_output(uint8_t channel, uint16_t level)
{
  uint8_t enable = (level == 0) ? OFF : ON;
  GPIO_PinState _state = (enable == OFF) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  
  switch(channel){
    case CHANNEL_FORWARD:
      flagOutputStatus.forward = enable;
      tle9246x_set_current(TLE9246x_CHANNEL_FORWARD, level);
      break;
    case CHANNEL_BACKWARD:
      flagOutputStatus.backward = enable;
      tle9246x_set_current(TLE9246x_CHANNEL_BACKWARD, level);
      break;
    case CHANNEL_RESERVED3:
      flagOutputStatus.reserved3 = enable;
      tle9246x_set_current(TLE9246x_CHANNEL_RESERVED3, level);
      HAL_GPIO_WritePin(FOC_RESERVED_GPIO_Port,  FOC_RESERVED_Pin,  _state);
      break;
    case CHANNEL_REGEN:
      flagOutputStatus.regen = enable;
      tle9246x_set_current(TLE9246x_CHANNEL_REGEN, level);
      HAL_GPIO_WritePin(FOC_REGEN_GPIO_Port,  FOC_REGEN_Pin,  _state);
      break;

    case CHANNEL_TOPLINK_UP:
      flagOutputStatus.topLinkUp = enable;
      tle9246x_set_current(TLE9246x_CHANNEL_TOPLINK_UP, level);
      break;
    case CHANNEL_TOPLINK_DOWN:
      flagOutputStatus.topLinkDown = enable;
      tle9246x_set_current(TLE9246x_CHANNEL_TOPLINK_DOWN, level);
      break;
    case CHANNEL_BALANCE_UP:
      flagOutputStatus.balanceUp = enable;
      tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_UP, level);
      HAL_GPIO_WritePin(FOC_BALANCE_UP_GPIO_Port,  FOC_BALANCE_UP_Pin,  _state);
      break;
    case CHANNEL_BALANCE_DOWN:
      flagOutputStatus.balanceDown = enable;
      tle9246x_set_current(TLE9246x_CHANNEL_BALANCE_DOWN, level);
      HAL_GPIO_WritePin(FOC_BALANCE_DOWN_GPIO_Port,  FOC_BALANCE_DOWN_Pin,  _state);
      break;

    case CHANNEL_AD_MOTOR_RIGHT:
      flagOutputStatus.adMotorRight = enable;
      bts7006_output(BTS7006_CHANNEL_AD_MOTOR_RIGHT, level);
      break;
    case CHANNEL_AD_MOTOR_LEFT:
      flagOutputStatus.adMotorLeft = enable;
      bts7006_output(BTS7006_CHANNEL_AD_MOTOR_LEFT, level);
      break;  
    case CHANNEL_PTO:
      flagOutputStatus.pto = enable;
      bts7006_output(BTS7006_CHANNEL_PTO, level);
      break;
    
    case CHANNEL_HITCH_UP:
      flagOutputStatus.hitchUp = enable;
      vn5025_output(VN5025_CHANNEL_UP, level);
      break;  
    case CHANNEL_HITCH_DOWN:
      flagOutputStatus.hitchDown = enable;
      vn5025_output(VN5025_CHANNEL_DOWN, level);
      break;
      
    case CHANNEL_STARTER_OUT:
      HAL_GPIO_WritePin(FOC_START_GPIO_Port, FOC_START_Pin, (enable == ON) ? GPIO_PIN_RESET : GPIO_PIN_SET);            // The starter output is activated LOW voltage (TTL-0V)
      flagOutputStatus.starterOut = enable;
      break;
    case CHANNEL_CHECK_BUZZER:
      HAL_GPIO_WritePin(FOC_LAMP_GPIO_Port,  FOC_LAMP_Pin,  _state);
      flagOutputStatus.checkBuzzer = enable;
      break;
    case CHANNEL_FOURWD:
      HAL_GPIO_WritePin(FOC_SOL4_GPIO_Port,  FOC_SOL4_Pin,  _state);
      flagOutputStatus.fourWD = enable;
      break;
    case CHANNEL_UNLOAD:
      HAL_GPIO_WritePin(FOC_SOL5_GPIO_Port,  FOC_SOL5_Pin,  _state);
      flagOutputStatus.unload = enable;
      break;
    case CHANNEL_QUICK_TURN:
      HAL_GPIO_WritePin(FOC_SOL6_GPIO_Port,  FOC_SOL6_Pin,  _state);
      flagOutputStatus.qt = enable;
      break;
    case CHANNEL_BACK_BUZZER:
      HAL_GPIO_WritePin(FOC_SOL7_GPIO_Port,  FOC_SOL7_Pin,  _state);
      flagOutputStatus.backBuzzer = enable;
      break;
  
    default:
      break;
  }
}

void adMotorLeftOn()                    {       write_output(CHANNEL_AD_MOTOR_LEFT, ON);        }
void adMotorLeftOff()                   {       write_output(CHANNEL_AD_MOTOR_LEFT, OFF);       }

void adMotorRightOn()                   {       write_output(CHANNEL_AD_MOTOR_RIGHT, ON);       }
void adMotorRightOff()                  {       write_output(CHANNEL_AD_MOTOR_RIGHT, OFF);      }

void fourWDOn()                         {       write_output(CHANNEL_FOURWD, ON);               }
void fourWDOff()                        {       write_output(CHANNEL_FOURWD, OFF);              }

void unloadOn()                         {       write_output(CHANNEL_UNLOAD, ON);               }
void unloadOff()                        {       write_output(CHANNEL_UNLOAD, OFF);              }

void qtOn()                             {       write_output(CHANNEL_QUICK_TURN, ON);           }
void qtOff()                            {       write_output(CHANNEL_QUICK_TURN, OFF);          }

void backBuzzerOn()                     {       write_output(CHANNEL_BACK_BUZZER, ON);          }
void backBuzzerOff()                    {       write_output(CHANNEL_BACK_BUZZER, OFF);         }

void checkBuzzerOn()                    {       write_output(CHANNEL_CHECK_BUZZER, ON);         }
void checkBuzzerOff()                   {       write_output(CHANNEL_CHECK_BUZZER, OFF);        }

void starterOn()                        {       write_output(CHANNEL_STARTER_OUT, ON);          }
void starterOff()                       {       write_output(CHANNEL_STARTER_OUT, OFF);         }

void ptoOn(uint16_t duty)               {       write_output(CHANNEL_PTO, duty);                }
void ptoOff()                           {       write_output(CHANNEL_PTO, OFF);                 }

void hitchDownOn(uint16_t duty)         {       write_output(CHANNEL_HITCH_DOWN, duty);         }
void hitchDownOff()                     {       write_output(CHANNEL_HITCH_DOWN, OFF);          }

void hitchUpOn(uint16_t duty)           {       write_output(CHANNEL_HITCH_UP, duty);           }
void hitchUpOff()                       {       write_output(CHANNEL_HITCH_UP, OFF);            }


void forwardOn(uint16_t outCurrent)     {       write_output(CHANNEL_FORWARD, outCurrent);      }
void forwardOff()                       {       write_output(CHANNEL_FORWARD, OFF);             }
void backwardOn(uint16_t outCurrent)    {       write_output(CHANNEL_BACKWARD, outCurrent);     }
void backwardOff()                      {       write_output(CHANNEL_BACKWARD, OFF);            }
void regenOn(uint16_t outCurrent)       {       write_output(CHANNEL_REGEN, outCurrent);        }
void regenOff()                         {       write_output(CHANNEL_REGEN, OFF);               }
void reserved3On(uint16_t outCurrent)   {       write_output(CHANNEL_RESERVED3, outCurrent);    }
void reserved3Off()                     {       write_output(CHANNEL_RESERVED3, OFF);           }

void topLinkUpOn(uint16_t outCurrent)   {       write_output(CHANNEL_TOPLINK_UP, outCurrent);   }
void topLinkUpOff()                     {       write_output(CHANNEL_TOPLINK_UP, OFF);          }
void topLinkDownOn(uint16_t outCurrent) {       write_output(CHANNEL_TOPLINK_DOWN, outCurrent); }
void topLinkDownOff()                   {       write_output(CHANNEL_TOPLINK_DOWN, OFF);        }
void balanceDownOn(uint16_t outCurrent) {       write_output(CHANNEL_BALANCE_DOWN, outCurrent); }
void balanceDownOff()                   {       write_output(CHANNEL_BALANCE_DOWN, OFF);          }
void balanceUpOn(uint16_t outCurrent)   {       write_output(CHANNEL_BALANCE_UP, outCurrent);   }
void balanceUpOff()                     {       write_output(CHANNEL_BALANCE_UP, OFF);          }


void output_controller_shuttle(uint8_t forceUpdate)
{
  static uint16_t lastUpdatedCurrent = 0;
  
  if((flagOutputControl.backward == ON) && (flagOutputControl.forward == ON)) {
    flagOutputControl.forward = OFF;
    flagOutputControl.backward = OFF;
    // For safety
    forwardOff();
    backwardOff();
    return;
  }
  
  if((lastUpdatedCurrent != get_finalCurrent()) || (forceUpdate == TRUE)) {
    lastUpdatedCurrent = get_finalCurrent();
    
    if(flagOutputControl.forward == ON)         forwardOn(lastUpdatedCurrent);
    else if(flagOutputControl.backward == ON)   backwardOn(lastUpdatedCurrent);
    else {
      forwardOff();
      backwardOff();
    }
  }
}

void output_controller()
{
  static uint16_t timerTLE92464Update = 0;
  uint8_t flagTLE92464Update = FALSE;
  static uint8_t updateTimer = 0;
  
  static uint8_t topLinkUpPrevious;
  static uint8_t topLinkDownPrevious;
  static uint8_t balanceUpPrevious;
  static uint8_t balanceDownPrevious;
    
  /*
  static uint16_t topLinkUpLastUpdatedCurrent = 0;
  static uint16_t topLinkDownLastUpdatedCurrent = 0;
  static uint16_t balanceUpLastUpdatedCurrent = 0;
  static uint16_t balanceDownLastUpdatedCurrent = 0;
  */
    
  if(tle92464_init == 0) {
    updateTimer += 2;
    if(updateTimer >= 10) {
      updateTimer = 0;
      output_init();                                                   // Every 10ms it is called
    }
  }
  else {
    
    if(flagTimer.tenMs == TRUE) {
      timerTLE92464Update += 10;
    }
    if(timerTLE92464Update >= 300) {
      flagTLE92464Update = TRUE;
      timerTLE92464Update = 0;
    }
    
    output_controller_shuttle(flagTLE92464Update);
    
    if((flagOutputControlPrevious.regen != flagOutputControl.regen)  || (flagTLE92464Update == TRUE)) {
      flagOutputControlPrevious.regen = flagOutputControl.regen;
      /* Disabled on 2025.07.30
      if(flagOutputControl.regen == ON)         regenOn(1500);               // changed the current on 500mA
      else                                      regenOff();
      */
      regenOff();
    }
/*
    if((flagOutputControlPrevious.reserved3 != flagOutputControl.reserved3)  || (flagTLE92464Update == TRUE)) {
      flagOutputControlPrevious.reserved3 = flagOutputControl.reserved3;
      if(flagOutputControl.reserved3 == ON)     reserved3On(1500);               // changed the current on 500mA
      else                                      reserved3Off();
    }
*/
    if((topLinkUpPrevious != flagOutputControl.topLinkUp) || (flagTLE92464Update == TRUE)) {
      topLinkUpPrevious = flagOutputControl.topLinkUp;
      if(flagOutputControl.topLinkUp == ON)     topLinkUpOn(1500);
      else                                      topLinkUpOff();
    }
    
    if((topLinkDownPrevious != flagOutputControl.topLinkDown) || (flagTLE92464Update == TRUE)) {
      topLinkDownPrevious = flagOutputControl.topLinkDown;
      if(flagOutputControl.topLinkDown == ON)   topLinkDownOn(1500);
      else                                      topLinkDownOff();
    }
    
    if((balanceUpPrevious != flagOutputControl.balanceUp) || (flagTLE92464Update == TRUE)) {
      balanceUpPrevious = flagOutputControl.balanceUp;
      if(flagOutputControl.balanceUp == ON)     balanceUpOn(1500);//balanceUpOn(get_balanceUpCurrent());
      else                                      balanceUpOff();
    }
  
    if((balanceDownPrevious != flagOutputControl.balanceDown) || (flagTLE92464Update == TRUE)) {
      balanceDownPrevious = flagOutputControl.balanceDown;
      if(flagOutputControl.balanceDown == ON)   balanceDownOn(1500);//balanceDownOn(get_balanceDownCurrent());
      else                                      balanceDownOff();
    }
    
    if(flagTimer.hundredMs == TRUE)
    {
      if((flagOutputControl.forward == OFF) && (flagOutputControl.backward == OFF) && (flagOutputControl.regen == OFF)) {
        tle9246x_clear_errors(TLE9246x_FIRST_IC, TRUE);
      }
      else {
        tle9246x_get_diagnostic(TLE9246x_FIRST_IC);
        flagErrorSensors.forwardValve = tle9246x_outputError[TLE9246x_CHANNEL_FORWARD];
        flagErrorSensors.backwardValve = tle9246x_outputError[TLE9246x_CHANNEL_BACKWARD];
        flagErrorSensors.regenValve = tle9246x_outputError[TLE9246x_CHANNEL_REGEN];
        flagErrorSensors.reserved3 = tle9246x_outputError[TLE9246x_CHANNEL_RESERVED3];

        if((flagErrorSensors.forwardValve == FALSE) && (flagErrorSensors.backwardValve == FALSE) && (flagErrorSensors.regenValve == FALSE))
        {
          tle9246x_clear_errors(TLE9246x_FIRST_IC, FALSE);
        }
        else {
          if(flagOutputControl.forward == OFF)          tle9246x_outputError_local[TLE9246x_CHANNEL_FORWARD] = FALSE;
          if(flagOutputControl.backward == OFF)         tle9246x_outputError_local[TLE9246x_CHANNEL_BACKWARD] = FALSE;
          if(flagOutputControl.regen == OFF)            tle9246x_outputError_local[TLE9246x_CHANNEL_REGEN] = FALSE;
          if(flagOutputControl.reserved3 == OFF)        tle9246x_outputError_local[TLE9246x_CHANNEL_RESERVED3] = FALSE;
        }
      }
    }

    if(flagTimer.hundredMs == TRUE)
    {
      if((flagOutputControl.topLinkUp == OFF) && (flagOutputControl.topLinkDown == OFF) && 
         (flagOutputControl.balanceUp == OFF) && (flagOutputControl.balanceDown == OFF)) {
        tle9246x_clear_errors(TLE9246x_SECOND_IC, TRUE);
      }
      else {
        tle9246x_get_diagnostic(TLE9246x_SECOND_IC);
        flagErrorSensors.topLinkUpValve = tle9246x_outputError[TLE9246x_CHANNEL_TOPLINK_UP];
        flagErrorSensors.topLinkDownValve = tle9246x_outputError[TLE9246x_CHANNEL_TOPLINK_DOWN];
        flagErrorSensors.balanceUpValve = tle9246x_outputError[TLE9246x_CHANNEL_BALANCE_UP];
        flagErrorSensors.balanceDownValve = tle9246x_outputError[TLE9246x_CHANNEL_BALANCE_DOWN];
        
        if((flagErrorSensors.topLinkUpValve == FALSE) && (flagErrorSensors.topLinkDownValve == FALSE) &&
           (flagErrorSensors.balanceUpValve == FALSE) && (flagErrorSensors.balanceDownValve == FALSE))
        {
          tle9246x_clear_errors(TLE9246x_SECOND_IC, FALSE);
        }
        else {
          if(flagOutputControl.topLinkUp == OFF)                tle9246x_outputError_local[TLE9246x_CHANNEL_TOPLINK_UP] = FALSE;
          if(flagOutputControl.topLinkDown == OFF)              tle9246x_outputError_local[TLE9246x_CHANNEL_TOPLINK_DOWN] = FALSE;
          if(flagOutputControl.balanceUp == OFF)                tle9246x_outputError_local[TLE9246x_CHANNEL_BALANCE_UP] = FALSE;
          if(flagOutputControl.balanceDown == OFF)              tle9246x_outputError_local[TLE9246x_CHANNEL_BALANCE_DOWN] = FALSE;
        }
      }
    }
  }
  
  flagOutputControl.adMotorRight = OFF;
  flagOutputControl.adMotorLeft = OFF;
  if(flagOutputControl.adMotorLeft == ON)       adMotorLeftOn();
  else                                          adMotorLeftOff();

  if(flagOutputControl.adMotorRight == ON)      adMotorRightOn();
  else                                          adMotorRightOff();
  
  if(flagOutputControl.pto == ON)               ptoOn(get_ptoDuty());
  else                                          ptoOff();
  
  if(flagOutputLamp.checkBuzzer == ON)          checkBuzzerOn();
  else                                          checkBuzzerOff();
  
  if(flagOutputControl.starterOut == ON)        starterOn();
  else                                          starterOff();
  
  if(flagOutputControl.hitchUp == ON)           hitchUpOn(get_threePUpDuty());
  else                                          hitchUpOff();

  if(flagOutputControl.hitchDown == ON)         hitchDownOn(get_threePDownDuty());
  else                                          hitchDownOff();
  
  if(flagOutputControl.backBuzzer == ON)        backBuzzerOn();
  else                                          backBuzzerOff();
  
  if(flagOutputControl.qt == ON)                qtOn();
  else                                          qtOff();
  
  if(flagOutputControl.unload == ON)            unloadOn();
  else                                          unloadOff();
  
  if(flagOutputControl.fourWD == ON)            fourWDOn();
  else                                          fourWDOff();
  
}

void output_clear()
{
  // The forward and backward output state is controlled in power shuttle function, please see the shuttle.c source file
  flagOutputControl.balanceDown = OFF;
  flagOutputControl.balanceUp = OFF;
  
  flagOutputControl.topLinkDown = OFF;
  flagOutputControl.topLinkUp = OFF;
  flagOutputControl.qt = OFF;
  
  flagOutputControl.pto = OFF;
  flagOutputControl.fourWD = OFF;
  flagOutputControl.backBuzzer = OFF;
  flagOutputControl.starterOut = OFF;
  
  flagOutputLamp.checkBuzzer = OFF;
  flagOutputLamp.pto = OFF;
  flag.buzzerOn = FALSE;
}