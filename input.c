#include "input.h"
#include "adc.h"
#include "main.h"
#include "tic12400.h"

uint8_t speedCounter;
flagInputStatus_t flagInputStatus;
flagInputStatus_t flagInputStatusPrevious;

uint8_t                 get_speedCounter()                      {       return speedCounter;            }
void                    set_speedCounter(uint8_t _speedCounter) {       speedCounter = _speedCounter;   }

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void gpio_input_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : FPC_SW1_Pin */
  GPIO_InitStruct.Pin = FPC_SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FPC_SW1_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FPC_SW2_Pin */
  GPIO_InitStruct.Pin = FPC_SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FPC_SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FPC_SW3_Pin */
  GPIO_InitStruct.Pin = FPC_SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FPC_SW3_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FPC_SW4_Pin */
  GPIO_InitStruct.Pin = FPC_SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FPC_SW4_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : FPC_SPEED_INT_Pin */
  GPIO_InitStruct.Pin = FPC_SPEED_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FPC_SPEED_INT_GPIO_Port, &GPIO_InitStruct);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  
  tic12400_Init(TIC12400_FIRST_IC);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == FPC_SPEED_INT_Pin) {
    speedCounter++;
    if ( speedCounter >= 254) speedCounter = 254;
  }
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

void read_digital_inputs()
{
  static uint8_t rawData[NUMBER_OF_INPUTS]; 
  uint32_t data;
  uint8_t i;
    
  for(i = 0; i < NUMBER_OF_INPUTS; i++) {
    rawData[i] <<= 1;
  }
  
  rawData[0] = (HAL_GPIO_ReadPin(FPC_SW2_GPIO_Port, FPC_SW2_Pin) == GPIO_PIN_SET)   ? (rawData[0]+1) : rawData[0];                // Seat switch
  rawData[1] = (HAL_GPIO_ReadPin(FPC_SW3_GPIO_Port, FPC_SW3_Pin) == GPIO_PIN_RESET) ? (rawData[1]+1) : rawData[1];                // Brake switch
  rawData[2] = (HAL_GPIO_ReadPin(FPC_SW4_GPIO_Port, FPC_SW4_Pin) == GPIO_PIN_RESET) ? (rawData[2]+1) : rawData[2];                // Side brake switch
  rawData[3] = (HAL_GPIO_ReadPin(FPC_SW1_GPIO_Port, FPC_SW1_Pin) == GPIO_PIN_SET) ? (rawData[3]+1) : rawData[3];                  // PTO Auto switch

  data = tic12400_getInputDataProved();
      
  flagInputStatus.data = data & 0x00FFFFFF;

  flagInputStatus.seat          = (rawData[0] == 255) ? ON :  OFF;
  flagInputStatus.brake         = (rawData[1] == 255) ? ON :  OFF;
  flagInputStatus.sideBrake     = (rawData[2] == 255) ? ON :  OFF;
  flagInputStatus.ptoAuto       = (rawData[3] == 255) ? ON :  OFF;
}

uint16_t getRawBackwardPressure()       {  return getAverageADCValue(ADC_BACKWARD_PRESSURE);            }
uint16_t getRawForwardPressure()        {  return getAverageADCValue(ADC_FORWARD_PRESSURE);             }
uint16_t getRawShuttleLever()           {  return getAverageADCValue(ADC_SHUTTLE_LEVER);                }
uint16_t getRawClutchLevel()            {  return getAverageADCValue(ADC_CLUTCH_LEVEL);                 }
uint16_t getRawOilTemperature()         {  return getAverageADCValue(ADC_OIL_TEMPERATURE);              }
uint16_t getRawFootAccelerator()        {  return getCurrentADCValue(ADC_FOOT_ACCELERATOR);             }            // Changed on 2022.12.12
uint16_t getRawMotorPosition()          {  return getAverageADCValue(ADC_MOTOR_POSITION);               }
uint16_t getRawWirelessChargingLevel()  {  return getAverageADCValue(ADC_WIRELESS_CHARGING_LEVEL);      }
uint16_t getRawHitchPosition()          {  return getAverageADCValue(ADC_HITCH_POSITION_SENSOR);        }
uint16_t getRawStrokeSensor()           {  return getAverageADCValue(ADC_STROKE_SENSOR);                }
uint16_t getRawHitchDepth()             {  return getAverageADCValue(ADC_HITCH_DEPTH_SENSOR);           }
uint16_t getRawSteering()               {  return getAverageADCValue(ADC_STEERING_SENSOR);              }
uint16_t getRawHitchDraft()             {  return getAverageADCValue(ADC_HITCH_DRAFT_SENSOR);           }
uint16_t getRawBalanceRolling()         {  return getAverageADCValue(ADC_BALANCE_ROLLING_SENSOR);       }
uint16_t getRawBatteryVoltage()         {  return getAverageADCValue(ADC_BATTERY_POWER);                }
uint16_t getRawSensorVoltage()          {  return getAverageADCValue(ADC_SENSOR_POWER);                 }