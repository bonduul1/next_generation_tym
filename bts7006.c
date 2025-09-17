 /*
  *     File:           bts7006.c
  *     Author:         Enkhbat Batbayar
  *     Date:           2022.10.13
  *     Version:        1.0
  *     Note:           bts7006 IC driver and testing source code
  *                     We do not need INx pins for controlling
  */

#include "bts7006.h"
#include "main.h"
#include "timer.h"

#define HARDWARE_PWM_ENABLED_FOR_PTO            1

void bts7006_init()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
#ifndef HARDWARE_PWM_ENABLED_FOR_PTO
  HAL_GPIO_WritePin(FOC_PTO_GPIO_Port,          FOC_PTO_Pin,            GPIO_PIN_RESET);
#endif
  HAL_GPIO_WritePin(FOC_AD_MOTOR_LEFT_GPIO_Port,        FOC_AD_MOTOR_LEFT_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_AD_MOTOR_RIGHT_GPIO_Port,       FOC_AD_MOTOR_RIGHT_Pin, GPIO_PIN_RESET);

#ifdef HARDWARE_PWM_ENABLED_FOR_PTO
    MX_TIM3_Init();                                                               // Timer enable with output comapre mode and PWM1
#else
  /*Configure GPIO pins : FOC_PTO_Pin */
  GPIO_InitStruct.Pin = FOC_PTO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_PTO_GPIO_Port, &GPIO_InitStruct);
#endif
  
  /*Configure GPIO pins : FOC_AD_MOTOR_LEFT_Pin */
  GPIO_InitStruct.Pin = FOC_AD_MOTOR_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_AD_MOTOR_LEFT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FOC_AD_MOTOR_RIGHT_Pin */
  GPIO_InitStruct.Pin = FOC_AD_MOTOR_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_AD_MOTOR_RIGHT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FST_PTO_Pin */
  GPIO_InitStruct.Pin = FST_PTO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FST_PTO_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FST_AD_MOTOR_LEFT_Pin */
  GPIO_InitStruct.Pin = FST_AD_MOTOR_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FST_AD_MOTOR_LEFT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FST_AD_MOTOR_LEFT_SEL_Pin */
  GPIO_InitStruct.Pin = FST_AD_MOTOR_LEFT_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FST_AD_MOTOR_LEFT_SEL_GPIO_Port, &GPIO_InitStruct);
}

void bts7006_output(uint8_t channel, uint16_t level)
{
  if(channel == BTS7006_CHANNEL_AD_MOTOR_RIGHT){
    HAL_GPIO_WritePin(FOC_AD_MOTOR_RIGHT_GPIO_Port, FOC_AD_MOTOR_RIGHT_Pin, ((level == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET));
  }
  else if(channel == BTS7006_CHANNEL_AD_MOTOR_LEFT){
    HAL_GPIO_WritePin(FOC_AD_MOTOR_LEFT_GPIO_Port, FOC_AD_MOTOR_LEFT_Pin, ((level == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET));
  }
  else if(channel == BTS7006_CHANNEL_PTO){
#ifdef HARDWARE_PWM_ENABLED_FOR_PTO
    update_tim_pto(level);
#else
    HAL_GPIO_WritePin(FOC_PTO_GPIO_Port, FOC_PTO_Pin, ((level == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET));
#endif
  }
  else if(channel == BTS7006_CHANNEL_FOUR){
  }
}

int8_t bts7006_status(uint8_t channel)
{
  uint8_t ret = -1;
  if(channel == BTS7006_CHANNEL_AD_MOTOR_LEFT){
    HAL_GPIO_WritePin(FST_AD_MOTOR_LEFT_SEL_GPIO_Port, FST_AD_MOTOR_LEFT_SEL_Pin, GPIO_PIN_SET);
    ret = HAL_GPIO_ReadPin(FST_AD_MOTOR_LEFT_GPIO_Port, FST_AD_MOTOR_LEFT_Pin);
  }
  else if(channel == BTS7006_CHANNEL_PTO){
    ret = HAL_GPIO_ReadPin(FST_PTO_GPIO_Port, FST_PTO_Pin);
  }
  return ret;
}