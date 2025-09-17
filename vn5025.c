#include "vn5025.h"
#include "main.h"
#include "timer.h"

/*
 * Output of the status pins are analog, so I changed program as follows on 2021.12.17
 */

#define HARDWARE_PWM_ENABLED_FOR_3P            1

void vn5025_init()
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FOC_TOPLINK_UP_GPIO_Port, FOC_TOPLINK_UP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FOC_TOPLINK_DOWN_GPIO_Port, FOC_TOPLINK_DOWN_Pin, GPIO_PIN_RESET);

#ifdef HARDWARE_PWM_ENABLED_FOR_3P
  MX_TIM4_Init();                                                               // Timer enable with output comapre mode and PWM1
#else
  /*Configure GPIO pins : FOC_TOPLINK_UP_Pin */
  GPIO_InitStruct.Pin = FOC_TOPLINK_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_TOPLINK_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FOC_TOPLINK_DOWN_Pin */
  GPIO_InitStruct.Pin = FOC_TOPLINK_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FOC_TOPLINK_DOWN_GPIO_Port, &GPIO_InitStruct);
#endif                                                                          // it is moved on 2021.12.16
  
  /*Configure GPIO pins : FST_TOPLINK_DOWN_Pin */
  GPIO_InitStruct.Pin = FST_TOPLINK_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;                           // Always high/low, enable - pull up, can not detect
  HAL_GPIO_Init(FST_TOPLINK_DOWN_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FST_TOPLINK_UP_Pin */
  GPIO_InitStruct.Pin = FST_TOPLINK_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;                           // Always high/low, enable - pull up, can not detect
  HAL_GPIO_Init(FST_TOPLINK_UP_GPIO_Port, &GPIO_InitStruct);
}

void vn5025_output(uint8_t channel, uint16_t level)
{
#ifdef HARDWARE_PWM_ENABLED_FOR_3P
  update_tim_channel(channel, level);                                                               // Timer enable with output comapre mode and PWM1
#else
  if(channel == VN5025_CHANNEL_DOWN){
    HAL_GPIO_WritePin(FOC_TOPLINK_DOWN_GPIO_Port, FOC_TOPLINK_DOWN_Pin, ((level == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET));
  }
  else if(channel == VN5025_CHANNEL_UP){
    HAL_GPIO_WritePin(FOC_TOPLINK_UP_GPIO_Port, FOC_TOPLINK_UP_Pin, ((level == 0) ? GPIO_PIN_RESET : GPIO_PIN_SET));
  }
#endif
}

int8_t vn5025_status(uint8_t channel)
{
  int8_t ret = -1;
  if(channel == VN5025_CHANNEL_DOWN){
    ret = HAL_GPIO_ReadPin(FST_TOPLINK_DOWN_GPIO_Port, FST_TOPLINK_DOWN_Pin);
  }
  else if(channel == VN5025_CHANNEL_UP){  
    ret = HAL_GPIO_ReadPin(FST_TOPLINK_UP_GPIO_Port, FST_TOPLINK_UP_Pin);
  }
  return ret;
}