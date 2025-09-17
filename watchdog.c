
#include "watchdog.h"
#include "main.h"

IWDG_HandleTypeDef hiwdg;

void watchdog_init()
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

void watchdog_trigger()
{
  HAL_IWDG_Refresh(&hiwdg); 
}