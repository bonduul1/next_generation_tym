/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines  ------------------------------------------------------------------*/ 
#define TIMER_CHANNEL_1         1
#define TIMER_CHANNEL_2         2
#define TIMER_CHANNEL_3         3
#define TIMER_CHANNEL_4         4
/* Variables------------------------------------------------------------------*/
  
void MX_TIM4_Init();
uint8_t update_tim_channel(uint8_t channel, uint16_t duty);
uint8_t start_timers();

void MX_TIM3_Init();
uint8_t update_tim_pto(uint16_t duty);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */
