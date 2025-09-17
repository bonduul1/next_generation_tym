/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BTS7006_H
#define __BTS7006_H

/*
  *     File:           bts7006.h
  *     Author:         Enkhbat Batbayar
  *     Date:           2022.10.14
  *     Version:        1.0
  *     Note:           bts7006 IC driver and testing source code
  *     
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Defines  ------------------------------------------------------------------*/

/*---------------------------- Other   Functions -----------------------------*/
#define FOC_PTO_Pin                     GPIO_PIN_9
#define FOC_PTO_GPIO_Port               GPIOC
#define FOC_AD_MOTOR_LEFT_Pin           GPIO_PIN_9
#define FOC_AD_MOTOR_LEFT_GPIO_Port     GPIOB
#define FOC_AD_MOTOR_RIGHT_Pin          GPIO_PIN_8
#define FOC_AD_MOTOR_RIGHT_GPIO_Port    GPIOB

#define FST_PTO_Pin                     GPIO_PIN_8
#define FST_PTO_GPIO_Port               GPIOA
#define FST_AD_MOTOR_LEFT_Pin           GPIO_PIN_0
#define FST_AD_MOTOR_LEFT_GPIO_Port     GPIOE
#define FST_AD_MOTOR_LEFT_SEL_Pin       GPIO_PIN_1
#define FST_AD_MOTOR_LEFT_SEL_GPIO_Port GPIOE
  
#define BTS7006_CHANNEL_ONE             1
#define BTS7006_CHANNEL_TWO             2
#define BTS7006_CHANNEL_THREE           3
#define BTS7006_CHANNEL_FOUR            4

#define BTS7006_CHANNEL_AD_MOTOR_RIGHT  BTS7006_CHANNEL_ONE
#define BTS7006_CHANNEL_AD_MOTOR_LEFT   BTS7006_CHANNEL_TWO
#define BTS7006_CHANNEL_PTO             BTS7006_CHANNEL_THREE

  
void bts7006_init();
void bts7006_output(uint8_t channel, uint16_t level);
int8_t bts7006_status(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __BTS7006_H */
