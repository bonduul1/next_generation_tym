/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN5025_H
#define __VN5025_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
#define FST_TOPLINK_DOWN_Pin            GPIO_PIN_12
#define FST_TOPLINK_DOWN_GPIO_Port      GPIOD
#define FST_TOPLINK_UP_Pin              GPIO_PIN_13
#define FST_TOPLINK_UP_GPIO_Port        GPIOD

#define FOC_TOPLINK_DOWN_Pin            GPIO_PIN_14
#define FOC_TOPLINK_DOWN_GPIO_Port      GPIOD
#define FOC_TOPLINK_UP_Pin              GPIO_PIN_15
#define FOC_TOPLINK_UP_GPIO_Port        GPIOD

#define VN5025_CHANNEL_ONE              3                                 // Timer-4 and channel-3 is used for TOPLINK down control pin
#define VN5025_CHANNEL_TWO              4                                 // Timer-4 and channel-4 is used for TOPLINK up control pin
#define VN5025_CHANNEL_DOWN             VN5025_CHANNEL_ONE
#define VN5025_CHANNEL_UP               VN5025_CHANNEL_TWO
  
  
void vn5025_init();
void vn5025_output(uint8_t channel, uint16_t level);
int8_t vn5025_status(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __VN5025_H */
