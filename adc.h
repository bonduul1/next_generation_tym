/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define ADC0_Pin                        GPIO_PIN_0
#define ADC0_GPIO_Port                  GPIOC
#define ADC1_Pin                        GPIO_PIN_1
#define ADC1_GPIO_Port                  GPIOC
#define ADC2_Pin                        GPIO_PIN_2
#define ADC2_GPIO_Port                  GPIOC
#define ADC3_Pin                        GPIO_PIN_3
#define ADC3_GPIO_Port                  GPIOC
#define ADC4_Pin                        GPIO_PIN_0
#define ADC4_GPIO_Port                  GPIOA
#define ADC5_Pin                        GPIO_PIN_1
#define ADC5_GPIO_Port                  GPIOA
#define ADC6_Pin                        GPIO_PIN_2
#define ADC6_GPIO_Port                  GPIOA
#define ADC7_Pin                        GPIO_PIN_3
#define ADC7_GPIO_Port                  GPIOA
#define ADC8_Pin                        GPIO_PIN_4
#define ADC8_GPIO_Port                  GPIOA
#define ADC9_Pin                        GPIO_PIN_5
#define ADC9_GPIO_Port                  GPIOA
#define ADC10_Pin                       GPIO_PIN_6
#define ADC10_GPIO_Port                 GPIOA
#define ADC11_Pin                       GPIO_PIN_7
#define ADC11_GPIO_Port                 GPIOA
#define ADC12_Pin                       GPIO_PIN_4
#define ADC12_GPIO_Port                 GPIOC
#define ADC13_Pin                       GPIO_PIN_5
#define ADC13_GPIO_Port                 GPIOC
#define ADC14_Pin                       GPIO_PIN_0
#define ADC14_GPIO_Port                 GPIOB
#define ADC15_Pin                       GPIO_PIN_1
#define ADC15_GPIO_Port                 GPIOB
  
#define NUMBER_OF_ADC_CHANNELS          16
#define NUMBER_OF_ADC_AVERAGE           4

#define ADC_BACKWARD_PRESSURE           0
#define ADC_FORWARD_PRESSURE            1
#define ADC_SHUTTLE_LEVER               2
#define ADC_CLUTCH_LEVEL                3
#define ADC_OIL_TEMPERATURE             4
#define ADC_FOOT_ACCELERATOR            5
#define ADC_MOTOR_POSITION              6
#define ADC_WIRELESS_CHARGING_LEVEL     7
#define ADC_HITCH_POSITION_SENSOR       8
#define ADC_STROKE_SENSOR               9
#define ADC_HITCH_DEPTH_SENSOR          10
#define ADC_STEERING_SENSOR             11
#define ADC_HITCH_DRAFT_SENSOR          12
#define ADC_BALANCE_ROLLING_SENSOR      13
#define ADC_BATTERY_POWER               14
#define ADC_SENSOR_POWER                15
  
  
#define VAC_AD_CONVER                   1023
#define VAC_AD_BIT_VOLTAGE              (float)(3.3/(float)4095)
#define VAC_AD_DEFAULT_RATE             (5/2)/VAC_AD_BIT_VOLTAGE

#define VAC_AD_SENSOR_POWER_LOW         (4/2)/VAC_AD_BIT_VOLTAGE
#define VAC_AD_SENSOR_POWER_HIGH        (6/2)/VAC_AD_BIT_VOLTAGE
   
/* Functions -----------------------------------------------------------------*/
void MX_DMA_Init(void);
void MX_ADC1_Init(void);

uint8_t  startConversation();
uint8_t  updateADC();
uint8_t  updateLastAverageADC();

uint16_t getCurrentADCValue(uint8_t channel);
uint16_t getAverageADCValue(uint8_t channel);
uint8_t  getCurrentADCValues(uint16_t *channels_data);
uint8_t  getAverageADCValues(uint16_t *channels_data);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
