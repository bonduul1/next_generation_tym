/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Defines  ------------------------------------------------------------------*/
  
#define SPI_ONE         1
#define SPI_TWO         2
#define SPI_THREE       3

#define SOFTWARE_SPI    0  
#define HARDWARE_SPI    1
  
/* --------------------------------------------------------- Definitions SPI2 ---------------------------------------------------*/
#define spi2_miso()              (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) ? 0 : 1

#define spi2_mosi_high()         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define spi2_mosi_low()          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)

#define spi2_sck_high()          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define spi2_sck_low()           HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)

/* --------------------------------------------------------- Definitions SPI3 ---------------------------------------------------*/
#define spi3_miso()              (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_RESET) ? 0 : 1

#define spi3_mosi_high()         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define spi3_mosi_low()          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)

#define spi3_sck_high()          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET)
#define spi3_sck_low()           HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET)

/* Functions -----------------------------------------------------------------*/  
void MX_SPI2_Init(void);
void MX_SPI3_Init(void);

void SOFT_SPI2_Init(void);
void SOFT_SPI3_Init(void);

uint32_t SPI_TransmitReceive(uint8_t channel, uint32_t txData, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
