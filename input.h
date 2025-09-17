/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INPUT_H
#define __INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define FPC_SPEED_INT_Pin               GPIO_PIN_2
#define FPC_SPEED_INT_GPIO_Port         GPIOE
#define FPC_SPEED_INT_EXTI_IRQn         EXTI2_IRQn

#define FPC_SW1_Pin                     GPIO_PIN_7                              // PTO auto switch
#define FPC_SW1_GPIO_Port               GPIOE
#define FPC_SW2_Pin                     GPIO_PIN_3                              // Seat switch
#define FPC_SW2_GPIO_Port               GPIOE
#define FPC_SW3_Pin                     GPIO_PIN_4                              // Brake switch
#define FPC_SW3_GPIO_Port               GPIOE
#define FPC_SW4_Pin                     GPIO_PIN_5                              // Side brake switch
#define FPC_SW4_GPIO_Port               GPIOE
  
#define NUMBER_OF_INPUTS                4

typedef union {
  uint32_t data;
  struct {
    uint32_t topLinkUp            : 1;
    uint32_t topLinkDown          : 1; 
    uint32_t transmit             : 1; 
    uint32_t res1                 : 1; 
    uint32_t transmissionOne      : 1; 
    uint32_t transmissionTwo      : 1; 
    uint32_t transmissionThree    : 1; 
    uint32_t transmissionFour     : 1;

    uint32_t res2                 : 1;
    uint32_t res3                 : 1;
    uint32_t transmissionL        : 1; 
    uint32_t transmissionM        : 1; 
    uint32_t parkingBrake         : 1; 
    uint32_t balanceUp            : 1; 
    uint32_t balanceDown          : 1; 
    uint32_t ptoManual            : 1; 
    
    uint32_t hitchManualUp        : 1; 
    uint32_t hitchManualDown      : 1; 
    uint32_t transmissionH        : 1;
    uint32_t model                : 1;
    uint32_t res5                 : 1;
    uint32_t footPedal            : 1;
    uint32_t res6                 : 1;
    uint32_t res7                 : 1;
    
    uint32_t brake                : 1;
    uint32_t sideBrake            : 1; 
    uint32_t seat                 : 1; 
    uint32_t ptoAuto              : 1; 
    uint32_t res8                 : 1; 
    uint32_t res9                 : 1;
    uint32_t res10                : 1;
    uint32_t res11                : 1;
  };
} flagInputStatus_t;

extern flagInputStatus_t        flagInputStatus;
extern flagInputStatus_t        flagInputStatusPrevious;

void     gpio_input_init(void);
void     read_digital_inputs();
uint8_t  get_speedCounter();
void     set_speedCounter(uint8_t _speedCounter);


uint16_t getRawBackwardPressure();
uint16_t getRawForwardPressure();
uint16_t getRawShuttleLever();
uint16_t getRawClutchLevel();
uint16_t getRawOilTemperature();
uint16_t getRawFootAccelerator();
uint16_t getRawMotorPosition();
uint16_t getRawWirelessChargingLevel();
uint16_t getRawHitchPosition();
uint16_t getRawStrokeSensor();
uint16_t getRawHitchDepth();
uint16_t getRawSteering();
uint16_t getRawHitchDraft();
uint16_t getRawBalanceRolling();
uint16_t getRawBatteryVoltage();
uint16_t getRawSensorVoltage();

#ifdef __cplusplus
}
#endif

#endif /* __INPUT_H */
