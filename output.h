/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OUTPUT_H
#define __OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define FOC_RESERVED_Pin                GPIO_PIN_15
#define FOC_RESERVED_GPIO_Port          GPIOE
#define FOC_REGEN_Pin                   GPIO_PIN_14
#define FOC_REGEN_GPIO_Port             GPIOE
#define FOC_BALANCE_UP_Pin              GPIO_PIN_13
#define FOC_BALANCE_UP_GPIO_Port        GPIOE
#define FOC_BALANCE_DOWN_Pin            GPIO_PIN_12
#define FOC_BALANCE_DOWN_GPIO_Port      GPIOE

#define FOC_SOL4_Pin                    GPIO_PIN_8
#define FOC_SOL4_GPIO_Port              GPIOC
#define FOC_SOL5_Pin                    GPIO_PIN_7
#define FOC_SOL5_GPIO_Port              GPIOC
#define FOC_SOL6_Pin                    GPIO_PIN_6
#define FOC_SOL6_GPIO_Port              GPIOC
#define FOC_SOL7_Pin                    GPIO_PIN_9
#define FOC_SOL7_GPIO_Port              GPIOD
#define FOC_SOL8_Pin                    GPIO_PIN_8
#define FOC_SOL8_GPIO_Port              GPIOD
  
#define FOC_LAMP_Pin                    GPIO_PIN_8
#define FOC_LAMP_GPIO_Port              GPIOE
#define FOC_START_Pin                   GPIO_PIN_11
#define FOC_START_GPIO_Port             GPIOA

#define CHANNEL_FORWARD                 1
#define CHANNEL_BACKWARD                2
#define CHANNEL_REGEN                   3
#define CHANNEL_RESERVED3               4
#define CHANNEL_TOPLINK_UP              5
#define CHANNEL_TOPLINK_DOWN            6
#define CHANNEL_BALANCE_UP              7
#define CHANNEL_BALANCE_DOWN            8

#define CHANNEL_AD_MOTOR_RIGHT          9
#define CHANNEL_AD_MOTOR_LEFT           10
#define CHANNEL_PTO                     11
  
#define CHANNEL_HITCH_UP                12
#define CHANNEL_HITCH_DOWN              13
  
#define CHANNEL_STARTER_OUT             14
#define CHANNEL_CHECK_BUZZER            15

#define CHANNEL_FOURWD                  16
#define CHANNEL_UNLOAD                  17
#define CHANNEL_QUICK_TURN              18
#define CHANNEL_BACK_BUZZER             19

  
typedef union {
  uint32_t data;
  struct {
    uint32_t backward           : 1;
    uint32_t forward            : 1;
    uint32_t regen              : 1;                            // PIN-38
    uint32_t reserved3          : 1;                            // PIN-63
    uint32_t topLinkUp          : 1;
    uint32_t topLinkDown        : 1;
    uint32_t balanceUp          : 1;
    uint32_t balanceDown        : 1;
    
    uint32_t adMotorRight       : 1;                            // PIN-3
    uint32_t adMotorLeft        : 1;
    uint32_t pto                : 1;
    uint32_t hitchUp            : 1;
    uint32_t hitchDown          : 1;
    uint32_t starterOut         : 1;
    uint32_t checkBuzzer        : 1;
    uint32_t fourWD             : 1;
    
    uint32_t qt                 : 1;
    uint32_t backBuzzer         : 1;
    uint32_t unload             : 1;
    
    uint32_t neutral            : 1;
    uint32_t res                : 14;
  };
} flagOutput_t;

extern flagOutput_t flagOutputControlPrevious;                                  // Added on 2021.11.19 for reducing access time of TLE82453

extern flagOutput_t flagOutputControl;
extern flagOutput_t flagOutputStatus;

extern flagOutput_t flagOutputLamp;
extern flagOutput_t flagOutputValve;


void     gpio_output_init();
uint8_t  output_init();

void output_controller_shuttle(uint8_t forceUpdate);
void output_controller();
void output_clear();

extern uint8_t tle92464_init;
extern uint8_t tle92464_icFirstInit;
extern uint8_t tle92464_icSecondInit;

#ifdef __cplusplus
}
#endif

#endif /* __OUTPUT_H */
