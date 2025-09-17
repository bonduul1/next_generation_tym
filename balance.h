/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BALANCE_H
#define __BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define VA_SET_CENTER                           (float)512
#define VA_SET_GAP                              (float)60
  
#define	VAC_EFFECTIVE_RANGE 	                (float)40
#define VAC_STOP_DEADBAND_DOWN                  (float)20
#define VAC_STOP_DEADBAND_UP                    (float)25
  
#define VAC_ROLL_CENTER_TO_STROKE               (float)1.4
  
#define VAC_BALANCE_SETTING_TIME                3000
#define VAC_BALANCE_SETTING_OVER_TIME           15000

#define VAC_BALANCE_OUTPUT_LIMIT_TIME           5000
  
#define VA_BALANCE_DELAY                        500    
#define VA_BALANCE_UP_DELAY                     100
#define VA_BALANCE_DOWN_DELAY                   100

#define VAB_STOP_MODE                           0
#define VAB_DOWN_MODE                           1
#define VAB_UP_MODE                             2

#define VAB_BALANCE_MANUAL_MODE                 0
#define VAB_BALANCE_FLAT_MODE                   1
#define VAB_BALANCE_SLOPE_MODE                  2

#define VAB_BALANCE_FAST_MODE                   0
#define VAB_BALANCE_NORMAL_MODE                 1
#define VAB_BALANCE_SLOW_MODE                   2

#define UNLOAD_VALVE_ON_AND_OFF_TIME            22                              // Unload valve on and off time = 20ms, the time is changed on 2024.01.04 in order to remove one cycle,
  
typedef union {
  uint32_t data;
  struct {
    uint32_t settingMode        : 1;
    uint32_t settingStroke      : 1;
    uint32_t settingRoll        : 1;
    uint32_t volumeMoved        : 1;
    uint32_t ptoStopped         : 1;
    uint32_t autoFlat           : 1;
    uint32_t autoSlope          : 1;
    uint32_t res                : 1;
    
    uint32_t balanceMode        : 8;
    
    uint32_t settingSequence    : 3;
    
    uint32_t res1               : 13;
  };
} flagBalance_t;

extern flagBalance_t flagBalance;
extern float targetStrokePosition;

void balance_setting(void);
void balance_init();
void balance_process(void);
uint8_t unload_valve_control();

uint16_t get_balanceUpCurrent();
uint16_t get_balanceDownCurrent();

#ifdef __cplusplus
}
#endif

#endif /* __BALANCE_H */
