/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PTO_H
#define __PTO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private defines -----------------------------------------------------------*/
#define NUMBER_OF_PTO_ON_POINTS                 8
#define NUMBER_OF_PTO_OFF_POINTS                8
#define PTO_DUTY_MAX                            1000
#define PTO_SAFETY_CHECK_TIME                   5000
  
#define PTO_MODE_OFF                            0
#define PTO_MODE_MANUAL                         1
#define PTO_MODE_AUTO                           2
#define PTO_MODE_STATIONARY                     3
  
extern uint8_t ptoSwitch;
extern uint8_t ptoMode;
  
uint8_t get_ptoSwitch();
uint16_t get_ptoDuty();

void updatePtoSw();
void pto_control_process();
void pto_init();
void calculate_pto_compensation();

#ifdef __cplusplus
}
#endif

#endif /* __PTO_H */
