/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHUTTLE_H
#define __SHUTTLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/

#define AD_SPEED_MIDDLE                         (float)127.0
#define AD_SPEED_MIDDLE_GAP                     7
#define SHUTTLE_DIALOG_OFFSET_CURRENT           100
#define SHUTTLE_COMPENSATION_OFFSET_CURRENT     200
    
#define VAC_NEUTRAL                             0
#define VAC_FORWARD                             10
#define VAC_BACKWARD                            100

#define VAC_SHUTTLE_MODE_OFF                    0
#define VAC_SHUTTLE_MODE_FILL                   1
#define VAC_SHUTTLE_MODE_CONTROL                2
#define VAC_SHUTTLE_MODE_CHANGE                 3
#define VAC_SHUTTLE_MODE_FREE                   4

#define VAC_SHUTTLE_MAX_TIME                    5000
#define VAC_SHUTTLE_MAX_CURRENT                 1000
#define VAC_SHUTTLE_MIN_CURRENT                 50

extern uint16_t        timeFill;
extern uint16_t        timeShuttle;
extern uint8_t         shuttleMode;

void shuttle_init();
void shuttle(uint8_t isTransmissionHappened);
void shuttle_fill_update();

void calculate_fill(uint8_t isUpdateMap);
void shuttle_valve_out();

void test_controller();
uint16_t get_finalCurrent();
uint8_t update_nv_data();

#ifdef __cplusplus
}
#endif

#endif /* __SHUTTLE_H */
