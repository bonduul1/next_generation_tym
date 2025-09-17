/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RPM_CONTROLLER_H
#define __RPM_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Definitions ---------------------------------------------------------------*/

#define VAC_HAND_ACCEL_MIN              0
#define VAC_HAND_ACCEL_MAX              250
    
#define VAC_FOOT_ACCEL_MIN              (uint16_t)((float)0.42 / (float)0.00488 )      // 0.5 * 621 = 310
#define VAC_FOOT_ACCEL_MAX              (uint16_t)((float)4.50 / (float)0.00488 )      // 4.5 * 621 = 2794

#define VAC_RPM_TO_BIT                  0.125f
#define VAC_RPM_MIN                     (float)( 900.0f/VAC_RPM_TO_BIT)      // rpm minimum value is 800 ==> 6400
#define VAC_RPM_MAX                     (float)(2330.0f/VAC_RPM_TO_BIT)      // rpm maximum value is 2350 ==> 18800    --> Diff = 12400

#define VAC_HAND_RPM_CAL                (float)(VAC_RPM_MAX - VAC_RPM_MIN)/(float)(VAC_HAND_ACCEL_MAX - VAC_HAND_ACCEL_MIN) // calculate hand raw data into rpm (scaling)
#define VAC_FOOT_RPM_CAL                (float)(VAC_RPM_MAX - VAC_RPM_MIN)/(float)(VAC_FOOT_ACCEL_MAX - VAC_FOOT_ACCEL_MIN) // calculate foot raw data into rpm (scaling)
  
#define VAC_RPM_CRUISE_SAVE_TIME        3000                                    // ms
  
/* Definitions of Registers --------------------------------------------------*/
  
typedef union {
  uint32_t data;
  struct {
    uint32_t footSensorError            : 1;
    uint32_t handSensorError            : 1; 
    uint32_t footSensorUsed             : 1; 
    uint32_t handSensorUsed             : 1; 
    uint32_t footIvs                    : 1; 
    uint32_t res0                       : 3; 
    
    uint32_t rpmCruiseUsedA             : 1; 
    uint32_t rpmCruiseSwLongPressedA    : 1; 
    uint32_t rpmCruiseUsedB             : 1; 
    uint32_t rpmCruiseSwLongPressedB    : 1; 
    uint32_t res1                       : 4; 
    
    uint32_t res2                       : 16;    
  } ;
} rpmControllerFlag_t;

extern rpmControllerFlag_t rpmControllerFlag;

extern uint16_t nvRpmCruiseNewA;
extern uint16_t nvRpmCruiseNewB;
extern uint16_t resultRpm;

void rpm_control_process();
void rpm_can_packet(uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __RPM_CONTROLLER_H */
