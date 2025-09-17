/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TOPLINK_H
#define __TOPLINK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private defines -----------------------------------------------------------*/
#define UNLOAD_VALVE_ON_AND_OFF_TIME            22                              // Unload valve on and off time = 20ms, the time is changed on 2024.01.04 in order to remove one cycle,

void toplink_init();
void toplink_control_process();

#ifdef __cplusplus
}
#endif

#endif /* __TOPLINK_H */
