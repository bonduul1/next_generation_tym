/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Defines -------------------------------------------------------------------*/
  
void watchdog_init();
void watchdog_trigger();
  
#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H */