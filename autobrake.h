/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUTOBRAKE_H
#define __AUTOBRAKE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
#define VA_AUTOBREAK_MODE_OFF           0
#define VA_AUTOBREAK_WEAK_MODE          1
#define VA_AUTOBREAK_MEDIUM_MODE        2
#define VA_AUTOBREAK_STRONG_MODE        3
  
#define VA_AUTOBREAK_DIR_NEUTRAL        0
#define VA_AUTOBREAK_DIR_RIGHT          1
#define VA_AUTOBREAK_DIR_LEFT           2

#define VA_MOTOR_RIGHT_ERROR            (uint16_t)((float)4.85/(float)0.00488)                  // 993
#define VA_MOTOR_LEFT_ERROR             (uint16_t)((float)0.15/(float)0.00488)                  // 30
#define VA_MOTOR_ERROR_BAND             (uint16_t)((float)0.25/(float)0.00488)                  // 51

#define VA_MOTOR_CENTER                 (uint16_t)((float)2.500/(float)0.00488)                 // 512

#define VA_MOTOR_PWM_BAND               (uint16_t)((float)0.70/(float)0.00488)                  // 143
#define VA_MOTOR_DEADBAND               (uint16_t)((float)0.10/(float)0.00488)                  // 20

#define VA_MOTOR_RIGHT_LARGE_LIMIIT     (uint16_t)((float)4.427/(float)0.00488)                 // 907
#define VA_MOTOR_RIGHT_MIDDLE_LIMIIT    (uint16_t)((float)4.154/(float)0.00488)                 // 851
#define VA_MOTOR_RIGHT_SMALL_LIMIIT     (uint16_t)((float)3.881/(float)0.00488)                 // 795

#define VA_MOTOR_LEFT_LARGE_LIMIIT      (uint16_t)((float)0.572/(float)0.00488)                 // 117
#define VA_MOTOR_LEFT_MIDDLE_LIMIIT     (uint16_t)((float)0.845/(float)0.00488)                 // 173
#define VA_MOTOR_LEFT_SMALL_LIMIIT      (uint16_t)((float)1.118/(float)0.00488)                 // 229

#define VAC_MOTOR_RIGHT_MAX             VA_MOTOR_RIGHT_ERROR  - (uint16_t)((float)0.1/(float)0.00488)           // 993 - 20  = 973
#define VAC_MOTOR_RIGHT_MIN             VA_MOTOR_RIGHT_ERROR  - (uint16_t)((float)1.5/(float)0.00488)           // 993 - 307 = 686
#define VAC_MOTOR_LEFT_MIN              VA_MOTOR_LEFT_ERROR   + (uint16_t)((float)0.1/(float)0.00488)           // 30  + 20  = 50
#define VAC_MOTOR_LEFT_MAX              VA_MOTOR_LEFT_ERROR   + (uint16_t)((float)1.5/(float)0.00488)           // 30  + 307 = 337

// The VAC_MOTOR_LEFT_MAX and VAC_MOTOR_LEFT_MIN are changed each other

#define VA_MOTOR_OUT_ERROR              15000
#define VA_MOTOR_OUT_CENTER             2000
  
#define VA_PWM_CYCLE                    300
#define VA_PWM_ON                       250
  
void autobrake_process();

#ifdef __cplusplus
}
#endif

#endif /* __AUTOBRAKE_H */
