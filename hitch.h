
/*
 *
 * File Name    : hitch.h
 * Program      : HITCH CONTROLL HEAD
 * Author       : Enkhbat
 * Company      : Kiwon Electronics Co.Ltd
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HITCH_H
#define __HITCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/

#define VAC_HITCH_DIRECTION_OFF                 0x01                            // LIFT PWM output direction valve
#define VAC_HITCH_DIRECTION_UP                  0x02
#define VAC_HITCH_DIRECTION_DOWN                0x03
#define VAC_HITCH_DIRECTION_RUN                 0x04

#define VAB_HITCH_POSITION_MODE                 0
#define VAB_HITCH_DRAFT_MODE                    1
#define VAB_HITCH_DEPTH_MODE                    2

#define VAB_HITCH_SETTING_MODE_OFF              0
#define VAB_HITCH_SETTING_POSITION_MODE         1
#define VAB_HITCH_SETTING_DRAFT_MODE            2
#define VAB_HITCH_SETTING_DEPTH_MODE            3

#define VAC_IMPULSE_CHECK_TIME                  50                              // ms, Impulse output check time after ouput is stopped
#define VAC_IMPULSE_UP_TIME                     22                              // ms
#define VAC_IMPULSE_DOWN_TIME                   22                              // ms
#define VAC_LIFT_UP_PWM_CHECK_TIME              800                             // ms
#define VAC_LIFT_DOWN_PWM_CHECK_TIME            400                             // ms
#define VAC_LIFT_FLOATING_TIME                  200                             // ms
#define VAC_LIFT_SETTING_ON_TIME                2000                            // ms
#define VAB_LIFT_SETTING_CHECK_TIME             3000                            // ms
#define VAB_LIFT_SETTING_TIME_OVER              20000                           // ms

#define VAC_LIFT_DOWN_SET_POS_MID               500                             // %, scale = 0.1
#define VAC_DOWN_SPEED_CUT_OFF                  140                             // %, scale = 0.1
#define VAC_DOWN_SPEED_MAX                      960                             // %, scale = 0.1

#define VAC_DRAFT_SET_MIN                       (uint16_t)((float)0.80/(float)0.00488)
#define VAC_DRAFT_SET_MAX                       (uint16_t)((float)1.10/(float)0.00488)
#define VAC_DRAFT_SENSOR_MIN                    VAC_DRAFT_SET_MAX + 4
#define VAC_DRAFT_SENSOR_MAX                    (uint16_t)((float)3.00/(float)0.00488)

#define VAC_DEPTH_SET_MIN                       (uint16_t)((float)0.70/(float)0.00488)
#define VAC_DEPTH_SET_MAX                       (uint16_t)((float)1.00/(float)0.00488)
#define VAC_DEPTH_UP_LIMIT                      (uint16_t)((float)3.50/(float)0.00488)

//#define VAC_LIFT_UP_MODE_LIMIT_ANGLE            220
//#define VAC_LIFT_DEADBAND_ANGLE_2               2
//#define VAC_LIFT_DEADBAND_ANGLE_3               12
//#define VAC_LIFT_FLOATING                       15                              // changed on 2021.12.14 -- 10% of the positions sensor

#define VAC_LIFT_UP_MODE_LIMIT_ANGLE            880  
#define VAC_LIFT_DEADBAND_ANGLE_2               8
#define VAC_LIFT_DEADBAND_ANGLE_3               48
#define VAC_LIFT_FLOATING                       60                              // changed on 2025.02.19

#define VAC_LIFT_COUNT_MAX                      980                             // 98% (uint8_t)((float)4.90/(float)0.0196)
  
typedef union {
  uint32_t data;
  struct {
    uint32_t settingMode                : 3;
    uint32_t positionSettingSequence    : 3;
    uint32_t leverThreePSettingMode     : 3;
    
    uint32_t res1                       : 23;
  };
} flagHitch_t;

extern flagHitch_t flagHitch;



uint16_t updateThreePPositionSensor(uint16_t _rawData);
uint16_t updateLeverSensor(uint16_t _rawData);

void lift_init();
void lift_control_process(void);
void hitch_setting_process(void);
void hitch_setting_lever_process(void);
void hitch_pwm_off_out(void);

uint16_t get_threePDownDuty();
uint16_t get_threePUpDuty();

void lever_rate_calculate();
void position_rate_calculate();

#ifdef __cplusplus
}
#endif

#endif /* __HITCH_H */
