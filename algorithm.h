/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/
  
#define BIT_NV_TURN_UP_MODE             0x0001
#define BIT_NV_BACK_UP_MODE             0x0002
#define BIT_NV_TURN_DOWN_MODE           0x0004
#define BIT_NV_BACK_DOWN_MODE           0x0008
#define BIT_NV_4WD_MANUAL_MODE          0x0010
#define BIT_NV_4WD_AUTO_MODE            0x0020
  
  
#define VAC_4WD_MODE_OFF                0
#define VAC_4WD_MANUAL_MODE             1
#define VAC_4WD_AUTO_MODE               2

#define VAC_STARTER_ON_DELAY            500
  
#define VAC_4WD_QT_DELAY                500
  
#define VAC_WHEEL_LAMP_CYCLE            800                                     // ms
#define VAC_WHEEL_LAMP_ON               400                                     // ms
  
typedef union {
  // 48 flag
  uint8_t data[7];
  struct {
    uint32_t oneUpRun           : 1;
    uint32_t oneDownRun         : 1;
    uint32_t powerOn            : 1;
    uint32_t parkStop           : 1;
    uint32_t liftLock           : 1;
    uint32_t engineRun          : 1;
    uint32_t rpmCruiseA         : 1;
    uint32_t rpmCruiseB         : 1;
    
    uint32_t checkBuzzer        : 1;
    uint32_t buzzerOn           : 1;
    uint32_t ngBuzzer           : 1;
    uint32_t liftDither         : 1;                                            // Hitch control
    uint32_t quickMove          : 1;
    uint32_t floating           : 1;
    uint32_t turnUpRun          : 1;
    uint32_t backUpRun          : 1;
    
    uint32_t armSetConnected    : 1;
    uint32_t draftRun           : 1;
    uint32_t ditherOn           : 1;                                            // Shuttle control
    uint32_t fillOn             : 1;                                            // Shuttle control
    uint32_t isFillFirst        : 1;                                            // Shuttle control
    uint32_t forwardUpdate      : 1;                                            // Shuttle control
    uint32_t backwardUpdate     : 1;                                            // Shuttle control
    uint32_t pedalPush          : 1;                                            // Shuttle control
    
    uint32_t pedalFree          : 1;                                            // Shuttle control
    uint32_t pedalDelay         : 1;                                            // Shuttle control
    uint32_t pedalControl       : 1;                                            // Shuttle control
    uint32_t pedalShuttle       : 1;                                            // Shuttle control
    uint32_t pedalError         : 1;                                            // Shuttle control
    uint32_t speedCompensation  : 1;                                            // Shuttle control
    uint32_t transmissionChange : 1;                                            // Shuttle control
    uint32_t warningTransmission: 1;                                            // Shuttle control
    
    uint32_t shuttleSwError     : 1;                                            // Shuttle control
    uint32_t adMotorError       : 1;
    uint32_t autoBreakError     : 1;
    uint32_t autoBreakNeutral   : 1;
    uint32_t forwardSearchRun   : 1;
    uint32_t backwardSearchRun  : 1;
    uint32_t qtDisable          : 1;
    uint32_t qtDelay            : 1;
    
    uint32_t wheelSpeedLimit    : 1;
    uint32_t diffDelay          : 1;
    uint32_t fillImplement      : 1;
    uint32_t leverUpRun         : 1;                                            // Added on 2021.12.08
    uint32_t leverDownRun       : 1;                                            // Added on 2021.12.10
    uint32_t ptoLiftLock        : 1;                                            // Added on 2022.09.22
    uint32_t isSteeringON       : 1;                                            // Added on 2025.01.23
    uint32_t isEUModel          : 1;
    
    uint32_t depthOnRun         : 1;
    uint32_t buzzerButton       : 1;
    uint32_t turnDownRpmRun     : 1;
    uint32_t backDownRpmRun     : 1;
    uint32_t rpmCruiseAuto      : 1;
    
    uint32_t steeringRight      : 1;
    uint32_t steeringLeft       : 1;
    uint32_t parkingLamp        : 1;
  } ;
} flag_t;

extern flag_t flag;
extern flag_t flagPrevious;



void control_init();
uint8_t control_previous_process();
void control_process();



uint8_t check_model();
void buzzer_control_process();
void starter_control_process();
void drive_process();





void safety_control_process();

#ifdef __cplusplus
}
#endif

#endif /* __ALGORITHM_H */
