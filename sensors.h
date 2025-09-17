
/*
 *
 * File Name    : sensors.h
 * Program      : sensors are calculated in here
 * Author       : Enkhbat
 * Company      : Kiwon Electronics
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORS_H
#define __SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Defines -------------------------------------------------------------------*/

/* Defines Analog inputs -----------------------------------------------------*/
//#define VA_BALANCE_ANALOG_SENSOR_USED                   TRUE
#define VA_BALANCE_CAN_SENSOR_USED                      TRUE

/* Defines Digital inputs ----------------------------------------------------*/
// Based on the pressure sensor datasheet
// The output is 0.5V ~ 4.5V (0 ~ 25Bar)
// 1023 <--> 5.0V

#define PRESSURE_SENSOR_ADC_MAX         928                                     // it is changed on 2022.04.13
#define PRESSURE_SENSOR_ADC_MIN         80                                      // it is changed on 2022.04.13
#define PRESSURE_SENSOR_BAR_MAX         250                                     // scale is 0.1
#define PRESSURE_SENSOR_BAR_MIN         0
  
#define VAC_SENSOR_CHECK_TIME           1000                                    // ms
  
// 3.3V is 4095 
#define VAC_BATTERY_8V                  1741
#define VAC_BATTERY_14V                 3047
#define VAC_BATTERY_18V                 3900

#define VAC_SENSOR_SHORT                (uint16_t)((float)4.90/(float)0.00488)  // 1004 --> 
#define VAC_SENSOR_OPEN                 (uint16_t)((float)0.10/(float)0.00488)  // 20
  
#define VAC_SHUTTLE_FORWARD_POSITION    (uint16_t)((float)3.5/(float)0.00488)   // 717
#define VAC_SHUTTLE_BACKWARD_POSITION   (uint16_t)((float)1.4/(float)0.00488)   // 286

/* --------------------------------------------------------- ADC definitions -------------------------------------------------------*/
//#define VAC_HAND_ACCEL_ERR_MIN          (float)(0.25f / (float)0.0196 )      // 0.25 * 621 = 155
// The following voltages are changed because the voltage is NOT matched when testing with multimeter
/*
  0.25V --> 0.17V
  0.50V --> 0.42V
*/
#define VAC_FOOT_ACCEL_ERR_MIN          (uint16_t)((float)0.17 / (float)0.00488 )      // 0.25 * 621 = 155
#define VAC_FOOT_ACCEL_ERR_MAX          (uint16_t)((float)4.80 / (float)0.00488 )      // 4.80 * 621 = 2980
    
#define VAC_FOOT_ACCEL_ERR_IVS          (uint16_t)((float)1.15 / (float)0.00488 )      // 1.15 * 621 = 714

typedef union {
  uint8_t data[8];
  struct {
    // 0
    uint8_t backwardValve       : 1;                                            // BIT 0
    uint8_t forwardValve        : 1;                                            // BIT 1
    uint8_t threePDown          : 1;                                            // BIT 2
    uint8_t threePUp            : 1;                                            // BIT 3
    uint8_t pto                 : 1;                                            // BIT 4
    uint8_t fourWD              : 1;                                            // BIT 5
    uint8_t balanceDownValve    : 1;                                            // BIT 6
    uint8_t balanceUpValve      : 1;                                            // BIT 7
    // 1
    uint8_t topLinkDownValve    : 1;                                            // BIT 0
    uint8_t topLinkUpValve      : 1;                                            // BIT 1
    uint8_t reserved3           : 1;                                            // BIT 2
    uint8_t regenValve          : 1;                                            // BIT 3
    uint8_t res1                : 4;                                            // BIT 4
    // 2
    uint8_t footPedal           : 1;
    uint8_t handPedal           : 1;
    uint8_t threePLever         : 1;
    uint8_t threePPosition      : 1;
    uint8_t threePDraft         : 1;
    uint8_t res2                : 3;
    // 3
    uint8_t shuttle             : 1;
    uint8_t clutch              : 1;
    uint8_t temperature         : 1;
    uint8_t res3                : 1;
    uint8_t backwardPressure    : 1;
    uint8_t forwardPressure     : 1;
    uint8_t battery             : 1;
    uint8_t res8                : 1;
    // 4,5,6,7
    
    uint8_t stroke              : 1;
    uint8_t balance             : 1;
    uint8_t threePDepth         : 1;
      
    uint8_t res4                : 8;
    uint8_t res5                : 8;
    uint8_t res6                : 8;
    uint8_t res7                : 8;
  } ;
} flagErrorSensors_t;

typedef union {
  uint32_t data;
  struct {
    uint32_t forward            : 1;
    uint32_t backward           : 1;
    uint32_t neutral            : 1;
    
    uint32_t res                : 29;
  } ;
} flagShuttle_t;

extern flagErrorSensors_t       flagErrorSensors;
extern flagErrorSensors_t       flagErrorSensorsHigh;
extern flagErrorSensors_t       flagErrorSensorsLow;
extern flagShuttle_t            flagShuttle;
extern flagShuttle_t            flagShuttlePrevious;


void lever_rate_calculate(void);
void position_rate_calculate(void);


void digital_sensors();
void analog_sensors();
void check_sensors_error();

uint16_t get_clutch_sensor();
uint16_t get_shuttle_sensor();
uint16_t get_threeP_lever_sensor();
uint16_t get_foot_accelerator();

uint8_t get_hand_accelerator();

uint16_t get_oil_temperature();
uint16_t get_threeP_draft_sensor();
uint16_t get_threeP_position_sensor();
uint16_t get_battery_power();
uint16_t get_sensor_power();
uint16_t get_forward_pressure();
uint16_t get_backward_pressure();
uint16_t get_offset_battery();

uint16_t get_rawThreePLeverSensor();
uint16_t get_rawThreePPositionSensor();
uint16_t get_threeP_depth_sensor();
uint16_t get_average_speed_hz();
float    get_average_speed();

uint16_t get_clutch_sensor_average();
uint16_t get_threeP_lever_sensor_average();

uint16_t get_steering_sensor();
uint16_t get_stroke_sensor();
uint16_t get_balance_sensor();
uint16_t get_motor_position();

#ifdef __cplusplus
}
#endif

#endif /* __SENSORS_H */
