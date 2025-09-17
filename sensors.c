

#include "sensors.h"
#include "adc.h"
#include "input.h"
#include "output.h"
#include "main.h"
#include "tic12400.h"
#include "can.h"
#include "hitch.h"
#include "algorithm.h"
#include "settings.h"
#include "settings_dtc.h"

// X750 - 3p hitch (UP/DOWN), balance (UP/DOWN), pto (ON/OFF) - MAP absent
// X780 - 3p hitch (UP/DOWN), balance (UP/DOWN), pto (ON/OFF) - We have maps --> map data default setting --> ( get from TYM )
// X780 - Implementation of the instrumental panel is same as X750


// backward switch is replaced with CHARGE input --> backward buzzer
// 00 -> ADC RAW data and calculated data should be transmitted to diagnostic program
// 00 -> Temperature sensor is changed on 2021.10.12 so the program should be rewritten again.

// 0 -> Transmission shift CAN protocol                                         // 
// 1 -> Transmission shift                                                      // map setting menu is finished and algorithm is not finished --> 70%
// 2 -> Dialog sensitive setting                                                // 50%                  
// 3- > Back buzzer                                                             // 50%
// 4 -> Check buzzer                                                            // 50%
// 5 -> PTO auto mode                                                           // 70% --> 


// 6 -> 3P auto mode (one touch up, down, 3p up down)                           // 50%
// 7 -> Balance auto mode                                                       // 70%
// 8 -> 4WD, 2WD, QT driver functions                                           // 50%

// 9 -> MAP number settings (constant and range )                               // 100%

// 10-> Default setting map numbers, map data                                   // It should be made by TYM

// 11 -> Test program routine time                                              // one time cycle test

//#define HELLA_TEMPERATURE_SENSOR           1
#define HYDAC_TEMPERATURE_SENSOR           1

#if defined(HELLA_TEMPERATURE_SENSOR)

#define HARDWARE_VOLTAGE_DIVIDER_RESISTOR_10000         1
//#define HARDWARE_VOLTAGE_DIVIDER_RESISTOR_4700          1
#define OIL_TEMPERATURE_OFFSET                          25
#define NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS            30
//const float oilTemperatureData[NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS] = { -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120 };
const float oilTemperatureData[NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS] = { -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120 };

#if defined(HARDWARE_VOLTAGE_DIVIDER_RESISTOR_10000)
const float oilTemperatureADCData[NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS] = { 3.7294, 3.4303, 3.1074, 2.7734, 2.4393, 2.1182, 1.8183, 1.5467, 1.3061, 1.0971, 0.9184, 0.7674, 0.6408, 0.5349, 0.4475, 0.3751, 0.3148, 0.2652, 0.2235, 0.1895, 0.1611, 0.1376, 0.1177, 0.1009, 0.0870, 0.0749, 0.0651, 0.0564, 0.0495, 0.0431 };
//  const float oilTemperatureADCData[NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS] = { 4628, 4257, 3856, 3442, 3027, 2628, 2256, 1919, 1621, 1361, 1140, 952, 795, 664, 555, 465, 391, 329, 277, 235, 200, 171, 146, 125, 108, 93, 81, 70, 61, 54 };
//#elif defined(HARDWARE_VOLTAGE_DIVIDER_RESISTOR_4700)
//  const float oilTemperatureADCData[NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS] = { 5348, 5106, 4824, 4505, 4155, 3785, 3405, 3028, 2664, 2322, 2009, 1727, 1478, 1260, 1073, 913, 776, 661, 562, 480, 411, 352, 303, 261, 225, 194, 169, 147, 129, 113};
#endif

#endif

//#define TRACTOR_T68_40KM        1
//#define TRACTOR_T68_30KM        1
#define TRACTOR_T78_40KM        1
//#define TRACTOR_T78_30KM        1

// Based on the data, the general equation of T68-30/40 is Y(km/h) = 0.018 * x(Hz) --> Y = ax
// Based on the data, the general equation of T78-30/40 is Y(Km/h) = 0.020 * x(Hz) --> Y = ax

// The following equation is improved
// Based on the data, the general equation of T68-30/40 is Y(km/h) = 0.018 * x(Hz) + 0.0011 --> Y = ax + b
// Based on the data, the general equation of T78-30/40 is Y(Km/h) = 0.020 * x(Hz) + 0.0011 --> Y = ax + b

#if defined(TRACTOR_T68_40KM) || defined(TRACTOR_T68_30KM)
//const float generalCoefficientOfHzToKmH = 0.018;
#define generalCoefficientOfHzToKmH(x)    ((0.018 * (float)x) + 0.0000)
#elif defined(TRACTOR_T78_40KM) || defined(TRACTOR_T78_30KM)
//const float generalCoefficientOfHzToKmH = 0.020;
#define generalCoefficientOfHzToKmH(x)    ((0.020 * (float)x) + 0.0000)
#endif

#if defined(TRACTOR_T68_40KM)
// Tractor speed sensor
#define NUMBER_OF_TRACTOR_SPEED_TABLE           24
const float speedSensorInKmh[NUMBER_OF_TRACTOR_SPEED_TABLE] = { 0.39, 0.57, 0.82, 1.09, 1.24, 1.28, 1.82, 1.89, 2.61, 2.71, 3.48, 3.62, 3.95, 4.10, 5.82, 6.04, 8.35, 8.66, 11.13, 11.55, 13.13, 19.33, 27.73, 36.97 };
const float speedSensorInHz[NUMBER_OF_TRACTOR_SPEED_TABLE] = { 20.93, 30.80, 44.19, 58.92, 66.87, 69.53, 98.39, 102.31, 141.16, 146.79, 188.22, 195.72, 213.97, 222.12, 314.84, 326.82, 451.72, 468.91, 602.3, 625.21, 710.77, 1045.81, 1500.51, 2000.68 };
#elif defined(TRACTOR_T68_30KM)
// Tractor speed sensor
#define NUMBER_OF_TRACTOR_SPEED_TABLE           24
const float speedSensorInKmh[NUMBER_OF_TRACTOR_SPEED_TABLE] = { 0.32, 0.47, 0.68, 0.90, 1.02, 1.07, 1.51, 1.57, 2.16, 2.25, 2.88, 3.00, 3.28, 3.40, 4.82, 5.01, 6.92, 7.18, 9.23, 9.58, 10.89, 16.02, 22.99, 30.65 };
const float speedSensorInHz[NUMBER_OF_TRACTOR_SPEED_TABLE] = { 17.35, 25.54, 36.64, 48.85, 55.44, 57.65, 81.57, 84.82, 117.04, 121.70, 156.05, 162.27, 177.4, 184.15, 261.03, 270.96, 374.51, 388.76, 499.35, 518.35, 589.29, 867.06, 1244.05, 1658.73 };
#elif defined(TRACTOR_T78_40KM)
// Tractor speed sensor
#define NUMBER_OF_TRACTOR_SPEED_TABLE           16
const float speedSensorInKmh[NUMBER_OF_TRACTOR_SPEED_TABLE] = { 0.39, 0.56, 0.81, 1.08, 1.29, 1.87, 2.68, 3.58, 4.12, 5.98, 8.55, 11.45, 13.17, 19.12, 27.36, 36.64 };
const float speedSensorInHz[NUMBER_OF_TRACTOR_SPEED_TABLE] = { 18.74, 27.20, 38.91, 52.12, 62.24, 90.34, 129.26, 173.13, 198.81, 288.59, 412.91, 553.04, 636.18, 923.49, 1321.3, 1769.74 };
#elif defined(TRACTOR_T78_30KM)
// Tractor speed sensor
#define NUMBER_OF_TRACTOR_SPEED_TABLE           16
const float speedSensorInKmh[NUMBER_OF_TRACTOR_SPEED_TABLE] = { 0.32, 0.47, 0.67, 0.89, 1.07, 1.55, 2.22, 2.97, 3.41, 4.95, 7.09, 9.49, 10.92, 15.85, 22.68, 30.38 };
const float speedSensorInHz[NUMBER_OF_TRACTOR_SPEED_TABLE] = { 15.53, 22.55, 32.26, 43.21, 51.60, 74.90, 107.17, 143.54, 164.83, 239.27, 342.33, 458.52, 527.45, 765.65, 1095.47, 1467.26 };
#endif

flagErrorSensors_t      flagErrorSensorsHigh;
flagErrorSensors_t      flagErrorSensorsLow;
flagErrorSensors_t      flagErrorSensors;                                       // If a sensor has an error, the corresponding flag should be ON
flagShuttle_t           flagShuttle;                                            // Curerntly, the forward, backward, and neutral is measured by VR so it is converted to on/off flag and saved in here
flagShuttle_t           flagShuttlePrevious;

float                   averageSpeedHz;                                         // The final real speed (Hz) is calculated in here
float                   averageSpeed;                                           // The final real speed (Km/h) is calculated in here

#if defined(VA_BALANCE_CAN_SENSOR_USED)
int16_t                 rawBalanceSensorCAN;
#endif

uint16_t                rawBalanceSensor;                                       // The calculated data with only sensor power 
uint16_t                rawSteeringSensor;
uint16_t                rawClutchSensor;
uint16_t                rawShuttleSensor;
uint16_t                rawThreePLeverSensor;
uint16_t                rawFootAccelerator;

uint16_t                rawOilTemperature;
uint16_t                rawThreePDraftSensor;
uint16_t                rawThreePPositionSensor;
uint16_t                rawBatteryPower;
uint16_t                rawSensorPower;
uint16_t                rawForwardPressure;
uint16_t                rawBackwardPressure;
uint16_t                rawADMotorPosition;
uint16_t                rawStrokeSensor;
uint16_t                rawThreePDepthSensor;

uint16_t                balanceSensor;                                          // The calculated value of sensors
uint16_t                steeringSensor;
uint16_t                clutchSensor;
uint16_t                shuttleSensor;
uint16_t                threePLeverSensor;
uint16_t                footAccelerator;

uint16_t                adMotorPosition;
uint16_t                strokeSensor;

uint16_t                oilTemperature;
uint16_t                threePDraftSensor;
uint16_t                threePPositionSensor;
uint16_t                threePDepthSensor;

uint16_t                batteryPower;
uint16_t                sensorPower;
uint16_t                forwardPressure;
uint16_t                backwardPressure;
uint16_t                offsetBattery;

uint16_t averageClutchSensor;

uint8_t calculate_average_clutch() {                                            // Every 10 ms, the new data is saved
  static uint32_t totalClutchSensor = 0;
  static uint32_t counterClutchSensor = 0;
  if(flagTimer.tenMs == FALSE)
  {
    return 0;
  }
  totalClutchSensor += clutchSensor;
  counterClutchSensor++;
  
  if(counterClutchSensor >= 10) {                                               // Every 100ms, the averaged data is updated.
//    averageClutchSensor = totalClutchSensor / counterClutchSensor;
    totalClutchSensor = 0;
    counterClutchSensor = 0;
    
    averageClutchSensor = clutchSensor;                                         // There is no calculation is needed for additional average
  }
  return 1;
}

uint16_t threePLeverSensorAverage;

uint8_t calculate_average_3p_lever_sensor() {                                            // Every 10 ms, the new data is saved
  static uint32_t totalLeverSensor = 0;
  static uint32_t counterLeverSensor = 0;
  if(flagTimer.tenMs == FALSE)
  {
    return 0;
  }
  totalLeverSensor += threePLeverSensor;
  counterLeverSensor++;
  
  if(counterLeverSensor >= 10) {                                               // Every 100ms, the averaged data is updated.
    threePLeverSensorAverage = totalLeverSensor / counterLeverSensor;
    totalLeverSensor = 0;
    counterLeverSensor = 0;
  }
  return 1;
}


uint16_t get_average_speed_hz()         { return (uint16_t)averageSpeedHz;      }
float    get_average_speed()            { return averageSpeed * 10;             }               // Changed on 2023.03.31

uint16_t get_balance_sensor()           { return balanceSensor;                 }

uint16_t get_clutch_sensor()            { return clutchSensor;                  }
uint16_t get_clutch_sensor_average()    { return averageClutchSensor;           }
uint16_t get_shuttle_sensor()           { return shuttleSensor;                 }
uint16_t get_threeP_lever_sensor()      { return threePLeverSensor;             }
uint16_t get_threeP_lever_sensor_average()      { return threePLeverSensorAverage;             }
uint16_t get_foot_accelerator()         { return footAccelerator;               }

uint8_t get_hand_accelerator()         { return canRxArmRest.acceleratorPosition;               }

uint16_t get_oil_temperature()          { return oilTemperature;                }
uint16_t get_threeP_draft_sensor()      { return threePDraftSensor;             }
uint16_t get_threeP_depth_sensor()      { return threePDepthSensor;             }
uint16_t get_threeP_position_sensor()   { return threePPositionSensor;          }
uint16_t get_battery_power()            { return batteryPower;                  }
uint16_t get_sensor_power()             { return sensorPower;                   }
uint16_t get_forward_pressure()         { return forwardPressure;               }
uint16_t get_backward_pressure()        { return backwardPressure;              }
uint16_t get_offset_battery()           { return offsetBattery;                 }

uint16_t get_rawThreePLeverSensor()     { return rawThreePLeverSensor;          }
uint16_t get_rawThreePPositionSensor()  { return rawThreePPositionSensor;       }

uint16_t get_steering_sensor()          { return steeringSensor;                }
uint16_t get_stroke_sensor()            { return strokeSensor;                  }
uint16_t get_motor_position()           { return adMotorPosition;               }

void analog_sensors(void)
{
  static uint8_t oilCounter = 0;
  static float rawOilTemperatureTotal = 0;
  float rate;
#if defined(VA_BALANCE_CAN_SENSOR_USED)
  float rollingSensor;
#endif
  
#if defined(HELLA_TEMPERATURE_SENSOR)
  uint8_t i;
  float tempCalc;
#elif defined(HYDAC_TEMPERATURE_SENSOR)
  float tempCalc;
#endif
// Disabled on 2025.07.30  float steeringAngle;
  
  updateLastAverageADC();                                                                  // If DMA is finished, then update ADC buffers
  
  if(VAC_AD_SENSOR_POWER_LOW >= sensorPower) {                                  // 센서 전압 4.5V이하.
    rate = VAC_AD_DEFAULT_RATE; 
  }
  else if(VAC_AD_SENSOR_POWER_HIGH <= sensorPower) {                            // 센서 전압 5.5V이상.
    rate = VAC_AD_DEFAULT_RATE; 
  }
  else {
    rate = (float)sensorPower / (float)VAC_AD_CONVER; 
  }

  rawClutchSensor               = getRawClutchLevel();
  rawShuttleSensor              = getRawShuttleLever();
  
  rawFootAccelerator            = getRawFootAccelerator();
  rawSteeringSensor             = getRawSteering();
  rawForwardPressure            = getRawForwardPressure();
  rawBackwardPressure           = getRawBackwardPressure();
  
  rawThreePLeverSensor          = ((canRxArmRest.hitchLeverMSB << 8) + canRxArmRest.hitchLeverLSB);     // 1000                     // getRaw3pLever();
  rawThreePPositionSensor       = getRawHitchPosition();
  rawThreePDraftSensor          = getRawHitchDraft();
  rawThreePDepthSensor          = getRawHitchDepth();
  
  rawADMotorPosition            = getRawMotorPosition();
  rawStrokeSensor               = getRawStrokeSensor();
  
  rawBatteryPower               = getRawBatteryVoltage();
  rawSensorPower                = getRawSensorVoltage();
  
  
  if(flagTimer.tenMs == TRUE)
  {
    rawOilTemperature           = getRawOilTemperature();
    rawOilTemperature           = (uint16_t)((float)rawOilTemperature / rate);
    rawOilTemperature           = (rawOilTemperature >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawOilTemperature;             // Between 0 and 1023
    rawOilTemperatureTotal     += rawOilTemperature;
    oilCounter++;
  }
  
  rawClutchSensor               = (uint16_t)((float)rawClutchSensor / rate);
  rawClutchSensor               = (rawClutchSensor >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawClutchSensor;
  rawClutchSensor               = VAC_AD_CONVER - rawClutchSensor;
  clutchSensor                  = rawClutchSensor;                                                                      // Between 0 and 1023           --> Changed on 2023.03.31
  
    
  calculate_average_clutch();                                                                                           // The clutch average is calculated in here.
    
  rawSteeringSensor             = (uint16_t)((float)rawSteeringSensor / rate);
  rawSteeringSensor             = (rawSteeringSensor >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawSteeringSensor;
  steeringSensor                = rawSteeringSensor;                                                                     // Between 0 and 1023           --> Changed on 2023.03.31
  
//  flag.steeringRight = FALSE;
//  flag.steeringLeft = FALSE;
  // 25 - 512, 
  // 235 ==> 340, 860 ==> 785
  /*
  steeringAngle = ((float)(steeringSensor - steeringOffset) * 0.25 * (float)0.431) - 55.0;
  if((steeringAngle <= -30) || (steeringAngle >= 30))
  {
    flag.isSteeringON = TRUE;
    if(steeringAngle <= -30)
      flag.steeringLeft = ON;
    else
      flag.steeringRight = ON;
  }
  else
  {
    flag.isSteeringON = FALSE;
  }
  */
  
  if(flag.isSteeringON == FALSE)
  {
    if(steeringSensor > nvSteeringRight)
    {
      flag.steeringRight = TRUE;
      flag.isSteeringON = TRUE;
    }
    else if(steeringSensor < nvSteeringLeft)
    {
      flag.steeringLeft = TRUE;
      flag.isSteeringON = TRUE;
    }
  }
  else
  {
    // steering on
    if(flag.steeringRight == TRUE)
    {
      if(steeringSensor < (nvSteeringRight - 20))
      {
        flag.steeringRight = FALSE;
        flag.isSteeringON = FALSE;
      }
    }
    
    if(flag.steeringLeft == TRUE)
    {
      if(steeringSensor > (nvSteeringLeft + 20))
      {
        flag.steeringLeft = FALSE;
        flag.isSteeringON = FALSE;
      }
    }
  }
  
  rawShuttleSensor              = (uint16_t)((float)rawShuttleSensor / rate);
  rawShuttleSensor              = (rawShuttleSensor >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawShuttleSensor;
  shuttleSensor                 = rawShuttleSensor;                                                                     // Between 0 and 1023           --> Changed on 2023.03.31
  
  threePLeverSensor             = updateLeverSensor(rawThreePLeverSensor);                                              // Between 0 and 1000
  
  calculate_average_3p_lever_sensor();

  rawFootAccelerator            = (uint16_t)((float)rawFootAccelerator / rate);
  rawFootAccelerator            = (rawFootAccelerator >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawFootAccelerator;
  footAccelerator               = rawFootAccelerator;                                                                   // Between 0 and 1023

  rawADMotorPosition            = (uint16_t)((float)rawADMotorPosition / rate);
  rawADMotorPosition            = (rawADMotorPosition >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawADMotorPosition;
  adMotorPosition               = rawADMotorPosition;                                                                   // Between 0 and 1023
  
  rawStrokeSensor               = (uint16_t)((float)rawStrokeSensor / rate);
  rawStrokeSensor               = (rawStrokeSensor >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawStrokeSensor;
  strokeSensor                  = rawStrokeSensor;                                                                      // Between 0 and 1023
  
#if defined(VA_BALANCE_ANALOG_SENSOR_USED)
  rawBalanceSensor              = getRawBalanceRolling();
  rawBalanceSensor              = (uint16_t)((float)rawBalanceSensor / rate);
  rawBalanceSensor              = (rawBalanceSensor >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawBalanceSensor;
  balanceSensor                 = rawBalanceSensor;
#elif defined(VA_BALANCE_CAN_SENSOR_USED)
  // This above part disabled due to rolling sensor data is received from CAN bus
  rawBalanceSensorCAN           = (int16_t)((canRxAngle.rollAngleMSB << 8) + canRxAngle.rollAngleLSB);
  rollingSensor                 = ((float)rawBalanceSensorCAN * 0.01) - 90;
  
  if(rollingSensor >= 15.0) {
    balanceSensor = 1023;
  }
  else if(rollingSensor <= -15.0) {
    balanceSensor = 0;
  }
  else {
    // Between -15 and 15
    balanceSensor = (uint16_t)(((15.0 + rollingSensor) * 1023.0) / 30.0);
  }
#endif
  
  if(oilCounter >= 50) {                                                        // Oil temperature update time 50*10 = 500ms
    rawOilTemperatureTotal /= oilCounter;
    oilCounter = 0;
    
    #if defined(HELLA_TEMPERATURE_SENSOR)
      // 320, 0.7V,  32C
      tempCalc                      = (((float)rawOilTemperatureTotal * 5.0) / (float)VAC_AD_CONVER) / 2.0;
      
      if(flagTimer.oneSecond == TRUE) {
        for(i = 0; i < NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS; i++) {
          if((oilTemperatureADCData[i]) < tempCalc) {
            if(i == 0) {
              oilTemperature = (uint8_t)(oilTemperatureData[0] + OIL_TEMPERATURE_OFFSET);                           // +25C so if the value is 0 it is -25C
            }else {
              oilTemperature = (uint8_t)((oilTemperatureData[i]) + ((oilTemperatureData[i] - oilTemperatureData[i - 1]) * 
                                                                (tempCalc - (oilTemperatureADCData[i])) / ((oilTemperatureADCData[i]) - (oilTemperatureADCData[i - 1]))) + OIL_TEMPERATURE_OFFSET);
            }
            break;
          }
        }
        
        if(i == NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS) {
          oilTemperature = (uint8_t)(oilTemperatureData[NUMBER_OF_ADC_OIL_TEMPERATURE_POINTS - 1] + OIL_TEMPERATURE_OFFSET);                           // 120C
        }
      }
    #elif defined(HYDAC_TEMPERATURE_SENSOR)
      // -25C to 125C temperature sensor between 0 and 5V, (ADC between 0 and 1023)
      // 5V --> 2.5V --> Hardware
      // 1023 <--> 3.3V
      // X    <--> 2.5V --> X = 775
      // 775 --> 125C
      // 0   --> -25C
      // The maximum value is 775
      
      // 2025.02.20
      // 1K pull up with 5V, therefore BOM should be changed next time
      // MIN = 7, MAX = 1017 on testing device = 1017
      
      //printf("rawOilTemperatureTotal = %.0f\r\n", rawOilTemperatureTotal);
      
      //tempCalc = ((float)rawOilTemperatureTotal / (float)VAC_AD_CONVER);
      tempCalc = ((float)rawOilTemperatureTotal / (float)775.0);
      oilTemperature = (int16_t)(tempCalc * 150.0);
      
      // Removed on 2025.06.11 tempCalc = ((float)rawOilTemperatureTotal / (float)1010.0);                                       // Enabled on 2025.05.27
      
      // y = 0.0001x2 + 0.0451x - 0.5771 --> This equation is added because of 1K om hardware; 
      // We do NOT need this equation for mass product, we should R131 resistor to 10K
      // =-0.00000003*C6*C6*C6+0.0002*C6*C6 + 0.0251*C6

      //tempCalc = 0.0001 * (float)rawOilTemperatureTotal * (float)rawOilTemperatureTotal + 0.0451 * (float)rawOilTemperatureTotal;
      //tempCalc = -0.00000003 * (float)rawOilTemperatureTotal * (float)rawOilTemperatureTotal * (float)rawOilTemperatureTotal
      //  + 0.0002 * (float)rawOilTemperatureTotal * (float)rawOilTemperatureTotal + 0.0251 * (float)rawOilTemperatureTotal;
        
      // Removed on 2025.06.11 oilTemperature = (int16_t)(tempCalc);
    #endif
      
    rawOilTemperatureTotal = 0;                                                               // Added on 2021.11.28
  }
  
  rawThreePDraftSensor          = (uint16_t)((float)rawThreePDraftSensor/ rate);
  rawThreePDraftSensor          = (rawThreePDraftSensor >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawThreePDraftSensor;
  threePDraftSensor             = rawThreePDraftSensor;                                                                 // Between 0 and 1023
  
  rawThreePDepthSensor          = (uint16_t)((float)rawThreePDepthSensor/ rate);
  rawThreePDepthSensor          = (rawThreePDepthSensor >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawThreePDepthSensor;
  threePDepthSensor             = rawThreePDepthSensor;                                                                 // Between 0 and 1023
  
  rawThreePPositionSensor       = (uint16_t)((float)rawThreePPositionSensor / rate);
  rawThreePPositionSensor       = (rawThreePPositionSensor >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawThreePPositionSensor;
  threePPositionSensor          = updateThreePPositionSensor(rawThreePPositionSensor);                                  // Between 0 and 1023
  
  batteryPower                  = rawBatteryPower;                                                                      // Between 0 and 4096
  sensorPower                   = rawSensorPower;                                                                       // No need for calculation, because based on the raw data other sensor value is calculated
    
  rawForwardPressure            = (uint16_t)((float)rawForwardPressure / rate);
  rawForwardPressure            = (rawForwardPressure >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawForwardPressure;
  
  if(rawForwardPressure <= PRESSURE_SENSOR_ADC_MIN) {
    // the pressure is 0 bar
    forwardPressure = PRESSURE_SENSOR_BAR_MIN;
  }
  else if(rawForwardPressure >= PRESSURE_SENSOR_ADC_MAX) {
    forwardPressure = PRESSURE_SENSOR_BAR_MAX;
  }
  else {
    forwardPressure = (uint16_t)(((float)(rawForwardPressure - PRESSURE_SENSOR_ADC_MIN) / (float)(PRESSURE_SENSOR_ADC_MAX - PRESSURE_SENSOR_ADC_MIN)) * (float)(PRESSURE_SENSOR_BAR_MAX - PRESSURE_SENSOR_BAR_MIN));
  }
  // unit of forwardPressure sensor now shows bar and scale is 0.1
  
  rawBackwardPressure           = (uint16_t)((float)rawBackwardPressure / rate);
  rawBackwardPressure           = (rawBackwardPressure >= VAC_AD_CONVER) ? VAC_AD_CONVER : rawBackwardPressure;
  
  if(rawBackwardPressure <= PRESSURE_SENSOR_ADC_MIN) {
    // the pressure is 0 bar
    backwardPressure = PRESSURE_SENSOR_BAR_MIN;
  }
  else if(rawBackwardPressure >= PRESSURE_SENSOR_ADC_MAX) {
    backwardPressure = PRESSURE_SENSOR_BAR_MAX;
  }
  else {
    backwardPressure = (uint16_t)(((float)(rawBackwardPressure - PRESSURE_SENSOR_ADC_MIN) / (float)(PRESSURE_SENSOR_ADC_MAX - PRESSURE_SENSOR_ADC_MIN)) * (float)(PRESSURE_SENSOR_BAR_MAX - PRESSURE_SENSOR_BAR_MIN));
  }
  // unit of backwardPressure sensor now shows bar and scale is 0.1  
}

void digital_sensors(void)
{
  static uint8_t  digitalInputTime = 0;
  static uint8_t  forwardSwitch;
  static uint8_t  neutralSwitch;
  static uint8_t  backwardSwitch;
   
  static uint8_t  timerSampling = 0;
  static uint8_t  rawSpeedCounter;
  static uint16_t rawSpeed;  

  
  // updated on 2021.12.13, the reading time is changed.
  digitalInputTime++;
  if(digitalInputTime >= 5) {                                                   // 5 * 2ms = 10ms
    digitalInputTime = 0;
    read_digital_inputs();                                                        // It is reading sensor, switch, buttons data on mcu pins, internally 8 points are checking, 10ms * 8 = 80ms
  }

  
  forwardSwitch  <<= 1;
  neutralSwitch  <<= 1;
  backwardSwitch <<= 1;

  flagShuttle.forward  = OFF;
  flagShuttle.backward = OFF;
  flagShuttle.neutral  = OFF;

  if((get_shuttle_sensor() > VAC_SENSOR_SHORT) || (get_shuttle_sensor() < VAC_SENSOR_OPEN)) {
    
    if(get_shuttle_sensor() > VAC_SENSOR_SHORT) 
      flagErrorSensorsHigh.shuttle = TRUE;
    else if(get_shuttle_sensor() < VAC_SENSOR_OPEN)
      flagErrorSensorsLow.shuttle = TRUE;
      
    flagShuttle.backward = OFF;
    flagShuttle.forward  = OFF;
    flagShuttle.neutral  = ON;
    
    flagErrorSensors.shuttle = TRUE;
    
    forwardSwitch = 0;
    backwardSwitch = 0;
    neutralSwitch = 0xFF;
  }
  else
  {
    flagErrorSensorsHigh.shuttle = FALSE;
    flagErrorSensorsLow.shuttle = FALSE;
    flagErrorSensors.shuttle = FALSE;                                                           // Added on 2021.12.06
    if(get_shuttle_sensor() <= VAC_SHUTTLE_BACKWARD_POSITION) {
      backwardSwitch++;
    }
    else if(get_shuttle_sensor() >= VAC_SHUTTLE_FORWARD_POSITION) {
      forwardSwitch++;
    }
    else {
      neutralSwitch++;
    }
  }
  
  if(forwardSwitch == 0xFF) {
    flagShuttle.forward = ON;
    flagOutputLamp.forward = ON;
  }
  if(neutralSwitch == 0xFF) {
    flagShuttle.neutral= ON;
    flagOutputLamp.neutral = ON;
  }
  if(backwardSwitch == 0xFF) {
    flagShuttle.backward = ON;
    flagOutputLamp.backward = ON;
  }
  
  if(flagTimer.tenMs == TRUE)
  {
    timerSampling += 10;
  }

  if(timerSampling >= 100)
  {
    timerSampling = 0;
   
    rawSpeed += get_speedCounter();
    rawSpeedCounter++;
    set_speedCounter(0);
   
    if(rawSpeedCounter >= 4){
      averageSpeedHz = (rawSpeed / rawSpeedCounter) * 10;                       // This is 
      averageSpeed = generalCoefficientOfHzToKmH(averageSpeedHz);
      rawSpeedCounter = 0;
      rawSpeed = 0;
    }
  }
}

void check_sensors_error()
{
  // Hand & foot pedal error is checked in "rpmController.c" source code
  // Shuttle sensor is checked in previous function
  //
  static uint16_t timerTemperature = 0;
  static uint16_t timerCarSpeed = 0;
  static uint16_t timerClutch = 0;
  static uint16_t timerLever = 0;
  static uint16_t timerPosition = 0;
  static uint16_t timerDraft = 0;
  static uint16_t timerForwardPressure = 0;
  static uint16_t timerBackwardPressure = 0;
  static uint16_t timerBatteryLow= 0;
  static uint16_t timerBatteryHigh = 0;
  static uint16_t timerFootPedalIVS = 0;
  static uint16_t timerFootPedal = 0;
  
  if(flagTimer.tenMs == FALSE)
  {
    return;
  }
  
  timerTemperature += 10;
  timerCarSpeed += 10;
  timerClutch += 10;
  timerLever += 10;
  timerPosition += 10;
  timerDraft += 10;
  timerForwardPressure += 10;
  timerBackwardPressure += 10;
  timerBatteryLow += 10;
  timerBatteryHigh += 10;
  timerFootPedalIVS += 10;
  timerFootPedal += 10;

  // Foot pedal sensor error is checking with IVS switch and HIGH, LOW error
  if((flagInputStatus.footPedal == ON) && (footAccelerator > VAC_FOOT_ACCEL_ERR_IVS))
  {
    if(VAC_SENSOR_CHECK_TIME <= timerFootPedalIVS) {
      timerFootPedalIVS = VAC_SENSOR_CHECK_TIME;
      flagErrorSensors.footPedal = TRUE;
    }
  }
  else {
    timerFootPedalIVS = 0;
    if((footAccelerator <= VAC_FOOT_ACCEL_ERR_MIN) || (footAccelerator >= VAC_FOOT_ACCEL_ERR_MAX))
    {
      if(footAccelerator <= VAC_FOOT_ACCEL_ERR_MIN)
        flagErrorSensorsLow.footPedal = TRUE;
      else if(footAccelerator >= VAC_FOOT_ACCEL_ERR_MAX)
        flagErrorSensorsHigh.footPedal = TRUE;
    
      if(VAC_SENSOR_CHECK_TIME <= timerFootPedal) {
        timerFootPedal = VAC_SENSOR_CHECK_TIME;
        flagErrorSensors.footPedal = TRUE;
      }
    }
    else {
      timerFootPedal = 0;
      flagErrorSensors.footPedal = FALSE;
      flagErrorSensorsLow.footPedal = FALSE;
      flagErrorSensorsHigh.footPedal = FALSE;
    }
  }
  
  // Hand pedal sensor
  if(get_hand_accelerator() == 0xFE)
  {
    flagErrorSensors.handPedal = TRUE;
  }
  else
  {
    flagErrorSensors.handPedal = FALSE;
  }
  
  // Temperature sensor
  if((rawOilTemperature > VAC_SENSOR_SHORT) || (rawOilTemperature < VAC_SENSOR_OPEN)) {
    if(VAC_SENSOR_CHECK_TIME <= timerTemperature) {
      timerTemperature = VAC_SENSOR_CHECK_TIME;
      flagErrorSensors.temperature = TRUE;
      
      if(rawOilTemperature > VAC_SENSOR_SHORT)
        flagErrorSensorsHigh.temperature = TRUE;
      else if(rawOilTemperature < VAC_SENSOR_OPEN)
        flagErrorSensorsLow.temperature = TRUE;
    }
  }
  else {
    flagErrorSensorsLow.temperature = FALSE;
    flagErrorSensorsHigh.temperature = FALSE;
    flagErrorSensors.temperature = FALSE;
    timerTemperature = 0;
  }
  
  /* Disabled on 2025.02.20 because of the clutch position
  // Clutch sensor
  if((get_clutch_sensor() > VAC_SENSOR_SHORT) || (get_clutch_sensor() < VAC_SENSOR_OPEN)) {
    if(VAC_SENSOR_CHECK_TIME <= timerClutch) {
      timerClutch = VAC_SENSOR_CHECK_TIME;
      flagErrorSensors.clutch = TRUE;
      if(get_clutch_sensor() > VAC_SENSOR_SHORT)
        flagErrorSensorsHigh.clutch = TRUE;
      if(get_clutch_sensor() < VAC_SENSOR_OPEN)
        flagErrorSensorsLow.clutch = TRUE;
    }
  }
  else {
    flagErrorSensorsLow.clutch = FALSE;
    flagErrorSensorsHigh.clutch = FALSE;
    flagErrorSensors.clutch = FALSE;
    timerClutch = 0;
  }
  */
  // 3P lever sensor
  if(rawThreePLeverSensor == 0x3F8) {
    if(VAC_SENSOR_CHECK_TIME <= timerLever) {
      timerLever = VAC_SENSOR_CHECK_TIME;
      flagErrorSensors.threePLever = TRUE;      
      flagErrorSensorsHigh.threePLever = TRUE;
      flagErrorSensorsLow.threePLever = TRUE;
    }
  }
  else  {
    flagErrorSensorsLow.threePLever = FALSE;
    flagErrorSensorsHigh.threePLever = FALSE;
    flagErrorSensors.threePLever = FALSE;
    timerLever = 0;
  }
  
  // 3P position sensor
  if((rawThreePPositionSensor > VAC_SENSOR_SHORT) || (rawThreePPositionSensor < VAC_SENSOR_OPEN)) {
    if(VAC_SENSOR_CHECK_TIME <= timerPosition) {
      timerPosition = VAC_SENSOR_CHECK_TIME;
      flagErrorSensors.threePPosition = TRUE;
      
      if(rawThreePPositionSensor > VAC_SENSOR_SHORT)
        flagErrorSensorsHigh.threePPosition = TRUE;
      else if(rawThreePPositionSensor < VAC_SENSOR_OPEN)
        flagErrorSensorsLow.threePPosition = TRUE;
    }
  }
  else {
    flagErrorSensorsLow.threePPosition = FALSE;
    flagErrorSensorsHigh.threePPosition = FALSE;
    flagErrorSensors.threePPosition = FALSE;
    timerPosition = 0;
  }
  /*
  // 3P draft sensor
  if((rawThreePDraftSensor > VAC_SENSOR_SHORT) || (rawThreePDraftSensor < VAC_SENSOR_OPEN)) {
    if(VAC_SENSOR_CHECK_TIME <= timerDraft) {
      timerDraft = VAC_SENSOR_CHECK_TIME;
      flagErrorSensors.threePDraft = TRUE;
      
      if(get_threeP_draft_sensor() > VAC_SENSOR_SHORT)
        flagErrorSensorsHigh.threePDraft = TRUE;
      else if(get_threeP_draft_sensor() < VAC_SENSOR_OPEN)
        flagErrorSensorsLow.threePDraft = TRUE;
    }
  }
  else {
    flagErrorSensorsLow.threePDraft = FALSE;
    flagErrorSensorsHigh.threePDraft = FALSE;
    flagErrorSensors.threePDraft = FALSE;
    timerDraft = 0;
  }
  */
    
  
  // Forward pressure sensor
  if((rawForwardPressure > VAC_SENSOR_SHORT) || (rawForwardPressure < VAC_SENSOR_OPEN)) {
    if(VAC_SENSOR_CHECK_TIME <= timerForwardPressure) {
      timerForwardPressure = VAC_SENSOR_CHECK_TIME;
// Disabled on 2022.08.23        flagErrorSensors.forwardPressure = TRUE;
    }
  }
  else {
    flagErrorSensors.forwardPressure = FALSE;
    timerForwardPressure = 0;
  }
  
  // Backward pressure sensor
  if((rawBackwardPressure > VAC_SENSOR_SHORT) || (rawBackwardPressure < VAC_SENSOR_OPEN)) {
    if(VAC_SENSOR_CHECK_TIME <= timerBackwardPressure ) {
      timerBackwardPressure = VAC_SENSOR_CHECK_TIME;
// Disabled on 2022.08.23        flagErrorSensors.backwardPressure = TRUE;
    }
  }
  else {
    flagErrorSensors.backwardPressure = FALSE;
    timerBackwardPressure = 0;
  }
  
  if(batteryPower <= VAC_BATTERY_8V) {
    flagErrorSensorsHigh.battery = FALSE;
    timerBatteryHigh = 0;
    if(VAC_SENSOR_CHECK_TIME <= timerBatteryLow) {
      timerBatteryLow = VAC_SENSOR_CHECK_TIME;
      flagErrorSensorsLow.battery = TRUE;
    }
  }
  else if(batteryPower >= VAC_BATTERY_18V) {
   flagErrorSensorsLow.battery = FALSE;
   timerBatteryLow = 0;
    if(VAC_SENSOR_CHECK_TIME <= timerBatteryHigh) {
      timerBatteryHigh = VAC_SENSOR_CHECK_TIME;
      flagErrorSensorsHigh.battery = TRUE;
    }
  }
  else
  {
    flagErrorSensorsHigh.battery = FALSE;
    flagErrorSensorsLow.battery = FALSE;
    timerBatteryHigh = 0;
    timerBatteryLow = 0;
  }
  
  update_dtc_test_result();
}