/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  *                     The source code development is begining
  * @Author             : Enkhbat Batbayar
  * @Company            : Kiwon Electronics LLC
  *
  * @Program            : In order to understand the program, please read the below description
  *                             1. definition is declared with CAPITAL SYMBOLS (ALL SYMBOLS) such as                    "CAN_CHANNEL_1"
  *                             2. function as named with underline such as                                             "can_transmit"
  *                             3. variables are declared with full name such as                                        "canOneTxData"
  *                             4. the non-volatile memory data are named with begining of "nv" symbols such as         "nvLiftHighAD"
  *
  *     2021.06.03      --> The source and header files are created
  *     2021.06.04      --> STM32 prephiral interface handler source codes were implemented
  *     2021.07.05      --> Added settings and fram functions
  *     2021.10.30      --> 
  *                             1. 계기판 발송 CAN CHANNEL 2로 변경 
  *                             2. 계기판 발송 CAN ID 19FFB020 추가 ( 에러 발송은 can_transmit_error()  DISABLE 시킴 )
  *                             3. transmissionL -> transmissionH로 수정. ( SHUTTLE MAP 설정등 정확하게 수정되었는지 다시 확인 할 것 ) 
  *                             4. CAN 발송 3P liftLock threePUpFlashing/threePDownFlashing 수정. ( 계속 점멸 문제 발생 ) 
  *                             5. CAN ID 0x19FFA000  발송 삭제
  *                             6. CAN 19FFD220 DATA7 = PROGRAM VERSION 발송으로 수정.
  *
  *     2021.11.01      -->     
  *                             1. T-BOX CAN 19FFA100 100ms cycle 발송 추가. (임시 사용, 앞으로 사용 하지 않음)
  *
  *     2021.11.02      -->     
  *                             1. RPM of hand and foot pedal sensor calculation bug is fixed (some definition calculation is not matched the voltage such as IVS switch )
  *
  *     2021.11.03      -->
  *                             1. Program version is changed to 1.2
  *                             2. CAN channel of the Instrumental panel is AUTOMATICCALY selected based on the received CAN channel
  *                             3. The speed dialog VR configuration is changed
  *                                     A. The CAN protocol is added
  *                                     B. Shuttle speed calculation algorithm is little bit changed and tested with oscilloscope
  *     
  *     2021.11.04      -->
  *                             1. Program version is changed to 1.3
  *                             2. 기계식(셔틀 사용) 여부(31번: OPEN (OFF) = 기계식, 연결 (ON) = 전자식) implementation is added
  *
  *     2021.11.08.            
  *                             1. CAN 채널 확정 시킴( CAN1과 CAN2 연결시 두번 수신 동작 되는 것 수정)
  *                             2. CAN2 진단기 ENABLE DATA[0]=F4, DATA[1]=1, DATA[6]=1 일때 Enablee
  *                                CAN2 진단기 ENABLE DATA[0]=F4, DATA[1]=1, DATA[6]=0 일때 Disable 
  *                                진단기 프로그램과 맞게 수정.
  *                             3. 프로그램 버젼 1.3으로 수정.
  *                             4. 변속 스위치 항상 OFF로 설정함. ( INPUT.C 내용 )
  *                             5. 진단용 CAN ID 19FFA100, 19FFA120, 19FFA020을 CAN2로도 발송 되도록 수정. ( CAN ID 삭제 할 것. ) 
  *     2021.11.16
  *                             1. Program version is changed to 1.4
  *                             2. Transmit switch is enabled
  *                             3. AD CAN packet transmission bug is fixed
  *                             4. Shuttle currents and pressures, oil temperature, and car speed data packet send to diagnostic program every 10ms
  *                             5. The analog inputs data are matched with diagnostic monitoring data
  *                             6. Transmission map selection is changed
  *
  *     2021.11.18
  *                             1. Program version is changed to 1.5
  *                             2. Fill time calculation bug is fixed.
  *                             3. Speed rate calculation is changed from percentage to current
  *                                     a. offset is 100 --> if the setting is 100, then it is 0 mA
  *                             4. Oil temperature is saved every 10ms and calculated every 500ms. Which means 50 data is averaged.
  *
  *     2021.11.22
  *                             1. Program version is changed to 1.6
  *                             2. TLE82453 IC configuration is changed (auto limit is enabled )
  *     2021.11.23
  *                             1. Program version is changed to 1.7
  *                             2. The maximum and minimum value of the speed rate settings are changed.
  *                             3. The default settings are changed based on the T59_69_79_TCU_설정값_20211116.xls (sheet name is 20211123)
  *                             4. Map number setting process is checking the MIN and MAX value of the FILL & MAP numbers
  *                             5. 
  *                             
  *     2021.11.25
  *                             1. Program version is changed to 1.8
  *                             2. "get_clutch_sensor_average()" is added in used in shuttle.c file, please see the calculate_clutch() function
  *                             3. Map number saving bug is fixed
  *     2021.11.26
  *                             1. Program version is changed to 1.9
  *                             2. Model switch is checked. If model switch is "ON", the shuttle is not working anymore.
  *                             3. Clutch pedal checking algorithm is little bit changed.
  *                             4. CAN transmission time management is changed little bit. Only one transmission should be happened in 2ms.
  *                             5. CAN2 is used 
  *     2021.11.29
  *                             1. Program version is changed to 1.9
  *                             2. The hitch controller manual mode starting time implementation is added.
  *                             3. The ADPro valve controller algorithm is added
  *                             4. The Fill time and current calculation range is increased from 0-100% to 0-150%
  *                             5. Checked N->1->N->2->N->3->N->4 && 4->N->3->N->2->N->1 condition is tested with shift map
  *                             6. 3P map data is updated same as X750 but extended points
  *                             7. 3P Duty values are transmitted to diagnostic program and tested.
  *     2021.11.30
  *                             1. Program version is changed to 2.0
  *                             2. The sensor input of the TIC12400 IC's inputs are checked 8 times and proved the value ("tic12400_getInputDataProved"  function is added)
  *                             3. Shuttle mode is absent condition (model sw is always on) is applied.
  *                                     a. if "isShuttleControlled" value is FALSE, it is NOT used shuttle controller.
  *     2021.12.03
  *                             1. The PTO lamp status on the instrumental panel is implemented. please see the "flagOutputLamp.pto" variable.
  *                             2. 3P MANUAL UP/DOWN 출력 제어 방법 수정.
  *                             3. 수평제어 평행화 출력 오류 수정.
  *     2021.12.06
  *                             1. Program version is changed to 2.1
  *                             2. Sensor errors CAN packet reprogrammed and bugs are fixed.
  *                             3. The "VAC_TRACTOR_MODEL" definition is added for 5 types of tractor (but program is 3 types)
  *                                     when VAC_TRACTOR_MODEL is equal to "VAC_TRACTOR_TX69_MODEL", it is using shuttle controller.
  *                                     when VAC_TRACTOR_MODEL is equal to "VAC_TRACTOR_TX76_MODEL", it is using shuttle controller. However, AD error is not detected.
  *                                     when VAC_TRACTOR_MODEL is equal to "VAC_TRACTOR_TX59_MODEL", it is NOT using shuttle controller and AD error is NOT detected.
  *                             4. Back buzzer implementation is little bit changed.
  *                             5. Check buzzer implementation is created when car speed over 7km/h
  *                             6. Quick turn implementation bug is fixed
  *                             7. AD pro output implementation bug is fixed.
  *                             8. 
  *     2021.12.07
  *                             1. Program version is changed to 2.2
  *                             2. Errors of the transmission LMH and 1234 inputs are checked. it is transmitted to Instrumental panel via CAN bus.
  *     2021.12.08
  *                             1. Program version is changed to 2.3
  *                             2. 3P lever controller is implemented.
  *                                     a. "flag.leverUpRun" flag is used for implementing this part.
  *                             3. PTO switch on/off checking time is changed to 0.5s
  *                                     a. "ptoOffTime" timer value is changed from 20 to 50
  *     2021.12.09
  *                             1. Program version is changed to 2.4
  *                             2. Added model selection (TX69=1 or TX76=2) program based on the CAN protocol which is transmitted from 계기판
  *                             3. when shuttle forward or backward and break switch, buzzer is ON.
  *     2021.12.10
  *                             1. Program version is changed to 2.5
  *                             2. The bug which 3P one touch down button is clicked (the 3P position is in the deadband), it is going to down.
  *                                     Description: it should NOT go down anymore, so I changed the program, please search "2021.12.10"
  *                             3. PTO off map should be implemented after PTO on map is finished. If PTO on map is not finished, PTO should be off without using PTO OFF MAP.
  *                                     a. "ptoOffTime" should be considered when the PTO on map is finished.
  *                             4. Buzzer bug is fixed --> Shuttle is switched to forward/backward when car speed is over 7km/h. The buzzer should be ON.
  *                             5. 3P up and down duty increasing time is changed from 250ms to 750ms
  *                             6. 3P lever up and down bug is fixed by using (flag.leverUpRun & flag.leverDownRun)
  *                             7. PTO ON to OFF case, check buzzer is on in one time.
  *                             8. 3P lever control is working when up limit is high, otherwise is not working --> the bug is fixed.
  *                             9. 기계식, CAN back switch should NOT transmitted.
  *                             10. 3P map data is updated
  *                             11. 3P 8bit to 10bit changes
  *                                     a. Only deadband checking sections are changed to 10bits
  *                             12. 3P speed controlling dialog MIN, MAX, and MEDIUM values changed
  *                                     a. VAC_LIFT_DOWN_SET_MAX, VAC_LIFT_DOWN_SET_MIN, VAC_LIFT_DOWN_SET_MID
  *                             13. PTO overtime bug is fixed
  *                                     a. timerPtoOff and timerPtoOn variables are updated if they have last time of the position (PTO on and off MAP time point)
  *     2021.12.13
  *                             1. Program version is changed to 2.6
  *                             2. 3P up and down output should be on when RPM is over 400.
  *                                     a. if(canRealRpm >= 400) condition is added
  *                             3. 3P position and lever limit calculation is changed bu using following variables
  *                                     a. uint16_t leverUpLimit;
  *                                     b. uint16_t leverDownLimit;
  *                                     c. uint16_t positionUpLimit;
  *                                     d. uint16_t positionDownLimit;
  *                             4. 3P if car speed over 7km/h, the buzzer is on when shifting MHL, and 1234. such as 1-->2 state.
  *                             5. 3P draft & depth map is used 26-->29 MAP data is used.
  *                                     a. Draft up map --> 26
  *                                     b. Draft down map --> 27
  *                                     c. Depth up map --> 28
  *                                     d. Depth down map --> 29
  *                             6. Digital input read function is changed to 16ms to 80ms
  *                                     a. PTO read function is implemented because of this changes.
  *     2021.12.14
  *                             1. Program version is changed to 2.7
  *                             2. RPM calculation is changed
  *                             3. "MX_CAN_FILTER_Init(0, J1939_EEC1_ECU_ID, J1939_EEC1_ECU_ID);" filter is added for CAN2
  *                             4. Temperature sensor calculation bug is fixed. Please see the implementation of the "HYDAC_TEMPERATURE_SENSOR" definition
  *                             5. CAN protocol of the diagnostic program is changed for matching LabView new version.
  *                             6. 3P safety timer is removed because we are checking RPM when controlling 3P
  *                             7. 3P floating implementation is changed. Please see the "flag.floating" the variable implementation.
  *     2021.12.15
  *                             1. Program version is changed to 2.8
  *                             2. Forward and backward pressure sensor calculation is added. Now we can see the pressure sensor in bar
  *                             3. Dither frequency, dither current, and PWM are changed because of it is using one variables for different outputs
  *                             4. When 3P lock is TRUE, the deadband zone is changed.
  *                             5. Forward and backward pressure sensor error checking and send it to diagnostic program
  *     2021.12.16
  *                             1. Program version is changed to 2.9
  *                             2. Diagnostic CAN protocol is updated that model checking bits
  *
  *     2021.12.17
  *                             1. Program version is changed to 3.0
  *                             2. Output of the status pins are analog, so I changed program. See the "vnd5025.c" source code
  *                             3. control algorithm is cycle time is checked   -->  390us ~ 670us
  *                             4. "FOC_SOL3_Pin" is no more used.
  *                             5. The DRAG output is now using the TLE82453 (not LOW SIDE). Therefore, program is changed --> Checked working well
  *                             6. Tested forward, backward, pto, valve outputs with open and short cases.
  *                             7. 3P status pin has a analog output so we do NOT need the hardware PULL-UP resistors
  *                             8. ENGINE 400 RPM over next, 1sec delay after 3P safty lock free buzzer on 되도록 수정.
  *                             9. 3P short can be detected. But open can NOT detected for this hardware because of the 3P status pin output is connected to digital input.
  *                             10.
  *     2021.12.22
  *                             1. Program version is changed to 3.1
  *                             2. 3P Floating is changed from 20 to 15
  *                             3. Starter map selection is changed (now it is the transmission 1,2,3, and 4 are changed by checking start or shift map). Please see the "2021.12.22"
  *                             4. 3P down and up outputs errors are disabled. So, we should check error by changing IC or schematic.
  *                             5. "VAC_DOWN_SPEED_CUT_OFF" is changed from 10 to 35. It is matched to key panel with controller.
  *
  *     2021.12.27
  *                             1. Program version is changed to 1.0 --> TX69
  *                             2. FRAM Default settings values are changed.
  *                             3.
  *
  *     2021.12.28
  *                             1. Program version is changed to 1.1 --> TX69
  *                             2. The CAN pakcet is changed whose ID is "0x19FFA100".
  *     2022.01.03
  *                             1. Program version is changed to 1.2 --> TX69
  *                             2. The 3P position setting range value bug is fixed. (it is only neccessary when calibrating the 3P position sensor.)
  *
  *     2022.01.05
  *                             1. Program version is changed to 1.3 --> TX69
  *                             2. CAN initialization sequence is changed
  *                                     a. OLD: INIT ( INTR.ENBALE is in the middle of initialization process )--> FILTER --> ACTIVATION --> START
  *                                     b. NEW: INIT --> FILTER --> ACTIVATION --> START --> INTR.ENBALE
  *                                     Reason: If the interrupt function is called during the initialization process, the CAN controller may have an error (pointer error, register error, etc).
  *                             3. "canRealRpm" calculation is moved into main program from CAN interrupt and added two buffer for saving the Engine RPM value
  *                             4. When CAN transmission is happened with an error, the error handler function is disabled and counter is added.
  *                             5. UART interrupt is disabled --> UART interface is NOT used in this program
  *                             6. Car speed calculation is improved --> equation is changed from Y=AX to Y=AX+B
  *     2022.01.07
  *                             1. Program version is changed to 1.4 --> TX69
  *                             2. Shuttle output is changed based on the transmission 1,2,3 and 4 shift inputs. The all implementation is in the "shuttle.c" source file. Please search with "2022.01.07" this word
  *                             3.
  *     2022.01.12
  *                             1. Program version is changed to 1.5 --> TX69
  *                             2. Cruise (AD_PRO) output algorithm is improved with clutch input
  *     2022.01.12
  *                             1. Program version is changed to 1.6 --> TX69
  *                             2. The mechanical shuttle controller is updated.
  *     2022.01.14
  *                             1. Program version is changed to 1.7 --> TX69
  *                             2. KEY OFF->ON 3P레버와 3P위치 않아서 3P작동 되지 않음
  *                                     a. #define VAC_LIFT_DEADBAND_ANGLE_3            12
  *                             3. 3P 위치센서가 250일 떄 외부 3P 상승 ON후 OFF 하면 3P가 상승 되었다가 다시 내려옴
  *                                     a. if(positionAngle >= 1000) { positionAngle = 1000; }
  *                             4. AUTO BALANCE ON시 DEADBAND CONTROL 수정 --> 3, 6, 12
  *                             5. "updateADC" function is changed with "updateLastAverageADC" function. it is the adc averaging method is changed.
  *     2022.01.24
  *                             1. Program version is changed to 1.8 --> TX69
  *                             2. DIFF valve output is used as "언로드 빨브"
  *                                     a. The implementation is in "balance.c" source file
  *                             3. Forward & backward shuttle compensation map is used. In this case, we are using map number 5 and 6
  *                                     a. Map 5 is forward shuttle compensation map
  *                                     b. Map 6 is backward shuttle compensation map
  *                                     c. Setting map is updated, please see "settings.c" and "settings.h" files
  *                                     d. shuttle control function is updated, please see "shuttle.c" file
  *                             4. Fixed the calculation of "temperatureFillTimePercent" and "temperatureFillCurrentPercent" variables.
  *										a. Map-2 is changed. (temperature and time compensation of FILL)
  *     2022.01.27
  *                             1. Program version is changed to 1.9 --> TX69
  *                             2. Power problem bug is fixed. Now the battery voltage is higher than 10V, it will start the TCU
  *     2022.01.27
  *                             1. Program version is changed to 2.0 --> TX69
  *                             2. Back buzzer should be on eventough backward output is NOT on.
  *
  *     2022.01.28
  *                             1. Program version is changed to 2.1 --> TX69
  *                             2. Default setting data are changed
  *     2022.01.28
  *                             1. Program version is changed to 2.2 --> TX69
  *                             2. Clutch switch input is not used for clutch forward and backward output, only map information is used for clutch control
  *     2022.02.03
  *                             1. Program version is changed to 2.3 --> TX69
  *                             2. Starter bug is fixed
  *                             3. Map data is changed
  *     2022.02.04
  *                             1. Program version is changed to 2.4 --> TX69
  *                             2. Map data is changed
  *                             3. Drag map is moved from 5,6,7,8 to 47,48,49,50
  *                             4. TLE82453 second IC is NOT used so the configuration and output implementation are DISABLED.
  *                             5. When transmission is 1,2,3 or 4단, and shuttle is forward or backward and 변속SW is OFF, then clutch switch is OFF to ON, the output should be shuttle happened.
  *                                     a. But in this case on 2022.01.28, we disabled the clutch switch. Thus, instead of checking switch, we are using pedal position which is pushed or not. 
  *                                     b. instead of clutch switch, "flag.pedalPush" is checking.
  *     2022.02.04
  *                             1. Program version is changed to 2.5 --> TX69
  *                             2. Map data is changed
  *                             3. When default setting is happened via CAN, it should be updated map data and numbers without shutting down the TCU
  *
  *     2022.02.07
  *                             1. Program version is changed to 2.6 --> TX69
  *                             2. Balance STOP_DEADBAND is changed from 2 to 3, Starting deadband is also changed from 3 to 4, and from 6 to 7
  *                             3. Stroke sensor averaging points is changed from 4 to 2
  *                             4. Balance timeOut is changed from 2500ms to 5000ms
  *     2022.02.08
  *                             1. Program version is changed to 2.7 --> TX69
  *                             2. in hitch control, added the lever control section, (lever is checked in every 200ms).  If lever is changed (-2 to +2), the hitch is controlled with lever position.
  *                             3. Disabled unload_valve_control() function
  *                                     a. It removed stop delay (50ms) of balance up and down outputs
  *                             4. Balance up stop deadband is changed from 3 to 4, 5
  *                             5. Balance 민감 default setting is changed from 4 to 5
  *     2022.02.11
  *                             1. Program version is changed to 2.8 --> TX69
  *                             2. 3P position sensor up and down range is changed
  *                                     a. Down range 0.1V ~ 1.0V
  *                                     b. Up range 3.9V ~ 4.8V
  *                             3. When 3P lever setting mode is activated, the check buzzer is ON one time.
  *     2022.03.02
  *                             1. Program version is changed to 2.9 --> TX69
  *                             2. CAN packets are lost sometimes
  *                                     a. Because of the engine CAN packets (there are too many packets) --> such as 0x19FFA005 packet is lost sometimes randomly
  *                                     b. Therefore this flag is enabled --> hcan1.Init.AutoRetransmission = ENABLE
  *                             3. 0x19FFA020 packet is never transmitted (This problem is never happened in 시험기)
  *                                     a. we changed CAN algorithm little bit --> it solved the problem but packets are lost randomly
  *                                     b. Therefore this flag is enabled --> hcan1.Init.AutoRetransmission = ENABLE  
  *
  *
  *     2022.03.15 --> STAGE V development is beginning
  *                             1. Program version is changed to 1.0
  *                             2. RPM contoller is used
  *                                     a. CAN packet transmission is enabled
  *                                     b. RPM controller is used
  *                             3. Switch panel is changed so the corresponding CAN packet and controller is changed
  *                                     a. Switch panel CAN packet is changed
  *                                     b. PTO Controller is changed
  *                                     c. liftStop variable calculation is changed
  *                             4. DIFF (unload) control is added
  *                             5. Check buzzer is added on topLink and balance controller switch checking
  *
  *     2022.04.12
  *                             1. Program version is changed 1.1
  *                             2. TSC1 control byte is changed from CD to C1 in CAN message
  *     2022.04.13
  *                             1. Program version is changed 1.2
  *                             2. Pressure sensor calculation is fixed.
  *     2022.04.15
  *                             1. Program version is changed 0.3
  *                             2. 4WD output is disabled for mechanical tractor (Model: TX59)
  *                             3. RPM Cruise control is changed (PTO state does NOT matter)
  *     2022.04.15
  *                             1. Program version is changed 0.4
  *                             2. 4WD CAN output is disabled for mechanical tractor (Model: TX59)
  *                             3. Unload valve controller is disabled for mechanical tractor (Model: TX59)
  *     2022.04.18
  *                             1. Program version is changed 0.5
  *                             2. RPM Valve output is disabled (for TSC1 communication)
  *                             3. RPM min and max value are changed
  *     2022.04.18
  *                             1. Program version is changed 0.6
  *                             2. Panel model number VAC_MODEL_1005_648_230_0 & VAC_MODEL_1007_648_230_0 are swapped each other (in main.h)
  *
  *     2022.04.22         -->  The program is changed for T100, T115
  *                             1. Program version is changed 0.1
  *                             2. "C" transmission is added, the memory map is little bit changed (see settings.c file)
  *     2022.05.10
  *                             1. Setting data and CAN protocol is changed and fixed
  *     2022.06.28      
  *                             1. Setting data and CAN protocol is changed and fixed
  *                             2. 
  *     2022.08.11      
  *                             1. Program version 0.2
  *                             2. PTO bug is fixed.
  *                             3. Temperature sensor is changed so, temperature calculation is changed (Testing device is changed 5K to 20K)
  *                             4. Battery High/Low error checking threshold value is changed. (bug is fixed)
  *     2022.08.12
  *                             1. Program version 0.3
  *                             2. Brake lamp output is changed to PIN=3
  *     2022.08.15
  *                             1. Program version 0.4
  *                             2. Brake lamp output bug is fixed
  *                             3. HYDAC Temperature sensor is used 
  *                             4. Oil pressure & strainer blocking sensor errors are disabled
  *                             5. TOPLINK Unload valve output current is changed from 500mA to 1500mA
  *                             6. Hardware changes
  *                                     a. Because of clutch valve (D4 diode is removed and connected short) --> Hardware fix
  *                                     b. PTO & brake lamp short is happened (only one board) --> Hardware fix
  *     2022.08.18
  *                             1. Program version 0.5
  *                             2. Parking brake input is added into Instrumental panel CAN transmission message
  *                             3. Drag sensor error is disabled
  *
  *     2022.08.22
  *                             1. Program version 0.6
  *                             2. TLE92464 testing with debug interface
  *                                     a. The PERIOD_EXP and POW2PERIOD_EXP definitions are changed because of the minimum frequency.
  *                                     b. Theshold values are added into setting functions
  *                                     c. Deep dither is disabled and autolimit feature is also disabled
  *                             4. 
  *                             5. 
  *     2022.08.23              
  *                             1. Program version 0.7
  *                             2. Steering switch 1 and 2 replaced each other (1 --> 2), (2 --> 1)
  *				3. Hella temperature sensor is used and 300 Om is soldered instead of 10K so HARDWARE_VOLTAGE_DIVIDER_RESISTOR_300 definition is enabled
  *                             4. When 4WD auto mode, the steering sensor check condition is changed.
  *                             5. PTO OFF setting MAX and MIN bug is fixed.
  *     2022.10.26
  *                             1. Program version 0.8
  *                             2. Temperature sensor points are added based on excel data
  *     2022.10.26
  *                             1. Program version 0.9
  *                             2. PTO bug is fixed based on TIMER PWM configuration
  *                             3. If oil pressure sensor error is occurred then buzzer is ON (not flashing)
  *                             4. Strainer blocking sensor is detected an error, it should be flashing to CAN data
  *     2022.12.01
  *                             1. Program version 1.0
  *                             2. Seat switch is using
  *                             3. Strainory blocking CAN bus protocol is changed from FLASHING to ON
  *                             4. Clutch hardware schematic is chnaged
  *                             5. Strainory blocking switch is checking condition is changed in INPUT SIDE so diagnostic side is back
  *     2022.12.05
  *                             1. Program version 1.1
  *                             2. Buzzer output is changed when power shuttle (F->R->F) is moved (tractor speed is over 10km/h).
  *                             3. Engine RPM is NOT received, so the canRpm should be 0 (engine is OFF state)
  *                             4. 
  *                             5. 
  *                             6. 
  *     2022.12.08
  *                             1. Program version 2.3
  *                             2. "EngineLoad" is transmitted to CAN diagnostic protocol from engine CAN ID = 0x0CF00300
  *     2022.12.12
  *                             1. Program version changed 2.4
  *                             2. Foot pedal is avraged from 4 to 20, because it is so sensitivity
  *     2022.12.13 ~ 14
  *                             1. Program version changed 2.5
  *                             2. TLE92464 TOPLINKK UNLOAD VALVE is using DIR pin
  *                             3. TLE92464 ERROR Clear function is added
  *     2022.12.16
  *                             1. Program version changed 2.6
  *                             2. Temperature sensor offset is changed from -25 to -40 in Smartkey CAN protocol.
  *     2022.12.20
  *                             1. Program version changed 2.7
  *                             2. Strainer blocking sensor condition is changed
  *                             3. Software watchdog is added
  *                             4. Foot sampling time is changed from Diagnostic program (Sampling buffer is now constant)
  *     2022.12.27
  *                             1. Program version changed 2.8
  *                             2. The shuttle forward to backward shift algorithm is changed
  *     2023.01.02
  *                             1. Program version changed 2.9
  *                             2. The shuttle forward to backward shift algorithm is changed
  *     2023.01.03
  *                             1. Program version changed 3.0
  *                             2. The shuttle forward to backward shift algorithm is changed
  *                             3. The forward/backward setting data is changed little bit (added +5 to reduce gap)
  *     2023.01.04
  *                             1. Program version changed 3.1
  *                             2. The shuttle forward to backward shift algorithm is changed
  *                             3. Drag output is enabled in "Drag control" and drag control algorithm is changed
  *                             4.
  *     2023.01.05
  *                             1. Program version changed 3.2
  *                             2. Drag control bug is fixed and added clutch sensor is checked in this control
  *                             3. "Real time MAP DATA is NOT updated" bug is fixed --> "isUpdateHappenedMapNumber = TRUE" is added
  *     2023.01.06
  *                             1. Program version changed 3.3
  *                             2. Strainer blocking switch is disabled because of request from TYM
  *                             3. Idle RPM control is changed from 800 to 900
  *                             4. EU version program bug of PTO part is fixed. 
  *     2023.01.09
  *                             1. Program version changed 3.4
  *                             2. Drag control part is changed by request from TYM
  *     2023.01.10
  *                             1. Program version changed 3.5
  *                             2. Map data selection bug is fixed (Map selection is changed)
  *     2023.01.17
  *                             1. Program version changed 3.6
  *                             2. Setting position is changed
  *                                     a. Foot pedal position is used instead of clutch compensation variable (Clutch compensation is NOT used)
  *                                     b. Engine load compensation is added
  *                             3. Engine torque control is added (in SHUTTLE FILL MODE)
  *                             4. Foot pedal sampling calculation is done
  *     2023.01.19 ~ 01.20
  *                             1. Program version changed 3.7
  *                             2. Foot pedal sampling calculation is changed (same as previous one)
  *                             3. Hand, Foot pedal and IVS switch error checking time bug is fixed. (rpmController.c & sensor.c files are changed)
  *                             4. High-Low output maps are added when clutch is NOT free (setting.c & transmission.c files are changed)
  *
  *     2023.01.19 ~ 01.20
  *                             1. Program version changed 3.8
  *                             2. Foot pedal sampling calculation is changed  "LOG is used, sampling time is 50ms, sampling time buffer is selected by Diagnostic program, max buffer is 20 --> means 1 seconds"
  *
  *     2023.01.30
  *                             1. Program version changed 3.9
  *                             2. Hand pedal algorithm is changed same as foot pedal (LOG algorithm is used)
  *                             3. Clutch pedal is using moving average method instead of get_clutch_sensor_average() function (Actually it is updated every 100ms)
  *
  *     2023.01.31
  *                             1. Program version changed 4.0
  *                             2. Default data is changed
  *                                     a. Drag ON time is 100, OFF time is 105
  *                             3. Hand pedal LOG algorithm is removed , just only MA is used.
  *
  *                             1. Program version changed 4.1
  *                             2. Clutch pedal checking method is changed to previous version, And average time is changed 100ms to 300ms
  *                             3. Shuttle mode is check --> NO change, if MAP DATA is changed, the problem can be solved.
  *     2023.02.01
  *                             1. Program version changed 4.2
  *                             2. Default data is changed
  *                                     a. Foot sampling time is 500, shuttle compensation dialogue is low=90
  *                                     b. Starter map number is changed
  *                                     c. MAP data is changed --> 17, 18, 15, 16, 2, 4, 11, 12, 13, 14, 31, 32, 41 ~ 47, 51 ~ 55, 66, 67, 69 ~ 72
  *     2023.02.09
  *                             1. Program version changed 4.3
  *                             2. Strainer blocking alarm function is added
  *     2023.02.10
  *                             1. Program version changed 4.4
  *                             2. Strainer blocking alarm function is removed
  *                             3. Default MAP data is changed. 41, 52, 53, 54, 55
  *                             4. nvShuttlePtoCompensation, nvShuttle4WDCompensation variables are NOT used.
  *     2023.02.13
  *                             1. Program version changed 4.5
  *                             2. "nvQtAutoConnectSpeed = 7", "nvDiffAutoConnectSpeed = 14", 69, 70 map's first point is 7
  *                             3. 
  *     2023.02.15
  *                             1. Program version changed 1.0
  *                             2. Default map is changed:
  *                                     a. Transmission map (Forward M, 3->4), (Backward M, 3->4), (Backward H, 2->3), (Backward H, 1->2)
  *                                     b. Map data: 51, 52, 55 
  *     2023.02.16
  *                             1. Program version changed 1.0
  *                             2. TLE92464 Error detection time is changed from 1000ms to 200ms
  *                             3. TLE92464 Open Error detection threshold is changed from fixed 125ma to "Setpoint/8"
  *                             4. TLE92464 SPI current updating method is added (300ms --> Forward, Backward, High, Low, Drag)
  *     2023.02.24
  *                             1. Program version changed 1.0
  *                             2. TLE92464 Error detection is 200ms. If an error is occurred during 1 seconds, the error should sent via CAN bus.
  *     2023.02.28 ~ 03.01
  *                             1. Program version changed 1.0
  *                             2. TLE92464 Error detection bug is fixed. Also, each channel error detection action is in ON-state of an output.
  *     2023.03.06
  *                             1. Program version changed 1.0
  *                             2. In EU versio, the PTO IP lamp should be flashing with when seat sw = off & stationary sw=off & PTO sw is on states.
  *                                     a. PTO IP lamp is OFF --> PTO IP lamp flashing
  *     2023.03.07
  *                             1. Program version changed 2.0
  *     2023.03.20
  *                             1. Program version changed 2.1
  *                             2. Default setting is changed --> MAP 41, 42, 43, 44, 45, 46, 47
  *     2023.03.21
  *                             1. Program version changed 1.1
  *     2023.03.22
  *                             1. Program version changed 2.1
  *                             2. Shift map bug is fixed. When speed sensor is over 5 km/h, the shift map should be used eventhough CLUTCH PEDAL is pushed and back
  *                             3. Default setting is changed as previous state (2.0) --> MAP 41, 42, 43, 44, 45, 46, 47
  *                             4. nvSensetiveSettingDownLimit = -15
  *                             5. MAP DATA is changed -> 17, 18, 41 ~ 47, 51 ~ 55
  *     2023.03.27
  *                             1. Program version changed 2.2
  *                             2. Clutch map 17 and 18 are changed. (Added -10)
  *                             3. Clutch map 15 and 16 are changed.
  *     2023.04.21
  *                             1. Program version changed 2.3
  *                             2. Backward shuttle starting rate current setting value is added
  *                                     a. Shuttle starting rate current is used for FORWARD
  *                                     b. CAN protocol is added
  *                                     c. Diagnostic program is changed,
  *     2023.05.04
  *                             1. Program version changed 3.0
  *                             2. UDS function is added
  *     2023.07.07
  *                             1. High/Low DID implementation is added
  *                             2. 
  *
  *     2023.11.07
  *                             1. Seat switch is removed from Starter function
  *
  *     2023.12.21
  *                             1. Program version is changed to 1.0 (VAC_PROGRAM_VERSION = 10)
  *                             2. CAN data of function switch bug is fixed.
  *     2023.12.26
  *                             1. CAN data of 0x19FFA120 bug is fixed.
  *     2023.12.27
  *                             1. Added: Buzzer output is flashing when all of sub transmission switches are OFF (C,L,M, and H) or all of main transmission switches are OFF (1,2,3, and 4)
  *                                Also, the forward and reverse outputs are OFF.
  *                             2. Fixed: Searching algorithm is changed when all C,L,M,H,1,2,3, and 4 are OFF.
  *     2024.01.02
  *                             1. The check buzzer is ON (always ON) when parking brake switch is ON and shuttle switch is forward or reverse
  *                             2. Changed: Forward and Reverse lamp (CAN) output is flashing when all of sub transmission switches are OFF (C,L,M, and H) or all of main transmission switches are OFF (1,2,3, and 4)
  *     2024.01.04
  *                             1. TOPLINK Unload valve time 18ms to 20ms changed
  *                             2. Buzzer time bug is fixed (14.5 seconds TO 15 seconds) when seat & parking switch are OFF.
  *                             3. Real RPM and control RPM are transmitted with UDS protocol (DID is 0xA401, 0xA402)
  *
  *     2024.01.30
  *                             1. Program version is changed to 1.1
  *                             2. Active session is saved into the NV
  *                             3. Software reset need security
  *                             4. uds_main() function is called in init loop section, because the CAN transmission begin as early as possible.
  *     2025.03.24
  *                             1. Program version is changed to 1.2
  *                             2. RPM down algorithm is changed.
  *     2025.04.02
  *                             1. CAN reinit function is added and called from uds_main file
  *     2025.05.23
  *                             1. Program version is changed to 1.3
  *                             2. Default map data is changed
  *     2025.06.11
  *                             1. Program version is changed to 1.3
  *                             2. Temperature calculation is changed as same as TX78
  *     2025.06.11
  *                             1. Program version is changed 1.4
  *                             2. Base current is added (setting and shuttle/Clutch)
  *                                     a. Added: uint16_t nvForwardBaseCurrent; uint16_t nvBackwardBaseCurrent; uint16_t nvClutchGap;
  *                             3. Clutch setting function is added (save id of factory reset, but data is different = 6666)
  *                                     a. Based on the setting 17, 18 map data will be changed
  *     2025.07.17
  *                             1. Program version is changed 1.5
  *                             2. PTO temperature map is added, only DUTY compensation is used
  *                             3. PTO ON MAP is changed
  *                             4. 3P quick down function is removed (Onetouch down button long pressed)
  *                             5. 3P Stall function is changed
  *                             6. 
  *     2025.07.23
  *                             1. Regen CAN ID is added
  *                             2. Removed the balance up and down button from DASH PANEL
  *
  *     2025.07.30
  *                             1. Program version is changed 1.6
  *                             2. Maximum current is 1000mA (Shuttle) = VAC_SHUTTLE_MAX_CURRENT
  *                             3. 46 map is used for clutch to shuttle map
  *                             4. Regen CAN ID is changed
  *                             5. Power Assist CAN data is changed
  *                             6. Regen output is NOT used, so output is disabled
  *                             7. Shuttle MAP number is transmitted instead of 3P depth sensor
  *     2025.08.04
  *                             1. Program version is 1.7
  *                             2. Clutch is pushed, the map data is updated
  *     2025.08.07
  *                             1. Program version is 1.8
  *                             2. Default setting data is changed
  *     2025.08.12
  *                             1. Default setting data is changed
  *                             2. Hitch algorithm is changed. When it is going up then never going down until, onetouch up, down or lever is moved
  *                             3. Hitch lever checking is change (250/1000 --> 237/950)
  *
  *
  *     NOTICE:                 BEFORE COMPILE THE SOURCE CODE, PLEASE CHECK THE "VAC_TRACTOR_MODEL" DEFINITION WHICH IS CORRECTLY CONFIGURED OR NOT.
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system.h"
#include "adc.h"
#include "input.h"
#include "output.h"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer.h"

#include "fram.h"
#include "tic12400.h"
#include "tle92464.h"
#include "settings.h"

#include "algorithm.h"
#include "shuttle.h"
#include "sensors.h"
#include "watchdog.h"
#include "uds_main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
// global timer flags
flagTimer_t flagTimer;

// local timer counter variables
uint8_t tm1ms;
uint8_t tm2ms;
uint8_t tm10ms;
uint8_t tm100ms;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void timer_process();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
//  uint8_t test_counter = 0;
  /* USER CODE BEGIN 1 */
  uint8_t outputInitDone = FALSE;
  uint8_t inputInitDone = FALSE;
  uint8_t controlInitDone = FALSE;
  uint8_t initDone = FALSE;
   
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  __enable_irq();
  
  gpio_input_init();                                                            // General IO configuration --> Inputs --> general inputs & TIC12400 chip select and Irq pins
  gpio_output_init();                                                           // General IO configuration --> Outputs

  fram_init();                                                                  // General IO configuration --> Chip select pin

  // Interfaces configuration of the MCU
  MX_DMA_Init();                
  MX_ADC1_Init();
  
  // Software watchdog is added
  watchdog_init();                                                              // Because of the software watchdog, we added initialization function in here after the MCU configuration
  
  can_init(CAN_CHANNEL_1);
  can_init(CAN_CHANNEL_2);
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  
  /* USER CODE BEGIN 2 */
  printf("Starting application.\r\n");
  
  control_init();  
  
  inputInitDone = tic12400_GPIO_Input_Init();
  start_timers();
  startConversation();
  
  uds_init();
  
  while (!initDone)                                                                     // Init loop
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(flagTimer.oneMs == TRUE)
    {
      watchdog_trigger();
      timer_process();
      
      if(flagTimer.twoMs == TRUE) {
        if(inputInitDone == FALSE) {
          inputInitDone = tic12400_GPIO_Input_Init();
        }
        
        if(outputInitDone == FALSE) {
          outputInitDone = output_init();        
        }
        if((controlInitDone == FALSE) && (inputInitDone == TRUE)) {
          controlInitDone = control_previous_process();                         // In here we are updating digital and analog inputs, and checking the model switch, if model switch is checked and finished the while loop. and go to main loop
          uds_main();                                                           // Added on 2024.01.30
        }
        
        if((outputInitDone == TRUE) && (inputInitDone == TRUE) && (controlInitDone == TRUE)) {
          initDone = TRUE;
        }        
      }
      flagTimer.data = 0;                                                       // All timer flag is cleared
    }
  }

  while (1)
  {
    if(flagTimer.oneMs == TRUE)
    {
      flagTimer.oneMs = FALSE;
      watchdog_trigger();
      timer_process();
      if(flagTimer.twoMs == TRUE) {
        control_process();
      }
      uds_main();
      can_transmit_process_2();
      
      flagTimer.data = 0;                                                       // All timer flag is cleared
    }
  }
}

/* USER CODE BEGIN 4 */
void userTimerISR(void)
{
  flagTimer.oneMs = TRUE;
  if(flag.fillOn == TRUE) {
    timeFill += 1;                                                              // Because of program cycle it was increased by 2
  }
}

void timer_process()
{
  tm1ms++;                                                                      // every 1ms it is added by one
  
  if( tm1ms < 2 ) return;
  tm1ms = 0;
  flagTimer.twoMs = TRUE;
  
  tm2ms++;                                                                      // every 2ms it is added by one
  if( tm2ms < 5 ) return;
  tm2ms = 0;
  flagTimer.tenMs = TRUE;
  
  
  tm10ms++;                                                                     // every 10 ms it is added by one, so 10*10 = 100ms
  if(tm10ms < 10) return;
  tm10ms = 0;
  flagTimer.hundredMs = TRUE;
  
  tm100ms++;                                                                    // every 100 ms it is added by one, so 100*10 = 1000ms = 1 second
  if(tm100ms < 10) return;
  tm100ms = 0;
  flagTimer.oneSecond = TRUE;
}

void delay300ns()                                       // 13 * 20 = 260ns --> Updated on 2022.07.12
{
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
