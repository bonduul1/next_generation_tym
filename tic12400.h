/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIC12400_H
#define __TIC12400_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
  
/* Definitions ---------------------------------------------------------------*/
#define SW_INT_Pin                      GPIO_PIN_10
#define SW_INT_GPIO_Port                GPIOB
#define SW_INT_EXTI_IRQn                EXTI15_10_IRQn

#define SW_CS_Pin                       GPIO_PIN_12
#define SW_CS_GPIO_Port                 GPIOB
  
#define NUMBER_OF_TIC12400                      1
#define TIC12400_FIRST_IC                       1
  
/* Definitions of Registers --------------------------------------------------*/
#define TIC12400_DEVICE_ID                      0x20

#define TIC12400_DEVICE_ID_REG                  0x01                            // Read only
#define TIC12400_INT_STAT_REG                   0x02                            // Read only
#define TIC12400_CRC_REG                        0x03                            // Read only
#define TIC12400_IN_STAT_MISC_REG               0x04                            // Read only
#define TIC12400_IN_STAT_COMP_REG               0x05                            // Read only
#define TIC12400_IN_STAT_ADC0_REG               0x06                            // Read only
#define TIC12400_IN_STAT_ADC1_REG               0x07                            // Read only
#define TIC12400_IN_STAT_MATRIX0_REG            0x08                            // Read only
#define TIC12400_IN_STAT_MATRIX1_REG            0x09                            // Read only
#define TIC12400_ANA_STAT0_REG                  0x0A                            // Read only
#define TIC12400_ANA_STAT1_REG                  0x0B                            // Read only
#define TIC12400_ANA_STAT2_REG                  0x0C                            // Read only
#define TIC12400_ANA_STAT3_REG                  0x0D                            // Read only
#define TIC12400_ANA_STAT4_REG                  0x0E                            // Read only
#define TIC12400_ANA_STAT5_REG                  0x0F                            // Read only
#define TIC12400_ANA_STAT6_REG                  0x10                            // Read only
#define TIC12400_ANA_STAT7_REG                  0x11                            // Read only
#define TIC12400_ANA_STAT8_REG                  0x12                            // Read only
#define TIC12400_ANA_STAT9_REG                  0x13                            // Read only
#define TIC12400_ANA_STAT10_REG                 0x14                            // Read only
#define TIC12400_ANA_STAT11_REG                 0x15                            // Read only
#define TIC12400_ANA_STAT12_REG                 0x16                            // Read only
                                                                                // 0x17 to 0x19 are reserved
#define TIC12400_CONFIG_REG                     0x1A                            // Read and write
#define TIC12400_IN_EN_REG                      0x1B                            // Read and write
#define TIC12400_CS_SELECT_REG                  0x1C                            // Read and write
#define TIC12400_WC_CFG0_REG                    0x1D                            // Read and write
#define TIC12400_WC_CFG1_REG                    0x1E                            // Read and write
#define TIC12400_CCP_CFG0_REG                   0x1F                            // Read and write
#define TIC12400_CCP_CFG1_REG                   0x20                            // Read and write
#define TIC12400_THRES_COMP_REG                 0x21                            // Read and write
#define TIC12400_INT_EN_COMP1_REG               0x22                            // Read and write
#define TIC12400_INT_EN_COMP2_REG               0x23                            // Read and write
#define TIC12400_INT_EN_CFG0_REG                0x24                            // Read and write
#define TIC12400_INT_EN_CFG1_REG                0x25                            // Read and write
#define TIC12400_INT_EN_CFG2_REG                0x26                            // Read and write
#define TIC12400_INT_EN_CFG3_REG                0x27                            // Read and write
#define TIC12400_INT_EN_CFG4_REG                0x28                            // Read and write
#define TIC12400_THRES_CFG0_REG                 0x29                            // Read and write
#define TIC12400_THRES_CFG1_REG                 0x2A                            // Read and write
#define TIC12400_THRES_CFG2_REG                 0x2B                            // Read and write
#define TIC12400_THRES_CFG3_REG                 0x2C                            // Read and write
#define TIC12400_THRES_CFG4_REG                 0x2D                            // Read and write
#define TIC12400_THRESMAP_CFG0_REG              0x2E                            // Read and write
#define TIC12400_THRESMAP_CFG1_REG              0x2F                            // Read and write
#define TIC12400_THRESMAP_CFG2_REG              0x30                            // Read and write
#define TIC12400_MATRIX_REG                     0x31                            // Read and write
#define TIC12400_MODE_REG                       0x32                            // Read and write

typedef union {
  uint32_t data;
  struct {
    uint32_t topLinkUp            : 1;
    uint32_t topLinkDown          : 1; 
    uint32_t transmit             : 1; 
    uint32_t res1                 : 1; 
    uint32_t transmissionOne      : 1; 
    uint32_t transmissionTwo      : 1; 
    uint32_t transmissionThree    : 1; 
    uint32_t transmissionFour     : 1;

    uint32_t res2                 : 1;
    uint32_t res3                 : 1;
    uint32_t transmissionL        : 1; 
    uint32_t transmissionM        : 1; 
    uint32_t parkingBrake         : 1; 
    uint32_t balanceUp            : 1; 
    uint32_t balanceDown          : 1; 
    uint32_t ptoManual            : 1; 
    
    uint32_t hitchManualUp        : 1; 
    uint32_t hitchManualDown      : 1; 
    uint32_t transmissionH        : 1;
    uint32_t model                : 1;
    uint32_t res5                 : 1;
    uint32_t footPedal            : 1;
    uint32_t res6                 : 1;
    uint32_t res7                 : 1;
    
    uint32_t ticStatus            : 8;
  } ;
} tic12400Input_t;

uint8_t  tic12400_Init(uint8_t selectedIC);  
uint8_t  tic12400_GPIO_Input_Init(void);

uint32_t tic12400_writeRegister(uint8_t address, uint32_t data);
uint32_t tic12400_readRegister(uint8_t address);
uint8_t  tic12400_getDeviceID();
uint32_t tic12400_getInputData();
uint32_t tic12400_getInputDataProved();
uint8_t  tic12400_getStatus(uint8_t fromIC);

void tic12400_disableSPI(uint8_t selectedIC);
void tic12400_enableSPI(uint8_t selectedIC);

#ifdef __cplusplus
}
#endif

#endif /* __TIC12400_H */
