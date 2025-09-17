#include "tic12400.h"

#include "spi.h"

/* --------------------------------------------------------- Local variables ---------------------------------------------------------*/
uint8_t statusFlag;
uint32_t status;

extern void fram_disable();

void tic12400_disableSPI(uint8_t selectedIC)
{
  if(selectedIC == TIC12400_FIRST_IC)
  {
    HAL_GPIO_WritePin(SW_CS_GPIO_Port, SW_CS_Pin, GPIO_PIN_SET);
  }
}

void tic12400_enableSPI(uint8_t selectedIC)
{
  fram_disable();
  if(selectedIC == TIC12400_FIRST_IC)
  {
    HAL_GPIO_WritePin(SW_CS_GPIO_Port, SW_CS_Pin, GPIO_PIN_RESET);
  }
}

uint8_t tic12400_Init(uint8_t selectedIC)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  tic12400_disableSPI(selectedIC);
  
  if(selectedIC == TIC12400_FIRST_IC)
  {
    /*Configure GPIO pins : SW_CS_Pin */
    GPIO_InitStruct.Pin = SW_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SW_CS_GPIO_Port, &GPIO_InitStruct);

    // The implementation of SPI initialization is in "main.c"
    
    /*Configure GPIO pin : SW_INT_Pin */
    GPIO_InitStruct.Pin = SW_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SW_INT_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    return 1;
  }
  return 0;
}

uint8_t id_tic;
uint8_t tic12400_GPIO_Input_Init(void)
{
  uint32_t wr_data = 0;
  id_tic = tic12400_getDeviceID();
  if(id_tic)
  {
    // Configure input channels
    status = tic12400_readRegister(TIC12400_INT_STAT_REG);
    wr_data = 0xFFFFFF;
    tic12400_writeRegister(TIC12400_IN_EN_REG, wr_data);
    
    // Configure current source
    status = tic12400_readRegister(TIC12400_INT_STAT_REG);
    wr_data = 0;
    tic12400_writeRegister(TIC12400_CS_SELECT_REG, wr_data);
      
    // Configure wetting current as 0mA
    status = tic12400_readRegister(TIC12400_INT_STAT_REG);
    wr_data = 0;
    tic12400_writeRegister(TIC12400_WC_CFG0_REG, wr_data);
    
    // Configure wetting current as 0mA
    status = tic12400_readRegister(TIC12400_INT_STAT_REG);
    wr_data = 0;
    tic12400_writeRegister(TIC12400_WC_CFG1_REG, wr_data);
    
    // Configure mode 0 is select as switch / 1 is select as adc
    status = tic12400_readRegister(TIC12400_INT_STAT_REG);
    wr_data = 0;
    tic12400_writeRegister(TIC12400_MODE_REG, wr_data);
    
    // Configure select switch mode or polling mode and timing...(0x1A)
    status = tic12400_readRegister(TIC12400_INT_STAT_REG);
    wr_data = (0x1BA);
    tic12400_writeRegister(TIC12400_CONFIG_REG, wr_data);
    
    // Configure select threshold voltage
    status = tic12400_readRegister(TIC12400_INT_STAT_REG);
    wr_data = (0xAAA);
    tic12400_writeRegister(TIC12400_THRES_COMP_REG, wr_data);
    
    // start operation
    status = tic12400_readRegister(TIC12400_INT_STAT_REG);
    wr_data = (1 << 11);
    tic12400_writeRegister(TIC12400_CONFIG_REG, wr_data);
    return TRUE;
  }
  return FALSE;
}

uint32_t tic12400_getInputData()
{
  tic12400Input_t result;
  result.data = 0;
  // start operation
  status = tic12400_readRegister(TIC12400_INT_STAT_REG);
  result.data = (~tic12400_readRegister(TIC12400_IN_STAT_COMP_REG)) & 0x00FFFFFF;
  
  return result.data;
}

uint32_t tic12400_getInputDataProved()
{
  tic12400Input_t ret;
  static tic12400Input_t result[8];
  static uint8_t readingCounter = 0;
  
  result[readingCounter].data = 0;
  // start operation
  status = tic12400_readRegister(TIC12400_INT_STAT_REG);
  result[readingCounter].data = (~tic12400_readRegister(TIC12400_IN_STAT_COMP_REG)) & 0x00FFFFFF;
  
  readingCounter++;
  if(readingCounter >= 8) {
    readingCounter = 0;
  }
  
  ret.data = result[0].data & result[1].data & result[2].data & result[3].data & result[4].data & result[5].data & result[6].data & result[7].data;
  
  return ret.data;
}

uint8_t tic12400_calculate_parity(uint32_t data)
{
  uint8_t i;
  uint8_t ret;
  uint8_t ones = 0;
  uint32_t check_data = data;
  
  for(i = 0; i < 32; i++){
    if(check_data & 0x01){
      ones++;
    }
    check_data >>= 1;
  }
  if(ones & 0x01)
    ret = 1;
  else
    ret = 0;
  
  return ret;
}

uint32_t tic12400_readRegister(uint8_t address)
{
  uint32_t txData = (address << 25) & 0xFFFFFFFF;  
  uint32_t rxData;
  
  // calculate parity bit and put it at last bit
  // txData --> 31 is Read = 0
  // txData --> 30 to 25 is address
  // txData --> 24 to 1 is Do NOT care
  // txData --> 0 is Parity bit
  txData = txData | tic12400_calculate_parity(txData);
  
  // rxData --> 31 to 25 is Status flag
  // rxData --> 24 to 1  is data output
  // rxData --> 0 is Parity bit
  
  tic12400_enableSPI(TIC12400_FIRST_IC);
  delay300ns();
  rxData = SPI_TransmitReceive(SPI_TWO, txData, 32);
  delay300ns();
  tic12400_disableSPI(TIC12400_FIRST_IC);
    
  return rxData;
}

uint32_t tic12400_writeRegister(uint8_t address, uint32_t data)
{
  uint32_t txData = (address << 25) | 0x80000000 | (data << 1);
  uint32_t rxData;
  
  // calculate parity bit and put it at last bit
  // txData --> 31 is Write = 1
  // txData --> 30 to 25 is address
  // txData --> 24 to 1 is data
  // txData --> 0 is Parity bit
//  txData = txData | tic12400_calculate_parity(txData);
  
  // rxData --> 31 to 25 is Status flag
  // rxData --> 24 to 1  is previous content of the register addressed
  // rxData --> 0 is Parity bit
  
  tic12400_enableSPI(TIC12400_FIRST_IC);
  delay300ns();
  rxData = SPI_TransmitReceive(SPI_TWO, txData, 32);
  delay300ns();
  tic12400_disableSPI(TIC12400_FIRST_IC);
  return rxData;
}

uint8_t tic12400_getDeviceID()
{
  return ((tic12400_readRegister(TIC12400_DEVICE_ID_REG) & 0xFF) == TIC12400_DEVICE_ID) ? 1 : 0;
}

uint8_t tic12400_getStatus(uint8_t fromIC)
{
  return (fromIC == 1) ? status = tic12400_readRegister(TIC12400_INT_STAT_REG) : status;
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}