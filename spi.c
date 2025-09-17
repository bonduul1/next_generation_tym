#include "spi.h"
#include "main.h"
/* --------------------------------------------------------- STM32 SPI variables ---------------------------------------------------*/
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* --------------------------------------------------------- Local functions ---------------------------------------------------*/
uint32_t softwareSPI2_TransmitReceive(uint32_t txData, uint8_t len);
uint32_t softwareSPI3_TransmitReceive(uint32_t txData, uint8_t len);

/* --------------------------------------------------------- Variables ---------------------------------------------------*/
uint8_t isSPI2UseHardware;
uint8_t isSPI3UseHardware;

uint32_t SPI_TransmitReceive(uint8_t channel, uint32_t txData, uint8_t len)
{
  uint8_t _rxData[4];
  uint8_t _txData[4];
  uint32_t ret = 0;
  
  _txData[0] = (uint8_t)(txData >> 24);
  _txData[1] = (uint8_t)(txData >> 16);
  _txData[2] = (uint8_t)(txData >> 8);
  _txData[3] = (uint8_t)(txData);
  
  if(channel == SPI_ONE){
  }
  else if(channel == SPI_TWO){
    if(isSPI2UseHardware == HARDWARE_SPI)
    {
      HAL_SPI_TransmitReceive (&hspi2, _txData, _rxData, (len/8), 100);
      ret = (_rxData[0] << 24) | (_rxData[1] << 16) | (_rxData[2] << 8) | _rxData[3];
    }
    else if(isSPI2UseHardware == SOFTWARE_SPI)
    {
      ret = softwareSPI2_TransmitReceive(txData, len);
    }
  }
  else if(channel == SPI_THREE){
    if(isSPI3UseHardware == HARDWARE_SPI)
    {
      if(len == 32) {
        HAL_SPI_TransmitReceive (&hspi3, _txData, _rxData, (len/8), 100);
        /*
        HAL_SPI_TransmitReceive (&hspi3, &_txData[0], &_rxData[0], 1, 100);
        HAL_SPI_TransmitReceive (&hspi3, &_txData[1], &_rxData[1], 1, 100);
        HAL_SPI_TransmitReceive (&hspi3, &_txData[2], &_rxData[2], 1, 100);
        HAL_SPI_TransmitReceive (&hspi3, &_txData[3], &_rxData[3], 1, 100);
          */
        ret = (_rxData[0] << 24) | (_rxData[1] << 16) | (_rxData[2] << 8) | _rxData[3];
        //ret = (_rxData[3] << 24) | (_rxData[2] << 16) | (_rxData[1] << 8) | _rxData[0];
      }
      else if(len == 8) {
        _txData[0] = _txData[3];
        HAL_SPI_TransmitReceive (&hspi3, _txData, _rxData, 1, 100);
        ret = _rxData[0];
      }
    }
    else if(isSPI3UseHardware == SOFTWARE_SPI)
    {
      ret = softwareSPI3_TransmitReceive(txData, len);
    }
  }
  return ret;
}

void delaySCK()                 // 13 * 5 = 65ns --> 13 * 8 = 104ns ( Changed on 2022.07.12 because of BTS72220 datasheet, minimum time is 90ns, minimum one cycle is 200ns)
{
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");
  __asm volatile("NOP");

}

uint32_t softwareSPI2_TransmitReceive(uint32_t txData, uint8_t len)
{
  uint8_t i;
  uint32_t rxData = 0;
  __disable_irq();
  if(len == 32)
  {
    for(i = 0; i < len; i++)
    {
      rxData <<= 1;
      spi2_sck_high();
      delaySCK();
      
      if(txData & 0x80000000){
        spi2_mosi_high();
      }else{
        spi2_mosi_low();
      }
      txData <<= 1; 
      
      spi2_sck_low();
      delaySCK();
      if(spi2_miso()){
        rxData |= 0x00000001;
      }
    }
  }
  else if(len == 8)
  {
    for(i = 0; i < len; i++)
    {
      rxData <<= 1;
      spi2_sck_high();
      delaySCK();
      
      if(txData & 0x80) {
        spi2_mosi_high();
      } else {
        spi2_mosi_low();
      }
      txData <<= 1; 
      
      spi2_sck_low();
      delaySCK();
      if(spi2_miso()) {
        rxData |= 0x01;
      }
    }
  }
  __enable_irq();
  return rxData;
}

uint32_t softwareSPI3_TransmitReceive(uint32_t txData, uint8_t len)
{
  uint8_t i;
  uint32_t rxData = 0;
  
  __disable_irq();
  if(len == 32)
  {
    for(i = 0; i < len; i++)
    {
      rxData <<= 1;
      spi3_sck_high();
      delaySCK();
      
      if(txData & 0x80000000){
        spi3_mosi_high();
      }else{
        spi3_mosi_low();
      }
      txData <<= 1; 
      
      spi3_sck_low();
      delaySCK();
      if(spi3_miso()){
        rxData |= 0x00000001;
      }
    }
  }
  else if(len == 8)
  {
    delaySCK();
    
    for(i = 0; i < len; i++)
    {
      rxData <<= 1;
      spi3_sck_high();
      delaySCK();
      
      if(txData & 0x80) {
        spi3_mosi_high();
      } else {
        spi3_mosi_low();
      }
      txData <<= 1; 

      spi3_sck_low();
      delaySCK();
      if(spi3_miso()) {
        rxData |= 0x01;
      }
    }
  }
  
  __enable_irq();
  return rxData;
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
  isSPI2UseHardware = HARDWARE_SPI;
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */
  isSPI3UseHardware = HARDWARE_SPI;
  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;                                        // Phase edge is changed on 2022.07.14
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x1D;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

void SOFT_SPI2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  isSPI2UseHardware = SOFTWARE_SPI;
}

void SOFT_SPI3_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  isSPI3UseHardware = SOFTWARE_SPI;
}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
  else if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_SPI3_ENABLE();

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
  else if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }

}