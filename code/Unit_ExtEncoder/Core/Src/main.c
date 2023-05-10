/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_ex.h"
#include "flash.h"
#include "modecfg.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION 2
#define I2C_ADDRESS 0x59
#define APPLICATION_ADDRESS     ((uint32_t)0x08001000) 
#define FLASH_DATA_SIZE 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
__IO float meter;
__IO int32_t meter_int32;
__IO int32_t perimeter = 1000;
__IO int32_t pulse = 2000;
__IO int32_t pulse_reg = 1000;
char temp_tx_char[9] = {0};
char temp_integer_part[8] = {0};
uint8_t flash_data[FLASH_DATA_SIZE] = {0};
uint8_t i2c_address[1] = {0};
volatile uint8_t fm_version = FIRMWARE_VERSION;
volatile uint8_t z_mode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//复制中断向量表到SRAM首地址
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //重映射 SRAM 地址到 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void z_gpio_setting(uint8_t mode)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  switch (mode)
  {
  case 0:
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    break;
  case 1:
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    break;
  case 2:
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    break;
  
  default:
    break;
  }
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void user_i2c_init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA9   ------> I2C1_SCL
  PA10   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x2000090E;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = i2c_address[0]<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void cover_data_to_flash(void)
{
  flash_data[0] = I2C_ADDRESS;
  flash_data[1] = 0;
  flash_data[2] = perimeter & 0xff;
  flash_data[3] = (perimeter >> 8) & 0xff;
  flash_data[4] = (perimeter >> 16) & 0xff;
  flash_data[5] = (perimeter >> 24) & 0xff;     
  flash_data[6] = pulse & 0xff;
  flash_data[7] = (pulse >> 8) & 0xff;
  flash_data[8] = (pulse >> 16) & 0xff;
  flash_data[9] = (pulse >> 24) & 0xff;    
}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    i2c_address[0] = I2C_ADDRESS;
    cover_data_to_flash();     
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    perimeter = flash_data[2] | (flash_data[3] << 8) | (flash_data[4] << 16) | (flash_data[5] << 24);
    pulse = flash_data[6] | (flash_data[7] << 8) | (flash_data[8] << 16) | (flash_data[9] << 24);
    pulse_reg = pulse / 2;
  }
  set_i2c_slave_address(i2c_address[0]);
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len)
{
  char temp_sign_part[2] = {0};
  char temp_decimal_part[4] = {0}; 
  uint8_t buf[4];
  uint8_t rx_buf[16];
  uint8_t rx_mark[16] = {0};  

  if (len == 1 && (rx_data[0] <= 0x03))
  {
    i2c1_set_send_data((uint8_t *)&encoder_countAB, 4);
  }    
  else if (len == 1 && (rx_data[0] >= 0x10 && rx_data[0] <= 0x13))
  {
    meter_int32 = (int32_t)(meter * 1000);
    i2c1_set_send_data((uint8_t *)&meter_int32, 4);
  }   
  else if (len == 1 && ((rx_data[0] >= 0x20) && (rx_data[0] <= 0x26)))
  {
	  int32_t temp, dec;

    if (meter > 0) {
      temp_sign_part[0] = 0x2B;
    } else {
      temp_sign_part[0] = 0x2D;
    }
    temp = ((int32_t)(meter * 1000));
    if (meter < 0) {
      temp = -temp;
    }
    dec = temp;
    temp = temp / 1000;
    dec = dec - temp * 1000;
    if (temp <= 9999)
      sprintf(temp_integer_part, "%04d", temp);
    if (dec <= 999)
      sprintf(temp_decimal_part, "%c%03d", '.', dec);
    memcpy(temp_tx_char, temp_sign_part, 1);
    memcpy(temp_tx_char+1, temp_integer_part, 4);
    memcpy(temp_tx_char+5, temp_decimal_part, 4);
    i2c1_set_send_data((uint8_t *)temp_tx_char, sizeof(temp_tx_char));  
  }  
  else if (len > 1 && (rx_data[0] == 0x30))
  {
    if (rx_data[1] == 1) {
      meter = 0;
      encoder_countAB = 0;
    }
  }     
  else if (len > 1 && ((rx_data[0] >= 0x40) && (rx_data[0] <= 0x43)))
  {
    for(int i = 0; i < len - 1; i++) {
      rx_buf[rx_data[0]-0x40+i] = rx_data[1+i];
      rx_mark[rx_data[0]-0x40+i] = 1;     
    }    
    if (rx_mark[0] && rx_mark[1] && rx_mark[2] && rx_mark[3]) {
      perimeter = (rx_buf[0] | (rx_buf[1] << 8) | (rx_buf[2] << 16) | (rx_buf[3] << 24));
      if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
        cover_data_to_flash();
        writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
      }
    }
	}
  else if (len == 1 && ((rx_data[0] >= 0x40) && (rx_data[0] <= 0x43)))
  {
    i2c1_set_send_data((uint8_t *)&perimeter, 4);
	}
  else if (len == 1 && ((rx_data[0] >= 0x50) && (rx_data[0] <= 0x53)))
  {
    i2c1_set_send_data((uint8_t *)&pulse_reg, 4);
	}
  else if (len > 1 && ((rx_data[0] >= 0x50) && (rx_data[0] <= 0x53)))
  {
    for(int i = 0; i < len - 1; i++) {
      rx_buf[rx_data[0]-0x50+i] = rx_data[1+i];
      rx_mark[rx_data[0]-0x50+i] = 1;     
    }    
    if (rx_mark[0] && rx_mark[1] && rx_mark[2] && rx_mark[3]) {
      pulse_reg = (rx_buf[0] | (rx_buf[1] << 8) | (rx_buf[2] << 16) | (rx_buf[3] << 24));
      pulse = pulse_reg * 2;
      if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
        cover_data_to_flash();
        writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
      }
    }
	}  
  else if (len == 1 && ((rx_data[0] >= 0x60) && (rx_data[0] <= 0x63)))
  {
    i2c1_set_send_data((uint8_t *)&encoder_counter, 4);
	}
  else if (len > 1 && ((rx_data[0] >= 0x60) && (rx_data[0] <= 0x63)))
  {
    for(int i = 0; i < len - 1; i++) {
      rx_buf[rx_data[0]-0x60+i] = rx_data[1+i];
      rx_mark[rx_data[0]-0x60+i] = 1;     
    }    
    if (rx_mark[0] && rx_mark[1] && rx_mark[2] && rx_mark[3]) {
      encoder_counter = (rx_buf[0] | (rx_buf[1] << 8) | (rx_buf[2] << 16) | (rx_buf[3] << 24));
    }
	}
  else if (len > 1 && (rx_data[0] == 0x70))
  {
    if (len == 2) {
      z_mode = rx_data[1];
      z_gpio_setting(z_mode);
    }
	}
  else if (len == 1 && (rx_data[0] == 0x70))
  {
    i2c1_set_send_data((uint8_t *)&z_mode, 1);
	}
  else if (len == 1 && (rx_data[0] == 0xFE))
  {
    i2c1_set_send_data((uint8_t *)&fm_version, 1);
  }  
  else if (len > 1 && (rx_data[0] == 0xFF))
  {
    if (len == 2) {
      if (rx_data[1] < 128) {
        if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
          i2c_address[0] = rx_data[1];
          flash_data[0] = i2c_address[0];
          flash_data[1] = 0;
          writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
          set_i2c_slave_address(i2c_address[0]);
          user_i2c_init();
        } 
      }
    }     
  }
  else if (len == 1 && (rx_data[0] == 0xFF))
  {
    i2c1_set_send_data(i2c_address, 1);    
  }       
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
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
  MX_GPIO_Init();
  // MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  advanceModeInit(0);
  init_flash_data();
  user_i2c_init();
  i2c1_it_enable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    encode_update();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C1 GPIO Configuration
  PA9   ------> I2C1_SCL
  PA10   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* I2C1 interrupt Init */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = 0x2000090E;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 0;
  I2C_InitStruct.OwnAddress1 = 0x59<<1;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2C1);
  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
