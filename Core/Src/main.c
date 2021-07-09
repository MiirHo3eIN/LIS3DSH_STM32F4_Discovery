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
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
  int16_t x_raw; 
  int16_t y_raw; 
  int16_t z_raw; 
}RawDataValues;

typedef struct 
{
  float x_raw; 
  float y_raw; 
  float z_raw; 

}mGVibrationsTypeDef;

RawDataValues raw_data; 
mGVibrationsTypeDef mG_data; 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define __LIS_CS_EN     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET); 
#define __LIS_CS_DIS    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET); 

#define REG_SET_SUCCESSFULL     1U
#define REG_SET_FAIL            0U

#define SWITCH_RED_LED_ON           HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
#define TOGGLE_RED_LED              HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); 


#define SWITCH_GREEN_LED_ON         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
#define SWITCH_GREEN_LED_OFF        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const uint8_t REG3_ADDR = 0x23;     // for interrupt
const uint8_t REG4_ADDR = 0x20; 
const uint8_t REG5_ADDR = 0x24; 

const uint8_t REG3_DATA = 0x88;     //for interrupt 
const uint8_t REG4_DATA = 0x67;     
const uint8_t REG5_DATA = 0x68;

const uint8_t OUT_X_L_ADDR = 0x28; 
const uint8_t OUT_X_H_ADDR = 0x29; 

const uint8_t OUT_Y_L_ADDR = 0x2A; 
const uint8_t OUT_Y_H_ADDR = 0x2B; 

const uint8_t OUT_Z_L_ADDR = 0x2C; 
const uint8_t OUT_Z_H_ADDR = 0x2D; 

uint16_t full_value;
int dx = 0U; 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int LIS_Write(uint8_t addr, uint8_t data); 
uint16_t LIS_Read_Raw_Data(uint8_t addr_low, uint8_t addr_high);
void LIS_Init(void);
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
  /* USER CODE BEGIN 1 */


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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  __LIS_CS_DIS;

  // Configuring the LIS with our configurations 
   
  LIS_Init(); 

   

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (dx == 1)
    {
        dx = 0; 
        raw_data.x_raw = LIS_Read_Raw_Data(OUT_X_L_ADDR, OUT_X_H_ADDR);
        raw_data.y_raw = LIS_Read_Raw_Data(OUT_Y_L_ADDR, OUT_Y_H_ADDR);
        raw_data.z_raw = LIS_Read_Raw_Data(OUT_Z_L_ADDR, OUT_Z_H_ADDR);
        
        /* this is not needed for the PCA trainning... */ 
        mG_data.x_raw  = raw_data.x_raw *  0.122;
        mG_data.y_raw  = raw_data.y_raw *  0.122;
        mG_data.z_raw  = raw_data.z_raw *  0.122;
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int LIS_Write(uint8_t addr, uint8_t data)
{
    uint8_t rx_buf[1];
    uint8_t tx_buf[2];
    __LIS_CS_EN; 
    tx_buf[0] = addr; 		// for Write
    tx_buf[1] = data;
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&tx_buf[0], 1, 100);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&tx_buf[1], 1, 100);
    __LIS_CS_DIS;

    __LIS_CS_EN;
    tx_buf[0] = addr | 0x80;  // for read
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&tx_buf[0], 1, 100);
    HAL_SPI_Receive(&hspi1, (uint8_t *)&rx_buf[0], 1, 100);
    __LIS_CS_DIS;

    if (rx_buf[0] == data)         return REG_SET_SUCCESSFULL;
    else                           return REG_SET_FAIL;
}

uint16_t LIS_Read_Raw_Data(uint8_t addr_low, uint8_t addr_high)
{
    uint8_t tx_buff; 
    uint8_t raw_data_high;
    uint8_t raw_data_low;

    __LIS_CS_EN;
    tx_buff = addr_high | 0x80 ; 
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&tx_buff, 1, 100); 
    HAL_SPI_Receive(&hspi1, (uint8_t *)&raw_data_high, 1, 100); 
    __LIS_CS_DIS; 


    __LIS_CS_EN; 
    tx_buff = addr_high | 0x80; 
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&tx_buff, 1, 100);
    HAL_SPI_Receive(&hspi1, (uint8_t *)&raw_data_low, 1, 100);
    __LIS_CS_DIS; 


    return  (raw_data_low | raw_data_high<<8) ;
    
}


void LIS_Init(void)
{


  int status; 
  status = LIS_Write(REG3_ADDR, REG3_DATA);
  


  if (status == REG_SET_FAIL)
  {
          while (1)
              {
                        TOGGLE_RED_LED;
                        SWITCH_GREEN_LED_OFF;
                        HAL_Delay(300);
              }
  } 
  status = LIS_Write(REG4_ADDR, REG4_DATA); 
  if (status == REG_SET_FAIL)
  {
          while (1)
              {
                        TOGGLE_RED_LED;
                        SWITCH_GREEN_LED_OFF;
                        HAL_Delay(300);
              }
  }
  status = LIS_Write(REG5_ADDR, REG5_DATA); 
  if (status == REG_SET_FAIL)
  {
          while (1)
              {
                        TOGGLE_RED_LED;
                        SWITCH_GREEN_LED_OFF;
                        HAL_Delay(300);
              }
  } 
  if (status == REG_SET_SUCCESSFULL)    SWITCH_GREEN_LED_ON;
}



 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
 // UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

  	  	  	  dx =1;
              
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
