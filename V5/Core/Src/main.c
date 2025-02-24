/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#include "rplidar.h"
#include "byteswap.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UART_RX_BUFFER_SIZE 2048
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t rx_buffer[UART_RX_BUFFER_SIZE] = {0};
scan_data sample[UART_RX_BUFFER_SIZE * 2] = {0};
uint8_t half = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (half == 0) {
		memcpy(sample, rx_buffer, Size);
		half = 1;
	}
	else {
		memcpy(sample, rx_buffer + UART_RX_BUFFER_SIZE, Size);
		half = 0;
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, 1024);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, 1024);
}

enum state_scan state_lidar = STANDBY;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//  uint8_t dma_buffer[0] = {0};
  descriptor desc_res = {0};
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
  MX_FDCAN2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LIDAR_PWM_GPIO_Port, LIDAR_PWM_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if (HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin) != GPIO_PIN_RESET) {
		switch (state_lidar) {
		case STANDBY:
			// Start Lidar rotation (PWM pin -> on)
			printf("on\n\r");
			HAL_GPIO_WritePin(LIDAR_PWM_GPIO_Port, LIDAR_PWM_Pin, GPIO_PIN_SET);
			HAL_Delay(1000);

			state_lidar = REQUEST;
			printf("req\n\r");
			break;

		case REQUEST:
			// Request
			desc_res = new_req(&huart1, SCAN);

			state_lidar = DESCRIPTOR;
			printf("desc\n\r");
			break;

		case DESCRIPTOR:
			// Read descriptor
			if (desc_res.start_flag1 != START_FLAG1) {
				state_lidar = UART_ERROR;
				printf("error flag 1\n\r");
			}
			else if (desc_res.start_flag2 != START_FLAG2) {
				state_lidar = UART_ERROR;
				printf("error flag 2\n\r");
			}
			else if (desc_res.res_length_mode != RES_LENGTH_MODE) {
				state_lidar = UART_ERROR;
				printf("error reslength\n\r");
			}
			else if (desc_res.type != DATA_TYPE) {
				state_lidar = UART_ERROR;
				printf("error type 1\n\r");
			}
			else {
				state_lidar = SCANNING; // Everything fine !
				printf("scan\n\r");
			}
			break;

		case SCANNING:
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &rx_buffer, UART_RX_BUFFER_SIZE);
			break;
	//
	//		case STOPPING:
	//			stop(&huart1);
	//			HAL_Delay(1000);
	//			state_lidar = STANDBY;
	//
	//			break;
		case UART_ERROR:
			break;

		default:
			printf("error\n\r");
			state_lidar = ERROR;

			break;
		}

		// start scanning
		//		  scan_data res_data;
		//		  HAL_Delay(500);
		//
		//		  while (HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin)) {
		//			  get_res_data(&huart1, (uint8_t *)&res_data, &res_desc);
		//			  HAL_Delay(100);
		//			  HAL_UART_Transmit(&huart2, (uint8_t *)&res_data, (uint16_t)sizeof(res_data), 500);
		//			  HAL_Delay(100);
		//			  HAL_UART_Transmit(&huart2, (uint8_t *)"\n\r", 2, 500);
		//			  HAL_Delay(100);
		//		  }
		if (HAL_GPIO_ReadPin(AU_GPIO_Port, AU_Pin) == GPIO_PIN_RESET) {
			printf("off\n\r");
			stop(&huart1);
			HAL_GPIO_WritePin(LIDAR_PWM_GPIO_Port, LIDAR_PWM_Pin, GPIO_PIN_RESET);
			state_lidar = STANDBY;
		}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
