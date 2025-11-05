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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_ENABLED 0 // Set to 1 to enable printf, 0 to disable sdsd

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t calculate_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0x0000;       // Initial value
    uint16_t polynomial = 0x8001;
    uint16_t xorout = 0x0000;

    for (size_t i = 0; i < length; ++i) {
        crc ^= (data[i] << 8);   // XOR byte into high byte of CRC
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {  // If MSB is set
                crc = (crc << 1) & 0xFFFF;
                crc ^= polynomial;
            } else {
                crc = (crc << 1) & 0xFFFF;
            }
        }
    }
    return crc ^ xorout;
}


// Joint data structure matching Arduino response
typedef struct {
    uint8_t joint_id;
    int16_t motor_angle;      // In tenths of degrees
    int16_t gearbox_angle;    // In tenths of degrees
    uint8_t limit_switch_1;
    uint8_t limit_switch_2;
    uint8_t system_ready;
    uint8_t last_error;       // 0 = OK, 1 = Timeout, 2 = CRC Error
} JointData_t;

JointData_t joint1;  // Data for joint 1


// Redirect printf to UART2
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\n=== STM32 Clock Status ===\r\n");
  printf("SystemCoreClock: %lu Hz\r\n", SystemCoreClock);
  printf("HCLK: %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
  printf("PCLK1 (APB1): %lu Hz\r\n", HAL_RCC_GetPCLK1Freq());
  printf("PCLK2 (APB2): %lu Hz\r\n", HAL_RCC_GetPCLK2Freq());
  printf("========================\r\n\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint8_t poll_msg[6];
	      uint8_t response[13];
	      uint16_t crc_calculated, crc_received;
	      HAL_StatusTypeDef status;

	      // 1. Build the poll message for Joint 1
	      poll_msg[0] = 0x02; // STX
	      poll_msg[1] = 1;    // Joint ID
	      poll_msg[2] = 0x06; // Live Data Command

	      // 2. Calculate the correct CRC using your verified function
	      crc_calculated = calculate_crc16(poll_msg, 3);
	      poll_msg[3] = (crc_calculated >> 8) & 0xFF; // CRC High Byte
	      poll_msg[4] = crc_calculated & 0xFF;        // CRC Low Byte
	      poll_msg[5] = 0x03; // ETX

	      // Debug: Print the message we are about to send
	      if (DEBUG_ENABLED) {
			  printf("TX: ");
			  for (int i = 0; i < 6; i++) {
				  printf("%02X ", poll_msg[i]);
			  }
			  printf("\r\n");
	      }

	      // 3. Transmit the message
	      __HAL_UART_FLUSH_DRREGISTER(&huart1); // Clear any old data
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // Set MAX485 to Transmit mode
	      HAL_Delay(2);
	      HAL_UART_Transmit(&huart1, poll_msg, 6, 100);
	      HAL_Delay(2); // Ensure transmission completes
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Set MAX485 back to Receive mode

	      // 4. Wait for and receive the response
	      status = HAL_UART_Receive(&huart1, response, 13, 200);

	      // 5. Process the response
	      if (status == HAL_OK) {
	    	  if (DEBUG_ENABLED) {
				  printf("RX: ");
				  for (int i = 0; i < 13; i++) {
					  printf("%02X ", response[i]);
				  }
				  printf("\r\n");
	    	  }

	          // Verify frame and CRC of the received data
	          if (response[0] == 0x02 && response[12] == 0x03) {
	              crc_calculated = calculate_crc16(response, 10);
	              crc_received = ((uint16_t)response[10] << 8) | response[11];

	              if (crc_calculated == crc_received) {
	            	  if (DEBUG_ENABLED) {
	            		  printf("✓✓✓ CRC Matched!\r\n");
	            	  }

	                  // Parse the payload
	                  int16_t motor_angle_raw = ((int16_t)response[3] << 8) | response[4];
	                  int16_t gearbox_angle_raw = ((int16_t)response[5] << 8) | response[6];
	                  uint8_t limit_switches = response[7];
	                  uint8_t system_status = response[8];

//	                  if (DEBUG_ENABLED) {
						  // Print the decoded data
						  printf("✓ Joint %u | Motor: %.1f° | Gearbox: %.1f° | L1:%u L2:%u | Ready:%u\r\n\r\n",
								 response[1],
								 motor_angle_raw / 10.0f,
								 gearbox_angle_raw / 10.0f,
								 (limit_switches & 0x01),
								 (limit_switches & 0x02) >> 1,
								 system_status);
//	                  }

	                  // Blink the on-board LED (PC13) to indicate success
	                  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	              } else if (DEBUG_ENABLED) {
	                  printf("❌ RX CRC Mismatch! Expected: 0x%04X, Got: 0x%04X\r\n\r\n", crc_calculated, crc_received);
	              }
	          } else if (DEBUG_ENABLED) {
	              printf("❌ Invalid RX Frame Markers!\r\n\r\n");
	          }
	      } else if (status == HAL_TIMEOUT) {
	    	  if (DEBUG_ENABLED) {
	    		  printf("❌ Timeout - No response from Joint 1.\r\n\r\n");
	    	  }
	      } else if (DEBUG_ENABLED) {
	          printf("❌ UART Receive Error! Status: %d\r\n\r\n", status);
	      }

	      // Wait before polling again
	      HAL_Delay(20);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
