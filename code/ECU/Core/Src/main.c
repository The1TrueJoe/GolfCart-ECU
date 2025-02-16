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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

// UART Receive Buffer
#define UART_RX_BUFFER_SIZE 2
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void setDutyCycle(TIM_HandleTypeDef* const htim, uint32_t channel, float duty_cycle) {
	if (duty_cycle > 100) duty_cycle = 100;
	if (duty_cycle < 0) duty_cycle = 0;

	float pw_resolution = (((float)(*htim).Init.Period + 1.0f) / 100.0f);

	uint16_t pw_desired = pw_resolution * duty_cycle;
	__HAL_TIM_SET_COMPARE(htim, channel, pw_desired);
}

// Increment speed controller (MCP4151 using SPI4)
void increment_speed() {
  // Set the CS pin low
  HAL_GPIO_WritePin(Speed_CS_GPIO_Port, Speed_CS_Pin, GPIO_PIN_RESET);
  // Send the increment command
  HAL_SPI_Transmit(&hspi4, 0x01, 1, 100);
  // Set the CS pin high
  HAL_GPIO_WritePin(Speed_CS_GPIO_Port, Speed_CS_Pin, GPIO_PIN_SET);
}

// Decrement speed controller (MCP4151 using SPI4)
void decrement_speed() {
  // Set the CS pin low
  HAL_GPIO_WritePin(Speed_CS_GPIO_Port, Speed_CS_Pin, GPIO_PIN_RESET);
  // Send the decrement command
  HAL_SPI_Transmit(&hspi4, 0x02, 1, 100);
  // Set the CS pin high
  HAL_GPIO_WritePin(Speed_CS_GPIO_Port, Speed_CS_Pin, GPIO_PIN_SET);
}

// Set the speed controller (MCP4151 using SPI4)
void set_speed(uint8_t speed) {
  // Set the CS pin low
  HAL_GPIO_WritePin(Speed_CS_GPIO_Port, Speed_CS_Pin, GPIO_PIN_RESET);
  // Send the set command
  HAL_SPI_Transmit(&hspi4, 0x00, 1, 100);
  // Send the speed value
  HAL_SPI_Transmit(&hspi4, speed, 1, 100);
  // Set the CS pin high
  HAL_GPIO_WritePin(Speed_CS_GPIO_Port, Speed_CS_Pin, GPIO_PIN_SET);
}

// Enable the drive relay
void enable_drive() {
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);
}

// Disable the drive relay
void disable_drive() {
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);
}

// Enable the forward relay
void enable_forward() {
  HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_SET);
}

// Enable the reverse relay
void enable_reverse() {
  HAL_GPIO_WritePin(Direction_GPIO_Port, Direction_Pin, GPIO_PIN_RESET);
}


/* 
  Motor 0:
  Steering Left
  Enable: Steering_L_EN
  Timer 2 Channel 4
  PWM: Steering_L_PWM
  Motor 1:
  Steering Right
  Enable: Steering_R_EN
  Timer 2 Channel 1
  PWM: Steering_R_PWM
  Motor 2:
  Brake Left
  Enable: Brake_L_EN
  Timer 3 Channel 4
  PWM: Brake_L_PWM
  Motor 3:
  Brake Right
  Enable: Brake_R_EN
  Timer 3 Channel 3
  PWM: Brake_R_PWM 
*/

// Run Steering Motor Forward
void steer_forward() {
  // Disable the other motor
  HAL_GPIO_WritePin(Steering_R_EN_GPIO_Port, Steering_R_EN_Pin, GPIO_PIN_RESET);

  // Write the PWM value
  setDutyCycle(&htim2, TIM_CHANNEL_1, 0);
  setDutyCycle(&htim2, TIM_CHANNEL_4, 100);

  // Enable the motor
  HAL_GPIO_WritePin(Steering_L_EN_GPIO_Port, Steering_L_EN_Pin, GPIO_PIN_SET);

}

// Run Steering Motor Backward
void steer_backward() {
  // Disable the other motor
  HAL_GPIO_WritePin(Steering_L_EN_GPIO_Port, Steering_L_EN_Pin, GPIO_PIN_RESET);

  // Write the PWM value
  setDutyCycle(&htim2, TIM_CHANNEL_4, 0);
  setDutyCycle(&htim2, TIM_CHANNEL_1, 100);

  // Enable the motor
  HAL_GPIO_WritePin(Steering_R_EN_GPIO_Port, Steering_R_EN_Pin, GPIO_PIN_SET);
}

// Stop the Steering Motor
void steer_stop() {
  // Disable the motor
  HAL_GPIO_WritePin(Steering_L_EN_GPIO_Port, Steering_L_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Steering_R_EN_GPIO_Port, Steering_R_EN_Pin, GPIO_PIN_RESET);
}

// Run Brake Motor Forward
void brake_forward() {
  // Disable the other motor
  HAL_GPIO_WritePin(Brake_R_EN_GPIO_Port, Brake_R_EN_Pin, GPIO_PIN_RESET);

  // Write the PWM value
  setDutyCycle(&htim3, TIM_CHANNEL_3, 0);
  setDutyCycle(&htim3, TIM_CHANNEL_4, 100);

  // Enable the motor
  HAL_GPIO_WritePin(Brake_L_EN_GPIO_Port, Brake_L_EN_Pin, GPIO_PIN_SET);
}

// Run Brake Motor Backward
void brake_backward() {
  // Disable the other motor
  HAL_GPIO_WritePin(Brake_L_EN_GPIO_Port, Brake_L_EN_Pin, GPIO_PIN_RESET);

  // Write the PWM value
  setDutyCycle(&htim3, TIM_CHANNEL_4, 0);
  setDutyCycle(&htim3, TIM_CHANNEL_3, 100);

  // Enable the motor
  HAL_GPIO_WritePin(Brake_R_EN_GPIO_Port, Brake_R_EN_Pin, GPIO_PIN_SET);
}

// Stop the Brake Motor
void brake_stop() {
  // Disable the motor
  HAL_GPIO_WritePin(Brake_L_EN_GPIO_Port, Brake_L_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Brake_R_EN_GPIO_Port, Brake_R_EN_Pin, GPIO_PIN_RESET);
}

// Read the UART data from uart6
void read_uart_data() {
  // Read the UART data
  HAL_UART_Receive_DMA(&huart6, uart_rx_buffer, UART_RX_BUFFER_SIZE);
}

// Decoder function from the UART data buffer. 
void decode_uart_data() {
  // Check the first byte of the buffer
  switch (uart_rx_buffer[0]) {
    case 0x01: // Drive
      // Check the second byte of the buffer
      switch (uart_rx_buffer[1]) {
        case 0x01: // Forward
          // Enable the drive relay
          enable_drive();
          // Enable the forward relay
          enable_forward();
          break;
        case 0x02: // Reverse
          // Enable the drive relay
          enable_drive();
          // Enable the reverse relay
          enable_reverse();
          break;
        case 0x03: // Stop
          // Disable the drive relay
          disable_drive();
          break;
      }
      break;
    case 0x02: // Steering
      // Check the second byte of the buffer
      switch (uart_rx_buffer[1]) {
        case 0x01: // Forward
          // Run the steering motor forward
          steer_forward();
          break;
        case 0x02: // Reverse
          // Run the steering motor backward
          steer_backward();
          break;
        case 0x03: // Stop
          // Stop the steering motor
          steer_stop();
          break;
      }
      break;
    case 0x03: // Brake
      // Check the second byte of the buffer
      switch (uart_rx_buffer[1]) {
        case 0x01: // Forward
          // Run the brake motor forward
          brake_forward();
          break;
        case 0x02: // Reverse
          // Run the brake motor backward
          brake_backward();
          break;
        case 0x03: // Stop
          // Stop the brake motor
          brake_stop();
          break;
      }
      break;
    case 0x04: // Speed with 2 preset speeds
      // Check the second byte of the buffer
      switch (uart_rx_buffer[1]) {
        case 0x01: // Low
          // Set the speed to low
          set_speed(64);
          break;
        case 0x02: // High
          // Set the speed to high
          set_speed(128);
          break;
        case 0x03: // Increment
          // Increment the speed
          increment_speed();
          break;
        case 0x04: // Decrement
          // Decrement the speed
          decrement_speed();
          break;
        case 0x05: // Stop
          // Stop the drive motor
          set_speed(0x00);
          disable_drive();
          break;
      }
      break;
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	read_uart_data();
	decode_uart_data();

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 6;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
