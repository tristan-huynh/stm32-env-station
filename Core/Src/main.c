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
#include "dht22.h"
#include "rgb_led.h"
#include <stdio.h>
#include <string.h>

#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "fan_control.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define GRAPH_HISTORY 64  // Number of data points to store
float temp_history[GRAPH_HISTORY] = {0};
float humid_history[GRAPH_HISTORY] = {0};
uint8_t history_index = 0;
uint8_t humid_index = 0;

uint8_t fan_status = 0;
uint8_t temperature_threshold = 20; 
uint8_t humidity_threshold = 50;
uint8_t fan_speed = 0; // 0-100%
uint8_t alert_condition = 0; // 0 = no alert, 1 = temp alert, 2 = humidity alert
uint8_t led_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void drawLineGraph(float* data, uint8_t length, uint8_t y_offset, uint8_t graph_height, float min_val, float max_val) {
    uint8_t y_axis_x = 22;  // Leave 22 pixels for labels
    uint8_t x_axis_y = y_offset + graph_height - 1;
    
    // x-axis
    for(int i = y_axis_x; i < 128; i++) {
        ssd1306_DrawPixel(i, x_axis_y, White);
    }
    
    // y-axis
    for(int i = y_offset; i < y_offset + graph_height; i++) {
        ssd1306_DrawPixel(y_axis_x, i, White);
    }

    char label[6];
    uint8_t num_ticks = 5;
    
    for(uint8_t i = 0; i < num_ticks; i++) {
        float value = max_val - (i * (max_val - min_val) / (num_ticks - 1));
        uint8_t tick_y = y_offset + (i * (graph_height - 1) / (num_ticks - 1));
        
        // Draw tick mark
        ssd1306_Line(y_axis_x - 2, tick_y, y_axis_x, tick_y, White);
        
        // Convert to integer for consistent formatting with your main display
        int value_int = (int)value;
        sprintf(label, "%d", value_int);  // Use %d for integer formatting
        
        // Ensure label doesn't go above screen bounds
        uint8_t label_y = tick_y;
        if (label_y < 4) label_y = 4;
        if (label_y > 60) label_y = 60;
        
        ssd1306_SetCursor(0, label_y - 4);
        ssd1306_WriteString(label, Font_6x8, White);
    }
    
    // Calculate X scaling
    float x_scale = (128.0f - y_axis_x - 1) / (length - 1);
    
    // Draw data points - only if we have valid data
    for(int i = 0; i < length - 1; i++) {
        // Skip drawing if data is zero (uninitialized)
        if (data[i] == 0.0f && data[i+1] == 0.0f) continue;
        
        uint8_t y1 = y_offset + graph_height - 2 - (uint8_t)((data[i] - min_val) / (max_val - min_val) * (graph_height - 3));
        uint8_t y2 = y_offset + graph_height - 2 - (uint8_t)((data[i+1] - min_val) / (max_val - min_val) * (graph_height - 3));
        
        uint8_t x1 = y_axis_x + (uint8_t)(i * x_scale);
        uint8_t x2 = y_axis_x + (uint8_t)((i + 1) * x_scale);
        
        // Clamp Y values
        if (y1 < y_offset) y1 = y_offset;
        if (y1 >= y_offset + graph_height) y1 = y_offset + graph_height - 1;
        if (y2 < y_offset) y2 = y_offset;
        if (y2 >= y_offset + graph_height) y2 = y_offset + graph_height - 1;
        
        ssd1306_Line(x1, y1, x2, y2, White);
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim3);

  RGB_Init();
  DHT22_Init(&htim3);
  FAN_Init();
  

  // TODO: Remove when done debugging
  uint8_t found_devices = 0;
  for(uint8_t addr = 0; addr < 128; addr++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 3, 100) == HAL_OK) {
          // Device found - blink green once
          RGB_SetRGB(0, 255, 0);
          HAL_Delay(500);
          RGB_SetRGB(0, 0, 0);
          HAL_Delay(500);
          found_devices++;
      }
  }
  if (found_devices == 0) {
      for(int i = 0; i < 5; i++) {
          RGB_SetRGB(255, 0, 0);
          HAL_Delay(200);
          RGB_SetRGB(0, 0, 0);
          HAL_Delay(200);
      }
  }

  ssd1306_Init();
  HAL_Delay(2000);

  // initalize motor controller code
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);    // IN1 = HIGH PC7
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);  // IN2 = LOW PA9 

  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);   // EN = HIGH PA11

  uint8_t screen = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    float temperature = 0.0f;
    float humidity = 0.0f;
    char buffer[32];
    
    DHT22_ReadData(&temperature, &humidity);

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) {
      screen++;
      if (screen > 4) {
        screen = 0;
      }
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET) {
      screen--;
      if (screen > 4) {
        screen = 4;
      }
    }

    if (alert_condition == 1) {
      RGB_SetRGB(255, 0, 0); // red for temperature alerta
    } else {
      RGB_SetRGB(0, 255, 0); // green for normal
    }

    if (screen == 0) {
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("1", Font_6x8, White);

      ssd1306_SetCursor(18, 0);
      ssd1306_WriteString("Current", Font_7x10, White);
      if (temperature != -999.0f && humidity != -999.0f) {
        // use integers for display 
        int temp_int = (int)temperature;
        int temp_dec = (int)((temperature - temp_int) * 10);
        int hum_int = (int)humidity;
        int hum_dec = (int)((humidity - hum_int) * 10);

        ssd1306_SetCursor(0, 15);
        sprintf(buffer, "Temp: %d.%dC", temp_int, temp_dec);
        ssd1306_WriteString(buffer, Font_7x10, White);
        
        ssd1306_SetCursor(0, 30);
        sprintf(buffer, "Humid: %d.%d%%", hum_int, hum_dec);
        ssd1306_WriteString(buffer, Font_7x10, White);

        ssd1306_SetCursor(0, 45);
        sprintf(buffer, "Fan: %d RPM", fan_speed);
        ssd1306_WriteString(buffer, Font_7x10, White);
      }
    } else if (screen == 2) {
      temp_history[history_index] = temperature;
      history_index = (history_index + 1) % GRAPH_HISTORY;
      
      ssd1306_Fill(Black);
      
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("2", Font_6x8, White);
      ssd1306_SetCursor(18, 0);
      ssd1306_WriteString("Temperature (C)", Font_7x10, White);
      drawLineGraph(temp_history, GRAPH_HISTORY, 12, 50, 10.0f, 40.0f);
      int temp_int = (int)temperature;
      int temp_dec = (int)((temperature - temp_int) * 10);
      ssd1306_SetCursor(100, 0);
      sprintf(buffer, "%d.%d%%", temp_int, temp_dec);
      ssd1306_WriteString(buffer, Font_7x10, White);
    } else if (screen == 3) {
      humid_history[humid_index] = humidity;
      humid_index = (humid_index + 1) % GRAPH_HISTORY;
      
      ssd1306_Fill(Black);
      
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("3", Font_6x8, White);

      ssd1306_SetCursor(18, 0);
      ssd1306_WriteString("Humidity (%)", Font_7x10, White);
      drawLineGraph(humid_history, GRAPH_HISTORY, 12, 50, 0.0f, 100.0f);
        int hum_int = (int)humidity;
        int hum_dec = (int)((humidity - hum_int) * 10);

      ssd1306_SetCursor(100, 0);
      sprintf(buffer, "%d.%d%%", hum_int, hum_dec);
      ssd1306_WriteString(buffer, Font_7x10, White);
    } else if (screen == 4) {
      ssd1306_Fill(Black);
      ssd1306_SetCursor(0, 0);
      ssd1306_WriteString("4", Font_6x8, White);
      ssd1306_SetCursor(18, 0);
      ssd1306_WriteString("Fan Control", Font_7x10, White);
      
      ssd1306_SetCursor(0, 15);
      sprintf(buffer, "Speed: %s", "FULL");
      ssd1306_WriteString(buffer, Font_7x10, White);
      
      ssd1306_SetCursor(0, 30);
      sprintf(buffer, "Status: ON");
      ssd1306_WriteString(buffer, Font_7x10, White);
      
      FAN_setSpeed(100);  

      ssd1306_SetCursor(0, 45);
      sprintf(buffer, "PA5:%d PA6:%d", 
              HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5),
              HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
      ssd1306_WriteString(buffer, Font_6x8, White);
    }
    ssd1306_UpdateScreen();
    HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;        // Alternate Function Open Drain
  GPIO_InitStruct.Pull = GPIO_NOPULL;            // External pull-ups (you have 5k)
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;     // I2C1 alternate function
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
