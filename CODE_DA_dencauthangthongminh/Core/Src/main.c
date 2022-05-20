/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body 
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 *//*Tien*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*                      <3 Trang pig -- pisces girl <3                        */
/*                                    3000                                    */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t TT_time_watting = 2000u;
uint32_t TT_time_step = 100u;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TT_effect_init()
{
  HAL_GPIO_WritePin(GPIOB, b1_Pin | b2_Pin | b3_Pin | b4_Pin | b5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, b6_Pin | b7_Pin | b8_Pin | b9_Pin | b10_Pin, GPIO_PIN_RESET);
}

void TT_set_effect_led(uint32_t data)
{
  HAL_GPIO_WritePin(b1_GPIO_Port, b1_Pin, (GPIO_PinState)((data >> 0) & 0x1));
  HAL_GPIO_WritePin(b2_GPIO_Port, b2_Pin, (GPIO_PinState)((data >> 1) & 0x1));
  HAL_GPIO_WritePin(b3_GPIO_Port, b3_Pin, (GPIO_PinState)((data >> 2) & 0x1));
  HAL_GPIO_WritePin(b4_GPIO_Port, b4_Pin, (GPIO_PinState)((data >> 3) & 0x1));
  HAL_GPIO_WritePin(b5_GPIO_Port, b5_Pin, (GPIO_PinState)((data >> 4) & 0x1));
  HAL_GPIO_WritePin(b6_GPIO_Port, b6_Pin, (GPIO_PinState)((data >> 5) & 0x1));
  HAL_GPIO_WritePin(b7_GPIO_Port, b7_Pin, (GPIO_PinState)((data >> 6) & 0x1));
  HAL_GPIO_WritePin(b8_GPIO_Port, b8_Pin, (GPIO_PinState)((data >> 7) & 0x1));
  HAL_GPIO_WritePin(b9_GPIO_Port, b9_Pin, (GPIO_PinState)((data >> 8) & 0x1));
  HAL_GPIO_WritePin(b10_GPIO_Port, b10_Pin, (GPIO_PinState)((data >> 9) & 0x1));
}

void TT_effect(uint32_t time, uint32_t wait_time)
{
  HAL_GPIO_WritePin(b1_GPIO_Port, b1_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b2_GPIO_Port, b2_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b3_GPIO_Port, b3_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b4_GPIO_Port, b4_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b5_GPIO_Port, b5_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b6_GPIO_Port, b6_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b7_GPIO_Port, b7_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b8_GPIO_Port, b8_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b9_GPIO_Port, b9_Pin, GPIO_PIN_RESET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b10_GPIO_Port, b10_Pin, GPIO_PIN_RESET);
  HAL_Delay(wait_time);

  HAL_GPIO_WritePin(b1_GPIO_Port, b1_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b2_GPIO_Port, b2_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b3_GPIO_Port, b3_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b4_GPIO_Port, b4_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b5_GPIO_Port, b5_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b6_GPIO_Port, b6_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b7_GPIO_Port, b7_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b8_GPIO_Port, b8_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b9_GPIO_Port, b9_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
  HAL_GPIO_WritePin(b10_GPIO_Port, b10_Pin, GPIO_PIN_SET);
  HAL_Delay(time);
}


void TT_effect1() // bat duoi len
{
  uint16_t led = 0;
  for(int i = 0; i<=10; i++)
  {
    HAL_Delay(TT_time_step);
    TT_set_effect_led(led);
    led |= 1 << i;
  }
}
void TT_effect2() // tat duoi len
{
  uint16_t led = 0x3FF;
  for(int i = 0; i<=10; i++)
  {
    HAL_Delay(TT_time_step);
    TT_set_effect_led(led);
    led &= ~(1 << i);
  }
}

void TT_effect3() // bat tren xuong
{
  uint16_t led = 0;
  for(int i = 0; i<=10; i++)
  {
    HAL_Delay(TT_time_step);
    TT_set_effect_led(led);
    led |= (0x200) >> i;
  }
}

void TT_effect4() // tat tren xuong
{
    uint16_t led = 0x3FF;
    for(int i = 0; i<=10; i++)
    {
        HAL_Delay(TT_time_step);
        TT_set_effect_led(led);
        led &= ~(0x200 >> i);
    }
}

void TT_effect5() //sang so le
{
    TT_set_effect_led(0x5555u);
    HAL_Delay(TT_time_step);
    TT_set_effect_led(0xAAAAu);
    HAL_Delay(TT_time_step);
}
void TT_dim_light(void)
{
  HAL_GPIO_TogglePin(b1_GPIO_Port, b1_Pin);
  HAL_GPIO_TogglePin(b2_GPIO_Port, b2_Pin);
  HAL_GPIO_TogglePin(b3_GPIO_Port, b3_Pin);
  HAL_GPIO_TogglePin(b4_GPIO_Port, b4_Pin);
  HAL_GPIO_TogglePin(b5_GPIO_Port, b5_Pin);
  HAL_GPIO_TogglePin(b6_GPIO_Port, b6_Pin);
  HAL_GPIO_TogglePin(b7_GPIO_Port, b7_Pin);
  HAL_GPIO_TogglePin(b8_GPIO_Port, b8_Pin);
  HAL_GPIO_TogglePin(b9_GPIO_Port, b9_Pin);
  HAL_GPIO_TogglePin(b10_GPIO_Port, b10_Pin);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  TT_effect_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*                      <3 Trang pig -- pisces girl <3                        */
/*                                    3000                                    */
      
      
//    effect1(70, 300);

    if(HAL_GPIO_ReadPin(cb_tren_GPIO_Port, cb_tren_Pin) == GPIO_PIN_RESET) // duoi len
    {
//        HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1);
        TT_effect1();
        HAL_Delay(TT_time_watting);
        TT_effect2();
        TT_set_effect_led(0);
    }
    else if (HAL_GPIO_ReadPin(cb_duoi_GPIO_Port, cb_duoi_Pin) == GPIO_PIN_RESET) //tren xuong
    {
//        HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1);
        TT_effect3();
        HAL_Delay(TT_time_watting);
        TT_effect4();
        TT_set_effect_led(0);
    }
    else
    {
        if (HAL_GPIO_ReadPin(cb_as_GPIO_Port,cb_as_Pin) == GPIO_PIN_SET)
        {
            if(HAL_GPIO_ReadPin(cb_cd_GPIO_Port,cb_cd_Pin) == GPIO_PIN_SET)
            {
//                HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1);
                TT_set_effect_led(0x3FF);
            }
            else
            {
                TT_set_effect_led(0u);
            }
        }
        else
        {
//            HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1);
            TT_set_effect_led(0u);
        }
    }

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, b13_Pin|b14_Pin|b15_Pin|b16_Pin
                          |b5_Pin|b4_Pin|b3_Pin|b1_Pin
                          |b2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, b10_Pin|b9_Pin|b8_Pin|b7_Pin
                          |b6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : btn1_Pin btn2_Pin */
  GPIO_InitStruct.Pin = btn1_Pin|btn2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : cb_power_Pin cb_as_Pin cb_cd_Pin */
  GPIO_InitStruct.Pin = cb_power_Pin|cb_as_Pin|cb_cd_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : cb_tren_Pin cb_duoi_Pin */
  GPIO_InitStruct.Pin = cb_tren_Pin|cb_duoi_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : b13_Pin b14_Pin b15_Pin b16_Pin
                           b5_Pin b4_Pin b3_Pin b1_Pin
                           b2_Pin */
  GPIO_InitStruct.Pin = b13_Pin|b14_Pin|b15_Pin|b16_Pin
                          |b5_Pin|b4_Pin|b3_Pin|b1_Pin
                          |b2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : b10_Pin b9_Pin b8_Pin b7_Pin
                           b6_Pin */
  GPIO_InitStruct.Pin = b10_Pin|b9_Pin|b8_Pin|b7_Pin
                          |b6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
