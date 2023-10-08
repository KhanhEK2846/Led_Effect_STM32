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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//khai bao trang thai hieu ung led
typedef enum
{
	LED_BLINK_ORIGIN,
	LED_BLINK_SEQ,
	LED_BLINK_ALTER,
} Led_Effect_Status;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//khai bao bien luu trang thai
Led_Effect_Status led_eff_state;

//----- UART com debug variable -----
uint8_t Tx_Buffer[50] = "Hello CE437"; //buffer truyen di
uint16_t Tx_flag = 0;
//----- UART com debug variable -----

//---- led_time_cycle [min:max] variable ----
uint16_t max_cycle_led_time = 2000, min_cycle_led_time = 0;
//---- led_time_cycle [min:max] variable ----

//---- Led counter effect variable -----
uint8_t count_led_alt = 0, count_led_seq = 0;
//---- Led counter effect variable -----

uint16_t divine_time_val = 0; //bien dung de luu gia tri chia deu thoi gian hieu ung led (bam xung - pwm thu cong)

//test_var
int count = 0;

//---- button handle variable ----
uint8_t button_current = 1; //trang thai hien tai cua nut bam, khoi tao la 1 de tranh dieu kien ban da
uint8_t button_last = 1; //trang thai cuoi cua nut bam
uint8_t button_filter = 1;
uint8_t is_debouncing; //biet nhan biet nut dang rung do thay doi trang thai
uint32_t time_debounce; //lay thoi gian thay doi trang thai
uint32_t time_btn_press; //luu thoi gian ke tu luc nhan nut
uint8_t is_press_timeout, longpress_timeout;
uint32_t time_longpress_keep;
//---- button handle variable ----


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void led_blink_origin(void); //effect 1
void led_blink_sequence(void); //effect 2
void led_blink_alternate(void); //effect 3
void button_hanlde(void);
void button_pressing_callback(void);
void button_release_callback(void);
void button_shortpressing_callback_500ms(void); //button1 < 500ms handle fnct
void button_longpressing_callback_500ms(void); ////button1 > 500ms handle fnct
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int test;

void button_handle(void)
{
	//-------- Xu ly loc nhieu ------
	//khong dung timer interrupt phuc vu loc nhieu
	//giai thuat kieu:
	//1. phat hien thay doi trang thai
	//2. cu co nhieu, reset time_debounce
	uint8_t sta = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	//cu co nhieu gan lai button filter
	if(sta != button_filter) //trang thai doc duoc, khac voi trang thai truoc (button_filter)
	{
		//co su thay doi trang thai thi se vao day gan lien tuc vao button filter
		button_filter = sta;
		is_debouncing = 1;
		time_debounce = HAL_GetTick(); //moi lan co xung nhieu time debounce duoc cap nhat lai
	}
	//-------- Xu ly loc nhieu ------
	//-------- Xac lap tin hieu ------
	if(is_debouncing == 1 && (HAL_GetTick()-time_debounce >= 15)) //check nhieu
	{
		button_current = button_filter;
		is_debouncing = 0;
	}
	//-------- Xac lap tin hieu ------
	//-------- Xu ly nut nhan --------
	if(button_current != button_last)
	{
		if(button_current == 0) //truong hop dang nhan nut
		{
			//nhan xuong lam gi do thi nen viet ham callback - lap trinh nen quen callback - Nhan nut tao ra su kien
			//button_pressing_callback();
			is_press_timeout = 1;
			time_btn_press = HAL_GetTick();
		} else { //nha nut
			//nha nut se lam su kien gi day - viet dang callback
			//tuc la khoang thoi gian ke tu luc nhan cho den luc nha ra < 500ms
			if(HAL_GetTick() - time_btn_press <= 1000)
			{
				test = 1;
				button_shortpressing_callback_500ms();
			}
			//button_release_callback();
		}
		button_last = button_current; //muc dich neu trang thai nut bam sau khi xac lap khac trang thai truoc thi xu ly callback
	}
	if(is_press_timeout == 1 && (HAL_GetTick() - time_btn_press >= 3000))
	{
			button_longpressing_callback_500ms();
			is_press_timeout = 0;
			//longpress_timeout = 1;
			test++;
			//time_longpress_keep = HAL_GetTick();
	}
	//-------- Xu ly nut nhan --------
}

void button_pressing_callback(void)
{
	//test giam chu ky led timer
	//chua tinh truong hop nhan nhanh, nhan lau
	//thu nhan doi hieu ung
	//count++;
	//tam thoi do nothing
}

void button_shortpressing_callback_500ms(void)
{
	if(max_cycle_led_time == 0)
	{
		max_cycle_led_time = 2000;
	}
	max_cycle_led_time -= 100;
}


void button_longpressing_callback_500ms(void)
{
	//sau moi khoang thoi gian 200ms duoc nhan giu
	//giam chu ky led di dan tuc la "nut duoc nhan va giu hoai luon cu moi 200ms thi -100ms max-cycle
	if(longpress_timeout && (HAL_GetTick() - time_longpress_keep == 200))
	{
		test = 3;
		if(max_cycle_led_time == 0)
		{
			max_cycle_led_time = 2000;
		}
		max_cycle_led_time -= 100;
	}
}

void button_release_callback(void)
{
	//do nothinng
	//co the lam gi do nhung chua biet la nen lam gi
}


void led_blink_origin(void)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
}

void led_blink_sequence(void)
{
	if(count_led_seq == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		count_led_seq += 1;
	} else if(count_led_seq == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		count_led_seq += 1;
	} else if (count_led_seq == 2)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		count_led_seq += 1;
	} else if (count_led_seq == 3)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		count_led_seq = 0;
	}
}

void led_blink_alternate(void)
{
	if(count_led_alt == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET); //off
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET); //on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); //off
		count_led_alt += 1;
	} else if (count_led_alt == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET); //on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET); //off
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); //on
		count_led_alt = 0;
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
  led_eff_state = LED_BLINK_ORIGIN; //ke tu luc chuong trinh bat dau se vao hieu ung blink led
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE BEGIN 2 */
  //uint8_t btn_state = 0;
  //uint8_t btn2_state = 0;
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//----- Phan code test debug UART gui den man hinh OLED -----
//	if(Tx_flag)
//	{
//		 HAL_UART_Transmit(&huart1, Tx_Buffer,strlen(Tx_Buffer), 100);
//	}
//	HAL_Delay(300); // cu moi 300 ms gui 1 lan - thu 2 len truong test
	//----- Phan code test debug UART gui den man hinh OLED -----
	 button_handle();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim3.Init.Prescaler = 3599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = led_TIM_counter;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3599;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = led_TIM_counter;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDR_Pin|LEDG_Pin|LEDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDR_Pin LEDG_Pin LEDB_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin|LEDG_Pin|LEDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //!WARNING: Don't change
{
	static uint16_t led_timer_cnt = 0;
	if(htim == &htim3) //behavior led function timer
	{
		if(max_cycle_led_time > 0)
		{
			led_timer_cnt++;
			switch(led_eff_state)
			{
				case LED_BLINK_ORIGIN:
				{
					divine_time_val = max_cycle_led_time / 2;
					if(led_timer_cnt == divine_time_val){
						led_blink_origin();
						led_timer_cnt = 0;
					}
					break;
				}
				case LED_BLINK_SEQ:
				{
					divine_time_val = max_cycle_led_time / 4;
					if(led_timer_cnt == divine_time_val){
						led_blink_sequence();
						led_timer_cnt = 0;
					}
					break;
				}
				case LED_BLINK_ALTER:
				{
					divine_time_val = max_cycle_led_time / 2;
					if(led_timer_cnt == divine_time_val){
						led_blink_alternate();
						led_timer_cnt = 0;
					}
					break;
				}
				default:
					break;
			}
		} else if(max_cycle_led_time == 0)
		{
			max_cycle_led_time = 2000; //neu chu ky bi nut bam tuong tac giam ve 0 thi reset lai thoi gian chu ky
		}
	}
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
