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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
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
TIM_HandleTypeDef htim1;

IRDA_HandleTypeDef hirda1;

/* USER CODE BEGIN PV */

typedef enum{
		DESLIGADO,
		BOTAO_1,
		BOTAO_2,
		BOTAO_3,
		BOTAO_4,
		BOTAO_5,
		BOTAO_6,
		BOTAO_7,
		BOTAO_8,
		BOTAO_9,
		OK,
		SETA_BAIXO,
		SETA_ESQUERDA,
		SETA_CIMA,
		SETA_DIREITA,
		ASTERISCO,
		HASHTAG
} botoes_e;

botoes_e recebeu;

uint8_t data, contagem;
uint16_t micros;
//uint16_t
uint32_t teste=0;
uint8_t recebeu;
float delay;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_IRDA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void delay_us  ( uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0 );   // define o valor do contador como 0
	while  (__HAL_TIM_GET_COUNTER(&htim1)  <  us);   // espera o contador alcançar a entrada us no parâmetro
	//return us;
}

uint32_t recebendo_data(){

	while((HAL_GPIO_ReadPin(infra_GPIO_Port, infra_Pin))==0);
	 while(HAL_GPIO_ReadPin(infra_GPIO_Port, infra_Pin)==1);

	 for(int i = 0; i < 32; i++)
		  		 {
		  			 while((HAL_GPIO_ReadPin(infra_GPIO_Port, infra_Pin))==0);

		  			 contagem = 0;
		  			 while(HAL_GPIO_ReadPin(infra_GPIO_Port, infra_Pin)==1){
		  				 contagem++;
		  				 //micros=delay_us(100);
		  				 delay_us(100);
		  				 //delay = delay_us(0.0001);

		  			 }

		  			 if(contagem > 12)
		  			 {
		  				 //teste = 0;
		  				teste |= (1UL << (31-i));
		  			 }

		  			 else
		  			 {
		  				 //teste = 0;

		  				teste &= ~ (1UL << (31-i));
		  			 }
		  		 }
	 return teste;
}
void convert_code (uint32_t code)
{
	switch (code)
	{
		case (0xff4ab5):
			recebeu = DESLIGADO;
			break;

		case (0xff6897):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_1;
			break;

		case (0xff9867):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_2;
			break;

		case (0xffb04f):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_3;
			break;

		case (0xff30cf):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_4;
			break;

		case (0xff18e7):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_5;
			break;

		case (0xff7a85):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_6;
			break;

		case (0xff10ef):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_7;
			break;

		case (0xff38c7):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_8;
			break;

		case (0xff5aa5):
			//lcd_send_cmd (0x86);
			recebeu = BOTAO_9;
			break;

		case (0xff02fd):
			recebeu = OK;
			break;

		case (0xffa857):
			recebeu = SETA_BAIXO;
			break;

		case (0xff22dd):
			recebeu = SETA_ESQUERDA;
			break;

		case (0xff629d):
			recebeu = SETA_CIMA;
			break;

		case (0xffc23d):
			recebeu = SETA_DIREITA;
			break;

		case (0xff42bd):
			recebeu = ASTERISCO;
			break;

		case (0xff52ad):
			recebeu = HASHTAG;
			break;


		default :
			break;
		}
}

/*void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){

}*/
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
  MX_USART1_IRDA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //DWT_Delay_Init ();
  //micros=__HAL_TIM_GET_AUTORELOAD(&htim1);
  //micros=micros/9900;
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  while(HAL_GPIO_ReadPin(infra_GPIO_Port, infra_Pin)==1);

	  data = recebendo_data ();

	  		convert_code (teste);

	  		HAL_Delay (200);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_IRDA_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  hirda1.Instance = USART1;
  hirda1.Init.BaudRate = 115200;
  hirda1.Init.WordLength = IRDA_WORDLENGTH_8B;
  hirda1.Init.Parity = IRDA_PARITY_EVEN;
  hirda1.Init.Mode = IRDA_MODE_RX;
  hirda1.Init.Prescaler = 10;
  hirda1.Init.PowerMode = IRDA_POWERMODE_NORMAL;
  hirda1.Init.ClockPrescaler = IRDA_PRESCALER_DIV1;
  if (HAL_IRDA_Init(&hirda1) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : infra_Pin */
  GPIO_InitStruct.Pin = infra_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(infra_GPIO_Port, &GPIO_InitStruct);

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

