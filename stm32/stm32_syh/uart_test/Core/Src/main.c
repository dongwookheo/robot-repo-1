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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
uint8_t g_rx_buf[256] = {0, };
uint8_t g_recv_data[256] = {0, };
uint8_t g_rx_index = 0;
uint8_t data_len = 0;

int16_t g_ma_motor_speed = 0;
int16_t g_mb_motor_speed = 0;
int16_t g_mc_motor_speed = 0;
int16_t g_md_motor_speed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t process_protocol(void);
uint8_t calc_checksum(uint8_t*, uint8_t);
void send_resonse_protocol(uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		uint8_t len = 0;
		len = process_protocol();

		if(len != 0)
		{
			// response for command
			uint8_t cmd = g_recv_data[0];
			uint8_t need_res = g_recv_data[len - 1];

			if(cmd == 0x01)
			{
				g_mc_motor_speed = (int16_t)((g_recv_data[1] << 8) | g_recv_data[2]);
				g_md_motor_speed = (int16_t)((g_recv_data[3] << 8) | g_recv_data[4]);
				g_ma_motor_speed = (int16_t)((g_recv_data[5] << 8) | g_recv_data[6]);
				g_mb_motor_speed = (int16_t)((g_recv_data[7] << 8) | g_recv_data[8]);

				if(need_res)
				{
					send_resonse_protocol(len);
				}
			}

		}
		HAL_UART_Receive_IT(&huart1, &g_rx_buf[g_rx_index], 1);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, &g_rx_buf[g_rx_index], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

/* USER CODE BEGIN 4 */
uint8_t process_protocol()
{
	g_rx_index++;

	if(g_rx_index > 6)
	{
		if(g_rx_buf[g_rx_index - 1] == 0xFD)
		{
		  if(g_rx_buf[g_rx_index - 2] == 0xFA)
		  {
			uint8_t packet_len = g_rx_buf[g_rx_index - 4];
			// Check Header
			if((g_rx_buf[g_rx_index - packet_len - 5] == 0xFE) && (g_rx_buf[g_rx_index -packet_len - 6] == 0xFA))
			{
			  // Check checksum
			  uint8_t calc_crc = calc_checksum(&g_rx_buf[g_rx_index - packet_len - 4], packet_len + 1);

			  if(calc_crc == g_rx_buf[g_rx_index - 3])
			  {
				// Check completed.
				for(int i = 0; i < packet_len; i++)
				{
				  g_recv_data[i] = g_rx_buf[g_rx_index - packet_len - 4 + i];
				}

				memset(g_rx_buf, 0, 256);
				g_rx_index = 0;

				return packet_len;
			  }
			}
		  }
		}
	}

  return 0;
}

void send_resonse_protocol(uint8_t len)
{
	uint8_t send_data[len+6];

	//header
	send_data[0] = 0xFA;
	send_data[1] = 0xFE;

	// cmd + data + response
	for(int i = 0; i < len; i++)
	{
		send_data[2+i] = g_recv_data[i];
	}

	//LEN
	send_data[2+len] = len;

	//CMD에 0x90 더하기 : response를 보내는거니까
	send_data[2] = send_data[2] + 0x90;

	// checksum
	uint16_t sum = 0;
	for(int i = 0; i < 2+len; i++)
	{
		sum += send_data[i];
	}
	send_data[3+len] = (uint8_t)sum;

	// footer
	send_data[4+len] = 0xFA;
	send_data[5+len] = 0xFD;

	HAL_UART_Transmit(&huart1, send_data, len+6 ,10000);
}

uint8_t calc_checksum(uint8_t* data, uint8_t len)
{
	uint16_t sum = 0;
	for(int i = 0; i < len; i++)
	{
		sum += data[i];
	}

	return (uint8_t)sum;
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
