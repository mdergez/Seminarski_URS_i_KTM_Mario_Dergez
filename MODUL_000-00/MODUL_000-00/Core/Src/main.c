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
#include "can.h"
#include "tim.h"
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

uint8_t flags = 0b00000000; //LSB flag0(timer condition), flag1(mutural condition), flag2(10s condition), flag3(1.5s condition)
uint8_t booleans = 0b00000000;
//0bxx00000000 ---> PWM signal rise or fall
//0b00xx000000 --->
//0b0000xx0000 ---> actuator on or off (11 == on)
//0b000000xx00 ---> current direction (00 == normal)
//0b00000000xx ---> actuator direction (11 == normal)


uint16_t PWM = 25;
uint16_t PWM_send = 0;
uint16_t time_counter = 0; //start with 0, max value 301

uint32_t current = 0;

//KTM CAN bus communication
uint8_t msg_flag = 0b00000000; // LSB - need to send data, 2nd from last - need to process received data, 3rd from last - there is received data, 4th from last - need to send request for data

//KTM end
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//KTM

//CAN bus Tx config
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailBox;
uint8_t TxData[8] = {0};

//CAN bus Rx config
CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8] = {0};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	msg_flag = msg_flag || 0b00000100; //when Rx interrupt is activated, change flag - there is data that needs to be processed

}

//KTM end

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
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //KTM
  HAL_CAN_Start(&hcan1);

  //CAN bus Tx config
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 127;

  //CAN bus Rx config
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);


//KTM end

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  //every 0.1s time_counter ++
	  if (!flags && __HAL_TIM_GET_COUNTER(&htim3) == htim3.Init.Period)
	  {
		  time_counter++;
		  flags = flags || 0b00000001; //put flag
	  }
	  else if (flags && __HAL_TIM_GET_COUNTER(&htim3) == 1)
	  {
		  flags = flags && 0b11111110; //reset flag
	  }



	  //PWM init value == 50(25+25)
	  //PWM signal sequence ---> 25->50->75->50->repeat

	  //every 30 sec change PWM and request data from modul 004-00
	  if ((time_counter % 300 == 0) && (!(flags >> 1)))
	  {
		  if (booleans >> 7)
		  {
			  PWM += 25;
			  if (PWM > 75)
			  {
				  PWM = 75;
				  booleans = booleans && 0b00111111; //change PWM seq to fall
			  }
		  }
		  else
		  {
			  PWM -= 25;
			  if (PWM < 25)
			  {
				  PWM = 25;
				  booleans = booleans || 0b11000000; //change PWM seq to rise
			  }
		  }


		  PWM_send = ((PWM/100) * 3900 + 100);

		  msg_flag = msg_flag || 0b00001001; //needs to send changed data and needs to request data
		  flags = flags || 0b00000010; //put on the flag
	  }

	  //every 10 sec change PWM
	  if ((time_counter % 100 == 0) && (!(flags >> 1)) && (!(flags >> 2)))
	  {
		  if (booleans >> 7)
		  {
			  PWM += 25;
			  if (PWM > 75)
			  {
				  PWM = 75;
				  booleans = booleans && 0b00111111; //change PWM seq to fall
			  }
		  }
		  else
		  {
			  PWM -= 25;
			  if (PWM < 25)
			  {
				  PWM = 25;
				  booleans = booleans || 0b11000000; //change PWM seq to rise
			  }
		  }


		  PWM_send = ((PWM/100) * 3900 + 100);

		  msg_flag = msg_flag || 0b00000001; //needs to send changed data
		  flags = flags || 0b00000100; //put on the flag
	  }

	  //every 1.5 sec request data from modul 004-00
	  if ((time_counter % 15 == 0) && (!(flags >> 1)) && (!(flags >> 3)))
	  {
		  msg_flag = msg_flag || 0b00001000; //needs to request data
		  flags = flags || 0b00001000; //put on the flag
	  }

	  //reset flags
	  if ((time_counter % 300 == 1) && (flags >> 1))
	  {
		  flags = flags && 0b111110001; //reset flags 1,2,3 (dont change flag0)
	  }
	  if ((time_counter % 100 == 1) && (flags >> 2))
	  {
		  flags = flags && 0b111111011; //reset flag 2
	  }
	  if ((time_counter % 15 == 1) && (flags >> 3))
	  {
		  flags = flags && 0b111110111; //reset flag 3
	  }


	  //CAN communication


	  if (msg_flag >> 2) //there is unprocessed received data
	  {
		  if (RxData[0] >> 1) //last 4 bits of RxData[0] high ---> needs to send data
		  {
			  msg_flag = msg_flag || 0b00000001;
		  }

		  if (RxData[0] >> 5) //first 4 bits of RxData[0] high ---> there is recieved data
		  {
			  msg_flag = msg_flag || 0b00000010;
		  }

		  msg_flag = msg_flag && 11111011; //reset flag
	  }
	  //send data
	  if (msg_flag) //needs to send data
	  {
		  TxData[0] = 0b11110000; //package contains data


		  TxData[1] = 0b00000000;
		  if (booleans)
		  {
			  TxData[1] = TxData[1] || 0b00000011; //actuator normal direction
		  }
		  if (booleans >> 5)
		  {
			  TxData[1] = TxData[1] || 0b00110000; //actuator is on
		  }


		  TxData[6] = PWM_send; //


		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
		  msg_flag = msg_flag && 0b11111110; //there is no more need to send data

		  TxData[0] = 0; //set array to 0
		  TxData[1] = 0;
		  TxData[2] = 0;
		  TxData[3] = 0;
		  TxData[4] = 0;
		  TxData[5] = 0;
		  TxData[6] = 0;
		  TxData[7] = 0;
	  }
	  if (msg_flag >> 3) //needs to send request
	  {
		  TxData[0] = 0b00001111; //package contains data request

		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
		  msg_flag = msg_flag && 0b11110111; //there is no more need to send data

		  TxData[0] = 0; //set array to 0
		  TxData[1] = 0;
		  TxData[2] = 0;
		  TxData[3] = 0;
		  TxData[4] = 0;
		  TxData[5] = 0;
		  TxData[6] = 0;
		  TxData[7] = 0;
	  }

	  if (msg_flag >> 1) //if there is received data, process it
	  {
		  if (RxData[1] >> 1) //actuator direction is normal
		  {
			  booleans = booleans || 0b00000011;
		  }
		  else //actuator direction is reverse
		  {
			  booleans = booleans && 0b11111100;
		  }
		  if (RxData[1] >> 3) //current is reverse
		  {
			  booleans = booleans || 0b00001100;
		  }
		  else //current is normal
		  {
			  booleans = booleans && 0b11110011;
		  }

		  if (RxData[1] >> 5) //actuator is turned on
		  {
			  booleans = booleans || 0b00110000;
		  }
		  else //actuator is turned off
		  {
			  booleans = booleans && 0b11001111;
		  }

		  //PWM_input = 0;
		  current = RxData[2];
		  //PWM_input = PWM_input << 8;
		  //PWM_input = PWM_input || RxData[7]

		  msg_flag = msg_flag && 0b11111101; //there is no more need to process data
		  RxData[0] = 0; //set array to 0
		  RxData[1] = 0;
		  RxData[2] = 0;
		  RxData[3] = 0;
		  RxData[4] = 0;
		  RxData[5] = 0;
		  RxData[6] = 0;
		  RxData[7] = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
