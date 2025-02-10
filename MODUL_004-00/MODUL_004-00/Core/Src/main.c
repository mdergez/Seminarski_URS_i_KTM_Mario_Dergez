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
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f7xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Default_PWM_value 0
#define Default_actuator_status 0b00100010
#define Default_PWM_scale_max 4000
#define Default_PWM_scale_min 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t sequence = 0;
uint8_t actuator_status = 0b00100010; //LSB - "is running" status, 2nd from last bit - "needs starting" status, 3rd from last bit - "PWM is present" status, 4th from last bit - "needs stopping" status, 5th from last bit - "actuator direction" status (normal = 1, reverse = 0), 6th from last bit - "turn of or on" - high on
uint8_t direction = 0b00000000; //LSB high --> current reverse direction
uint8_t coil_status = 0b00000000; //LSB - coil A, 2nd from last - coil B, 3rd from last - coil C
uint8_t coil_old_status = 0b00000000;
uint8_t last_change = 0b00000000; //from LSB to MSB ---> A-fall, A-rise, B-fall, B-rise, C-fall, C-rise

uint8_t digital_outputs = 0b00000000; //for 8 LED
uint8_t digital_old_outputs = 0b00000000; //used so that digital outputs change only if they need to

uint8_t tim_3_status = 0b00000000; //for communication - dont want transmit messages too fast - unused
uint8_t tim_4_status = 0b00000000; //for regulators - unused
uint8_t tim_5_status = 0b00000000; //MSB - timer on or off status, the rest is used for flags - used to count period between interrupts

uint16_t PWM_old_input = 0;
uint16_t PWM_input = 0; //PWM input
uint16_t PWM_value = 0; //scaled PWM input, desired PWM output (if regulators are present, they try to achieve that value)
uint16_t PWM_current_value = 0; //PWM output
uint16_t PWM_old_value = 0; //last PWM output
uint16_t ADC1_value; //ADC1 saves converted value inside this variable

uint32_t current = 0; //processed ADC1_value, in mA
uint32_t counter_1 = 0; //used to count time between interrupts (can be used to calculate speed if specs of actuator are know)
uint32_t counter_2 = 0; //used to count time between CAN bus messages - unused
uint32_t counter_3 = 0; //unused

uint32_t mils_current = 0; //ms counter for debouncing
uint32_t mils_old = 0;


//KTM CAN bus communication
uint8_t msg_flag = 0b00000000; // LSB - need to send data, 2nd from last - need to process received data, 3rd from last - there is received data, 4th from last - need to send request for data(not used on this module)

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //KTM
  HAL_CAN_Start(&hcan1);

  //CAN bus Tx config
  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 255;

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



	  //ADC get data
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 10); //do ADC every 10 ms

	  ADC1_value = HAL_ADC_GetValue(&hadc1);


	  //process ADC1 data
	  //reference voltage == 4096 milivolts
	  if (ADC1_value == 1650) //current sensor TCSM1123B5   --> 1.65V == 0A
	  {
		  current = 0;
	  }

	  //current sensor sensitivity == 150 mV/A
	  else if (ADC1_value > 1650)
	  {
		  current = (int)((float)((ADC1_value - 1650) * 1000) / 150); //current in mA
		  direction = direction && 0b11111110; //normal direction of current
	  }
	  else
	  {
		  current = (int)((float)((1650 - ADC1_value) * 1000) / 150); //current in mA
		  direction = direction || 0b00000001; //reverse direction of current
	  }


	  //KTM
	  //CAN bus data
	  //process CAN data
	  if (msg_flag >> 2) //there is unprocessed received data
	  {
		  if (RxData[0] >> 1) //last 4 bits of RxData[0] high ---> needs to send data
		  {
			  msg_flag = msg_flag || 0b00000001;
		  }

		  if (RxData[0] >> 5) //first 4 bits of RxData[0] high ---> there is received data
		  {
			  msg_flag = msg_flag || 0b00000010;
		  }

		  msg_flag = msg_flag &&11111011; //reset flag
	  }

	  if (msg_flag >> 1) //if there is received data, process it
	  {
		  if (RxData[1] >> 1) //actuator direction is set to normal
		  {
			  actuator_status = actuator_status || 0b00010000;
		  }
		  else //actuator direction is set to reverse
		  {
			  actuator_status = actuator_status && 0b11101111;
		  }

		  if (RxData[1] >> 5) //actuator is turned
		  {
			  actuator_status = actuator_status || 0b00100000;
		  }
		  else //actuator turned off
		  {
			  actuator_status = actuator_status && 0b11011111;
		  }

		  //PWM_input = 0;
		  PWM_input = RxData[6];
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

	  //KTM end


	  //process PWM
	  if (PWM_input != PWM_old_input) //there are changes to PWM_input value, recalculate
	  {
		  if (PWM_input < 100)
		  {
			  PWM_value = 0;
			  actuator_status = actuator_status && 0b11111011; //PWM is not present
			  actuator_status = actuator_status || 0b00001000; //actuator needs stopping
		  }

		  //if PWM is more than 100 then actuator status is set to running and PWM status is present
		  else if (PWM_input > 4000)
		  {
			  PWM_value = 100;
			  actuator_status = actuator_status || 0b00000101; //actuator is running and has PWM
		  }
		  else if (PWM_input > 4000)
		  {
			  PWM_value = (int)((float)((PWM_input-100)/3900)*100); // scale 100==0, 4000==100
			  actuator_status = actuator_status || 0b00000101; //actuator is running and has PWM
		  }
	  }


	  //actuator control


	  //running actuator
	  if ((actuator_status >> 2) && actuator_status && (actuator_status >> 5)) //actuator status is set to running, PWM is present and actuator is turned on
	  {
		  //start sequence
		  //does starting sequence until interrupts set "needs starting" status to low
		  if (actuator_status >> 1) //actuator needs starting
		  {
			  //start and reset timer 5 if not on
			  restart_tim_5();

			  //set coil B to GND and turn on PWM on coil A
			  if (last_change == 0b00000000)
			  {
				  AHigh_BLow(10); //PWM value is set to 10 (100max)
			  }
			  PWM_old_value = 10;
		  }

		  //running sequence
		  else
		  {
			  //regulator implementation
			  PWM_current_value = PWM_value;
			  //no regulator :)

			  if (!(actuator_status >> 4)) //normal direction
			  {
				  forward_sequence();
			  }
			  else //reverse direction
			  {
				  backward_sequence();
			  }

			  PWM_old_value = PWM_current_value;
		  }
	  }

	  //actuator is stopping
	  else if((actuator_status && (!(actuator_status >> 1)) && (!(actuator_status >> 2))) || (actuator_status && (!(actuator_status >> 5)))) //(actuator is running, doesnt need starting and PWM is not present) or (actuator is running but it is supposed to be turned off)
	  {
		  //disconnect power to actuator coils
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); //PWM coil A
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); //PWM coil B
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); //PWM coil C

		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

		  //connect actuator coils to GND
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); //GND coil A
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1); //GND coil B
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); //GND coil C

		  PWM_old_value = 0;
	  }

	  //actuator is not active
	  else
	  {
		  if ((tim_5_status >> 7) && (!tim_5_status)) //if timer 5 is on and flag is not set
		  {
			  //0.5s after it stopped moving
			  if (__HAL_TIM_GET_COUNTER(&htim5) == htim5.Init.Period)
			  {
				  actuator_status = actuator_status && 0b11111110; //change actuator status "is running" status to low
				  actuator_status = actuator_status || 0b00000010; //change actuator status "needs starting" status to high

				  //turn of timer 5 and reset timer
				  HAL_TIM_Base_Stop(&htim5);
				  __HAL_TIM_GET_COUNTER(&htim5) = 0;

				  //disconnect remaining connections to actuator coils
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0); //GND coil A
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0); //GND coil B
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0); //GND coil C

				  last_change = 0b00000000; //for starting sequence

				  tim_5_status = tim_5_status || 0b00000001; //put flag so it doesnt do this action each cycle
				  tim_5_status = tim_5_status && 0b01111111; //change timer status to inactive

				  PWM_old_value = 0;
			  }
		  }
	  }



	  //turn 8 LEDS on/off
	  //process PWM value to digital outputs
	  if (PWM_input != PWM_old_input)
	  {
		  if (PWM_value == 0)
		  {
			  digital_outputs = 0;
		  }
		  else
		  {
			  switch ((int)((float)((PWM_value + 12.5) /12.5)))
			  {
			  case 1:
				  digital_outputs = 0b00000001;
				  break;
			  case 2:
				  digital_outputs = 0b00000011;
				  break;
			  case 3:
				  digital_outputs = 0b00000111;
				  break;
			  case 4:
				  digital_outputs = 0b00001111;
				  break;
			  case 5:
				  digital_outputs = 0b00011111;
				  break;
			  case 6:
				  digital_outputs = 0b00111111;
				  break;
			  case 7:
				  digital_outputs = 0b01111111;
				  break;
			  case 8:
				  digital_outputs = 0b11111111;
				  break;
			  }
		  }

		  //set changes to LEDs (if there are any changes)
		  if (digital_outputs != digital_old_outputs)
		  {
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, digital_outputs >> 0);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, digital_outputs >> 1);
			  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, digital_outputs >> 2);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, digital_outputs >> 3);
			  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, digital_outputs >> 4);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, digital_outputs >> 5);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, digital_outputs >> 6);
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, digital_outputs >> 7);

			  digital_old_outputs = digital_outputs;
		  }
	  }

	  PWM_old_input = PWM_input;


	  //KTM
	  //CAN bus data
	  //send CAN data
	  if (msg_flag) //there is data to be sent
	  {
		  TxData[0] = 0b11110000; //package contains data


		  TxData[1] = 0b00000000;
		  if (actuator_status >> 4)
		  {
			  TxData[1] = TxData[1] || 0b00000011; //actuator normal direction
		  }
		  if (!(direction))
		  {
			  TxData[1] = TxData[1] || 0b00001100; //current reverse direction
		  }
		  if (actuator_status >> 5)
		  {
			  TxData[1] = TxData[1] || 0b00110000; //actuator is on
		  }


		  TxData[2] = current; //TxData 2,3,4,5 (32 bit number)


		  TxData[6] = ((PWM_value/100) * 3900 + 100); //scale 0-100 ---> 100-4000


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

	  //KTM end

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
void forward_sequence()
{
	if ((last_change != 0b11111111))
	{
		switch (last_change)
		{
		case 0b00000001: //A fall
			AHigh_BLow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00000010: //A rise
			BHigh_ALow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00000100: //B fall
			BHigh_CLow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00001000: //B rise
			CHigh_BLow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00010000: //C fall
			CHigh_ALow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00100000: //C rise
			AHigh_CLow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		}
	}
}

void backward_sequence()
{
	if (last_change != 0b11111111)
	{
		switch (last_change)
		{
		case 0b00000001: //A fall
			CHigh_ALow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00000010: //A rise
			AHigh_CLow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00000100: //B fall
			AHigh_BLow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00001000: //B rise
			BHigh_ALow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00010000: //C fall
			BHigh_CLow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		case 0b00100000: //C rise
			CHigh_BLow(PWM_current_value);
			last_change = 0b11111111;// change variable once PWM and GND pins are set so it doesnt do it on every cycle
			break;
		}
	}
}

void AHigh_BLow(uint16_t PWM)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM); //PWM coil A
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); //PWM coil B
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); //PWM coil C

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0); //GND coil A
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1); //GND coil B
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0); //GND coil C
}
void AHigh_CLow(uint16_t PWM)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM); //PWM coil A
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); //PWM coil B
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); //PWM coil C

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0); //GND coil A
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0); //GND coil B
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); //GND coil C
}
void BHigh_ALow(uint16_t PWM)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); //PWM coil A
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM); //PWM coil B
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); //PWM coil C

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); //GND coil A
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0); //GND coil B
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0); //GND coil C
}
void BHigh_CLow(uint16_t PWM)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); //PWM coil A
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM); //PWM coil B
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); //PWM coil C

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0); //GND coil A
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0); //GND coil B
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1); //GND coil C
}
void CHigh_ALow(uint16_t PWM)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); //PWM coil A
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); //PWM coil B
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM); //PWM coil C

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1); //GND coil A
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0); //GND coil B
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0); //GND coil C
}
void CHigh_BLow(uint16_t PWM)
{
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); //PWM coil A
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); //PWM coil B
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM); //PWM coil C

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0); //GND coil A
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1); //GND coil B
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0); //GND coil C
}

void restart_tim_3()
{
	if (!(tim_3_status >> 7)) //if timer is off, start it
	{
		HAL_TIM_Base_Start(&htim3);
		tim_3_status = tim_3_status || 0b1000000;
	}
	if (__HAL_TIM_GET_COUNTER(&htim3) != 0) //if timer counter is not 0, reset it
	{
		__HAL_TIM_GET_COUNTER(&htim3) = 0;
	}
}

void restart_tim_4()
{
	if (!(tim_4_status >> 7))//if timer is off, start it
	{
		HAL_TIM_Base_Start(&htim4);
		tim_4_status = tim_4_status || 0b1000000;
	}
	if (__HAL_TIM_GET_COUNTER(&htim4) != 0)//if timer counter is not 0, reset it
	{
		__HAL_TIM_GET_COUNTER(&htim4) = 0;
	}
}

void restart_tim_5()
{
	if (!(tim_5_status >> 7))//if timer is off, start it
	{
		HAL_TIM_Base_Start(&htim5);
		tim_5_status = tim_5_status || 0b1000000;
	}
	if (__HAL_TIM_GET_COUNTER(&htim5) != 0)//if timer counter is not 0, reset it
	{
		__HAL_TIM_GET_COUNTER(&htim5) = 0;
	}
}

void check_coil_status() //used in interrupts
{
	  //read pin PE2 (coil A)
	  switch(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2))
	  {
	  case 0:
		  coil_status = coil_status && 0b11111110; //status of coil A is lower than COM
		  break;
	  case 1:
		  coil_status = coil_status || 0b00000001; //status of coil A is higher than COM
		  break;
	  }

	  //read pin PE3 (coil B)
	  switch(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))
	  {
	  case 0:
		  coil_status = coil_status && 0b11111101; //status of coil B is lower than COM
		  break;
	  case 1:
		  coil_status = coil_status || 0b00000010; //status of coil B is higher than COM
		  break;
	  }

	  //read pin PE4 (coil C)
	  switch(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))
	  {
	  case 0:
		  coil_status = coil_status && 0b11111011; //status of coil C is lower than COM
		  break;
	  case 1:
		  coil_status = coil_status || 0b00000100; //status of coil C is higher than COM
		  break;
	  }

}

void find_change(uint8_t num) //used in interrupts
{
	switch (num)
	{
	case 0x0A:
		if(coil_status) //if current coil A status is high
		{
			last_change = 0b00000010; //coil A rise
		}
		else
		{
			last_change = 0b00000001; //coil A fall
		}
		break;
	case 0x0B: //if current coil B status is high
		if(coil_status>>1)
		{
			last_change = 0b00001000; //coil B rise
		}
		else
		{
			last_change = 0b00000100; //coil B fall
		}
		break;
	case 0x0C: //if current coil C status is high
		if(coil_status>>2)
		{
			last_change = 0b00100000; //coil C rise
		}
		else
		{
			last_change = 0b00010000; //coil C fall
		}
		break;
	}
	coil_old_status = coil_status;

}

void check_actuator_status() //used in interrupts
{
	if(actuator_status >> 1)// if actuator "needs starting" status is high, change it to low
	{
		actuator_status = actuator_status && 0b11111101;
	}
}




//interrupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //interrupt callback function
{
	GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
	switch(GPIO_Pin)
	{
	case GPIO_PIN_2: //interrupt on coil A

		// switch from interrupt to input
		GPIO_InitStructPrivate.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStructPrivate);


		check_coil_status();
		find_change(0x0A);// find out if its rising or falling edge

		//if interrupt is activated, actuator is moving -> "needs starting" status change to low
		check_actuator_status();

		//get time interval between interrupts (can be used to calculate speed)
		counter_2 = __HAL_TIM_GET_COUNTER(&htim5);
		restart_tim_5();

		//switch from input to interrupt
		GPIO_InitStructPrivate.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStructPrivate);

		break;
	case GPIO_PIN_3: //interrupt on coil B
		// switch from interrupt to input
		GPIO_InitStructPrivate.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStructPrivate);


		check_coil_status();
		find_change(0x0B);// find out if its rising or falling edge

		//if interrupt is activated, actuator is moving -> "needs starting" status change to low
		check_actuator_status();

		//get time interval between interrupts (can be used to calculate speed)
		counter_2 = __HAL_TIM_GET_COUNTER(&htim5);
		restart_tim_5();

		//switch from input to interrupt
		GPIO_InitStructPrivate.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStructPrivate);

		break;
	case GPIO_PIN_4:  //interrupt on coil C
		// switch from interrupt to input
		GPIO_InitStructPrivate.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStructPrivate);

		check_coil_status();
		find_change(0x0C);// find out if its rising or falling edge

		//if interrupt is activated, actuator is moving -> "needs starting" status change to low
		check_actuator_status();

		//get time interval between interrupts (can be used to calculate speed)
		counter_2 = __HAL_TIM_GET_COUNTER(&htim5);
		restart_tim_5();

		//switch from input to interrupt
		GPIO_InitStructPrivate.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
		GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStructPrivate);

		break;
	case GPIO_PIN_11: //interrupt on push button
		//button debouncing
		mils_current = HAL_GetTick();
		if (mils_current - mils_old > 10)
		{
			// switch from interrupt to input
			GPIO_InitStructPrivate.Pin = GPIO_PIN_11;
			GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
			GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
			GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOG, &GPIO_InitStructPrivate);

			//read button state
			//HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_11);

			if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_11))
			{
				if (actuator_status >> 5) //actuator is turned on, turn it off
				{
					actuator_status = actuator_status && 0b11011111;
				}
				else //turn it on
				{
					actuator_status = actuator_status || 0b00100000;
				}
			}


			//switch form input to interrupt
			GPIO_InitStructPrivate.Pin = GPIO_PIN_11;
			GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
			GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
			HAL_GPIO_Init(GPIOG, &GPIO_InitStructPrivate);

			mils_current = 0;
			mils_old = 0;

		}
		break;
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
