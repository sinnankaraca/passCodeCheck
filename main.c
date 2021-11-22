/* USER CODE BEGIN Header */
/*******************************************************************************
 * File Name          : main.c
 * Description        : This is a password management project, user with valid
 * passcode can change the any passcode he/she desires or see the all passcodes.
 * Green and Red Leds are showing the successfull or not successfull input.
 *
 * Author:              Sinan KARACA
 * Date:                27.06.2021
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

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

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

uint8_t rxData[4]; // Buffer array for receive data from uart

uint8_t cmdstate = 0; // cmdstate checking the user input it is written.

uint8_t receiveMode = 4; // The variable shows the 4 byte or 1 byte serial input modes

// 10 passcode database
uint8_t passCodes[10][4] = { "1000", "0050", "2587", "1234", "8854", "4253", "1921","2015", "3095", "3574" };

uint8_t set = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void comparePasscode(void);
void checkValidity(int);
void verifiedLogin(void);
void callForVerifiedLogin(void);
void changeReceiveMode(int);
void changePasscode(void);
void loginApproved (void);
void errorOccured (void);
void welcomeScreen(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FUNCTION      : changeSpeakerFrequency
// DESCRIPTION   :
//   Control the frequency on TIM1 PWM output on PA7
// PARAMETERS    :
//   *htim - pointer to hal timer structure for this timer
//   newFrequency - actual frequency desired
//
// RETURNS       :
//   Nothing
// Code is created by Allan Smith
static void changeSpeakerFrequency(TIM_HandleTypeDef *htim,
		uint32_t newFrequency) {

		HAL_TIMEx_PWMN_Stop(&htim, TIM_CHANNEL_1);


	// calculate the new period based off of frequency input
	uint32_t newPeriod = 1000000000 / (newFrequency * 250);

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	htim->Instance = TIM1;
	htim->Init.Prescaler = 0;
	htim->Init.CounterMode = TIM_COUNTERMODE_UP;
	htim->Init.Period = newPeriod;
	htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim->Init.RepetitionCounter = 0;
	htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(htim) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(htim) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = htim1.Init.Period/2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(htim, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_MspPostInit(htim);

	// must restart the timer once changes are complete
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // Initializing the receive huart2 with receive mode = 4.
  HAL_UART_Receive_IT(&huart2, (uint8_t*) &rxData, receiveMode);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		welcomeScreen(); // reset all the leds

		//USER WELCOME INTERFACE
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n----------------------------------------\r\n", 44, 1000);
		HAL_UART_Transmit(&huart2, (uint8_t*) "Welcome to Password Manangement System\r\n", 40, 1000);
		HAL_UART_Transmit(&huart2, (uint8_t*) "Please type your 4 digit passcode:\r\n", 36, 1000);

		//Compare the passcode with the database in passcode array database
		//Check the validity of 4 digit input
		//Indicate with leds
		comparePasscode();

		//Verified login, the user can see the all passcodes or change any of them.
		verifiedLogin();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, Led3_Pin|Led2_Pin|Led1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Led3_Pin Led2_Pin Led1_Pin */
  GPIO_InitStruct.Pin = Led3_Pin|Led2_Pin|Led1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// FUNCTION      : comparePasscode
// DESCRIPTION   :
//    The function created to ensure the user input inside the database
//    Get 4 digits from user and check the validity
//	  Afterwards compare with the database and indicate with led
// PARAMETERS    :
//                 Void
// RETURNS       :
//				   Void

void comparePasscode(void) {

	changeReceiveMode(4); // Change to 4 digit serial input mode
	checkValidity(4); // Check 4 bit user input

	while (1) {
		// Listen the serial line
		if (cmdstate == 1) {
			int check = 0;

			// for loop for compare user input with [10][4] array
			for (int i = 0; i < 10; i++) {
				for (int y = 0; y < 4; y++) {
					if (rxData[y] == passCodes[i][y]) {
						check = check + 1;
					}
				}
				if (check == 4) {
					HAL_UART_Transmit(&huart2, (uint8_t*) "   -  Access Granted\r\n", 22, 1000);
					HAL_UART_Transmit(&huart2,(uint8_t*) "----------------------------------------\r\n", 42, 1000);
					loginApproved();
					cmdstate = 0;
					return;
				} else if (i == 9) {

					HAL_UART_Transmit(&huart2, (uint8_t*) "  -  Access Denied\r\n", 20, 1000);
					HAL_UART_Transmit(&huart2, (uint8_t*) "----------------------------------------\r\n", 42, 1000);
					HAL_UART_Transmit(&huart2, (uint8_t*) "Please Try Again\r\n", 18, 1000);
					HAL_UART_Transmit(&huart2, (uint8_t*) "Type your 4 digit passcode:\r\n", 29, 1000);
					HAL_UART_Transmit(&huart2, (uint8_t*) "----------------------------------------\r\n", 42, 1000);
					errorOccured();
				}
				check = 0; // Be prepared for next element in array.
			}
			cmdstate = 0; // Be prepared for next user input.
		}
	}
}

// FUNCTION      : checkValidity
// DESCRIPTION   :
//    The function checks if the user input digits are in [0-9]
//    It is used for 1 byte or 4 byte input inside the code.
//	  1 byte to select option inside the user menu
//    4 byte to compare 4 digits inputs
// PARAMETERS    :
//                 checkVar - Used to change 1 or 4 digit input for serial input.
// RETURNS       :
//				   Void

void checkValidity(int checkVar) {
	while (1) {
		// Listen the serial line
		if (cmdstate == 1) {
			int checkValidityState = 0;

			for (char c = '0'; c <= '9'; ++c) {
				for (int y = 0; y < checkVar; y++) {
					if (rxData[y] == c) {
						checkValidityState++;
					}
				}
			}

			if (checkValidityState != checkVar) {
				HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 1000);
				HAL_UART_Transmit(&huart2, (uint8_t*) rxData, checkVar, 1000);
				HAL_UART_Transmit(&huart2, (uint8_t*) " - Not valid\r\n", 14, 1000);
				HAL_UART_Transmit(&huart2, (uint8_t*) "Only use [0-9]\r\n", 16, 1000);
				HAL_UART_Transmit(&huart2, (uint8_t*) "----------------------------------------\r\n", 42, 1000);
				checkValidityState = 0;
				errorOccured();
			} else {
				return;
			}

			checkValidityState = 0;// Be prepared for next validation.
			cmdstate = 0;// Be prepared for next user input.
		}
	}
}

// FUNCTION      : verifiedLogin
// DESCRIPTION   :
//    When user typed verified passcode.
//	  A menu pops up with 3 selection.
// 	  User can select by typing 1,2,3.
// 	  For only 1 digit input, it needs to change 1 digit for serial input
// PARAMETERS    :
//                 Void
// RETURNS       :
//				   Void

void verifiedLogin(void) {
	changeReceiveMode(1); //Change the input mode of serial line. (1 digit)
	callForVerifiedLogin();
	while (1) {
		// Listen the serial line
		if (cmdstate == 1) {
			if (rxData[0] - '0' == 1) {
				int x;
				loginApproved();
				HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n----------------------------------------", 42, 1000);
				for (x = 0; x < 10; x++) {
					HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 1000);
					HAL_UART_Transmit(&huart2, (uint8_t*) passCodes[x], 4, 1000);

				}
				// Show user all the passcodes and show the menu
				callForVerifiedLogin();

			} else if (rxData[0] - '0' == 2) {

				changePasscode();

			} else if (rxData[0] - '0' == 3) {
				cmdstate = 0;
				welcomeScreen();
				return;

			} else {
				HAL_UART_Transmit(&huart2, (uint8_t*) "\r\nPlease type valid input. [1,2,3]\r\n", 36, 1000);
				callForVerifiedLogin();
				errorOccured();
			}
			cmdstate = 0; // Be prepared for next user input.
		}

	}

}

// FUNCTION      : verifiedLogin
// DESCRIPTION   :
//
//    The function is to change number of characters for serial input.
//
// PARAMETERS    :
//                 Void
// RETURNS       :
//				   Void

void changeReceiveMode(int receiveModeVar) {
	receiveMode = receiveModeVar;
	MX_USART2_UART_Init();
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rxData, receiveMode);
}

// FUNCTION      : callForVerifiedLogin
// DESCRIPTION   :
//
//    Only user interface
//	  Shows the 3 selection menu to user.
//
// PARAMETERS    :
//                 Void
// RETURNS       :
//				   Void
void callForVerifiedLogin(void) {
	HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n----------------------------------------\r\n", 44, 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*) "\r\nChoose From Menu:\r\n", 21, 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*) "Type 1 to see all passcodes\r\n", 29, 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*) "Type 2 to change any of the passcode\r\n", 38, 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*) "Type 3 to log out\r\n", 19, 1000);
}


// FUNCTION      : HAL_UART_RxCpltCallback
// DESCRIPTION   :
//  	Function captures the user input over DMA interrupt
//  	Waits for the user to make cmdstate high.
// PARAMETERS    :
//   			   UART_HandleTypeDef *huart
//
// RETURNS       :
//   			   void
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	cmdstate = 1;
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rxData, receiveMode);
}

// FUNCTION      : welcomeScreen
// DESCRIPTION   :
//  	Reset green and red leds
// PARAMETERS    :
//				   void
// RETURNS       :
//   			   void

void welcomeScreen (void){
	HAL_GPIO_WritePin(GPIOB, Led1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Led3_Pin, GPIO_PIN_RESET);

}
// FUNCTION      : loginApproved
// DESCRIPTION   :
//  	Set the green led and voice
// PARAMETERS    :
//				   void
// RETURNS       :
//   			   void

void loginApproved (void){
	HAL_GPIO_WritePin(GPIOB, Led1_Pin, GPIO_PIN_SET);
	changeSpeakerFrequency(&htim1, 850);
	HAL_Delay(500);
	changeSpeakerFrequency(&htim1, 0);


}

// FUNCTION      : errorOccured
// DESCRIPTION   :
//  	Toggling red led and voice
// PARAMETERS    :
//				   void
// RETURNS       :
//   			   void
void errorOccured (void) {
	//Red LED toggle 5 times with 250 ms.
	for (int i = 0 ; i<4 ; i++){
		HAL_GPIO_WritePin(GPIOB, Led3_Pin,GPIO_PIN_SET);
		changeSpeakerFrequency(&htim1, 600);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOB, Led3_Pin,GPIO_PIN_RESET);
		changeSpeakerFrequency(&htim1, 0);
		HAL_Delay(250);
	}
}

// FUNCTION      : changePasscode
// DESCRIPTION   :
//  	User can change the passcode if typed password is valid.
//	    Function controls if the 4 digits input if it is valid.
//		Store them inside the database array.
// PARAMETERS    :
//				   void
// RETURNS       :
//   			   void

void changePasscode(void) {
	HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n----------------------------------------\r\n", 44, 1000);
	HAL_UART_Transmit(&huart2, (uint8_t*) "Please type [0-9] for choosing the passcode\r\n", 45, 1000);
	cmdstate = 0; // Wait for another user input.
	int num;
	while (1) {
		if (cmdstate == 1) {
			checkValidity(1);
			num = rxData[0] - '0';
			changeReceiveMode(4);//Change the input mode of serial line. (4 digit)
			HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*) rxData, 1, 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*) ". passcode is ", 14, 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*) passCodes[num], 4, 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*) "Please type 4 digit number to change it\r\n", 41, 1000);

			cmdstate = 0;
			checkValidity(4);
			int i;
			for (i = 0; i < 4; i++) {
				passCodes[num][i] = rxData[i];
			}
			HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n----------------------------------------\r\n", 44, 1000);
			HAL_UART_Transmit(&huart2, (uint8_t*) "Passcode has been changed!\r\n", 28, 1000);
			callForVerifiedLogin();
			loginApproved();
			changeReceiveMode(1); //Change the input mode of serial line. (1 digit)
			cmdstate = 0;// Be prepared for next user input.
			return;
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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
