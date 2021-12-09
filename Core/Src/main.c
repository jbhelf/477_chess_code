/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CS_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

#define CS_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)



#define CLK_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)

#define CLK_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)



#define DATA_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)

#define DATA_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim8_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	void setup_bb() {
		CS_HIGH;
		CLK_LOW;
	}

	void small_delay() {
		HAL_Delay(1);
	}

	void bb_write_bit(int b) {
		if (b) DATA_HIGH;
		else   DATA_LOW;
		small_delay();
		CLK_HIGH;
		small_delay();
		CLK_LOW;
	}

	void bb_write_byte(int b) {
		for (int i = 7; i >= 0; --i) bb_write_bit((b >> i) & 1);
	}

	void bb_cmd(int data) {
		CS_LOW;
		small_delay();
		bb_write_bit(0);
		bb_write_bit(0);
		bb_write_byte(data);
		small_delay();
		CS_HIGH;
		small_delay();
	}

	void bb_data(int data) {
		CS_LOW;
		small_delay();
		bb_write_bit(1);
		bb_write_bit(0);
		bb_write_byte(data);
		small_delay();
		CS_HIGH;
		small_delay();
	}

	void bb_init_oled() {
		HAL_Delay(1); // 1 ms
		bb_cmd(0x38);
		bb_cmd(0x08);
		bb_cmd(0x01);
		HAL_Delay(2); // 2 ms
		bb_cmd(0x06);
		bb_cmd(0x02);
		bb_cmd(0x0c);
	}

	void bb_display1 (const char* str) {
		bb_cmd(0x02);
		for (int i = 0; i < strlen(str); ++i) if (str[i] != '\0') bb_data(str[i]);
	}

	void bb_display2 (const char* str) {
		bb_cmd(0xc0);
		for (int i = 0; i < strlen(str); ++i) if (str[i] != '\0') bb_data(str[i]);
	}

	//For row board select lines:
	#define S0_Pin GPIO_PIN_2
	#define S1_Pin GPIO_PIN_1
	#define S2_Pin GPIO_PIN_0

	//For superMux (trademark pending) select and data lines:
	#define SUPER_MUX_DATA GPIO_PIN_3 //Data input for super mux
	#define S0_SUPER GPIO_PIN_10 //Super select line 0
	#define S1_SUPER GPIO_PIN_11 //Super select line 1
	#define S2_SUPER GPIO_PIN_12 //Super select line 2

	#define PAWN 1
	#define KNIGHT 3
	#define BISHOP 3
	#define ROOK 5
	#define QUEEN 9
	#define KING 40

	/*
	* I think this is mostly standard, but
	* 	Empty: 0
	* 	Pawn: 1
	* 	Knight, Bishop: 3
	* 	Rook: 5
	* 	Queen: 9
	* 	King: 10 for now
	* Positive indicates white (+=white)
	* Negative indicates black (-=black)
	* */

	int boardData[8][8] = {
						{-ROOK,-KNIGHT,-BISHOP,-KING,-QUEEN,-BISHOP,-KNIGHT,-ROOK},
						{-PAWN,-PAWN,-PAWN,-PAWN,-PAWN,-PAWN,-PAWN,-PAWN},
						{0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,0},
						{0,0,0,0,0,0,0,0},
						{PAWN,PAWN,PAWN,PAWN,PAWN,PAWN,PAWN,PAWN},
						{ROOK,KNIGHT,BISHOP,QUEEN,KING,BISHOP,KNIGHT,ROOK}};
//	int boardData[8][8] = {
//							{0,0,0,0,0,0,0,0},
//							{0,0,0,0,0,0,0,0},
//							{0,0,0,0,0,0,0,0},
//							{0,0,0,0,0,0,0,0},
//							{0,0,0,0,0,0,0,0},
//							{0,0,0,0,0,0,0,0},
//							{0,0,0,0,0,0,0,0},
//							{0,0,0,0,0,0,0,0}};

	int onPins[8][3] = {
					{0,0,0},
					{0,0,1},
					{0,1,0},
					{0,1,1},
					{1,0,0},
					{1,0,1},
					{1,1,0},
					{1,1,1}};

	const int selectPins[3] = {S0_Pin, S1_Pin, S2_Pin};
	const int superPins[3] = {S0_SUPER, S1_SUPER, S2_SUPER};

	int findMissingPiece(){
		//This code determines what value piece was just put down

		int pieceCount[] = {0,0,0,0,0}; //King, Queen, Bishop, Rook, Pawn

		/*EXPLANATION:
		 * If there is a white piece present, we add 1 * (piece value) to the respective pieceCount
		 * If there is a black piece present, we subtract 1 * (piece value) to the respective pieceCount
		 *
		 * At the end of the program, we can check this list and easily determine which piece was not present & return
		 */

		for(int row = 0;row < 7;row++){
			for(int col = 0;col < 7;col++){
				int current_board_val = boardData[row][col];
				switch(current_board_val){
					case KING:
					case -KING:
						pieceCount[0] += current_board_val;
						break;
					case QUEEN:
					case -QUEEN:
						pieceCount[1] += current_board_val;
						break;
					case BISHOP:
					case -BISHOP:
						pieceCount[2] += current_board_val;
						break;
					case ROOK:
					case -ROOK:
						pieceCount[3] += current_board_val;
						break;
					case PAWN:
					case -PAWN:
						pieceCount[4] += current_board_val;
						break;
					default:
						break;

				}
			}
		}

		for(int i = 0;i < 6;i++){
			if(pieceCount[i] != 0){
				return pieceCount[i];
			}
		}
	}

	//Works
	void choseSelectLineMux(int muxPin)
	{
		//Reset all pins:
		for(int i=0;i<3;i++){
			HAL_GPIO_WritePin(GPIOD, selectPins[i], GPIO_PIN_RESET);
		}

		//Turn on correct pins:
		for(int i=2;i>=0;i--){
			if(onPins[muxPin][i]){
				HAL_GPIO_WritePin(GPIOD, selectPins[i], GPIO_PIN_SET);
			}
		}
	}

	//Works
	void choseSelectLineSuper(int muxPin)
		{
			//Reset all pins:
			for(int i=0;i<3;i++){
				HAL_GPIO_WritePin(GPIOC, superPins[i], GPIO_PIN_RESET);
			}

			//Turn on correct pins:
			for(int i=2;i>=0;i--){
				if(onPins[muxPin][2-i]){
					HAL_GPIO_WritePin(GPIOC, superPins[i], GPIO_PIN_SET);
				}
			}
		}

	//To be verified
	void readSuperMux(){
		//MIGHT WANT TO NOT USE A GLOBAL SOURCE!
		int dest_row=-1, dest_hall=-1, source_row=-1, source_hall=-1; //x,y coordinates for source and destination of piece

		int change_detected = 0; //0 means no, 1 means yes

		for(int row=0;row<8;row++){
			//Iterate through each ROW BOARD (line below selects 1 row at a time)
			choseSelectLineSuper(row);

			for(int hall=0;hall<8;hall++){
				//Iterate through each HALL SENSOR (line below selects 1 hall at a time)
				choseSelectLineMux(hall);

				/* KEY (i.e. code meaning in english):
				 * (first if statement): If there isn't a piece where there previously was
				 * - !(HAL_GPIO_ReadPin(GPIOD, SUPER_MUX_DATA)): 1 means piece detected
				 * - boardData[row][col] == 0: Means position at {row, col} previously didn't have a piece
				 *
				 * (second if statement): If there is a piece where there previously wasn't
				 * - (HAL_GPIO_ReadPin(GPIOD, SUPER_MUX_DATA)): 1 means piece NOT detected
				 * - boardData[row][col] == 0: Means position at {row, col} previously had a piece
				 * */

				//If the current location is nonzero, but the previous state is 0, we are moving a piece here
				if((!(HAL_GPIO_ReadPin(GPIOD, SUPER_MUX_DATA))) && boardData[row][hall] == 0){
					change_detected = 1;
					dest_row = row;
					dest_hall = hall;
				}

				//If the current location is zero, but the previous state is nonzero, we are taking a piece from here
				if((HAL_GPIO_ReadPin(GPIOD, SUPER_MUX_DATA)) && boardData[row][hall] != 0){
					change_detected = 1;
					source_row = row;
					source_hall = hall;
				}
			}
		}

		//If a change was detected (flag was raised)

//		bb_display2("             ");
		if(change_detected){ //Continue here...
			change_detected = 0; //Reset value (redundant)

			char move_to_send[2]; //The array of information to send
			char test_with_display[3];


			if(dest_row != -1){ //Destination changed
				move_to_send[0] = (char)(((8*dest_row) + dest_hall) + 40);
				sprintf(test_with_display, "%2d", ((8*dest_row) + dest_hall));
				move_to_send[1] = test_with_display[2] = 'd';
			}else{ //Source changed
				move_to_send[0] = (char)(((8*source_row) + source_hall) + 40);
				sprintf(test_with_display, "%2d", ((8*source_row) + source_hall));
				move_to_send[1] = test_with_display[2] = 's';
//				move_to_send[2] = 's';
			}


			//comment this out
			bb_display1(move_to_send);
			bb_display2(test_with_display);
//			Send to Vik via USB UART code ...

		    HAL_UART_AbortReceive(&huart2);
		    HAL_UART_Transmit(&huart2, move_to_send, sizeof(move_to_send), 10);

		    //Wait to recieve squares to light up from Vik...
		    char squares_to_light[28];

		    //Initially render all LEDs black
		    all_black_render();

		    HAL_UART_AbortTransmit(&huart2);
		    HAL_UART_Receive(&huart2, squares_to_light, sizeof(squares_to_light), 10000000);

		    for(int i = 0;i < 28;i++){
		    	int square = ((int)squares_to_light[i]) - 40;

		    	if(square < 0 || square > 63){
		    		break;
		    	}else{
		    		 //1. As there are 15 LEDs/row, determine which specific LED goes on
		    		 if(square > 8)
		    			 int square_to_light = (square*2) - (int)(8/square); //The first component accounts for 1 'working' LED for every 'hidden' LED.  The second component accounts for the fact that there are 15/row, not 16/row
		    		 else
		    			 int square_to_light = (square*2);

					 //2. With how LEDs are wired, odd # rows (starting at 0) are 'backwards'
					 int row = (int)square_to_light/8;

					 if(row % 2){ //If the row is odd...
						 int flip_key[8] = {7, 5, 3, 1, -1, -3, -5, -7};

						 square_to_light += flip_key[square_to_light % 8];
					 }

					 render_one_led(square, 50, 50, 0); //Render LED purple
		    	}
		    }

		    render_neopixel();

		    //BREAK (stuff below here works as expected)

		    //RECEIVE EVALUATION FROM VIK & DISPLAY (CODE FROM RYAN):
			HAL_UART_Receive(&huart2, evaluation, sizeof(evaluation), 10000000);

			bb_display1("Evaluation:");
			bb_display2(evaluation);

		    //If the destination changed, update array representation of board:
			if(dest_row != -1 && dest_hall != -1){
				boardData[dest_row][dest_hall] = 1; //findMissingPiece(); //boardData[SOURCE_X][SOURCE_Y];
			}

			//If the source changed, update array representation of board:
			if(source_row != -1 && source_hall != -1){
				boardData[source_row][source_hall] = 0;
			}
		}

		return boardData;
	}

/*
*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*
NOTE:
	- To actually use this, we just need to call: readSuperMux() in main
	- Near the end of readSuperMux(), we still need to add the method used to send USB data ({FUNCTION CALL HERE})
*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*_*-*
*/

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
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  setup_bb();
  bb_init_oled();
//  init_neopixel(WS2812B);
  all_black_render();

  uint8_t pData[1];

//  bb_display1("                       ");
//  bb_display2("                       ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  readSuperMux();
//	  int sum = 0;
//	  for(int j = 0;j < 8;j++){
//		  //For each row..
//		  choseSelectLineSuper(j);
//		  for(int i = 0;i < 8;i++){
//			  //For each hall effect sensor...
//			  choseSelectLineMux(i);
////			  for(int wait = 0;wait < 100;wait++){
////				  for(int something = 0;something < 100;something++){
////					  continue;
////				  }
////			  }
////			  HAL_GPIO_ReadPin(GPIOD, SUPER_MUX_DATA);
//
//			  if(!(HAL_GPIO_ReadPin(GPIOD, SUPER_MUX_DATA))){
//				  sum += 1;
//			  }
//		  }
//	  }
//	  char test[16];
//	  int num = 3;
//	  //assign to pdata
//	  sprintf(test, "%d", sum);
////			  (int)0, pData, 1);
////	  continue;
//
////	  HAL_UART_AbortTransmit(&huart2);
////	  HAL_UART_Receive(&huart2, pData, sizeof(pData), 10000000);
////	  HAL_UART_AbortReceive(&huart2);
////	  HAL_UART_Transmit(&huart2, pData, sizeof(pData), 10);
//	  bb_display2(test);


//	  HAL_GPIO_ReadPin(GPIOB, ROW_MUX_DATA);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 104;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, S0_Super_Pin|S1_Super_Pin|S2_Super_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, S0_Pin_Pin|S1_Pin_Pin|S2_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Super_Pin S1_Super_Pin S2_Super_Pin */
  GPIO_InitStruct.Pin = S0_Super_Pin|S1_Super_Pin|S2_Super_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin_Pin S1_Pin_Pin S2_Pin_Pin */
  GPIO_InitStruct.Pin = S0_Pin_Pin|S1_Pin_Pin|S2_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Mux_Input_Pin */
  GPIO_InitStruct.Pin = Mux_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Mux_Input_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
