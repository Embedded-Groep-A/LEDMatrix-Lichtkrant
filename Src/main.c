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

#include "CHARS.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int maxDevices = 4;
unsigned char spidata[16];

char InteruptData[2];

uint8_t status[64];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void shiftOut(uint8_t val)
{
      uint8_t i;

      for (i = 0; i < 8; i++)  {

           	HAL_GPIO_WritePin(GPIOB,SPI_MOSI_TEST_Pin, !!(val & (1 << (7 - i))));

            HAL_GPIO_WritePin(GPIOB,SPI_CLK_TEST_Pin, SET);
            HAL_GPIO_WritePin(GPIOB,SPI_CLK_TEST_Pin, RESET);
      }
}

void spiTransfer(int addr, volatile uint8_t opcode, volatile uint8_t data) {
    //Create an array with the data to shift out
    int offset=addr*2;
    int maxbytes=maxDevices*2;

    for(int i=0;i<maxbytes;i++)
        spidata[i]=(uint8_t)0;
    //put our device data into the array
    spidata[offset+1]=opcode;
    spidata[offset]=data;
    //enable the line
    HAL_GPIO_WritePin(GPIOB,SPI_CS_TEST_Pin,RESET);

    //Now shift out the data
    for(int i=maxbytes;i>0;i--)
    {
    	shiftOut(spidata[i-1]);
    }

    //latch the data onto the display
    HAL_GPIO_WritePin(GPIOB,SPI_CS_TEST_Pin,SET);
}

void setLed(int addr, int row, int column, int state) {
    int offset;
    uint8_t val=0x00;

    offset=addr*8;
    val=0b10000000 >> column;
    if(state)
        status[offset+row]=status[offset+row]|val;
    else {
        val=~val;
        status[offset+row]=status[offset+row]&val;
    }
    spiTransfer(addr, row+1,status[offset+row]);
}

void setRow(int addr, int row, uint8_t value) {
    int offset;
    offset=addr*8;
    status[offset+row]=value;
    spiTransfer(addr, row+1,status[offset+row]);
}

void setColumn(int addr, int col, uint8_t value) {
	uint8_t val;
    for(int row=0;row<8;row++) {
        val=value >> (7-row);
        val=val & 0x01;
        setLed(addr,row,col,val);
    }
}

void shutdown(int addr, int b) {

    spiTransfer(addr, OP_SHUTDOWN,b);
}

void setScanLimit(int addr, int limit) {

    if(limit>=0 && limit<8)
        spiTransfer(addr, OP_SCANLIMIT,limit);
}

void setIntensity(int addr, int intensity) {

    if(intensity>=0 && intensity<16)
        spiTransfer(addr, OP_INTENSITY,intensity);
}

void clearDisplay(int addr) {
	int offset = addr*8;

    for(int i=0;i<8;i++) {
    	status[offset+i]=0;
    	spiTransfer(addr, i+1,status[offset+i]);
    }
}

void printthingy(const uint8_t* Narray, int ofset, int size, int spacing)
{
  for (int i = 0; i < size; i++)
  {
    setColumn((3-(ofset/8)), (i + ofset%8), Narray[i]);//3-(ofset/8) vind t juiste address van de matrix, i + ofset%8 de juiste colom binnen die matrix
  }
  for (int i = 0; i < spacing; i++)
  {
    setColumn((3-(ofset/8)), (size + ofset%8), 0x00);//voeg extra emptys toe na de geprinte char
  }
}

void displayTekst(char* txt, int size, int ofset)
{
  for (int i = 0; i < size; i++)
  {
	if (txt[i] > 64)//is t een letter
	{
		printthingy(&LetterArray[(txt[i]-65)*3],i*4+ofset, 3, 1);
//txt[i]-65 vertaald het ascii char naar een bruikbare int, an *3 omdat de chars in de letterarray 3bytes groot zijn

	}
	else if (txt[i] >= 48)//is t een getal
	{
		printthingy(&NumberArray[(txt[i]-48)*3],i*4+ofset, 3, 1);
	}
  }
}

void displayTempVocht(int temperatuur, int vochtgehalte)
{
	printthingy(&LetterArray[19*3],0,3,1);//T
	printthingy(&NumberArray[(temperatuur/10)*3],4,3,1);//tiental van temp
	printthingy(&NumberArray[(temperatuur%10)*3],8,3,1);//tal van temp
	printthingy(&SpecialCharArray[0],12,3,1);//graden teken
	printthingy(&LetterArray[21*3],16,3,1);//V
	printthingy(&NumberArray[(vochtgehalte/10)*3],20,3,1);//tiental van vocht
	printthingy(&NumberArray[(vochtgehalte%10)*3],24,3,1);//tal van vocht
  	printthingy(&SpecialCharArray[3],28,3,1);//Percent teken
}


void spiInit(void)
{
	for(int i=0;i<64;i++) status[i]=0x00;//set de status array op 0, er is niks te zien
    for(int i=0;i<maxDevices;i++) {
        spiTransfer(i,OP_DISPLAYTEST,0);
        //scanlimit is set to max on startup
        setScanLimit(i,7);
        //decode is done in source
        spiTransfer(i,OP_DECODEMODE,0);
        clearDisplay(i);
        //we go into shutdown-mode on startup
        shutdown(i,0);
    }
}



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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  char msg[] = "Begin matrix powah!!";
  HAL_UART_Transmit(&huart2, msg, sizeof(msg),HAL_MAX_DELAY);

  HAL_UART_Receive_IT(&huart1, InteruptData, 1);

  spiInit();
  for (int i = 0; i < 4; i++)
  {
	  shutdown(i,1);
  	  setIntensity(i,4);
  	  clearDisplay(i);
  }

//  displayTempVocht(25, 67);
//  char msg2[] = "HELP1234";
//  displayTekst(msg2,8,0);


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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|SPI_CLK_TEST_Pin|SPI_MOSI_TEST_Pin|SPI_CS_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD3_Pin SPI_CLK_TEST_Pin SPI_MOSI_TEST_Pin SPI_CS_TEST_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|SPI_CLK_TEST_Pin|SPI_MOSI_TEST_Pin|SPI_CS_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

char interuptstring[8] = "[[[[[[[[";
int input = 0;
int ACCEPT = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{

		if(ACCEPT)// als er over de bus ACCEPT is gelezen
		{
			if (InteruptData[0] == '\r')//stop met lezen
			{
				displayTekst(interuptstring,8,0);//gooi de tekst op de matrix
				ACCEPT = 0;//t bericht is voorbij
				for (int i = 0; i < 8; i++)//reset de tekst array
				{
					interuptstring[i] = '[';
					input = 0;
				}
			}
			else if (input < 8)//zo niet stop, dan zet de char in de tekst array
			{
				if (InteruptData[0] >= 48)//checken of t een getal/letter is
				{
					interuptstring[input] = InteruptData[0];
					input++;
				}
			}
		}
		else if (InteruptData[0] == 0x66) ACCEPT = 1;
		HAL_UART_Receive_IT(&huart1, InteruptData, 1);
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
