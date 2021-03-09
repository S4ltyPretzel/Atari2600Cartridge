
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "nokia5110_LCD.h"
#include "tm_stm32_fatfs.h"
#include "defines.h"

/* Fatfs structure */
FATFS FS;
FIL fil;
FRESULT fres;

/* Size structure for FATFS */
TM_FATFS_Size_t CardSize;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init_To_Program(void);
static void MX_GPIO_Init_To_Work(void);
static void MX_GPIO_Spi_Init(void);
void writeFlash(uint8_t dataVal, uint16_t adVal);
uint8_t readFlash(uint16_t adVal);
void eraseFlash();
void programFlash();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
char buffer[100],txt[100];
uint16_t plik=0;
uint16_t dane=0;
FILINFO finfo;
DIR dir;
FILE file;
volatile uint8_t gameBuf[4096];
volatile int active=0;
uint8_t gameBuf2[4096];
uint8_t ok=1;
uint16_t i=0;
uint16_t bytesRead=0;
GPIO_InitTypeDef GPIO_InitStruct;
char Received[3];
uint8_t test=49;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	//konfiguracja przycisku SW3 "OK"
	GPIO_InitStruct.Pin = BUT3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUT3_GPIO_Port, &GPIO_InitStruct);

	MX_GPIO_Init_To_Work();

	//Do³¹czenie flasha do magistrali Atari 2600
	//Init Flash - not selected

	HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flash_WE_GPIO_Port, Flash_WE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flash_RP_GPIO_Port, Flash_RP_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(Flash_G_GPIO_Port,Flash_G_Pin,RESET);
	HAL_GPIO_WritePin(Buf1_OE_GPIO_Port, Buf1_OE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Buf2_OE_GPIO_Port, Buf2_OE_Pin, GPIO_PIN_RESET);


	MX_GPIO_Spi_Init();
	LCD_setRST(LCD_RST_GPIO_Port, LCD_RST_Pin);
	LCD_setCE(LCD_CE_GPIO_Port, LCD_CE_Pin);
	LCD_setDC(LCD_DC_GPIO_Port, LCD_DC_Pin);
	LCD_setDIN(LCD_MOSI_GPIO_Port, LCD_MOSI_Pin);
	LCD_setCLK(LCD_SCK_GPIO_Port, LCD_SCK_Pin);
	LCD_init();

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,SET);
	LCD_clrScr();
	LCD_print("Press 'ok' ",0,0);
	LCD_print("button to",0,1);
	LCD_print("choose",0,2);
	LCD_print("a different",0,3);
	LCD_print("game.",0,4);

	HAL_UART_Receive(&huart1, &Received, 1, 2000);
	//HAL_UART_Transmit(&huart1, &test, 1, 3000);
	HAL_UART_Receive_IT(&huart1, &Received, 3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin)==0)
		{
			HAL_Delay(10);
			if (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin)==0)
			{
				MX_GPIO_Init_To_Program();

				//od³¹czenie flasha do magistrali Atari 2600
				HAL_GPIO_WritePin(Buf1_OE_GPIO_Port, Buf1_OE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Buf2_OE_GPIO_Port, Buf2_OE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Buf3_OE_GPIO_Port, Buf3_OE_Pin, GPIO_PIN_SET);

				MX_USART1_UART_Init();
				MX_SPI1_Init();
				LCD_setRST(LCD_RST_GPIO_Port, LCD_RST_Pin);
				LCD_setCE(LCD_CE_GPIO_Port, LCD_CE_Pin);
				LCD_setDC(LCD_DC_GPIO_Port, LCD_DC_Pin);
				LCD_setDIN(LCD_MOSI_GPIO_Port, LCD_MOSI_Pin);
				LCD_setCLK(LCD_SCK_GPIO_Port, LCD_SCK_Pin);
				LCD_init();

				//Zerowanie bufora gry
				for (int i=0; i<4096; i++) gameBuf[i]=0x00;

				//Init Flash - not selected
				HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Flash_WE_GPIO_Port, Flash_WE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Flash_RP_GPIO_Port, Flash_RP_Pin, GPIO_PIN_SET);

				if(f_mount(&FS, "SD:", 1) != FR_OK)
				{
					LCD_print("No SD found!",0,5);
				}
				else
					LCD_print("SD found!",0,5);

				fres = f_findfirst(&dir, &finfo, "", "*.bin");  // Start to search for files
				plik=1;
				LCD_print(finfo.fname, 0, 0);
				ok = 0;
				HAL_Delay(1000);
				while (ok==0)
				{
					if (HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin)==0)
					{
						HAL_Delay(10);
						if (HAL_GPIO_ReadPin(BUT1_GPIO_Port, BUT1_Pin)==0)
						{
							HAL_Delay(100);
							if (plik>=2)
							{
								fres = f_findfirst(&dir, &finfo, "", "*.bin");
								i=1;
								while (i != (plik-1))
								{
									fres = f_findnext(&dir, &finfo);
									i++;
								}
								plik--;
								LCD_clrScr();
								LCD_print(finfo.fname, 0, 0);
							}
						}
					}
					if (HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)==0)
					{
						HAL_Delay(10);
						if (HAL_GPIO_ReadPin(BUT2_GPIO_Port, BUT2_Pin)==0)
						{
							HAL_Delay(100);
							fres = f_findnext(&dir, &finfo);
							if (fres == FR_OK && finfo.fname[0])
							{
								plik++;
								LCD_clrScr();
								LCD_print(finfo.fname, 0, 0);
							}
							else
							{
								fres = f_findfirst(&dir, &finfo, "", "*.bin");
								i=1;
								while (i != (plik))
								{
									fres = f_findnext(&dir, &finfo);
									i++;
								}
							}
						}
					}
					if (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin)==0)
					{
						HAL_Delay(10);
						if (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin)==0)
						{
							ok=1;
						}
					}
				}
				f_open(&file, finfo.fname, FA_READ );
				//LCD_clrScr();
				LCD_print("Reading file",0,1);
				f_read(&file, &gameBuf, 4096, &bytesRead);
				f_close(&file);
				f_closedir(&dir);

				eraseFlash();
				programFlash();

				//Init Flash - not selected

				HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Flash_WE_GPIO_Port, Flash_WE_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(Flash_RP_GPIO_Port, Flash_RP_Pin, GPIO_PIN_SET);

				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,SET);
				LCD_clrScr();
				LCD_print("Ready!",0,0);
				LCD_print("Please reset",0,1);
				LCD_print("cartridge and",0,2);
				LCD_print("console!",0,3);
				while(1)
				{
					if (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin)==0)
					{
						HAL_Delay(100);
						if (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin)==0)
						{
							HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
						}
					}
				}
			}
		}
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Flash_D6_Pin|Flash_D7_Pin|Flash_D8_Pin|Flash_A8_Pin 
                          |Flash_A9_Pin|Flash_A10_Pin|Flash_A11_Pin|Flash_Slot_Pin 
                          |Flash_G_Pin|Flash_RP_Pin|Flash_D1_Pin|Flash_D2_Pin 
                          |Flash_D3_Pin|Flash_D4_Pin|Flash_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Flash_A0_Pin|Flash_A1_Pin|Flash_A2_Pin|Flash_A3_Pin 
                          |Flash_A4_Pin|Flash_A5_Pin|Flash_A6_Pin|Flash_A7_Pin 
                          |Buf1_OE_Pin|Buf2_OE_Pin|Buf3_OE_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUT1_Pin|BUT2_Pin|Flash_CS_Pin|LCD_RST_Pin 
                          |LCD_CE_Pin|LCD_SCK_Pin|Flash_WE_Pin|LCD_MOSI_Pin 
                          |Flash_A_1_Pin|BUT3_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Flash_D6_Pin Flash_D7_Pin Flash_D8_Pin Flash_A8_Pin 
                           Flash_A9_Pin Flash_A10_Pin Flash_A11_Pin Flash_Slot_Pin 
                           Flash_G_Pin Flash_RP_Pin Flash_D1_Pin Flash_D2_Pin 
                           Flash_D3_Pin Flash_D4_Pin Flash_D5_Pin */
  GPIO_InitStruct.Pin = Flash_D6_Pin|Flash_D7_Pin|Flash_D8_Pin|Flash_A8_Pin 
                          |Flash_A9_Pin|Flash_A10_Pin|Flash_A11_Pin|Flash_Slot_Pin 
                          |Flash_G_Pin|Flash_RP_Pin|Flash_D1_Pin|Flash_D2_Pin 
                          |Flash_D3_Pin|Flash_D4_Pin|Flash_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Flash_A0_Pin Flash_A1_Pin Flash_A2_Pin Flash_A3_Pin 
                           Flash_A4_Pin Flash_A5_Pin Flash_A6_Pin Flash_A7_Pin 
                           Buf1_OE_Pin Buf2_OE_Pin Buf3_OE_Pin SD_CS_Pin */
  GPIO_InitStruct.Pin = Flash_A0_Pin|Flash_A1_Pin|Flash_A2_Pin|Flash_A3_Pin 
                          |Flash_A4_Pin|Flash_A5_Pin|Flash_A6_Pin|Flash_A7_Pin 
                          |Buf1_OE_Pin|Buf2_OE_Pin|Buf3_OE_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT1_Pin BUT2_Pin Flash_CS_Pin LCD_RST_Pin 
                           LCD_CE_Pin LCD_SCK_Pin Flash_WE_Pin LCD_MOSI_Pin 
                           Flash_A_1_Pin BUT3_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = BUT1_Pin|BUT2_Pin|Flash_CS_Pin|LCD_RST_Pin 
                          |LCD_CE_Pin|LCD_SCK_Pin|Flash_WE_Pin|LCD_MOSI_Pin 
                          |Flash_A_1_Pin|BUT3_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Flash_RB_Pin */
  GPIO_InitStruct.Pin = Flash_RB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Flash_RB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void MX_GPIO_Init_To_Program(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Flash_D6_Pin|Flash_D7_Pin|Flash_D8_Pin|Flash_A8_Pin
                          |Flash_A9_Pin|Flash_A10_Pin|Flash_A11_Pin|Flash_Slot_Pin
                          |Flash_G_Pin|Flash_RP_Pin|Flash_D1_Pin|Flash_D2_Pin
                          |Flash_D3_Pin|Flash_D4_Pin|Flash_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Flash_A0_Pin|Flash_A1_Pin|Flash_A2_Pin|Flash_A3_Pin
                          |Flash_A4_Pin|Flash_A5_Pin|Flash_A6_Pin|Flash_A7_Pin
                          |Buf1_OE_Pin|Buf2_OE_Pin|Buf3_OE_Pin|SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUT1_Pin|BUT2_Pin|Flash_CS_Pin|LCD_RST_Pin 
                          |LCD_CE_Pin|LCD_SCK_Pin|Flash_WE_Pin|LCD_MOSI_Pin 
                          |Flash_A_1_Pin|BUT3_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Flash_D6_Pin Flash_D7_Pin Flash_D8_Pin Flash_A8_Pin 
                           Flash_A9_Pin Flash_A10_Pin Flash_A11_Pin Flash_Slot_Pin 
                           Flash_G_Pin Flash_RP_Pin Flash_D1_Pin Flash_D2_Pin 
                           Flash_D3_Pin Flash_D4_Pin Flash_D5_Pin */
  GPIO_InitStruct.Pin = Flash_D6_Pin|Flash_D7_Pin|Flash_D8_Pin|Flash_A8_Pin
                          |Flash_A9_Pin|Flash_A10_Pin|Flash_A11_Pin|Flash_Slot_Pin
                          |Flash_G_Pin|Flash_RP_Pin|Flash_D1_Pin|Flash_D2_Pin
                          |Flash_D3_Pin|Flash_D4_Pin|Flash_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Flash_A0_Pin Flash_A1_Pin Flash_A2_Pin Flash_A3_Pin 
                           Flash_A4_Pin Flash_A5_Pin Flash_A6_Pin Flash_A7_Pin 
                           Buf1_OE_Pin Buf2_OE_Pin Buf3_OE_Pin SD_CS_Pin */
  GPIO_InitStruct.Pin = Flash_A0_Pin|Flash_A1_Pin|Flash_A2_Pin|Flash_A3_Pin
                          |Flash_A4_Pin|Flash_A5_Pin|Flash_A6_Pin|Flash_A7_Pin
                          |Buf1_OE_Pin|Buf2_OE_Pin|SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT1_Pin BUT2_Pin Flash_CS_Pin LCD_RST_Pin 
                           LCD_CE_Pin LCD_SCK_Pin Flash_WE_Pin LCD_MOSI_Pin 
                           Flash_A_1_Pin BUT3_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = Flash_CS_Pin|LCD_RST_Pin
                          |LCD_CE_Pin|LCD_SCK_Pin|Flash_WE_Pin|LCD_MOSI_Pin 
                          |Flash_A_1_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Flash_RB_Pin */
  GPIO_InitStruct.Pin = Flash_RB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Flash_RB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}


static void MX_GPIO_Init_To_Work(void)

{
	// Zmiana trybów GPIO po³¹czonych z pamiêci¹ na tryb input i otwarcie buforów w celu umo¿liwienia komunikacji A2600 z pamieci¹ flash

	  GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();


	  //magistrala danych od strony MPU na Z
	  GPIO_InitStruct.Pin = Flash_D1_Pin|Flash_D2_Pin|Flash_D3_Pin|Flash_D4_Pin|Flash_D5_Pin|Flash_D6_Pin|Flash_D7_Pin|Flash_D8_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  //magistrala adresowa A0-A7 od strony MPU na Z
	  GPIO_InitStruct.Pin = Flash_A0_Pin|Flash_A1_Pin|Flash_A2_Pin|Flash_A3_Pin|Flash_A4_Pin|Flash_A5_Pin|Flash_A6_Pin|Flash_A7_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  //magistrala adresowa A8-A11 od strony MPU na Z
	  GPIO_InitStruct.Pin = Flash_A8_Pin|Flash_A9_Pin|Flash_A10_Pin|Flash_A11_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	 // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


	  // konfiguracja pinu A-1 jako wyjœcie w stanie niskim
	  GPIO_InitStruct.Pin = Flash_A_1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(Flash_A_1_GPIO_Port, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(Flash_A_1_GPIO_Port,Flash_A_1_Pin,RESET);

	  //Write Enable
	  GPIO_InitStruct.Pin = Flash_WE_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(Flash_WE_GPIO_Port, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(Flash_WE_GPIO_Port, Flash_WE_Pin, GPIO_PIN_SET);



	  //Slot
	  GPIO_InitStruct.Pin = Flash_Slot_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(Flash_Slot_GPIO_Port, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(Flash_Slot_GPIO_Port,Flash_Slot_Pin,RESET);

	  //RP
	  GPIO_InitStruct.Pin = Flash_RP_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(Flash_RP_GPIO_Port, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(Flash_RP_GPIO_Port, Flash_RP_Pin, GPIO_PIN_SET);

	  //RB
	  GPIO_InitStruct.Pin = Flash_RB_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Flash_RB_GPIO_Port, &GPIO_InitStruct);


	  // konfiguracja pinu Flash_G_Pin jako wyjœcie w stanie niskim- w³¹czenie wyjœc pamiêci flash
	  GPIO_InitStruct.Pin = Flash_G_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(Flash_G_GPIO_Port, &GPIO_InitStruct);
	 // HAL_GPIO_WritePin(Flash_G_GPIO_Port,Flash_G_Pin,RESET);



	  //magistrala adresowa A12 (CS) jako wejscie przerwania - ostatecznie nie u¿ywane
	  GPIO_InitStruct.Pin = Flash_CS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Flash_CS_GPIO_Port, &GPIO_InitStruct);

      HAL_NVIC_SetPriority(EXTI2_IRQn, 0x00, 0x00);
      HAL_NVIC_EnableIRQ(EXTI2_IRQn);



	  //konfiguracja wyjœc buforów - do³aczenie magistrali danych i adresowej do flash
	  GPIO_InitStruct.Pin =  Buf1_OE_Pin|Buf2_OE_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  //HAL_GPIO_WritePin(GPIOA, Buf1_OE_Pin|Buf2_OE_Pin|Buf3_OE_Pin, GPIO_PIN_RESET);

	  //KONFIGURACJA BUF3 jako wejœcie- linia w trybie pracy jest sterowana z sygna³u CS (A12) poprzez dolutowanie przewodu
	  GPIO_InitStruct.Pin =  Buf3_OE_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	  /*Configure GPIO pin : LED_Pin */
	  GPIO_InitStruct.Pin = LED_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);




}

static void MX_GPIO_Spi_Init(void)
{
	GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CE_Pin|LCD_SCK_Pin|LCD_MOSI_Pin|LCD_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB,LCD_RST_Pin|LCD_CE_Pin|LCD_SCK_Pin|LCD_MOSI_Pin|LCD_DC_Pin, GPIO_PIN_RESET);
}

void writeFlash(uint8_t dataVal, uint16_t adVal)
{

	//Configure data pins

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = Flash_D6_Pin|Flash_D7_Pin|Flash_D8_Pin|Flash_D1_Pin
						  |Flash_D2_Pin|Flash_D3_Pin|Flash_D4_Pin|Flash_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	//Init write command



	//Data set

	if ((dataVal&0x01)==0x01)
		HAL_GPIO_WritePin(Flash_D1_GPIO_Port, Flash_D1_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_D1_GPIO_Port, Flash_D1_Pin, GPIO_PIN_RESET);
	if ((dataVal&0x02)==0x02)
		HAL_GPIO_WritePin(Flash_D2_GPIO_Port, Flash_D2_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_D2_GPIO_Port, Flash_D2_Pin, GPIO_PIN_RESET);
	if ((dataVal&0x04)==0x04)
		HAL_GPIO_WritePin(Flash_D3_GPIO_Port, Flash_D3_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_D3_GPIO_Port, Flash_D3_Pin, GPIO_PIN_RESET);
	if ((dataVal&0x08)==0x08)
		HAL_GPIO_WritePin(Flash_D4_GPIO_Port, Flash_D4_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_D4_GPIO_Port, Flash_D4_Pin, GPIO_PIN_RESET);
	if ((dataVal&0x10)==0x10)
		HAL_GPIO_WritePin(Flash_D5_GPIO_Port, Flash_D5_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_D5_GPIO_Port, Flash_D5_Pin, GPIO_PIN_RESET);
	if ((dataVal&0x20)==0x20)
		HAL_GPIO_WritePin(Flash_D6_GPIO_Port, Flash_D6_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_D6_GPIO_Port, Flash_D6_Pin, GPIO_PIN_RESET);
	if ((dataVal&0x40)==0x40)
		HAL_GPIO_WritePin(Flash_D7_GPIO_Port, Flash_D7_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_D7_GPIO_Port, Flash_D7_Pin, GPIO_PIN_RESET);
	if ((dataVal&0x80)==0x80)
		HAL_GPIO_WritePin(Flash_D8_GPIO_Port, Flash_D8_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_D8_GPIO_Port, Flash_D8_Pin, GPIO_PIN_RESET);

	//Address set

	if ((adVal&0x001)==0x001)
		HAL_GPIO_WritePin(Flash_A0_GPIO_Port, Flash_A0_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A0_GPIO_Port, Flash_A0_Pin, GPIO_PIN_RESET);
	if ((adVal&0x002)==0x002)
		HAL_GPIO_WritePin(Flash_A1_GPIO_Port, Flash_A1_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A1_GPIO_Port, Flash_A1_Pin, GPIO_PIN_RESET);
	if ((adVal&0x004)==0x004)
		HAL_GPIO_WritePin(Flash_A2_GPIO_Port, Flash_A2_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A2_GPIO_Port, Flash_A2_Pin, GPIO_PIN_RESET);
	if ((adVal&0x008)==0x008)
		HAL_GPIO_WritePin(Flash_A3_GPIO_Port, Flash_A3_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A3_GPIO_Port, Flash_A3_Pin, GPIO_PIN_RESET);
	if ((adVal&0x010)==0x010)
		HAL_GPIO_WritePin(Flash_A4_GPIO_Port, Flash_A4_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A4_GPIO_Port, Flash_A4_Pin, GPIO_PIN_RESET);
	if ((adVal&0x020)==0x020)
		HAL_GPIO_WritePin(Flash_A5_GPIO_Port, Flash_A5_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A5_GPIO_Port, Flash_A5_Pin, GPIO_PIN_RESET);
	if ((adVal&0x040)==0x040)
		HAL_GPIO_WritePin(Flash_A6_GPIO_Port, Flash_A6_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A6_GPIO_Port, Flash_A6_Pin, GPIO_PIN_RESET);
	if ((adVal&0x080)==0x080)
		HAL_GPIO_WritePin(Flash_A7_GPIO_Port, Flash_A7_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A7_GPIO_Port, Flash_A7_Pin, GPIO_PIN_RESET);
	if ((adVal&0x100)==0x100)
		HAL_GPIO_WritePin(Flash_A8_GPIO_Port, Flash_A8_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A8_GPIO_Port, Flash_A8_Pin, GPIO_PIN_RESET);
	if ((adVal&0x200)==0x200)
		HAL_GPIO_WritePin(Flash_A9_GPIO_Port, Flash_A9_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A9_GPIO_Port, Flash_A9_Pin, GPIO_PIN_RESET);
	if ((adVal&0x400)==0x400)
		HAL_GPIO_WritePin(Flash_A10_GPIO_Port, Flash_A10_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A10_GPIO_Port, Flash_A10_Pin, GPIO_PIN_RESET);
	if ((adVal&0x800)==0x800)
		HAL_GPIO_WritePin(Flash_A11_GPIO_Port, Flash_A11_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A11_GPIO_Port, Flash_A11_Pin, GPIO_PIN_RESET);

	//Write enable

//	HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flash_WE_GPIO_Port, Flash_WE_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Flash_RP_GPIO_Port, Flash_RP_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flash_WE_GPIO_Port, Flash_WE_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(Flash_RP_GPIO_Port, Flash_RP_Pin, GPIO_PIN_SET);

}

uint8_t readFlash(uint16_t adVal)
{
volatile	uint8_t dataAcquired=0;

	//Configure data pins

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = Flash_D6_Pin|Flash_D7_Pin|Flash_D8_Pin|Flash_D1_Pin
						  |Flash_D2_Pin|Flash_D3_Pin|Flash_D4_Pin|Flash_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	//Address set

	if ((adVal&0x001)==0x001)
		HAL_GPIO_WritePin(Flash_A0_GPIO_Port, Flash_A0_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A0_GPIO_Port, Flash_A0_Pin, GPIO_PIN_RESET);
	if ((adVal&0x002)==0x002)
		HAL_GPIO_WritePin(Flash_A1_GPIO_Port, Flash_A1_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A1_GPIO_Port, Flash_A1_Pin, GPIO_PIN_RESET);
	if ((adVal&0x004)==0x004)
		HAL_GPIO_WritePin(Flash_A2_GPIO_Port, Flash_A2_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A2_GPIO_Port, Flash_A2_Pin, GPIO_PIN_RESET);
	if ((adVal&0x008)==0x008)
		HAL_GPIO_WritePin(Flash_A3_GPIO_Port, Flash_A3_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A3_GPIO_Port, Flash_A3_Pin, GPIO_PIN_RESET);
	if ((adVal&0x010)==0x010)
		HAL_GPIO_WritePin(Flash_A4_GPIO_Port, Flash_A4_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A4_GPIO_Port, Flash_A4_Pin, GPIO_PIN_RESET);
	if ((adVal&0x020)==0x020)
		HAL_GPIO_WritePin(Flash_A5_GPIO_Port, Flash_A5_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A5_GPIO_Port, Flash_A5_Pin, GPIO_PIN_RESET);
	if ((adVal&0x040)==0x040)
		HAL_GPIO_WritePin(Flash_A6_GPIO_Port, Flash_A6_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A6_GPIO_Port, Flash_A6_Pin, GPIO_PIN_RESET);
	if ((adVal&0x080)==0x080)
		HAL_GPIO_WritePin(Flash_A7_GPIO_Port, Flash_A7_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A7_GPIO_Port, Flash_A7_Pin, GPIO_PIN_RESET);
	if ((adVal&0x100)==0x100)
		HAL_GPIO_WritePin(Flash_A8_GPIO_Port, Flash_A8_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A8_GPIO_Port, Flash_A8_Pin, GPIO_PIN_RESET);
	if ((adVal&0x200)==0x200)
		HAL_GPIO_WritePin(Flash_A9_GPIO_Port, Flash_A9_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A9_GPIO_Port, Flash_A9_Pin, GPIO_PIN_RESET);
	if ((adVal&0x400)==0x400)
		HAL_GPIO_WritePin(Flash_A10_GPIO_Port, Flash_A10_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A10_GPIO_Port, Flash_A10_Pin, GPIO_PIN_RESET);
	if ((adVal&0x800)==0x800)
		HAL_GPIO_WritePin(Flash_A11_GPIO_Port, Flash_A11_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(Flash_A11_GPIO_Port, Flash_A11_Pin, GPIO_PIN_RESET);

	//Read enable

	HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Flash_WE_GPIO_Port, Flash_WE_Pin, GPIO_PIN_SET);

	//Data acquire

	if (HAL_GPIO_ReadPin(Flash_D1_GPIO_Port, Flash_D1_Pin))
		dataAcquired=dataAcquired+1;
	if (HAL_GPIO_ReadPin(Flash_D2_GPIO_Port, Flash_D2_Pin))
		dataAcquired=dataAcquired+2;
	if (HAL_GPIO_ReadPin(Flash_D3_GPIO_Port, Flash_D3_Pin))
		dataAcquired=dataAcquired+4;
	if (HAL_GPIO_ReadPin(Flash_D4_GPIO_Port, Flash_D4_Pin))
		dataAcquired=dataAcquired+8;
	if (HAL_GPIO_ReadPin(Flash_D5_GPIO_Port, Flash_D5_Pin))
		dataAcquired=dataAcquired+16;
	if (HAL_GPIO_ReadPin(Flash_D6_GPIO_Port, Flash_D6_Pin))
		dataAcquired=dataAcquired+32;
	if (HAL_GPIO_ReadPin(Flash_D7_GPIO_Port, Flash_D7_Pin))
		dataAcquired=dataAcquired+64;
	if (HAL_GPIO_ReadPin(Flash_D8_GPIO_Port, Flash_D8_Pin))
		dataAcquired=dataAcquired+128;
	HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_SET);
	return (dataAcquired);
}

void eraseFlash()
{
	HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_RESET);

	//Chip erase command

	LCD_clrScr();
	LCD_print("Erase...",0,0);

	HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_RESET);
	writeFlash(170, 1365);
	HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_SET);
	writeFlash(85, 682);
	HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_RESET);
	writeFlash(128, 1365);
	HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_RESET);
	writeFlash(170, 1365);
	HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_SET);
	writeFlash(85, 682);
	HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_RESET);
	writeFlash(16, 1365);

	//************************************************************************

	//LCD_clrScr();
	//LCD_print("Checking...",0,0);
	ok=HAL_GPIO_ReadPin(Flash_RB_GPIO_Port, Flash_RB_Pin);
	while (ok==0)
	{
		ok=HAL_GPIO_ReadPin(Flash_RB_GPIO_Port, Flash_RB_Pin);
		for (int i=0; i<4096; i++)
		{
			gameBuf2[i]=readFlash(i);
		}
	}
}

void programFlash()
{
	LCD_clrScr();
	LCD_print("Writing...",0,0);

	HAL_GPIO_WritePin(Flash_CS_GPIO_Port, Flash_CS_Pin, GPIO_PIN_RESET);

	// Zapis gry do pamiêci

	for (int i=0; i<4096; i++)
	{
		sprintf(txt,"%03d%%",((i*100)/4095));
		LCD_print(txt,0,1);
		HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_RESET);
		writeFlash(170, 1365);
		HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_SET);
		writeFlash(85, 682);
		HAL_GPIO_WritePin(Flash_A_1_GPIO_Port, Flash_A_1_Pin, GPIO_PIN_RESET);
		writeFlash(160, 1365);
		writeFlash(gameBuf[i], i);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}

	//************************************************************************

	// Sprawdzanie czy zapis przebieg³ poprawnie.

	LCD_clrScr();
	LCD_print("Verification...",0,0);
	for (int i=0; i<4096; i++)
	{
		gameBuf2[i]=readFlash(i);
		sprintf(txt,"%03d%%",((i*100)/4095));
		LCD_print(txt,0,1);
		if (gameBuf[i]!=gameBuf2[i])
		{
			LCD_clrScr();
			LCD_print("Verification Error",0,0);
			while(1){};
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*fres = f_findfirst(&dir, &finfo, "", "*.bin");
	i=1;
	while (i != (Received-60))
	{
		fres = f_findnext(&dir, &finfo);
		i++;
	}
	if (fres == FR_OK && finfo.fname[0])
	{
		f_open(&file, finfo.fname, FA_READ );
		//LCD_clrScr();
		LCD_print("Reading file",0,1);
		f_read(&file, &gameBuf, 4096, &bytesRead);
		f_close(&file);
		f_closedir(&dir);
		eraseFlash();
		programFlash();

		//Init Flash - not selected

		HAL_GPIO_WritePin(Flash_G_GPIO_Port, Flash_G_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Flash_WE_GPIO_Port, Flash_WE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Flash_RP_GPIO_Port, Flash_RP_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,SET);
		LCD_clrScr();
		LCD_print("Ready!",0,0);
		LCD_print("Please reset",0,1);
		LCD_print("cartridge and",0,2);
		LCD_print("console!",0,3);
		while(1)
		{
			if (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin)==0)
			{
				HAL_Delay(100);
				if (HAL_GPIO_ReadPin(BUT3_GPIO_Port, BUT3_Pin)==0)
				{
					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				}
			}
		}
	}*/
	LCD_clrScr();
	LCD_print("Received",0,0);
	HAL_UART_Receive_IT(&huart1, &Received, 3); // Ponowne w³¹czenie nas³uchiwania
}

void EXTI2_IRQHandler(void)
{

	//for (i=0;i<1;i++);

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)
	{
		//LCD_print("IRQ!",0,4);

		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

		if (HAL_GPIO_ReadPin(Flash_CS_GPIO_Port,Flash_CS_Pin)==0)
			{
			active=active+1;
			if (active>32000)
				{
				active=0;
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				}
			//HAL_GPIO_WritePin(Flash_G_GPIO_Port,Flash_G_Pin,RESET);
			//HAL_GPIO_WritePin(Buf3_OE_GPIO_Port, Buf3_OE_Pin, GPIO_PIN_RESET);
			}

		else
			{
			//HAL_GPIO_WritePin(Flash_G_GPIO_Port,Flash_G_Pin,SET);
			//HAL_GPIO_WritePin(Buf3_OE_GPIO_Port, Buf3_OE_Pin, GPIO_PIN_SET);
			}

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);

	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
