
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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm_dht22.h"
#include <math.h>
#define WIDTH 5
#define HEIGHT 9
#define NUM_OF_MOD 4
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t mode = 1;
uint8_t Przetworzoned = 1;
uint8_t buffer[WIDTH*HEIGHT*NUM_OF_MOD*24];
//uint8_t zero = 0b11100000;
//uint8_t one = 0b11111000;
uint8_t zero = 0b100;
uint8_t one = 0b110;
uint8_t jx = 0;
uint8_t wh = 1;
uint8_t hours;
uint8_t minutes;
uint8_t i2cbuf[3] = {0xFF, 0x06, 0x80};
uint8_t i2cbuf1 = 0x00;
uint8_t i2cbuf2 = 0xFF;
uint8_t i2cbuf3 = 0x06;
uint8_t i2cbufmin = 0x38;
uint8_t i2cbufh = 0x20;
uint8_t started = 0;
uint8_t intr = 0;
uint8_t klikacz = 0;
uint8_t klikaczToggle = 0;
char *godzina;
char str[4];
double bright = 1;
double globH = 0;
double globS = 1;
double globV = 0.05;
uint8_t Received[6] = {'0', '0', '0', '0', '0', '0'};
uint8_t rrr;
const uint8_t font[] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, // space
		4, 4, 4, 4, 0, 0, 4, 0, 0, // "!"
		10, 10, 10, 0, 0, 0, 0, 0, 0, // " " "
		10, 10, 31, 10, 31, 10, 10, 0, 0, // "#"
		4, 15, 20, 14, 5, 30, 4, 0, 0, // "$"
		24, 25, 2, 4, 8, 19, 3, 0, 0, // "%"
		12, 18, 20, 8, 21, 18, 13, 0, 0, // "&"
		12, 4, 8, 0, 0, 0, 0, 0, 0, // "'"
		2, 4, 8, 8, 8, 4, 2, 0, 0, // "("
		8, 4, 2, 2, 2, 4, 8, 0, 0, // ")"
		0, 4, 21, 14, 21, 4, 0, 0, 0, // "*"
		0, 4, 4, 31, 4, 4, 0, 0, 0, // "+"
		0, 0, 0, 0, 12, 4, 8, 0, 0, // ","
		0, 0, 0, 31, 0, 0, 0, 0, 0, // "-"
		0, 0, 0, 0, 0, 12, 12, 0, 0, // "."
		0, 1, 2, 4, 8, 16, 0, 0, 0, // "/"
		14, 17, 19, 21, 25, 17, 14, 0, 0, // "0"
		4, 12, 4, 4, 4, 4, 14, 0, 0, // "1"
		14, 17, 1, 2, 4, 8, 31, 0, 0, // "2"
		31, 2, 4, 2, 1, 17, 14, 0, 0, // "3"
		2, 6, 10, 18, 31, 2, 2, 0, 0, // "4"
		31, 16, 30, 1, 1, 17, 14, 0, 0, // "5"
		6, 8, 16, 30, 17, 17, 14, 0, 0, // "6"
		31, 1, 2, 4, 8, 8, 8, 0, 0, // "7"
		14, 17, 17, 14, 17, 17, 14, 0, 0, // "8"
		14, 17, 17, 15, 1, 2, 12, 0, 0, // "9"
		31, 31, 31, 31, 31, 31, 31, 0, 0, // fill 26, 58
		12, 18, 18, 12, 0, 0, 0, 0, 0, //degree 27, 59
		14, 17, 16, 16, 16, 17, 14, 0, 0//"C" 28, 60
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ustawbufor (uint8_t led, uint8_t red, uint8_t green, uint8_t blue);
void wyslij();
int ledpos(uint8_t x, uint8_t y, uint8_t m);
void printchar(uint8_t chr, uint8_t m);
void printstring(char *str);
void getRTC(char *str);
void czyscbufor();
void przejscie1(double h, double s, double v, char *oldstr, char *newstr);
void fill(double h, double s, double v);
void send_string(char* s);
struct RGB
{
	unsigned char R;
	unsigned char G;
	unsigned char B;
};

struct HSV
{
	double H;
	double S;
	double V;
};

struct RGB HSVToRGB(struct HSV hsv) {
	double r = 0, g = 0, b = 0;

	if (hsv.S == 0)
	{
		r = hsv.V;
		g = hsv.V;
		b = hsv.V;
	}
	else
	{
		int i;
		double f, p, q, t;
		if (hsv.H > 360)
			hsv.H = hsv.H - 360;
		if (hsv.H == 360)
			hsv.H = 0;
		else
			hsv.H = hsv.H / 60;

		i = (int)trunc(hsv.H);
		f = hsv.H - i;

		p = hsv.V * (1.0 - hsv.S);
		q = hsv.V * (1.0 - (hsv.S * f));
		t = hsv.V * (1.0 - (hsv.S * (1.0 - f)));

		switch (i)
		{
		case 0:
			r = hsv.V;
			g = t;
			b = p;
			break;

		case 1:
			r = q;
			g = hsv.V;
			b = p;
			break;

		case 2:
			r = p;
			g = hsv.V;
			b = t;
			break;

		case 3:
			r = p;
			g = q;
			b = hsv.V;
			break;

		case 4:
			r = t;
			g = p;
			b = hsv.V;
			break;

		default:
			r = hsv.V;
			g = p;
			b = q;
			break;
		}

	}

	struct RGB rgb;
	rgb.R = r * 255;
	rgb.G = g * 255;
	rgb.B = b * 255;

	return rgb;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_Intr_Pin){
		//intr = 1;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(Przetworzoned <= Received[1] && Received[0] == 0){
	Przetworzoned = Received[1] + 1;
	}else if(Received[0] == 1){
	send_string("STM32F103C8T6");
		uint8_t aaaaaaaa[] = { 's', 't','m'};
	HAL_UART_Transmit(&huart1, aaaaaaaa, 3, 1000);
	}else if(Received[0] == 2){
		mode = Received[1];
	}else if(Received[0] == 3){
		float rcv = Received[1];
		globV = rcv * 0.01;
	}

		HAL_UART_Receive_IT(&huart1, &rrr, 1);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //DWT_COUNTER_ENABLE();
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x0B, 1, &i2cbuf2, 1, 1000);
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x0C, 1, &i2cbuf2, 1, 1000);
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x0D, 1, &i2cbuf2, 1, 1000);
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x0E, 1, &i2cbuf3, 1, 1000);
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x0F, 1, &i2cbuf1, 1, 1000);
// HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x01, 1, &i2cbufmin, 1, 1000);
 // HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x02, 1, &i2cbufh, 1, 1000);
  getRTC(&str);
  for(uint8_t j = 0; j<180; j++){
		  struct HSV data = { 0, globS, globV};
		  struct RGB value = HSVToRGB(data);
		  ustawbufor(j, value.R, value.G, value.B);

  }
  DWT_COUNTER_ENABLE();
 uint8_t rawReadedData[5];
 Dht22_ReadedData_T DHTReadData;

 Dht22_Initial();
char put[5];
char putt[5];
int abcc;
int abccc;

		if(Dht22_ReadData(rawReadedData, sizeof(rawReadedData)))
 		{
 			float abc = Dht22_calculateTemperature(rawReadedData[2], rawReadedData[1]);
 			abcc = (int)abc;
 			snprintf(put, 5, "%d%c%c", abcc, 59, 60);
 		}
  printstring(put);
  wyslij();
  HAL_Delay(2000);
  started = 1;
  HAL_UART_Receive_IT(&huart1, &Received, 2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//	  HAL_Delay(1000);
//	  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen(data), 1000);


//	  for (uint8_t i = 0; i < 359; i++){
//		  for(uint8_t j = 0; j < 5; j++){
//			  for(uint8_t k = 0; k < 4; k++){
//				  struct HSV data = { i+((j+k)*6), 1, bright};
//				  struct RGB value = HSVToRGB(data);
//				  ustawbufor(ledpos(j, 0, k), value.R, value.G, value.B);
//			  }
//		  }
//		  wyslij();
//	  }
if (mode){
	/*if(1){
		HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x0F, 1, &i2cbuf1, 1, 1000);
			  	char newStr[4];
			  	getRTC(&newStr);
			  	przejscie1(globH, globS, globV, &str, &newStr);
			  	intr = 0;

	}*/
	if(Dht22_ReadData(rawReadedData, sizeof(rawReadedData)))
		{
			char pute[5];
			float abc = Dht22_calculateTemperature(rawReadedData[2], rawReadedData[1]);
			abcc = (int)abc;
			snprintf(pute, 5, "%d%c%c", abcc, 59, 60);
			//HAL_UART_Transmit(&huart1, (uint8_t*)pute, strlen(pute), 1000);
			przejscie1(globH, globS, globV, &str, &pute);
			snprintf(pute, 5, "T%d", abcc);
			send_string(pute);
		}
for(globH = 0; globH<360; globH += 1){


	if(globV>0.02){
	  for(uint8_t j = 0; j<180; j++){
			  struct HSV data = { globH, globS, globV};
			  struct RGB value = HSVToRGB(data);
			  ustawbufor(j, value.R, value.G, value.B);

	  }
	}
	else
	{
		  for(uint8_t j = 0; j<180; j++){
				  ustawbufor(j, 1, 0, 0);

		  }

	}
	  printstring(str);
	  /*if(klikacz<20){
		  struct HSV data = { globH, globS, 0};
		  struct RGB value = HSVToRGB(data);
		  ustawbufor(ledpos(0, 7, 2), value.R, value.G, value.B);
	  }else{
		  struct HSV data = { globH, globS, globV};
		  struct RGB value = HSVToRGB(data);
		  ustawbufor(ledpos(0, 7, 2), value.R, value.G, value.B);
		  if(klikacz>40)klikacz=0;
	  }
	  klikacz++;*/
	  wyslij();
	  if (intr){
	  	HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x0F, 1, &i2cbuf1, 1, 1000);
	  	char newStr[4];
	  	getRTC(&newStr);
	  	przejscie1(globH, globS, globV, &str, &newStr);
	  	intr = 0;
	  }
}


if(Dht22_ReadData(rawReadedData, sizeof(rawReadedData)))
	{
		char pute[5];
		float abc = Dht22_calculateHumidity(rawReadedData[4], rawReadedData[3]);
		abcc = (int)abc;
		snprintf(pute, 5, "%d%c%c", abcc, 37, 32);
		przejscie1(globH, globS, globV, &str, &pute);
		snprintf(pute, 5, "H%d", abcc);
		send_string(pute);
	}
for(globH = 0; globH<360; globH += 1){


	if(globV>0.02){
	  for(uint8_t j = 0; j<180; j++){
			  struct HSV data = { globH, globS, globV};
			  struct RGB value = HSVToRGB(data);
			  ustawbufor(j, value.R, value.G, value.B);

	  }
	}
	else
	{
		  for(uint8_t j = 0; j<180; j++){
				  ustawbufor(j, 1, 0, 0);

		  }

	}
	  printstring(str);
	  wyslij();

}


}
else
{
	  if(Przetworzoned > 0){
		  //jx++;
		 // if(jx == 3){
			  Przetworzoned--;
			//  jx=0;
		 // }
		  struct HSV data = { 100-Przetworzoned, globS, globV};
		  struct RGB value = HSVToRGB(data);
		  for (int i = 0; i<180; i++){
		  ustawbufor(i, value.R, value.G, value.B);
		  }
		  printstring(str);
		  wyslij();
		  if (intr){
		  	HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x0F, 1, &i2cbuf1, 1, 1000);
		  	char newStr[4];
		  	getRTC(&str);
		  	intr = 0;
}
	  }
}
  }
}


//	  for(uint8_t i = 0; i < 180; i++){
//		  struct HSV data = { i, 1, bright};
//		  struct RGB value = HSVToRGB(data);
//		  ustawbufor(i, value.R, value.G, value.B);
//	  }
//		  wyslij();
//	  for(uint8_t i = 0; i<360; i++){
//		  for(uint8_t k = 0; k < 9; k++){
//		  for(uint8_t j = 0; j<180; j++){
//				  struct HSV data = { i, 1, 0.3};
//				  struct RGB value = HSVToRGB(data);
//				  ustawbufor(j, value.R, value.G, value.B);
//
//		  }
//		 // printchar((hours & 15)+16, 0);
//		 // printchar((hours >> 4)+16, 1);
//		 // printchar((minutes & 15)+16, 2);
//		 // printchar(((minutes >> 4) & 3)+16, 3);
//		  printstring(str);
//		  wyslij();
//	  }
//	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_Intr_Pin */
  GPIO_InitStruct.Pin = GPIO_Intr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_Intr_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void ustawbufor (uint8_t led, uint8_t red, uint8_t green, uint8_t blue){
	    for (int i = 0; i <8; i++)
	    {
	    	  if (green & (1 << i)) {
	    		  buffer[led*24+7-i] = one;
	    	  }else{
	    		  buffer[led*24+7-i] = zero;
	    	  }
	    }
	for (int i = 0; i <8; i++)
	{
		  if (red & (1 << i)) {
			  buffer[led*24+15-i] = one;
		  }else{
			  buffer[led*24+15-i] = zero;
		  }
	}
	for (int i = 0; i <8; i++)
	{
		  if (blue & (1 << i)) {
			  buffer[led*24+23-i] = one;
		  }else{
			  buffer[led*24+23-i] = zero;
		  }
	}

}

void wyslij(){
    HAL_SPI_Transmit(&hspi1, &buffer, WIDTH*HEIGHT*NUM_OF_MOD*24, 1000);
    HAL_SPI_Transmit(&hspi1, 0b11111111, 1, 1000);


}

void czyscbufor(){
    for(int i = 0; i < WIDTH*HEIGHT*NUM_OF_MOD*24; i++){
  	  buffer[i] = zero;
    }
}

int ledpos(uint8_t x, uint8_t y, uint8_t m){

return((x+(WIDTH*m))+(y*WIDTH*NUM_OF_MOD));
}
void printchar(uint8_t chr, uint8_t m){

	for(uint8_t y = 0; y<9; y++){
		for(uint8_t i = 0; i<5; i++){
		  if (~font[(chr*9)+y] & (1 << i)) {
				ustawbufor(ledpos(i, y, m), 0, 0, 0);
		  }
		}
	}
}

void printstring(char *str){
	for (uint8_t i = 0; i < 4; i++){
		printchar(str[i]-32, 3-i);
	}
}

void getRTC(char *str){
	uint8_t hr;
	uint8_t mm;
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x02, 1, &hr, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x01, 1, &mm, 1, 1000);
	snprintf(str, 5, "%d%d%d%d", (((hr >> 4) & 3)), ((hr & 15)), ((mm >> 4)), ((mm & 15)));
	 // printchar((hours & 15)+16, 0);
	 // printchar((hours >> 4)+16, 1);
	 // printchar((minutes & 15)+16, 2);
	 // printchar(((minutes >> 4) & 3)+16, 3);
}

void przejscie1(double h, double s, double v, char *oldstr, char *newstr){
	uint8_t strdiff[4];
	double blind = 0.00;
	for (uint8_t i = 0; i < NUM_OF_MOD; i++){
		if(oldstr[i] != newstr[i]){
			strdiff[NUM_OF_MOD-1-i] = 1;
		}else{
			strdiff[NUM_OF_MOD-1-i] = 0;
		}
	}
	  for(uint8_t i = 0; i<v*100; i++){
		  struct HSV data = { h, s, v-blind};
		  struct RGB value = HSVToRGB(data);
		  for(uint8_t l = 0; l<4; l++){
		  if(strdiff[l]==1){
		  for(uint8_t j = 0; j<5; j++){
			  for(uint8_t k = 0; k<7; k++){
				  ustawbufor(ledpos(j, k, l), value.R, value.G, value.B);
			  }
		  }
		  }
		  }
		  blind = blind + 0.01;
		  printstring(oldstr);
		  wyslij();
	  }
	  blind = 0.00;
	  strcpy(oldstr, newstr);
	  for(uint8_t i = 0; i<v*100; i++){
		  struct HSV data = { h, s, 0.0+blind};
		  struct RGB value = HSVToRGB(data);
		  for(uint8_t l = 0; l<4; l++){
		  if(strdiff[l]==1){
		  for(uint8_t j = 0; j<5; j++){
			  for(uint8_t k = 0; k<7; k++){

				  ustawbufor(ledpos(j, k, l), value.R, value.G, value.B);
				  }
			  }
		  }
		  }
		  blind = blind + 0.01;
		  printstring(newstr);
		  wyslij();
	  }
}

void send_string(char* s)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 1000);
}

//void wejscie1(double h, double s, double v, uint8_t m, char *str){
//	  double blind = 0.0;
//	  for(uint8_t i = 0; i<v*100+1; i++){
//		  struct HSV data = { 0, 1, 0.0+blind};
//		  struct RGB value = HSVToRGB(data);
//		  for(uint8_t j = 0; j<5; j++){
//			  for(uint8_t k = 0; k<7; k++){
//				  ustawbufor(ledpos(j, k, m), value.R, value.G, value.B);
//			  }
//		  }
//		  blind = blind + 0.01;
//		  printstring(str);
//		  wyslij();
//	  }
//
//}
void fill(double h, double s, double v){
	  struct HSV data = { h, s, v};
	  struct RGB value = HSVToRGB(data);
	  for(uint8_t j = 0; j<180; j++){
		  	  ustawbufor(j, value.R, value.G, value.B);

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
