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
#include "DS1302.h"
#include "ssd1306.h"
#include "stdio.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MIN_TIME 1  // 1 second
#define MAX_TIME 300  // 5 minutes
#define MAX_ENCODER_VALUE 1024
#define TIME_STEP 10  // 10 seconds per detent
#define TOTAL_DETENTS ((MAX_TIME - MIN_TIME) / TIME_STEP)  // total number of detents
#define ENCODER_STEP (MAX_ENCODER_VALUE / TOTAL_DETENTS)
#define ADC_MAX 4095
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define NUM_CHANNELS 5
#define START_X 20
#define START_Y 10
#define GAP_WIDTH 18
#define BAR_WIDTH 5 // Width of the bars. Adjust as desired.
#define ELEVATION 10
#define EEPROM_ADDRESS 0x50 // Device address of 24C64

int adc_flag=0;
int uartflag=0;
int i=0;
uint32_t setTiming1=1;
uint32_t scaledTime;
uint32_t encoderReading;
char uartBuf[50];
char buffer[20];
char graphbuffer[100];
char graphTimeBuffer[100];
int adcraw0=0;
int adcraw1=0;
int adcraw4=0;
int adcraw6=0;
int adcraw7=0;

int adcarray0[100];
int adcarray1[100];
int adcarray4[100];
int adcarray6[100];
int adcarray7[100];

uint32_t counter1=0;
uint32_t counter2=0;


char datalogbuf0[100];
char datalogbuf1[100];
char datalogbuf4[100];
char datalogbuf6[100];
char datalogbuf7[100];
uint32_t counterclk=0;

char adcbuffer0[50];
char adcbuffer1[50];
char adcbuffer4[50];
char adcbuffer6[50];
char adcbuffer7[50];
char time[8];
char time_to_set[8]={0,23,6,11,16,14,00,1};//Contr,Year,Mounth,Date,Hour,Min,Sec,Day
char timeBuffer[100];
char timeBuffer2[100];
char TimeArray[100][50];  // Array to store time events

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//extern I2C_HandleTypeDef hi2c1;  // Assuming you have hi2c1 as your I2C handle
//
//void I2C_ScanBus(void)
//{
//    char msg[64];  // Message buffer
//
//    sprintf(msg, "Scanning I2C bus:\r\n");
//    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//
//    HAL_StatusTypeDef res;
//    for(uint16_t i = 0; i < 128; i++)
//    {
//        res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10000);
//        if(res == HAL_OK)
//        {
//            sprintf(msg, "0x%02X\r\n", i);  // Device found
//            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//        }
//        else
//        {
//            sprintf(msg, ".");  // No device found
//            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//        }
//    }
//
//    sprintf(msg, "Scan completed\r\n");
//    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
	  counterclk++;
	  counter2++;
	  adc_flag=0;
	  uartflag=0;
	  if(counterclk>=setTiming1)
	  {
//	  dataloggerflag=1;

	  adc_flag=1;
	  counterclk=0;
	  }
	  if(counter2>=5)
	  {
		  uartflag=1;
		  counter2=0;
	  }
    // Code to execute every 1 second interval
    // Place your desired actions here
  }
}

void datalogger() ////5 lepta
{


	HAL_Delay(500);
	if(HAL_GPIO_ReadPin(GPIOC, USER_Btn_Pin)==1)
	{
	for (i=0; i<100; i++) // 10 gia test  288 gia 5 lepta gia na bgalw ola ta apothikeumena apo to datalogger_array na ta metatrepsw kai na ta dw sto hyper terminal
	{

	char buffer2[50];
	ssd1306_Fill(Black);

	ssd1306_SetCursor(0, 0); // Position the cursor for the first line
	ssd1306_WriteString("DATA LOGGER", Font_11x18, White);

	ssd1306_SetCursor(0, 20); // Position the cursor for the second line. Adjust the y-coordinate as needed based on your font size.
	ssd1306_WriteString("SENDING", Font_11x18, White);

	ssd1306_SetCursor(0, 40); // Position the cursor for the second line. Adjust the y-coordinate as needed based on your font size.
	ssd1306_WriteString("TO PuTTY", Font_11x18, White);

	ssd1306_UpdateScreen();
    sprintf(graphbuffer, "\n\r~~~~~~~~~~~~~~DATALOGGER~~~~~~~~~~~~~~~~\n\r");
    HAL_UART_Transmit(&huart3, (uint8_t*)graphbuffer, strlen(graphbuffer), 100);
    sprintf(graphTimeBuffer, "\n\r~~~~~~~~~~~~~~%s~~~~~~~~~~~~~~\n\r",TimeArray[i]);
    HAL_UART_Transmit(&huart3, (uint8_t*)graphTimeBuffer, strlen(graphTimeBuffer), 100);
    sprintf(datalogbuf0, "\n\r%d Channel 0  %2d\n\r", i,adcarray0[i]);
    HAL_UART_Transmit(&huart3, (uint8_t*)datalogbuf0, strlen(datalogbuf0), 100);
    sprintf(datalogbuf1, "\n\r   Channel 1  %2d\n\r", adcarray1[i]);
    HAL_UART_Transmit(&huart3, (uint8_t*)datalogbuf1, strlen(datalogbuf1), 100);
    sprintf(datalogbuf4, "\n\r   Channel 4  %2d\n\r", adcarray4[i]);
    HAL_UART_Transmit(&huart3, (uint8_t*)datalogbuf4, strlen(datalogbuf4), 100);
    sprintf(datalogbuf6, "\n\r   Channel 6  %2d\n\r", adcarray6[i]);
    HAL_UART_Transmit(&huart3, (uint8_t*)datalogbuf6, strlen(datalogbuf6), 100);
    sprintf(datalogbuf7, "\n\r   Channel 7  %2d\n\r", adcarray7[i]);
    HAL_UART_Transmit(&huart3, (uint8_t*)datalogbuf7, strlen(datalogbuf7), 100);


//    sprintf(datalogbuf2, " %c\n\r", datalogger_array_status[i]);
//    HAL_UART_Transmit(&huart3, (uint8_t*)datalogbuf2, strlen(datalogbuf2), 100);
	}
	ssd1306_Fill(Black);

	}


}




void ADC_CH0_SELECT()
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void ADC_CH1_SELECT()
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void ADC_CH4_SELECT()
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void ADC_CH6_SELECT()
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_6;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
void ADC_CH7_SELECT()
{
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_7;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_TAKE_ALL()
{


	HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,SET);

	DS1302_ReadTime(time);
	sprintf(timeBuffer2,"\n\r %2d/%2d/%2d %2d:%2d:%2d \n\r",time[1],time[2],time[3],time[4],time[5],time[6]);
	strcpy(TimeArray[counter1], timeBuffer2);

    HAL_Delay(10);
    ADC_CH0_SELECT();
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    adcraw0 = HAL_ADC_GetValue(&hadc3);
    adcarray0[counter1]=adcraw0;
    HAL_ADC_Stop(&hadc3);
    HAL_Delay(10);
    ADC_CH1_SELECT();
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    adcraw1 = HAL_ADC_GetValue(&hadc3);
    adcarray1[counter1]=adcraw1;
    HAL_ADC_Stop(&hadc3);
    HAL_Delay(10);
    ADC_CH4_SELECT();
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    adcraw4 = HAL_ADC_GetValue(&hadc3);
    adcarray4[counter1]=adcraw4;
    HAL_ADC_Stop(&hadc3);
    HAL_Delay(10);
    ADC_CH6_SELECT();
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    adcraw6 = HAL_ADC_GetValue(&hadc3);
    adcarray6[counter1]=adcraw6;
    HAL_ADC_Stop(&hadc3);
    HAL_Delay(10);
    ADC_CH7_SELECT();
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 100);
    adcraw7 = HAL_ADC_GetValue(&hadc3);
    adcarray7[counter1]=adcraw7;
    HAL_ADC_Stop(&hadc3);
    HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,RESET);

    counter1++;
    adc_flag=0;
}

void uarttrigger()
{
	DS1302_ReadTime(time);
	sprintf(timeBuffer,"\n\r %2d/%2d/%2d %2d:%2d:%2d \n\r",time[1],time[2],time[3],time[4],time[5],time[6]);


	HAL_UART_Transmit(&huart3, (uint8_t*)timeBuffer, strlen(timeBuffer), HAL_MAX_DELAY);

	sprintf(uartBuf, "Encoder count: %d\r\n", (int)TIM4->CNT);
	HAL_UART_Transmit(&huart3, (uint8_t*)uartBuf, strlen(uartBuf), HAL_MAX_DELAY);
    sprintf(buffer, "SetTime ADC:%ds\n", scaledTime);
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

	sprintf(adcbuffer0,"\n\r Channel 0  %2d \n\r",adcraw0);
	HAL_UART_Transmit(&huart3, (uint8_t*)adcbuffer0, strlen(adcbuffer0), HAL_MAX_DELAY);
	sprintf(adcbuffer1,"\n\r Channel 1  %2d \n\r",adcraw1);
	HAL_UART_Transmit(&huart3, (uint8_t*)adcbuffer1, strlen(adcbuffer1), HAL_MAX_DELAY);
	sprintf(adcbuffer4,"\n\r Channel 4  %2d \n\r",adcraw4);
	HAL_UART_Transmit(&huart3, (uint8_t*)adcbuffer4, strlen(adcbuffer4), HAL_MAX_DELAY);
	sprintf(adcbuffer6,"\n\r Channel 6  %2d \n\r",adcraw6);
	HAL_UART_Transmit(&huart3, (uint8_t*)adcbuffer6, strlen(adcbuffer6), HAL_MAX_DELAY);
	sprintf(adcbuffer7,"\n\r Channel 7  %2d \n\r",adcraw7);
	HAL_UART_Transmit(&huart3, (uint8_t*)adcbuffer7, strlen(adcbuffer7), HAL_MAX_DELAY);
	for (uint16_t i = 0; i < 10; i++) {
	    EEPROM_Read(i, 0, (uint8_t*)&adcarray0[i], sizeof(int));

	    // Print the value using UART3
	    char uartBuffer[20];
	    sprintf(uartBuffer, "Value at index %d: %d\r\n", i, adcarray0[i]);
	    HAL_UART_Transmit(&huart3, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
	}
	HAL_Delay(10);

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM14_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  DS1302_Init();
  HAL_Delay(200);
	ssd1306_Fill(Black);

	ssd1306_SetCursor(0, 5); // Position the cursor for the first line
	ssd1306_WriteString("Teletos", Font_16x26, White);

	ssd1306_SetCursor(0, 35); // Position the cursor for the first line
	ssd1306_WriteString("Embedded", Font_16x26, White);
	ssd1306_UpdateScreen();
	 HAL_Delay(2000);
 // DS1302_WriteTime(time_to_set);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  I2C_ScanBus();
	  if(HAL_GPIO_ReadPin(EEPROM_BUTTON_GPIO_Port, EEPROM_BUTTON_Pin)==0)
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
		  for (uint16_t i = 0; i < 10; i++) {
		      EEPROM_Write(i, 0, (uint8_t*)&adcarray0[i], sizeof(int));
		  	ssd1306_Fill(Black);

		  	ssd1306_SetCursor(0, 0); // Position the cursor for the first line
		  	ssd1306_WriteString("EEPROM ", Font_11x18, White);
		  	ssd1306_SetCursor(0, 20); // Position the cursor for the first line
		  	ssd1306_WriteString("WRITE ", Font_11x18, White);
		  	ssd1306_UpdateScreen();
		  	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,RESET);
		  }
		  HAL_Delay(500);
	  }
	  ssd1306_Fill(Black);
	  if(uartflag==1)
	  {
		  uarttrigger();
		  uartflag=0;
	  }
      encoderReading = TIM4->CNT;  // Original reading
      int detents = encoderReading / ENCODER_STEP;
      scaledTime = MIN_TIME + (detents * TIME_STEP);
      if (scaledTime > MAX_TIME) {
          scaledTime = MIN_TIME;
      }
      if (scaledTime > MIN_TIME) {
          scaledTime = scaledTime-MIN_TIME;
      }
      if((encoderReading>1018)&&(encoderReading<=1024))
      {
    	  scaledTime=300;
      }
      // Scale the reading to a range of 1 second to 300 seconds
      // Scale the reading to a range of MIN_TIME to MAX_TIME


      // Round to the nearest multiple of 10


      setTiming1=(scaledTime);
      // Now scaledTime should be in the range of 1 second to 300 seconds
      sprintf(buffer, "SetTime ADC:%ds\n", scaledTime);

      ssd1306_SetCursor(13,56); // Adjust these values according to where you want the text to start
      ssd1306_WriteString(buffer, Font_6x8, White); // Replace with your font and color choice


      // take all the adc measurements
      if(counter1>99)
      {
    	  counter1=0;
      }

      if(adc_flag==1)
      {

    	  ADC_TAKE_ALL();
    	  adc_flag=0;

      }
      int barHeight1 = ((adcraw0 * (DISPLAY_HEIGHT - START_Y-ELEVATION)) / ADC_MAX);
      int barHeight2 = ((adcraw1 * (DISPLAY_HEIGHT - START_Y-ELEVATION)) / ADC_MAX);
      int barHeight3 = ((adcraw4 * (DISPLAY_HEIGHT - START_Y-ELEVATION)) / ADC_MAX);
      int barHeight4 = ((adcraw6 * (DISPLAY_HEIGHT - START_Y-ELEVATION)) / ADC_MAX);
      int barHeight5 = ((adcraw7 * (DISPLAY_HEIGHT - START_Y-ELEVATION)) / ADC_MAX);

      // Draw a filled rectangle for each bar
      ssd1306_DrawFilledRectangle(START_X + (0 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - barHeight1-ELEVATION, BAR_WIDTH, barHeight1, White);
      ssd1306_DrawFilledRectangle(START_X + (1 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - barHeight2-ELEVATION, BAR_WIDTH, barHeight2, White);
      ssd1306_DrawFilledRectangle(START_X + (2 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - barHeight3-ELEVATION, BAR_WIDTH, barHeight3, White);
      ssd1306_DrawFilledRectangle(START_X + (3 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - barHeight4-ELEVATION, BAR_WIDTH, barHeight4, White);
      ssd1306_DrawFilledRectangle(START_X + (4 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - barHeight5-ELEVATION, BAR_WIDTH, barHeight5, White);

      // Label for each bar
      ssd1306_SetCursor(START_X + (0 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - ELEVATION-45- 8);  // Assumes a font height
      ssd1306_WriteString("0", Font_6x8, White);  // Replace with your font
      ssd1306_SetCursor(START_X + (1 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - ELEVATION-45- 8);  // Assumes a font height
      ssd1306_WriteString("1", Font_6x8, White);  // Replace with your font
      ssd1306_SetCursor(START_X + (2 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - ELEVATION-45- 8);  // Assumes a font height
      ssd1306_WriteString("4", Font_6x8, White);  // Replace with your font
      ssd1306_SetCursor(START_X + (3 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - ELEVATION-45- 8);  // Assumes a font height
      ssd1306_WriteString("6", Font_6x8, White);  // Replace with your font
      ssd1306_SetCursor(START_X + (4 * (BAR_WIDTH + GAP_WIDTH)), DISPLAY_HEIGHT - ELEVATION-45- 8);  // Assumes a font height
      ssd1306_WriteString("7", Font_6x8, White);  // Replace with your font

      ssd1306_UpdateScreen();

      if(HAL_GPIO_ReadPin(GPIOC, USER_Btn_Pin)==1)
      {
    	  datalogger();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 37;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10B0DCFB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
//
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
//
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
//
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1024;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 64000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1024;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, RTC_CLK_Pin|RTC_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_IO_Pin */
  GPIO_InitStruct.Pin = RTC_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTC_IO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RTC_CLK_Pin RTC_RST_Pin */
  GPIO_InitStruct.Pin = RTC_CLK_Pin|RTC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF3 PF4 PF6 PF7
                           PF11 PF12 PF13 PF14
                           PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC4 PC5 PC6
                           PC7 PC8 PC9 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_MDC_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_MDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB15 PB3
                           PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG5 PG8 PG9
                           PG10 PG12 PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_BUTTON_Pin */
  GPIO_InitStruct.Pin = EEPROM_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EEPROM_BUTTON_GPIO_Port, &GPIO_InitStruct);

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
    char buffer[20];

    ssd1306_Fill(Black);

    sprintf(buffer, "gGMTXMMMMMMMMMMMM\n");
    ssd1306_SetCursor(0,0); // Adjust these values according to where you want the text to start
    ssd1306_WriteString(buffer, Font_7x10, White); // Replace with your font and color choice
    ssd1306_UpdateScreen();
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

