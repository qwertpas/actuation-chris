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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

#include "pca9534.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t UART2_rxBuffer[4l] = {0};
uint8_t keypress = 0;

uint32_t millis = 0;
float data_out[3] = {0}; //send to raspberry pi: (psi1, psi2, temp1)
uint8_t act_channels[11] = {0}; //first element is unused so 1-10 correspond to labeled channels

uint8_t ok_to_go;

int _write(int fd, char* ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
   return len;
}


//copied from daq-testbed-basic
uint8_t opcodes_to_spi(uint16_t* opcodes, uint8_t num_opcodes, uint8_t* spi_bytes) {
  for (uint8_t i = 0; i < num_opcodes; i++) {
    spi_bytes[i * 3 + 0] = opcodes[i] >> 8;
    spi_bytes[i * 3 + 1] = opcodes[i];
  }
  return num_opcodes * 3;
}

uint16_t read_single_reg(uint8_t address) {
  uint16_t opcodes[] = { 0xA000 | (address << 7) };

  uint8_t tx[3] = {0};
  opcodes_to_spi(opcodes, 1, tx);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0); // CS low
  HAL_SPI_Transmit(&hspi1, tx, 3, HAL_MAX_DELAY); // send command
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // CS high

  tx[0] = 0;
  tx[1] = 0;
  tx[2] = 0;

  uint8_t rx[3] = {0};

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0); // CS low
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, HAL_MAX_DELAY); // read response
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // CS high

  return (rx[0] << 8) | rx[1];
}

void write_single_reg(uint8_t address, uint16_t value) {
  uint16_t opcodes[] = { 0x6000 | (address << 7), value };

  uint8_t tx[6] = {0};
  opcodes_to_spi(opcodes, 2, tx);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0); // CS low
  HAL_SPI_Transmit(&hspi1, tx, 6, HAL_MAX_DELAY); // send command
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // CS high
}

float adc_to_volt(int32_t adc_count, int32_t gain){
	float volt = ((adc_count-1) / 8388608.0) * 1.2 / (float)(gain);
	return volt;
}

float adc_to_psi(int32_t adc_count){
	float mA = adc_to_volt(adc_count, 1) * 1000 / 55.5;
	float psi = 62.5 * mA + -250;
	return psi;
}


void actuate(int32_t channel, int32_t off){ //labeled channels 1-16
	if(channel <= 8){
		pca9534_set_channel(&hi2c1, PCA9534_OUTPUT_1, channel-1, !off);
	}else{
		pca9534_set_channel(&hi2c1, PCA9534_OUTPUT_2, channel-9, !off);
	}
}

void abort_valves(){
	for(uint8_t i=1; i<=8; i++){
		actuate(i, 0);
		act_channels[i] = 0;
	}
	actuate(9, 1);
	act_channels[9] = 1;
	actuate(10, 1);
	act_channels[10] = 1;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // disable printf buffering
  setvbuf(stdout, NULL, _IONBF, 0);
  
  HAL_TIM_Base_Start_IT(&htim1);

  pca9534_init_output(&hi2c1, PCA9534_OUTPUT_1);
  pca9534_init_output(&hi2c1, PCA9534_OUTPUT_2);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

  //set all channels off
   for (uint32_t i = 1; i <= 16; i++){
	   actuate(i, 0);
   }

   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);


//  for(uint8_t i = 0; i < 8; i++){
//	  pca9534_set_channel(&hi2c1, PCA9534_OUTPUT_1, i, 0);
//  }
//  for(uint8_t i = 0; i < 8; i++){
//  	  pca9534_set_channel(&hi2c1, PCA9534_OUTPUT_2, i, 0);
//    }

  //DAQ TESBED init
  HAL_Delay(50);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // deassert RESET
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1); // deassert CS


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0); // strobe RESET
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1); // strobe RESET

  HAL_Delay(100);

  write_single_reg(0x04, 0x0400);
  //set channel 2 gain to 16
  //0000 0100 0000 0000

  //start reciving commands
  HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);


printf("-------------------------------------------------------------------- \r\n");
printf("  ms         psi1        psi2       TC1 mV    | 1 2 3 4 5 6 7 8 9 10 \r\n");

	ok_to_go = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {




    uint8_t drdy = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5); // read DRDY
	if (drdy) {

		//request
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		uint8_t tx_data[32] = {0};
		uint8_t rx_data[32];
		HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 27, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);


		int32_t channels[8];
		for (uint8_t i = 0; i < 8; i++) {
		  channels[i] = (rx_data[i * 3 + 3] << 24) | (rx_data[i * 3 + 4] << 16) | (rx_data[i * 3 + 5] << 8);
		  channels[i] >>= 8;

		  data_out[0] = adc_to_psi(channels[0]);
		  data_out[1] = adc_to_psi(channels[1]);
		  data_out[2] = adc_to_volt(channels[2], 16) * 1000.0 - 0.5;

		}
	}

	if(keypress){
		char key = UART2_rxBuffer[0];
			 if(key == '1') {act_channels[1] = 1; actuate(1, 1);}
		else if(key == '2') {act_channels[2] = 1; actuate(2, 1);}
		else if(key == '3') {act_channels[3] = 1; actuate(3, 1);}
		else if(key == '4') {act_channels[4] = 1; actuate(4, 1);}
		else if(key == '5') {act_channels[5] = 1; actuate(5, 1);}
		else if(key == '6') {act_channels[6] = 1; actuate(6, 1);}
		else if(key == '7') {act_channels[7] = 1; actuate(7, 1);}
		else if(key == '8') {act_channels[8] = 1; actuate(8, 1);}
		else if(key == '9') {act_channels[9] = 1; actuate(9, 1);}
		else if(key == '0') {act_channels[10] = 1; actuate(10, 1);}

		else if(key == 'q') {act_channels[1] = 0; actuate(1, 0);}
		else if(key == 'w') {act_channels[2] = 0; actuate(2, 0);}
		else if(key == 'e') {act_channels[3] = 0; actuate(3, 0);}
		else if(key == 'r') {act_channels[4] = 0; actuate(4, 0);}
		else if(key == 't') {act_channels[5] = 0; actuate(5, 0);}
		else if(key == 'y') {act_channels[6] = 0; actuate(6, 0);}
		else if(key == 'u') {act_channels[7] = 0; actuate(7, 0);}
		else if(key == 'i') {act_channels[8] = 0; actuate(8, 0);}
		else if(key == 'o') {act_channels[9] = 0; actuate(9, 0);}
		else if(key == 'p') {act_channels[10] = 0; actuate(10, 0);}
		else if(key == 127) {abort_valves();}

		keypress = 0;
	}


	printf("%ld,   ", millis);
	for(uint8_t i = 0; i < 3; i++){
		printf("%f, ", data_out[i]);
	}
	printf("  ");
	for(uint8_t i = 1; i <= 10; i++){
		printf("%d ", act_channels[i]);
	}
	printf("\r\n");

//	HAL_Delay(1);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//    printf("1 milli!\r\n");
    millis++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	printf("recv: %c \r\n", UART2_rxBuffer[0]);
	keypress = 1;
    HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 1);
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
