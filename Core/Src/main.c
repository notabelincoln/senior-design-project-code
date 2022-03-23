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
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "sparkfun_as7265x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef SENSOR_DATA_LENGTH
#define SENSOR_DATA_LENGTH 18
#endif

#ifndef FOLDER_NAME
#define FOLDER_NAME "capstone-project"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef mkstr
#define mkstr(str)  #str
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t flags = 0;

const uint8_t sensor_grid[6] = {
		AS7265X_R_G_A_CAL,
		AS7265X_S_H_B_CAL,
		AS7265X_T_I_C_CAL,
		AS7265X_U_J_D_CAL,
		AS7265X_V_K_E_CAL,
		AS7265X_W_L_F_CAL};

const uint8_t spectra[3] = {
		AS72653_UV,
		AS72652_VISIBLE,
		AS72651_NIR};

const uint8_t bulb[3] = {
		AS7265x_LED_UV,
		AS7265x_LED_WHITE,
		AS7265x_LED_IR};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Gather data from sensor array and store it into buffer */
void get_data_bins(float *float_array, I2C_HandleTypeDef *hi2c);

/* Normalize a set of data */
void normalize_data(float *data, float *normal_data, float normal_max,
		float normal_min);

/* Write data to SD card */
void store_data(float *data, const char *filename);

/* Append data to file */
FRESULT open_append(FIL* fp, const char* path);
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
	uint8_t i;

	HAL_StatusTypeDef hal_status;

	float sample_data[SENSOR_DATA_LENGTH] = {0};
	float calibration_data[SENSOR_DATA_LENGTH] = {0};
	float normal_data[SENSOR_DATA_LENGTH] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	HAL_Delay(669); // give the peripherals time to power on

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	begin(&hi2c1, &huart2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		flags = 0;

		HAL_Delay(1); // Needed to prevent compiler from assuming flags defaults to 0

		switch(flags) {
		case (GPIO_PIN_4): { // Get sample data and normalize it
			uart_printf(&huart2, "Getting sample data...\r\n");

			/* Gather sample data */
			get_data_bins(sample_data, &hi2c1);

			/* Normalize the data */
			normalize_data(sample_data, normal_data, 1, 0);

			HAL_Delay(10);

			/* Print out each calibrated sample value to UART */
			for (i = 0; i < SENSOR_DATA_LENGTH; i++) {
				hal_status = uart_printf(&huart2, "%d nm: %0.3f\r\n", 410 + 25 * i,
						sample_data[i]);

				HAL_Delay(10);

				if (hal_status != HAL_OK)
					break;
			};
			continue;
		}
		case (GPIO_PIN_5): {
			/* Gather calibration data from sensor */
			uart_printf(&huart2, "Getting calibration data...\r\n");

			/* Gather sample data */
			get_data_bins(calibration_data, &hi2c1);

			HAL_Delay(10);

			/* Print out each calibrated sample value to UART */
			for (i = 0; i < SENSOR_DATA_LENGTH; i++) {
				hal_status = uart_printf(&huart2, "%d nm: %0.3f\r\n", 410 + 25 * i,
						calibration_data[i]);

				HAL_Delay(10);

				if (hal_status != HAL_OK)
					break;
			};
			continue;
		}
		case (GPIO_PIN_11): {
			HAL_Delay(100);
			store_data(sample_data, FOLDER_NAME"/samples.csv");
			store_data(calibration_data, FOLDER_NAME"/calibration.csv");
			store_data(normal_data, FOLDER_NAME"/normal.csv");
			continue;
		}
		default: continue;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* Print to serial port */
HAL_StatusTypeDef uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{

	static char buffer[256];
	va_list args;
	HAL_StatusTypeDef ret;

	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	ret = HAL_UART_Transmit(huart, (uint8_t *)buffer, len, HAL_MAX_DELAY);

	return ret;
}

/* Gather data from sensor array and store it into buffer */
void get_data_bins(float *float_array, I2C_HandleTypeDef *hi2c)
{
	uint8_t i;
	uint8_t j;

	takeMeasurements(hi2c); //This is a hard wait while all 18 channels are measured

	for (i = 0; i < 3; i++) {
		enableBulb(bulb[i], hi2c);
		for (j = 0; j < 6; j++) {
			float_array[6 * i + j] = getCalibratedValue(sensor_grid[j], spectra[i], hi2c);
		}
		disableBulb(bulb[i], hi2c);
	}
}

/* Normalize a set of data */
void normalize_data(float *data, float *normal_data, float normal_max,
		float normal_min)
{
	uint8_t i;
	float data_max;
	float data_min;

	data_max = data[0];
	data_min = data[0];

	for (i = 1; i < SENSOR_DATA_LENGTH; i++) {
		data_max = (data_max > data[i]) ? data_max : data[i];
		data_min = (data_min < data[i]) ? data_min : data[i];
	}

	for (i = 0; i < SENSOR_DATA_LENGTH; i++) {
		normal_data[i] = ((normal_max - normal_min) * (data[i] - data_min) / \
				(data_max - data_min)) + normal_min;
	}
}

/* Write data to SD card */
void store_data(float *data, const char *filename)
{
	uint8_t i;
	FATFS fatfs;
	FIL fil;
	FRESULT fres; //Result after operations
	UINT write_res;

	write_res = 0;

	char float_str[16];

	//Open the file system
	fres = f_mount(&fatfs, "", 1); //1=mount now
	if (fres != FR_OK) {
		uart_printf(&huart2, "f_mount error (%i)\r\n", fres);
		return;
	}

	fres = f_stat("0:/"FOLDER_NAME, NULL);
	if (fres == FR_NO_FILE) {
		uart_printf(&huart2, "Attempting to create directory %s...\r\n", FOLDER_NAME);
		fres = f_mkdir("0:/"FOLDER_NAME);
		if (fres != FR_OK) {
			uart_printf(&huart2, "Error creating directory %s\r\n", FOLDER_NAME);
			return;
		}
	}

	uart_printf(&huart2, "Attempting to write data to %s...\r\n", filename);

	for (i = 0; i < SENSOR_DATA_LENGTH - 1; i++) {
		fres = open_append(&fil, filename);
		if (fres == FR_OK) {
			sprintf(float_str, "%0.3f,", data[i]);
			uart_printf(&huart2, "Writing value %s to %s\r\n", float_str, filename);
			f_write(&fil, float_str, (UINT)strlen(float_str), &write_res);
		} else {
			uart_printf(&huart2, "Error writing data unit %u to %s\r\n", i, filename);
			return;
		}
		f_close(&fil);
	}

	fres = open_append(&fil, filename);
	if (fres == FR_OK) {
		sprintf(float_str, "%0.3f,", data[SENSOR_DATA_LENGTH]);
		uart_printf(&huart2, "Writing value %s to %s\r\n", float_str, filename);
		f_write(&fil, float_str, (UINT)strlen(float_str), &write_res);
	} else {
		uart_printf(&huart2, "Error writing data unit %u to %s\r\n", SENSOR_DATA_LENGTH - 1, filename);
		return;
	}
	f_close(&fil);


	fres = open_append(&fil, filename);
	if (fres == FR_OK) {
		f_printf(&fil, "\n");
	} else {
		uart_printf(&huart2, "Error opening %s\r\n", filename);
		return;
	}

	fres = f_close(&fil);
	if (fres != FR_OK)
		uart_printf(&huart2, "Error closing %s\r\n", filename);
	else
		uart_printf(&huart2, "Successfully wrote to %s\r\n", filename);

	f_mount(NULL, "", 0);
	HAL_Delay(100);
}

/* Append data to file */
FRESULT open_append(FIL* fp, const char* path)
{
	FRESULT fr;

	/* Opens an existing file. If not exist, creates a new file. */
	fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
	if (fr == FR_OK) {
		/* Seek to end of the file to append data */
		fr = f_lseek(fp, f_size(fp));
		if (fr != FR_OK)
			f_close(fp);
	}
	return fr;
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
