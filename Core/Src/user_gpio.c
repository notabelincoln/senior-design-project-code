#include "user_gpio.h"

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

float sensor_sample_data[18] = {0};
float sensor_calibration_data[18] = {0};

HAL_StatusTypeDef ret;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_Delay(200);
	switch (GPIO_Pin) {
	case GPIO_PIN_4: {
		uint8_t i;
		uint8_t buffer[32];
		getDataBins(sensor_sample_data, &hi2c1);
		ret = HAL_UART_Transmit(&huart2, "Sample Data:\r\n",
				strlen("Sample Data:\r\n"), HAL_MAX_DELAY);
		HAL_Delay(30);
		for (i = 0; i < 18; i++) {
			sprintf((char *)buffer,	"%d nm: %0.3f\r\n", 410 + 25 * i,
					sensor_calibration_data[i] - sensor_sample_data[i]);
			ret = HAL_UART_Transmit(&huart2, (char *)buffer,
					strlen((char *)buffer), HAL_MAX_DELAY);
			HAL_Delay(30);
		}
		ret = HAL_UART_Transmit(&huart2, "------------------------\r\n",
				strlen("------------------------\r\n"), HAL_MAX_DELAY);
		HAL_Delay(30);
	}
	break;
	case GPIO_PIN_5: {
		uint8_t i;
		uint8_t buffer[32];
		getDataBins(sensor_calibration_data, &hi2c1);
		ret = HAL_UART_Transmit(&huart2, "Calibration Data:\r\n",
				strlen("Calibration Data:\r\n"), HAL_MAX_DELAY);
		HAL_Delay(30);
		for (i = 0; i < 18; i++) {
			sprintf((char *)buffer,	"%d nm: %0.3f\r\n",
					410 + 25 * i, sensor_calibration_data[i]);
			ret = HAL_UART_Transmit(&huart2, (char *)buffer,
					strlen((char *)buffer), HAL_MAX_DELAY);
			HAL_Delay(30);
		}
		ret = HAL_UART_Transmit(&huart2, "------------------------\r\n",
				strlen("------------------------\r\n"), HAL_MAX_DELAY);
		HAL_Delay(30);
	}
	break;
	case GPIO_PIN_11:
		break;
	}

}
