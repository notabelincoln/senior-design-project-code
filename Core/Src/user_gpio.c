#include "user_gpio.h"

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

float sensor_data[18];
HAL_StatusTypeDef ret;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t i;
	uint8_t buffer[32];
	getDataBins(sensor_data, &hi2c1);
	for (i = 0; i < 18; i++) {
		sprintf((char *)buffer, "%d nm: %0.3f\r\n", 410 + 25 * i, sensor_data[i]);
		ret = HAL_UART_Transmit(&huart2, (char *)buffer, strlen((char *)buffer), HAL_MAX_DELAY);
		HAL_Delay(100);
		}
}
