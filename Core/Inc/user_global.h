#ifndef SENSOR_DATA_LENGTH
#define SENSOR_DATA_LENGTH 18
#endif

#ifndef USER_GLOBAL_H
#define USER_GLOBAL_H
#include "stm32f3xx_hal.h"

// variables are needed outside main.c, so they should be put here
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

extern float sensor_sample_data[SENSOR_DATA_LENGTH];
extern float sensor_calibration_data[SENSOR_DATA_LENGTH];
extern float sensor_sample_normal[SENSOR_DATA_LENGTH];

extern HAL_StatusTypeDef ret;

#endif
