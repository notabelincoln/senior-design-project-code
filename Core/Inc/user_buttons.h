#ifndef USER_BUTTONS_H
#define USER_BUTTONS_H
#include <stdarg.h> //for va_list var arg functions
#include "stm32f3xx_hal.h"
#include "user_global.h"

HAL_StatusTypeDef uart_printf(const char *fmt, ...);
HAL_StatusTypeDef get_sample_data();
HAL_StatusTypeDef get_calibration_data();
HAL_StatusTypeDef write_data_to_sd();

#endif
