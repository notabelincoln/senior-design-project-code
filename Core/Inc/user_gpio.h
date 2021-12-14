#ifndef USER_GPIO_H
#define USER_GPIO_H

#include "stm32f3xx_hal.h"
#include "user_buttons.h"
#include "user_global.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
HAL_StatusTypeDef uart_printf(const char *fmt, ...);
#endif
