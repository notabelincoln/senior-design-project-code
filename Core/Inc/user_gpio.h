#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include "stm32f3xx_hal.h"
#include "sparkfun_as7265x.h"
#include "user_global.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void user_printstr(const char *fmt, ...);
