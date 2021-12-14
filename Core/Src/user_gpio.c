#include "user_gpio.h"
#include "user_buttons.h"
#include "fatfs.h"
#include <string.h>

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_Delay(200);
	switch (GPIO_Pin) {
	case GPIO_PIN_4: {
		HAL_StatusTypeDef ret;
		ret = get_sample_data();
		break;
	}
	break;
	case GPIO_PIN_5: {
		HAL_StatusTypeDef ret;
		ret = get_calibration_data();
		break;
	}
	break;
	case GPIO_PIN_11: {
		HAL_StatusTypeDef ret;
		ret = write_data_to_sd();
		break;
	}
	}

}
