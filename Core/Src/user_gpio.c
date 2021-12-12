#include "user_gpio.h"
#include "user_math.h"
#include "fatfs.h"
#include <string.h>

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_Delay(200);
	switch (GPIO_Pin) {
	case GPIO_PIN_4: {
		// counter and UART string variables
		uint8_t i;

		// get sample data and store it into the sample array
		getDataBins(sensor_sample_data, &hi2c1);

		// display the sample over the uart connection
		ret = uart_printf("Sample Data:\r\n");

		// print out each calibrated sample value
		for (i = 0; i < 18; i++) {
			ret = uart_printf("%d nm: %7.3f\r\n", 410 + 25 * i,
					sensor_calibration_data[i] - sensor_sample_data[i]);
		}

		normalize_sample(sensor_sample_data, sensor_calibration_data, sensor_sample_normal);

		ret = uart_printf("------------------------\r\n");

		// display the sample over the uart connection
		ret = uart_printf("Normalized Sample Data:\r\n");

		// print out each calibrated sample value
		for (i = 0; i < 18; i++) {
			ret = uart_printf("%d nm: %7.3f\r\n", 410 + 25 * i,
					sensor_sample_normal[i]);
		}

		ret = uart_printf("------------------------\r\n");
	}
	break;
	case GPIO_PIN_5: {
		// counter and UART string variables
		uint8_t i;

		// get calibration data and store it into the calibration array
		getDataBins(sensor_calibration_data, &hi2c1);

		// display the calibration data over the uart connection
		ret = uart_printf("Calibration Data:\r\n");

		// print out each calibrated bin value
		for (i = 0; i < 18; i++) {
			ret = uart_printf("%d nm: %0.3f\r\n",
					410 + 25 * i, sensor_calibration_data[i]);
		}
		ret = uart_printf("------------------------\r\n");
	}
	break;
	case GPIO_PIN_11: {
		//some variables for FatFs
		FATFS FatFs; 	//Fatfs handle
		FIL fil; 		//File handle
		FRESULT fres; //Result after operations

		//Let's get some statistics from the SD card
		DWORD free_clusters, free_sectors, total_sectors;

		FATFS* getFreeFs;

		UINT bytesWrote;

		uart_printf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

		HAL_Delay(1000); //a short delay is important to let the SD card settle



		//Open the file system
		fres = f_mount(&FatFs, "", 1); //1=mount now
		if (fres != FR_OK) {
			uart_printf("f_mount error (%i)\r\n", fres);
			return;
		}



		fres = f_getfree("", &free_clusters, &getFreeFs);
		if (fres != FR_OK) {
			uart_printf("f_getfree error (%i)\r\n", fres);
			return;
		}

		//Formula comes from ChaN's documentation
		total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
		free_sectors = free_clusters * getFreeFs->csize;

		uart_printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

		//Now let's try to open file "test.txt"
		fres = f_open(&fil, "test.txt", FA_READ);
		if (fres != FR_OK) {
			uart_printf("f_open error (%i)\r\n");
			return;
		}
		uart_printf("I was able to open 'test.txt' for reading!\r\n");

		//Read 30 bytes from "test.txt" on the SD card
		BYTE readBuf[30];

		//We can either use f_read OR f_gets to get data out of files
		//f_gets is a wrapper on f_read that does some string formatting for us
		TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
		if(rres != 0) {
			uart_printf("Read string from 'test.txt' contents: %s\r\n", readBuf);
		} else {
			uart_printf("f_gets error (%i)\r\n", fres);
		}

		//Be a tidy kiwi - don't forget to close your file!
		f_close(&fil);

		//Now let's try and write a file "write.txt"
		fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS);
		if(fres == FR_OK) {
			uart_printf("I was able to open 'write.txt' for writing\r\n");
		} else {
			uart_printf("f_open error (%i)\r\n", fres);
		}

		//Copy in a string
		strncpy((char*)readBuf, "a new file is made!", 19);

		fres = f_write(&fil, readBuf, 19, &bytesWrote);
		if(fres == FR_OK) {
			uart_printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
		} else {
			uart_printf("f_write error (%i)\r\n");
		}

		//Be a tidy kiwi - don't forget to close your file!
		f_close(&fil);

		HAL_Delay(100);

		//Now let's try and write a file "write.txt"
		fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS);
		if(fres == FR_OK) {
			uart_printf("I was able to open 'write.txt' for writing\r\n");
		} else {
			uart_printf("f_open error (%i)\r\n", fres);
		}

		f_lseek(&fil, f_size(&fil));
		strncpy((char*)readBuf, "a new line is made!", 19);
		fres = f_write(&fil, readBuf, 19, &bytesWrote);
		if(fres == FR_OK) {
			uart_printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
		} else {
			uart_printf("f_write error (%i)\r\n");
		}

		//Be a tidy kiwi - don't forget to close your file!
		f_close(&fil);

		//We're done, so de-mount the drive
		f_mount(NULL, "", 0);
	}
	}

}

HAL_StatusTypeDef uart_printf(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	static HAL_StatusTypeDef ret;

	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (char *)buffer, len, HAL_MAX_DELAY);

	return ret;
}
