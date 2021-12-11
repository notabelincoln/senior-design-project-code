#include "user_gpio.h"
#include "user_math.h"
#include "fatfs.h"
#include <string.h>

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

float sensor_sample_data[SENSOR_DATA_LENGTH] = {0};
float sensor_calibration_data[SENSOR_DATA_LENGTH] = {0};

HAL_StatusTypeDef ret;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_Delay(200);
	switch (GPIO_Pin) {
	case GPIO_PIN_4: {
		// counter and UART string variables
		uint8_t i;
		uint8_t buffer[32];
		float sensor_sample_normal[SENSOR_DATA_LENGTH];

		// get sample data and store it into the sample array
		getDataBins(sensor_sample_data, &hi2c1);

		// display the sample over the uart connection
		ret = HAL_UART_Transmit(&huart2, "Sample Data:\r\n",
				strlen("Sample Data:\r\n"), HAL_MAX_DELAY);
		HAL_Delay(30);

		// print out each calibrated sample value
		for (i = 0; i < 18; i++) {
			sprintf((char *)buffer,	"%d nm: %0.3f\r\n", 410 + 25 * i,
					sensor_calibration_data[i] - sensor_sample_data[i]);
			ret = HAL_UART_Transmit(&huart2, (char *)buffer,
					strlen((char *)buffer), HAL_MAX_DELAY);
			HAL_Delay(30);
		}

		normalize_sample(sensor_sample_data, sensor_calibration_data, sensor_sample_normal);

		ret = HAL_UART_Transmit(&huart2, "------------------------\r\n",
				strlen("------------------------\r\n"), HAL_MAX_DELAY);
		HAL_Delay(30);

		// display the sample over the uart connection
		ret = HAL_UART_Transmit(&huart2, "Normalized Sample Data:\r\n",
				strlen("Normalized Sample Data:\r\n"), HAL_MAX_DELAY);
		HAL_Delay(30);

		// print out each calibrated sample value
		for (i = 0; i < 18; i++) {
			sprintf((char *)buffer,	"%d nm: %0.3f\r\n", 410 + 25 * i,
					sensor_sample_normal[i]);
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
		// counter and UART string variables
		uint8_t i;
		uint8_t buffer[32];

		// get calibration data and store it into the calibration array
		getDataBins(sensor_calibration_data, &hi2c1);

		// display the calibration data over the uart connection
		ret = HAL_UART_Transmit(&huart2, "Calibration Data:\r\n",
				strlen("Calibration Data:\r\n"), HAL_MAX_DELAY);
		HAL_Delay(30);

		// print out each calibrated bin value
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
	case GPIO_PIN_11: {
		myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

		HAL_Delay(1000); //a short delay is important to let the SD card settle

		//some variables for FatFs
		FATFS FatFs; 	//Fatfs handle
		FIL fil; 		//File handle
		FRESULT fres; //Result after operations

		//Open the file system
		fres = f_mount(&FatFs, "", 1); //1=mount now
		if (fres != FR_OK) {
			myprintf("f_mount error (%i)\r\n", fres);
			while(1);
		}

		//Let's get some statistics from the SD card
		DWORD free_clusters, free_sectors, total_sectors;

		FATFS* getFreeFs;

		fres = f_getfree("", &free_clusters, &getFreeFs);
		if (fres != FR_OK) {
			myprintf("f_getfree error (%i)\r\n", fres);
			while(1);
		}

		//Formula comes from ChaN's documentation
		total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
		free_sectors = free_clusters * getFreeFs->csize;

		myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

		//Now let's try to open file "test.txt"
		fres = f_open(&fil, "test.txt", FA_READ);
		if (fres != FR_OK) {
			myprintf("f_open error (%i)\r\n");
			while(1);
		}
		myprintf("I was able to open 'test.txt' for reading!\r\n");

		//Read 30 bytes from "test.txt" on the SD card
		BYTE readBuf[30];

		//We can either use f_read OR f_gets to get data out of files
		//f_gets is a wrapper on f_read that does some string formatting for us
		TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
		if(rres != 0) {
			myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
		} else {
			myprintf("f_gets error (%i)\r\n", fres);
		}

		//Be a tidy kiwi - don't forget to close your file!
		f_close(&fil);

		//Now let's try and write a file "write.txt"
		fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
		if(fres == FR_OK) {
			myprintf("I was able to open 'write.txt' for writing\r\n");
		} else {
			myprintf("f_open error (%i)\r\n", fres);
		}

		//Copy in a string
		strncpy((char*)readBuf, "a new file is made!", 19);
		UINT bytesWrote;
		fres = f_write(&fil, readBuf, 19, &bytesWrote);
		if(fres == FR_OK) {
			myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
		} else {
			myprintf("f_write error (%i)\r\n");
		}

		//Be a tidy kiwi - don't forget to close your file!
		f_close(&fil);

		//We're done, so de-mount the drive
		f_mount(NULL, "", 0);
	}
	}

}

void user_printstr(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}
