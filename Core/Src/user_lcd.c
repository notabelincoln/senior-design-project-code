/**
 ******************************************************************************
 * @file           : user_lcd.c
 * @brief          : Definitions for functions relating to lcd
 ******************************************************************************
 */
#include "user_lcd.h"
#include "user_math.h"
#include "fatfs.h"
#include <string.h>

int initialize_LCD(){
	return 0;
}

int display_LCD(float data[18]){
	return 0;
}

//pin order: PA0 PB3 PA9 PA10 PA12 PB0 PB1 PF1 (GPIOA_PIN_0 GPIOB_PIN_3 GPIOA_PIN_9 GPIOA_PIN_10 GPIOA_PIN_12 GPIOB_PIN_0 GPIOB_PIN_1 GPIOF_PIN_1)
int set_output(int pin){
	return 0;
}

int reset_output(int pin){
	return 0;
}
int reset_all(){
	return 0;
}
