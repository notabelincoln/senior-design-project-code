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

TIM_HandleTypeDef htim1;

void display();
void set_output(int pin);
void reset_output(int pin);
void reset_all();
int __io_putchar(int ch);
int __io_getchar(void);



void initialize_LCD(){
	reset_all();
	//Send function set
	set_output(5);	//keep high
	set_output(4);	//data length 1=8 bit
	set_output(3);	//display line number 1=2lines
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	reset_output(5);
	reset_output(4);
	reset_output(3);
	//send display on/off control
	set_output(0);	//cursor blink 1=on
	set_output(1);	//cursor 1=on
	set_output(2);	//display 1=on
	set_output(3);	//keep high
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	reset_output(0);
	reset_output(1);
	reset_output(2);
	reset_output(3);
	//send entry mode set
	set_output(1);	//moving direction 1=to the right
	set_output(2);	//keep high
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	reset_output(1);
	reset_output(2);
	//send display clear
	set_output(0);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_Delay(1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	reset_output(0);
}

void display_LCD(float data[18]){
	int Dom_wavelength;
	int max=0;
	int max_idx;
	for (int i=0;i<18;i++){
		float num=data[i];
		//printf("num: %f", num);
		if(num>max){
			max=num;
			max_idx=i;
		}
	}
	//printf("max_idx: %d\r\n", max_idx);
	Dom_wavelength= 410 + 25 * max_idx;
	while(Dom_wavelength){
		printf("dom wavelength: %d\r\n", Dom_wavelength);
		int digit_bin[8];
		printf("remainder: %d\r\n", Dom_wavelength%10+48);
		decToBin_oneDigit(Dom_wavelength%10+48, digit_bin);
		HAL_Delay(10);
		//set pins
		printf("digitbin:%d %d %d %d %d %d %d %d\r\n",digit_bin[0],digit_bin[1],digit_bin[2],digit_bin[3],digit_bin[4],digit_bin[5],digit_bin[6],digit_bin[7]);
		HAL_Delay(10);
		for (int i=0;i<8;i++){
			if (digit_bin[i]==1){
				set_output(i);
				printf("%d",i);
			}
		}
		HAL_Delay(1000);
		//enable
		printf("enable?\r\n");
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		printf("enabled\r\n");
		//reset pins
		HAL_Delay(100);
		reset_all();
		Dom_wavelength/=10;
	}

	reset_all();
	//ASCI to write A (65 01000001) pin order: PA0 PB3 PA9 PA10 PA12 PB0 PB1 PF1 (GPIOA_PIN_0 GPIOB_PIN_3 GPIOA_PIN_9 GPIOA_PIN_10 GPIOA_PIN_12 GPIOB_PIN_0 GPIOB_PIN_1 GPIOF_PIN_1)
	set_output(0);
	set_output(6);
	//write mode RS-PA3 RW-PA1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(5);
	//enable pin PA8 pulse
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	//reset pins
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	reset_output(0);
	reset_output(6);
}

//pin order: PA0 PB3 PA9 PA10 PA12 PB0 PB1 PF1 (GPIOA_PIN_0 GPIOB_PIN_3 GPIOA_PIN_9 GPIOA_PIN_10 GPIOA_PIN_12 GPIOB_PIN_0 GPIOB_PIN_1 GPIOF_PIN_1)
void set_output(int pin){
	switch (pin){
	case(0):
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			break;
	case(1):
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
			break;
	case(2):
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			break;
	case(3):
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			break;
	case(4):
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			break;
	case(5):
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			break;
	case(6):
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
			break;
	case(7):
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
			break;
	}
}

void reset_output(int pin){
	switch (pin){
	case(0):
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			break;
	case(1):
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
			break;
	case(2):
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
	case(3):
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			break;
	case(4):
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
	case(5):
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			break;
	case(6):
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			break;
	case(7):
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
			break;
	}
}
void reset_all(){
	for(int i=0;i<8;i++){
		reset_output(i);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}
