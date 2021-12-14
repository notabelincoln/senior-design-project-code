/*
 * user_lcd.h
 * Header file for code related to the lcd
 * Authors: Emily Currie & Abraham Jordan
 */
#ifndef USER_LCD_H
#define USER_LCD_H
#include "stm32f3xx_hal.h"
#include "user_global.h"
void initialize_LCD();
void display_LCD(float data[18]);
void set_output(int pin);
void reset_output(int pin);
#endif
