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

#define LCD_RS 8
#define LCD_RW 9
#define LCD_EN 10
#define LCD_DB0 0
#define LCD_DB1 1
#define LCD_DB2 2
#define LCD_DB3 3
#define LCD_DB4 4
#define LCD_DB5 5
#define LCD_DB6 6
#define LCD_DB7 7

#endif
