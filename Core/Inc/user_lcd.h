/*
 * user_lcd.h
 * Header file for code related to the lcd
 * Authors: Emily Currie & Abraham Jordan
 */
#ifndef USER_LCD_H
#define USER_LCD_H
#include "stm32f3xx_hal.h"
#include "user_global.h"
int initialize_LCD();
int display_LCD(float data[18]);
int set_output(int pin);
int reset_output(int pin);

#define LCD_RS_PIN 8
#define LCD_RW_PIN 9
#define LCD_EN_PIN 10

#define LCD_DB0_PIN 0
#define LCD_DB1_PIN 1
#define LCD_DB2_PIN 2
#define LCD_DB3_PIN 3
#define LCD_DB4_PIN 4
#define LCD_DB5_PIN 5
#define LCD_DB6_PIN 6
#define LCD_DB7_PIN 7

#endif
