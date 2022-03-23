/*
 * user_lcd.h
 * Header file for code related to the lcd
 * Authors: Emily Currie & Abraham Jordan
 */
#ifndef USER_LCD_H
#define USER_LCD_H

#include <stdint.h>

#include "stm32f3xx_hal_gpio.h"
#include "stm32f303x8.h"

/* Defines for LCD pins */
#define LCD_RS_PORT GPIOA
#define LCD_RW_PORT GPIOA
#define LCD_EN_PORT GPIOA

#define LCD_DB0_PORT GPIOA
#define LCD_DB1_PORT GPIOB
#define LCD_DB2_PORT GPIOA
#define LCD_DB3_PORT GPIOA
#define LCD_DB4_PORT GPIOA
#define LCD_DB5_PORT GPIOB
#define LCD_DB6_PORT GPIOB
#define LCD_DB7_PORT GPIOF

#define LCD_RS_PIN GPIO_PIN_3
#define LCD_RW_PIN GPIO_PIN_1
#define LCD_EN_PIN GPIO_PIN_8

#define LCD_DB0_PIN GPIO_PIN_0
#define LCD_DB1_PIN GPIO_PIN_3
#define LCD_DB2_PIN GPIO_PIN_9
#define LCD_DB3_PIN GPIO_PIN_10
#define LCD_DB4_PIN GPIO_PIN_12
#define LCD_DB5_PIN GPIO_PIN_0
#define LCD_DB6_PIN GPIO_PIN_1
#define LCD_DB7_PIN GPIO_PIN_1

/* Macros */

/* Function prototypes */
void lcd_init(void);
void lcd_display(uint8_t data, uint8_t size);

#endif
