/* user_lcd.h
 * Header file for code related to the lcd
 * Authors: Emily Currie & Abraham Jordan
 */
#ifndef USER_LCD_H
#define USER_LCD_H

#include <stdint.h>

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

/* Decide whether to operate lcd in 4 bit or 8 bit mode */
#define USE_8PIN 0

/* Macros */
#define LCD_PIN_HIGH(pin) LCD_##pin##_PORT |= LCD_##pin## _PIN
#define LCD_PIN_LOW(pin) LCD_##pin##_PORT &= ~(LCD_##pin##_PIN)

/* Function prototypes */

/* Set the values for the corresponding lcd pins */
void lcd_set_pins(uint8_t rs, uint8_t, rw, uint8_t data);

/* Read data from the lcd ram */
void lcd_read_data(uint8_t data);

/* Write data to lcd ram */
void lcd_write_data(uint8_t data);

/* Write to lcd instruction register */
void lcd_write_instruction(uint8_t data);

/* Send enable signal to lcd */
void lcd_enable(void);

/* Initialize the lcd */
void lcd_init(void);

/* Write a character to the lcd */
void lcd_write_char(char c);

/* Display an array of data (like a string) to the lcd */
void lcd_display(uint8_t data, uint8_t size);

/* Write a custom character to a rom address in the lcd */
void custom_char(uint8_t *char_shape, uint8_t address);

#endif
