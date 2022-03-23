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

#define lcd_clear_display() \
        lcd_write_instruction(0x01)

#define lcd_return_home() \
        lcd_write_instruction(0x02)

#define lcd_entry_mode_set(id, s) \
        lcd_write_instruction(0x04 | ((id && 1) << 1) | (s && 1))

#define lcd_display_control(d, c, b) \
        lcd_write_instruction(0x08 | ((d && 1) << 2) | ((c && 1) << 1) | (b && 1))

#define lcd_cursor_shift(sc, rl) \
        lcd_ write_instruction(0x10 | ((sc && 1) << 3) | ((rl && 1) << 2))

#define lcd_function_set(dl, n, f) \
        lcd_lcd_write_instruction(0x20 | ((dl && 1) << 4) | ((n && 1) << 3) | ((f && 1) << 2))

#define lcd_set_cgram_address(addr) \
        lcd_lcd_write_instruction(0x40 | ((addr & 0x07) << 3))

#define lcd_set_ddram_address(addr) \
        lcd_write_instruction(0x80 | (addr & 0x7f))

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
