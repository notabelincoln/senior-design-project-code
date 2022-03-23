/**
 ******************************************************************************
 * @file           : user_lcd.c
 * @brief          : Definitions for functions relating to lcd
 ******************************************************************************
 */
#include "user_lcd.h"

/* Set the values for the corresponding lcd pins */
void lcd_set_pins(uint8_t rs, uint8_t, rw, uint8_t data)
{
	(rs && 1) ? LCD_PIN_HIGH(RS) : LCD_PIN_LOW(RS);
	(rw && 1) ? LCD_PIN_HIGH(RW) : LCD_PIN_LOW(RW);
#if USE_8PIN
	(data & 0x01) ? LCD_PIN_HIGH(DB0) : LCD_PIN_LOW(DB0);
	(data & 0x02) ? LCD_PIN_HIGH(DB1) : LCD_PIN_LOW(DB1);
	(data & 0x04) ? LCD_PIN_HIGH(DB2) : LCD_PIN_LOW(DB2);
	(data & 0x08) ? LCD_PIN_HIGH(DB3) : LCD_PIN_LOW(DB3);
#endif
	(data & 0x10) ? LCD_PIN_HIGH(DB4) : LCD_PIN_LOW(DB4);
	(data & 0x20) ? LCD_PIN_HIGH(DB5) : LCD_PIN_LOW(DB5);
	(data & 0x40) ? LCD_PIN_HIGH(DB6) : LCD_PIN_LOW(DB6);
	(data & 0x80) ? LCD_PIN_HIGH(DB7) : LCD_PIN_LOW(DB7);
}

/* Read data from the lcd ram */
void lcd_read_data(uint8_t data)
{
	set_pins(1, 1, data & 0xff);
	enable();
#if !USE_8PIN
	set_pins(1, 1, (data << 4) & 0xf0);
	enable();
#endif
}

/* Write data to lcd ram */
void lcd_write_data(uint8_t data)
{
	set_pins(1, 0, data & 0xff);
	enable();
#if !USE_8PIN
	set_pins(1, 0, (data << 4) & 0xf0);
	enable();
#endif
}
/* Write to lcd instruction register */
void lcd_write_instruction(uint8_t data)
{
	set_pins(0, 0, data & 0xff);
	enable();
#if !USE_8PIN
	set_pins(0, 0, (data << 4) & 0xf0);
	enable();
#endif
}
/* Send enable signal to lcd */
void lcd_enable(void)
{
	user_usleep(100);
	LCD_PIN_LOW(EN);
	user_usleep(100);
	LCD_PIN_HIGH(EN);
	user_usleep(100);
	LCD_PIN_LOW(EN);
	user_usleep(100);
}
/* Initialize the lcd */
void lcd_init(void)
{
	set_pins(0, 0, 0x30);
	enable();
	user_usleep(4200);

	set_pins(0, 0, 0x30);
	enable();

	user_usleep(100);

	set_pins(0, 0, 0x30);
	enable();
	user_usleep(100);

	set_pins(0, 0, 0x20);
	enable();

	function_set(0, 1, 0);

	display_control(1, 1, 0);

	entry_mode_set(1, 0);

	clear_display();

	return_home();
}

/* Write a character to the lcd */
void lcd_write_char(char c)
{
	return;
}

/* Display an array of data (like a string) to the lcd */
void lcd_display(uint8_t data, uint8_t size)
{
	return;
}

/* Write a custom character to a rom address in the lcd */
void custom_char(uint8_t *char_shape, uint8_t address)
{
	return;
}
