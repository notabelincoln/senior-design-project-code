/**
 ******************************************************************************
 * @file           : user_lcd.c
 * @brief          : Definitions for functions relating to lcd
 ******************************************************************************
 */
#include "user_lcd.h"
#include "tim.h"

static volatile uint8_t col = 0; // identifies column position on LCD

/* Set the values for the corresponding lcd pins */
void lcd_set_pins(uint8_t rs, uint8_t rw, uint8_t data)
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
	lcd_set_pins(1, 1, data & 0xff);
	lcd_enable();
#if !USE_8PIN // Send two packets of data if in 4-bit mode
	lcd_set_pins(1, 1, (data << 4) & 0xf0);
	lcd_enable();
#endif
}

/* Write data to lcd ram */
void lcd_write_data(uint8_t data)
{
	lcd_set_pins(1, 0, data & 0xff);
	lcd_enable();
#if !USE_8PIN // Send two packets of data if in 4-bit mode
	lcd_set_pins(1, 0, (data << 4) & 0xf0);
	lcd_enable();
#endif
}
/* Write to lcd instruction register */
void lcd_write_instruction(uint8_t data)
{
	lcd_set_pins(0, 0, data & 0xff);
	lcd_enable();
#if !USE_8PIN // Send two packets of data if in 4-bit mode
	lcd_set_pins(0, 0, (data << 4) & 0xf0);
	lcd_enable();
#endif
}
/* Send enable signal to lcd */
void lcd_enable(void)
{
	HAL_Delay(1);
	LCD_PIN_LOW(EN);
	HAL_Delay(1);
	LCD_PIN_HIGH(EN);
	HAL_Delay(1);
	LCD_PIN_LOW(EN);
	HAL_Delay(1);
}
/* Initialize the lcd */
void lcd_init(void)
{
	HAL_Delay(100);
	lcd_set_pins(0, 0, 0x30);
	lcd_enable();
	HAL_Delay(10);

	lcd_set_pins(0, 0, 0x30);
	lcd_enable();

	HAL_Delay(1);

	lcd_set_pins(0, 0, 0x30);
	lcd_enable();
	HAL_Delay(1);

	lcd_set_pins(0, 0, 0x20);
	lcd_enable();

	lcd_function_set(0, 0, 1);

	lcd_display_control(1, 0, 0);

	lcd_entry_mode_set(1, 0);

	lcd_clear_display();
	lcd_return_home();
}

/* Write a character to the lcd */
void lcd_write_char(char c)
{
	if (col == ROW_LENGTH) {
		col = 0;
		lcd_set_ddram_address(0x00);
	}

	if ((c >= '0') && (c <= '9')) {
		lcd_write_data(c - '0' + 0x30);
		col++;
	} else if ((c >= 'a') && (c <= 'z')) {
		lcd_write_data(c - 'a' + 0x61);
		col++;
	} else if ((c >= 'A') && (c <= 'Z')) {
		lcd_write_data(c - 'A' + 0x41);
		col++;
	} else if ((c == ' ')) {
		lcd_write_data(0x10);
		col++;
	} else if (c == '\n') {
		col = 0;
		lcd_set_ddram_address(20);
	} else {
		col++;
		lcd_write_data(c);
	}
}

/* Display an array of data (like a string) to the lcd */
void lcd_display(uint8_t *data, uint8_t size)
{
	uint8_t i;

	for (i = 0; i < size; i++)
		lcd_write_char(data[i]);
}

/* Write a custom character to a rom address in the lcd */
void custom_char(uint8_t *char_shape, uint8_t address)
{
	uint8_t i;

	lcd_set_cgram_address(address & 0x07);
	for (i = 0; i < CHAR_HEIGHT; i++)
		lcd_write_data(char_shape[i] & 0x1f);
}
