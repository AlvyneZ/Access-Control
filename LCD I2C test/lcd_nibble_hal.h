/*
	Header: Library for LCD LM016L 4-bit mode operation

	The LCD can be used with or without the I2C module
		With the I2C module, the LCD is in 4 bit mode
		Without the I2C, 8-bit or 4-bit mode can be used

	This library solely deals with the 4-bit mode offering an abstraction layer

	This library is to be included in libraries for LCD_I2C or LCD_4_BIT
		Those libraries will define what pins are to be used
*/



#ifndef LCD_NIBBLE_HAL
//To prevent errors if the header file is included multiple times
#define LCD_NIBBLE_HAL



//Commands
#define LCD_4BIT_INIT			0x02
#define LCD_4BIT_2LINE_5_8_MAT	0x28
#define LCD_CLEAR				0x01
#define LCD_SET_CUR_INCREMENT	0x06
#define LCD_ON_CUR_OFF			0x0C
#define LCD_ON_CUR_BLINK		0X0E
#define LCD_CUR_LINE1			0X80
#define LCD_CUR_LINE2			0XC0
#define LCD_CUR_LEFT			0X10
#define LCD_CUR_RIGHT			0X14
#define LCD_DIS_LEFT			0X18
#define LCD_DIS_RIGHT			0X1C
#define LCD_CGRAM_ADDR			0x40




#include "main_timer.h"



//Prototype functions to be defined in the LCD_I2C or LCD_4_BIT libraries
void lcd_command(uint8_t cmd);
void lcd_char(uint8_t snd);
void lcd_interface_init();
//Note: For LCD_4_BIT, the commands need to have the necessary delays in them (40us min)





static inline void lcd_second_line(){
	lcd_command(LCD_CUR_LINE2);
}
static inline void lcd_first_line(){
	lcd_command (LCD_CUR_LINE1);
}

void lcd_clear(){
	lcd_command (LCD_CLEAR);		// Clear display command
	//Delay for the LCD to clear (3.2ms)
	uint64_t previous_t = main_timer_now();
	while ((main_timer_now() - previous_t) < 100);
	lcd_first_line();		// Cursor at home position
}


void lcd_init(){
	//Note: Needs to be called after main_timer_init()
	lcd_interface_init();

	//Delay for the LCD to power up (20ms) before sending it commands
	while ((main_timer_now()) < 625);

	lcd_command(LCD_4BIT_INIT);				//send for 4 bit initialization of LCD
	lcd_command(LCD_4BIT_2LINE_5_8_MAT);	// 2 line, 5*8 matrix in 4-bit mode
	lcd_command(LCD_ON_CUR_OFF);			// Display on cursor off
	lcd_command(LCD_SET_CUR_INCREMENT);		// Increment cursor (shift cursor to right)
	lcd_clear();							// Clear display screen
}


void lcd_print(uint8_t * str){
	uint64_t previous_t;
	uint8_t i;
	for (i = 0; str[i] != 0; i++){
		lcd_char(str[i]);
		//640us delay
		previous_t = main_timer_now();
		while ((main_timer_now() - previous_t) < 20);
	}
}



void save_custom_char(uint8_t char_map[8], uint8_t char_number){
	//Left shift the char_number by 3 to muliply it by 8
	lcd_command(LCD_CGRAM_ADDR | (char_number << 3));
	uint8_t row;
	for (row = 0; row < 8; row ++){
		lcd_char(char_map[row]);
	}
}




#endif		//LCD_NIBBLE_HAL
