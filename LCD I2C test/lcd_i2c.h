/*
	Header: Library file for LCD with I2C module

	Pins to use:
		Check the i2c.h header file
*/




#ifndef LCD_I2C
#define LCD_I2C



//Defining the 7-bit address of the PCF8574 I2C module
#define LCD_I2C_ADDR		0x27


#include "i2c.h"
#include "lcd_nibble_hal.h"


void lcd_command(uint8_t cmd){
	//For a command RS(Register Select) pin needs to be LOW (RS = 0)	[RS -> P0]
	//RW needs to be 0 to write to the LCD								[RW -> P1]
	//LCD latches onto the data on the data lines on a H-L pulse of EN	[EN -> P2]

	//RS & RW are low for both. EN is high for the first and goes low on the next
	uint8_t to_send [4] = {0b100, 0b000, 0b100, 0b000};
	//The first two are for the upper nibble and the next two are for the lower nibble

	//Setting the upper nibble
	to_send[0] |= (cmd & 0xF0);
	to_send[1] |= (cmd & 0xF0);

	//Setting the lower nibble
	to_send[2] |= (cmd << 4);
	to_send[3] |= (cmd << 4);

	twi_send(LCD_I2C_ADDR, to_send, 4);
}



void lcd_char(uint8_t snd){
	//For printing RS(Register Select) pin needs to be HIGH (RS = 1)	[RS -> P0]
	//RW needs to be 0 to write to the LCD								[RW -> P1]
	//LCD latches onto the data on the data lines on a H-L pulse of EN	[EN -> P2]

	//RW is low for both. EN is high for the first and goes low on the next
	uint8_t to_send [4] = {0b101, 0b001, 0b101, 0b001};
	//The first two are for the upper nibble and the next two are for the lower nibble

	//Setting the upper nibble
	to_send[0] |= (snd & 0xF0);
	to_send[1] |= (snd & 0xF0);

	//Setting the lower nibble
	to_send[2] |= (snd << 4);
	to_send[3] |= (snd << 4);

	twi_send(LCD_I2C_ADDR, to_send, 4);
}



void lcd_interface_init(){
	//Initializing i2c interface to 10kHz to allow for 400us between every byte sent
	//Most tasks on the LCD take 40us to execute
	twi_init(20);
}




#endif		//LCD_I2C
