/*
	Header: Library file for LCD with I2C module

	Pins to use:
		Check the i2c.h header file
*/




#ifndef LCD_I2C
#define LCD_I2C



//Defining the 7-bit address of the PCF8574 I2C module
#define LCD_I2C_ADDR		0x27

#define LCD_RS_HIGH			0x01		//Register Select	[RS -> P0]
#define LCD_RW_HIGH			0x02		//Read / not Write	[RW -> P1]
#define LCD_EN_HIGH			0x04		//Enable			[EN -> P2]
#define LCD_BL_HIGH			0x08		//LED Backlight		[BL -> P3]


#include "i2c_sync.h"
#include "lcd_nibble_hal.h"
#include "main_timer.h"



volatile uint8_t backlight_state = LCD_BL_HIGH;



void lcd_backlight_on(){
	backlight_state = LCD_BL_HIGH;
	uint8_t c = backlight_state;
	twi_send(LCD_I2C_ADDR, &c, 1);
}
void lcd_backlight_off(){
	backlight_state = 0;
	uint8_t c = backlight_state;
	twi_send(LCD_I2C_ADDR, &c, 1);
}



uint8_t lcd_read () {
	//To read we need RW pin to be high									[RW -> P1]

	uint8_t to_send[2];

	//Send the request for the lcd to send its data
	to_send[0] = 0xF0 | LCD_RW_HIGH | backlight_state;
	to_send[1] = 0xF0 | LCD_RW_HIGH | backlight_state | LCD_EN_HIGH;

	//Pulling Enable pin low
	twi_send(LCD_I2C_ADDR, to_send, 1);

	//Delaying for 64 us for the command to be settle
	uint64_t t = main_timer_now();
	while ((main_timer_now() - t) < 2);

	//Pulling enable pin high for the higher nibble to be loaded
	twi_send(LCD_I2C_ADDR, &(to_send[1]), 1);

	//Delaying for 64 us for the command to be settle
	t = main_timer_now();
	while ((main_timer_now() - t) < 2);

	//For holding received data
	uint8_t received[2];

	//Read the upper nibble from the lcd
	uint8_t temp = twi_receive(LCD_I2C_ADDR, received, 1);
	if (temp) return 0xFF;	//If the lcd could not be reached

	//Pulling Enable pin low for the higher nibble to be removed from the lcd output
	twi_send(LCD_I2C_ADDR, to_send, 1);

	//Delaying for 64 us for the command to be settle
	t = main_timer_now();
	while ((main_timer_now() - t) < 2);

	//Pulling enable pin high for the lower nibble to be loaded
	twi_send(LCD_I2C_ADDR, &(to_send[1]), 1);

	//Delaying for 64 us for the command to be settle
	t = main_timer_now();
	while ((main_timer_now() - t) < 2);

	//Read the lower nibble from the lcd
	temp = twi_receive(LCD_I2C_ADDR, &(received[1]), 1);
	if (temp) return 0xFF;	//If the lcd could not be reached

	//Pulling Enable pin low for the lower nibble to be removed from the lcd output
	twi_send(LCD_I2C_ADDR, to_send, 1);

	//Delaying for 64 us for the command to be settle
	t = main_timer_now();
	while ((main_timer_now() - t) < 2);


	//Combining the upper and lower nibbles
	temp |= (received[0] & 0x80);	//Upper nibble first
	temp |= (received[1] >> 4);		//Lower nibble next

	return temp;
}


uint8_t lcd_wait_busy(){
	uint64_t start_t = main_timer_now();
	uint8_t temp;

	//Using a timeout of 3.2 seconds
	while ((main_timer_now() - start_t) < 100000){
		temp = lcd_read();
		if (!(temp & (1 << 7))){
			return temp;
		}
	}
	return 0x00;
}


void lcd_send_nibble(uint8_t cmd, uint8_t Rs){
	//For a command, RS = 0; For writing a character, RS = 1			[RS -> P0]
	//RW needs to be 0 to write to the LCD								[RW -> P1]
	//LCD latches onto the data on the data lines on a H-L pulse of EN	[EN -> P2]
	//Backlight pin needs to be on for the LCD to go on					[BL -> P3]

	//RW are low for all. EN is high for the middle and goes low on the next.
	uint8_t to_send [3] = {0, 0, 0};

	//Setting the lower nibble and addign backlight state
	cmd = cmd << 4;
	to_send[0] |= cmd | backlight_state;
	to_send[1] |= cmd | backlight_state | LCD_EN_HIGH;
	to_send[2] |= cmd | backlight_state;

	//Setting RS if the Rs input parameter is high
	if (Rs){
		to_send[0] |= LCD_RS_HIGH; 
		to_send[1] |= LCD_RS_HIGH;
		to_send[2] |= LCD_RS_HIGH;
	}

	twi_send(LCD_I2C_ADDR, to_send, 3);

	//Delaying for 64 us for the command to be settle
	uint64_t t = main_timer_now();
	while ((main_timer_now() - t) < 2);
}



void lcd_command(uint8_t cmd){
	//For a command, RS(Register Select) pin needs to be LOW (RS = 0)	[RS -> P0]

	//Wait for busy flag to go low
	lcd_wait_busy();

	//To hold the nibbles to be sent
	uint8_t to_send[2] = {0, 0};

	//Upper nibble
	to_send[0] = (cmd >> 4) & 0x0F;
	//Lower nibble
	to_send[1] = (cmd & 0x0F);

	//Sending Upper nibble
	lcd_send_nibble(to_send[0], 0);
	//Sending Lower nibble
	lcd_send_nibble(to_send[1], 0);
}



void lcd_char(uint8_t snd){
	//For printing, RS(Register Select) pin needs to be HIGH (RS = 1)	[RS -> P0]

	//Wait for busy flag to go low
	lcd_wait_busy();

	//To hold the nibbles to be sent
	uint8_t to_send[2] = {0, 0};

	//Upper nibble
	to_send[0] = (snd >> 4) & 0x0F;
	//Lower nibble
	to_send[1] = (snd & 0x0F);

	//Sending Upper nibble
	lcd_send_nibble(to_send[0], 1);
	//Sending Lower nibble
	lcd_send_nibble(to_send[1], 1);
}



void lcd_interface_init(){
	//Initializing i2c interface to 10kHz to allow for 400us between every byte sent
	//Most tasks on the LCD take 40us to execute
	twi_init(100);
}




#endif		//LCD_I2C
