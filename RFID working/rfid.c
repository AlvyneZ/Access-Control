/*
	Program: To test the rfid on an atmega32a

	Pins to be used:
		SPI ->	MOSI -> PB2
				MISO -> PB1
				SCK  -> PB0
				CS   -> PB3
		RESET -> PB4
*/


#include "main_timer.h"
#include "MFRC522_c.h"


int main() {
	init_main_timer();

	lcd_init();

	MFRC522_PCD_Init();

	MFRC522_PCD_DumpVersionToOutput();

	output_heading("Scan Card");
	output_hold_delay();

	while(1){
		// Look for new cards
		if ( ! MFRC522_PICC_IsNewCardPresent()) {
			continue;
		}

		output_heading("Card Found");
		output_hold_delay();

		// Select one of the cards
		if ( ! MFRC522_PICC_ReadCardSerial()) {
			continue;
		}

		// Dump debug info about the card; PICC_HaltA() is automatically called
		MFRC522_PICC_DumpToOutput(&(uid));
	}

	output_heading("Stopped");
}
