/*
	Header: Library for 4-digit seven-segment display

	Pins to be used:
		PortA 				-> A,B,C,D..,H -> A0..A7
		PortB (Lower half)	-> D1,D2,D3,D4 -> B0..B3

	To output to the seven segment display, change the "to_display" array
*/


#ifndef SEVEN_SEG
#define SEVEN_SEG


#include <avr/io.h>

#include "main_timer.h"


char to_display[4] = {0,0,0,0};
const uint64_t hold_display_interval = 100;		//3.2ms (@ 8MHz & 256 prescaler)
volatile uint64_t previous_display_t = 0;
uint8_t displaying = 0;


void init_seven_seg () {
	//Initializing portA to outputs
	DDRA = 0xFF;
	PORTA = 0x00;
	//Initializing pins C0..C3 to outputs
	DDRB |= (0x0F);
	PORTB |= (0x0F);
}



void display_seg(char val){
	switch (val){
		case 0:
			PORTA = 0b11111111;
		break;
		case '0':
			PORTA = 0b11000000;
		break;
		case '1':
			PORTA = 0b11111001;
		break;
		case '2':
			PORTA = 0b10100100;
		break;
		case '3':
			PORTA = 0b10110000;
		break;
		case '4':
			PORTA = 0b10011001;
		break;
		case '5':
			PORTA = 0b10010010;
		break;
		case '6':
			PORTA = 0b10000010;
		break;
		case '7':
			PORTA = 0b11111000;
		break;
		case '8':
			PORTA = 0b10000000;
		break;
		case '9':
			PORTA = 0b10010000;
		break;
		case 'A':
			PORTA = 0b10001000;
		break;
		case 'B':
			PORTA = 0b10000011;
		break;
		case 'C':
			PORTA = 0b11000110;
		break;
		case 'D':
			PORTA = 0b10100001;
		break;
		case '*':
			PORTA = 0b11011110;
		break;
		case '#':
			PORTA = 0b10011100;
		break;
		default: //3 horizontal lines
			PORTA = 0b10110110;
		break;
	}
}


void full_display() {
	if ((main_timer_now() - previous_display_t) >= hold_display_interval) {
		displaying ++;
		if (displaying >= 4) displaying = 0;

		PORTB &= 0xF0;
		PORTB |= (1 << displaying);

		display_seg( to_display[displaying] );

		previous_display_t = main_timer_now();
	}
}





#endif		//SEVEN_SEG
