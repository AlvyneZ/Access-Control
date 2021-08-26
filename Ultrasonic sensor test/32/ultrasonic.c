/*
	Program: Ultrasonic sensor test code using ATmega32a

	Pins to Use:
		For the ultrasonic sensor:
			Check the Ultrasonic_sensor.h header file
		For the output to seven segment display:
			Check the seven_seg.h header file
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "main_timer.h"
#include "ultrasonic_sensor.h"
#include "seven_seg.h"




void received_distance(double distance){
	int i;
	uint8_t digit_found = 0;

	int temp = (int)(distance);
	i = temp / 1000;
	if (i == 0)
		to_display[0] = 0;
	else{
		to_display[0] = '0' + i;
		digit_found = 1;
	}
	temp = temp % 1000;

	i = temp / 100;
	if ((i == 0) && (digit_found == 0))
		to_display[1] = 0;
	else{
		to_display[1] = '0' + i;
		digit_found = 1;
	}
	temp = temp % 100;

	i = temp / 10;
	if ((i == 0) && (digit_found == 0))
		to_display[2] = 0;
	else{
		to_display[2] = '0' + i;
		digit_found = 1;
	}
	temp = temp % 10;

	to_display[3] = '0' + temp;
}



int main () {
	//Initializing main system timer
	init_main_timer();
	//Initializing ultrasonic sensor
	init_ultrasonic();

	//Setting up the output
	init_seven_seg();

	//Main loop
	while (1){
		sense_ultrasonic();
		full_display();
	}
	//End main loop
}
