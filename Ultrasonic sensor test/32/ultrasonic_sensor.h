/*
	Header: Library for using the ultrasonic sensor

	Pins Used For the ultrasonic sensor:
		TRIG -> Pin C3			-> pin25
		ECHO -> Pin D3(INT1)	-> pin17

	The "received_distance(double distance)" function will need to be defined in the code that includes
		the ultrasonic_sensor header file.
		It is the function to be run when the distance is received

	Note: The ECHO pin needs to be an external interrupt pin that allows
		triggerring on both rising and falling edges.

		For the ATmega32a, only D2(INT0) and D3(INT1) allow this
*/


#ifndef ULTRASONIC
//To prevent errors if the header file is included more than once
#define ULTRASONIC


#include <avr/io.h>		//Required for Data Direction Registers (DDR)
#include <avr/interrupt.h>	//Required for the external interrupts

//The Ultrasonic sensor depends on the main timer for its timing functions
#include "main_timer.h"



//To hold the time of the last read of the ultrasonic sensor
volatile uint64_t previous_ultrasonic_read_t = 0;


//Prototype definition of the function to be run when the distance is received
void received_distance(double distance);




void init_ultrasonic(){
	//Setting pin C3 to output
	DDRC |= (1 << 3);
	//Pulling pinC3 low (initial state)
	PORTC &= ~(1 << 3);

	//Setting pin D3 to input
	DDRD &= ~(1 << 3);
	//Setting pull down resistor on pin D3
	PORTD &= ~(1 << 3);

	//Enabling interrupts on pin D3 (INT1)
	GICR |= (1 << INT1);
	//Setting INT1 to trigger on both edges (logic change)
	MCUCR &= ~(1 << ISC11);
	MCUCR |= (1 << ISC10);
}


int echo_pin_low() {
	//Function for checking if the echo pin is low
	return (!(PORTD & (1 << 3)));
}



void calculate_distance(){
	//To get the signal travel time
	uint64_t signal_time = main_timer_now() - previous_ultrasonic_read_t;
	//To hold the distance in cm from the ultrasonic sensor
	double distance = (double)(34300 * signal_time * MAIN_TIMER_PRESCALER) / (double)(2 * F_CPU);
	/*
		Note: smallest distance determinable (resolution) is calculated for signal_time = 1
			For 256 prescaler and 8MHz clock, resolution is 0.5488cm
	*/

	received_distance(distance);
}


ISR(INT1_vect){
	if (echo_pin_low()){
		//If echo pin is low (ie after a falling edge)
		//Falling edge -> Use time difference from rising edge to get distance
		if ((main_timer_now_isr() - previous_ultrasonic_read_t) > 7){
			//To account for signal instability (D2 might be wrongly low immediately after triggerring)
			calculate_distance();
		}
	}
	//Setting previous read during both rising and falling edges
	previous_ultrasonic_read_t = main_timer_now_isr();
}


void trigger_ultrasonic() {
	//Triggerring the ultrasonic sensor is done by giving a pulse of at least 10us
	//	at the TRIG pin of the ultrasonic sensor

	//Pulling trigger pin high
	PORTC |= (1 << 3);
	//waiting for 32us (256 prescaler & 8MHz clk -> 1 count of the clock)
	previous_ultrasonic_read_t = main_timer_now();
	while ((main_timer_now() - previous_ultrasonic_read_t) < 1) {}
	//Pulling trigger pin back low
	PORTC &= ~(1 << 3);

	previous_ultrasonic_read_t = main_timer_now();

	//Reception of the echo signal is done via interrupts
}


void sense_ultrasonic() {
	//Triggerring ultrasonic every 320 milliseconds after last read
	if (echo_pin_low()){
		//Only trigger if the previous echo has been received
		if ((main_timer_now() - previous_ultrasonic_read_t) >= 10000){
			trigger_ultrasonic();
		}
	}
}



#endif		//ULTRASONIC
