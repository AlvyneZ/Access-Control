/*
	Program: Ultrasonic sensor test code using ATmega328p

	Pins to Use:
		For the ultrasonic sensor:
			TRIG -> Pin D5			-> pin11
			ECHO -> Pin D2(INT0)	-> pin4
		For the output LEDs:
			Under 100cm -> Pin B0 -> pin14
			Under 50cm  -> Pin B1 -> pin15
			Under 15cm  -> Pin B2 -> pin16
*/


//Defining internal RC oscillator clock frequency as 8MHz
#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>


//Defining prescaler to use for the timer
#define MAIN_TIMER_PRESCALER 256
/*
	Note: Timer1 is designed to be the clock for measuring the whole system's
		time so a prescaler of 256 is used so that it overflows every 2 seconds
		for an 8MHz driven system since timer1 is 16bit (max = 65535)
*/

//To hold the total count of the main timer
volatile uint64_t main_timer_t = 0;


//To hold the time of the last read of the ultrasonic sensor
volatile uint64_t previous_ultrasonic_read_t = 0;


void init_main_timer(){
	//Note: global interrupts will need to be enabled using sei()
	TCCR1A = 0;
	TCCR1B &= ~(1 << WGM13)& ~(1 << WGM12);
	if(MAIN_TIMER_PRESCALER == 1){//Setting CS12 to CS10 = 001 (prescaling of 1)
		TCCR1B &= ~(1<<2);
		TCCR1B &= ~(1<<1);
		TCCR1B |= (1<<0);
	}
	else if(MAIN_TIMER_PRESCALER == 8){//Setting CS12 to CS10 = 010 (prescaling of 8)
		TCCR1B &= ~(1<<2);
		TCCR1B |= (1<<1);
		TCCR1B &= ~(1<<0);
	}
	else if(MAIN_TIMER_PRESCALER == 64){//Setting CS12 to CS10 = 011 (prescaling of 64)
		TCCR1B &= ~(1<<2);
		TCCR1B |= (1<<1);
		TCCR1B |= (1<<0);
	}
	else if(MAIN_TIMER_PRESCALER == 256){//Setting CS12 to CS10 = 100 (prescaling of 256)
		TCCR1B |= (1<<2);
		TCCR1B &= ~(1<<1);
		TCCR1B &= ~(1<<0);
	}
	else if(MAIN_TIMER_PRESCALER == 1024){//Setting CS12 to CS10 = 101 (prescaling of 1024)
		TCCR1B |= (1<<2);
		TCCR1B &= ~(1<<1);
		TCCR1B |= (1<<0);
	}

	TCNT1 = 0;					//Initializing the counter to 0
	main_timer_t = 0;	//Initializing the timer's total count to 0

	TIMSK1 |= (1 << TOIE1);	//Enabling interrupts for timer1
}


uint16_t main_timer_count() {
	//In order to get a stable reading of TCNT1
	uint16_t counts[5] = {0,1,2,3,4};
	uint8_t read = 0;
	while ((counts[0] != counts[1]) || (counts[1] != counts[2]) ||
			(counts[2] != counts[3]) || (counts[3] != counts[4])){
		counts[read] = TCNT1;
		read ++;
		if (read >= 5) read = 0;
	}
	return counts[0];
}

uint64_t main_timer_now() {
	return main_timer_t + main_timer_count();
}


ISR(TIMER1_OVF_vect){
	//Since the main timer (timer 1) is a 16 bit timer
	main_timer_t += (1UL << 16);
}



void init_output_leds() {
	//Setting pins B0 to B2 as outputs for leds that show distance
	DDRB |= 0b0111;
}

void output_to_leds(double distance) {
	if (distance < 100){
		PORTB |= (1 << 0); //Under 100cm
		
		if (distance < 50){
			PORTB |= (1 << 1); //Under 50cm
			
			if (distance < 15)
				PORTB |= (1 << 2); //Under 15cm
			else
				PORTB &= ~(1 << 2); //Over 15cm
		}
		else{
			PORTB &= ~(1 << 1); //Over 50cm
			PORTB &= ~(1 << 2); //Over 15cm
		}
	}
	else{
		PORTB &= ~(1 << 0); //Over 100cm
		PORTB &= ~(1 << 1); //Over 50cm
		PORTB &= ~(1 << 2); //Over 15cm
	}
}




void init_ultrasonic(){
	//Setting pin D5 to output
	DDRD |= (1 << 5);
	//Pulling pinD5 low (initial state)
	PORTD &= ~(1 << 5);
	//Setting pin D2 to input
	DDRD &= ~(1 << 2);
	//Setting pull down resistor on pin D2
	PORTD &= ~(1 << 2);
	//Enabling interrupts on pin D2 (INT0)
	EIMSK |= (1 << INT0);
	//Setting INT0 to trigger on both edges (logic change)
	EICRA &= ~(1 << ISC01);
	EICRA |= (1 << ISC00);
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

	//TODO: Do something useful with the distance here
	output_to_leds(distance);
}


ISR(INT0_vect){
	if (!(PORTD & (1 << 2))){
		//If pin D2 is low (ie after a falling edge)
		//Falling edge -> Use time difference from rising edge to get distance
		if ((main_timer_now() - previous_ultrasonic_read_t) > 7){
			//To account for signal instability (D2 might be wrongly low immediately after triggerring)
			calculate_distance();
		}
	}
	//Setting previous read during both rising and falling edges
	previous_ultrasonic_read_t = main_timer_now();
}


void trigger_ultrasonic() {
	//Triggerring the ultrasonic sensor is done by giving a pulse of at least 10us
	//	at the TRIG pin of the ultrasonic sensor

	//Pulling pinD5 high
	PORTD |= (1 << 5);
	//waiting for 32us (256 prescaler & 8MHz clk -> 1 count of the clock)
	previous_ultrasonic_read_t = main_timer_now();
	while ((main_timer_now() - previous_ultrasonic_read_t) < 1) {}
	//Pulling pinD5 back low
	PORTD &= ~(1 << 5);

	//Reception of the echo signal is done via interrupts
}


int main () {
	//Enabling global interrupts
	sei();
	//Initializing main system timer
	init_main_timer();
	//Initializing ultrasonic sensor
	init_ultrasonic();

	//Setting up the output
	init_output_leds();

	//Main loop
	while (1){
		//Triggerring ultrasonic every 500 milliseconds after last read
		if (!(PORTD & (1 << 2))){
			//Only trigger if the previous echo has been received
			if ((main_timer_now() - previous_ultrasonic_read_t) >= 15625){
				trigger_ultrasonic();
			}
		}
	}
	//End main loop
}
