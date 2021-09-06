/*
	Header: Holds the functions required for the MAIN_TIMER

		The chosen timer for the main_timer is timer1
		It is a 16-bit timer
		For an 8MHz CPU frequency a 256 prescaler is used so that the counter overflows every 2 sec
*/


#ifndef MAIN_TIMER
//To prevent errors if the header file is included multiple times
#define MAIN_TIMER

#ifndef F_CPU		//To prevent defining F_CPU multiple times
//Defining internal RC oscillator clock frequency as 8MHz
#define F_CPU 8000000UL
#endif		//F_CPU


#include <avr/io.h>		//Required for Timer registers
#include <avr/interrupt.h>		//Required for Interrupt Service Routine (ISR)


//Defining prescaler to use for the timer
#define MAIN_TIMER_PRESCALER 256
/*
	Note: Timer1 is designed to be the clock for measuring the whole system's
		time so a prescaler of 256 is used so that it overflows every 2 seconds
		for an 8MHz driven system since timer1 is 16bit (max = 65535)
*/


//To hold the total count of the main timer
volatile uint64_t main_timer_t = 0;



void init_main_timer(){
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

	TIMSK |= (1 << TOIE1);	//Enabling interrupts for timer1

	sei();		//Enabling global interrupts
}



uint64_t main_timer_now() {
	//Save global interrupts flag
	uint8_t sreg = SREG;
	//Disable global interrupts
	cli();
	//Reading the TCNT1 16-bit register
	uint64_t to_return = TCNT1;
	to_return += main_timer_t;
	//Restore global interrupt flag
	SREG = sreg;
	return to_return;
}

uint64_t main_timer_now_isr() {
	//Reading the TCNT1 16-bit register
	uint64_t to_return = TCNT1;
	to_return += main_timer_t;
	return to_return;
}


void main_timer_clear() {
	//Shouldn't be called in the main loop

	//Save global interrupts flag
	uint8_t sreg = SREG;
	//Disable global interrupts
	cli();
	//Clearing TCNT1 and main_timer_t
	TCNT1 = 0;
	main_timer_t = 0;
	//Restore global interrupt flag
	SREG = sreg;
}




ISR(TIMER1_OVF_vect){
	//Since the main timer (timer 1) is a 16 bit timer
	main_timer_t += (1UL << 16);
}




#endif		//MAIN_TIMER
