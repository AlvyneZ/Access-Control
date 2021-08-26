/*
	Header: Library for getting 4-digit passcode input from the 4x4 keypad using an ATmega32a

	Pins to be used:
		Column pins:	Column 1..4	-> pin C4..7 (pins 26..29)
		Row pins:		Row 1..4	-> pin D4..7 (pins 18..21)

	The "validate_password()" and "detected_new_press()" functions will need to be
	  defined in the code that includes the keypad header file
	The functions can then access the "typed_password" array
*/



#ifndef KEYPAD
//To prevent errors if the header file is included multiple times
#define KEYPAD


#include <avr/io.h>		//Required for Data Direction Registers (DDR)

//The keypad depends on the main timer for its timing functions
#include "main_timer.h"


//The physical keys in order on the 4x4 keypad
/*const char KEYPAD_KEYS[4][4] = {
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}
	};*/
const char KEYPAD_KEYS[4][4] = {
		{'7','8','9','A'},
		{'4','5','6','B'},
		{'1','2','3','C'},
		{'*','0','#','D'}
	};
//The time taken for the system to forget what was typed if no key is pressed
const uint64_t clear_pass_interval = 156250UL;	//5 seconds (@ 8MHz & 256 prescaler)


//To hold the time of the last key press
volatile uint64_t previous_key_reg_t = 0;


//To hold the password typed in by the user
char typed_password[4] = {0,0,0,0};

//To hold the location in typed_password to input the next keyed in character
volatile uint8_t pass_offset = 0;

//To hold the pressed buttons in a certain period for detecting simultaneous presses
char pressed_buttons[4] = {0,0,0,0};
//To hold the number of detected presses
volatile uint8_t pressed_count = 0;




//Prototype definition of function that runs when full password is entered
void validate_password();
//Prototype definition of function that runs when another valid key press is detected
void detected_new_press();



void init_keypad() {
	//Setting Up row pins to be in TRISTATE (INPUT)
	DDRD &= 0x0F;		//DDR for pins D4..7 set to 0
	PORTD &= 0x0F;		//Setting output to 0
	//Setting Up column pins to be INPUTS
	DDRC &= 0x0F;		//DDR for pins C4..7 set to 0
	//Setting pull up resistors for column pins
	PORTC |= 0xF0;		//PORT for pins C4..7 set to 1
}


uint8_t detect_press() {
	//Pressed_buttons holds the pressed buttons that have been detected
	//Pressed_count shows how many have been detected since the last release of keys

	//Boolean for showing if a press has been detected
	uint8_t detected = 0;

	//Detecting Algorithm:
	//Rows need to be flipped LOW one at a time from top to bottom
	//At each moment, the columns are checked if they're still HIGH
	//Columns are low only if a button is pressed and shorts that column with the LOW row
	uint8_t row = 4;
	uint8_t column = 4;
	char key;
	while (row < 8){
		//Setting current row to output and pulling it LOW
		DDRD |= (1 << row);
		PORTD &= ~(1 << row);

		//Setting column to 0 for each row that is set to low
		column = 4;
		//Checking state of each column
		while (column < 8){
			if ( (PINC & (1 << column)) == 0 ) {
				//If pressed_buttons is full, just return that a button is still being pressed
				if (pressed_count >= 4){
					DDRD &= ~(1 << row);	//Setting row back to TRISTATE
					return 1;
				}
				//Only add key to pressed_buttons if it is not there
				key = KEYPAD_KEYS[row-4][column-4];
				if ((pressed_buttons[0] != key) && (pressed_buttons[1] != key) &&
						(pressed_buttons[2] != key) && (pressed_buttons[3] != key)){
					pressed_buttons[pressed_count] = key;
					pressed_count ++;
				}
				//Record that a key is being pressed whether or not it was originally in pressed_buttons
				detected = 1;
			}
			//Incrementing column
			column ++;
		}

		//Setting the row back to TRISTATE (INPUT)
		DDRD &= ~(1 << row);
		//Incrementing row
		row ++;
	}
	return detected;
}


void clear_pressed() {
	pressed_count = 0;
	pressed_buttons[0] = 0;
	pressed_buttons[1] = 0;
	pressed_buttons[2] = 0;
	pressed_buttons[3] = 0;
}


void clear_password() {
	pass_offset = 0;
	typed_password[0] = 0;
	typed_password[1] = 0;
	typed_password[2] = 0;
	typed_password[3] = 0;
}




void add_char_to_pass(char k) {
	if ((main_timer_now() - previous_key_reg_t) <= 3125){
		//The next key press should be at least 100ms after the last key press
		return;
	}

	//Save the character into the password variable
	typed_password[pass_offset] = k;
	//Increment the offset value
	pass_offset ++;

	//If the password variable is full
	if (pass_offset >= 4){
		validate_password();
		clear_password();
	}
	else{
		detected_new_press();
	}

	//Record the time as the last time a valid key press was registered
	previous_key_reg_t = main_timer_now();
}


void check_keypad(){
	//This is the function to be run in main() in the while(1) loop
	//	in order to poll the keypad for input

	//Register the press after the key is released
	//Detect_press() will return 0 if no key is being pressed
	uint8_t detected = detect_press();
	if ((detected == 0) && (pressed_count > 0)){
		if (pressed_count == 1){
			add_char_to_pass(pressed_buttons[0]);
		}
		else{
			//TODO: What to do when multiple keys are pressed simultaneously
			//Possibly enter admin mode
			add_char_to_pass('Z');
		}
		//Clear the pressed_buttons variable for the next period
		clear_pressed();

		previous_key_reg_t = main_timer_now();
	}
	

	//Forget previously typed password if the time limit has elapsed
	if ((pass_offset > 0) && ((main_timer_now() - previous_key_reg_t) >= clear_pass_interval)){
		clear_password();
	}
}





#endif		//KEYPAD
