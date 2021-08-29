/*

*/



#include <avr/io.h>
#include <avr/interrupt.h>

#include "main_timer.h"
#include "lcd_i2c.h"



void main (){
	init_main_timer();
	lcd_init();

	lcd_print("Hello");
	lcd_second_line();
	lcd_print("World!");

	//3.2 seconds delay
	uint64_t prev_t = main_timer_now();
	while ((main_timer_now() - prev_t) < 100000);

	lcd_clear();
	lcd_print("AlvyneZ was");
	lcd_second_line();
	lcd_print("   Here!!");

	while (1){}
}
