/*
	Header: Library for I2C or TWI interfacing on the ATmega32 in mater mode

	Pins to be used:
		SCL -> PC0 -> pin 22
		SDA -> PC1 -> pin 23

	Note: The receive part of this code is untested and is commented out for now
*/



#ifndef I2C_TWI
//To prevent errors if the header file is included multiple times
#define I2C_TWI



#ifndef F_CPU		//To prevent defining F_CPU multiple times
//Defining internal RC oscillator clock frequency as 8MHz
#define F_CPU 8000000UL
#endif		//F_CPU




#include <avr/io.h>



//To hold data to be sent on the twi interface
volatile uint8_t twi_out_buffer[64];
//To hold the next location of twi_out_buffer to write to when adding data to send
volatile uint8_t twi_out_add_offset = 0;
//To hold the next location of twi_out_buffer to read from when sending
volatile uint8_t twi_out_send_offset = 0;
//To tell if a start condition should be sent before a certain character
volatile uint64_t twi_sta = 0ULL;
//To tell if a stop condition should be sent before a certain character
volatile uint64_t twi_sto = 0ULL;

/*
//To hold data received from the twi interface
volatile uint8_t twi_in_buffer[32];
//To hold the next location of twi_in_buffer to write to when receiving data
volatile uint8_t twi_in_offset = 0x00;
*/


//Increment twi_out_send_offset
void increment_twi_out_send_offset(){
	twi_out_send_offset++;
	if (twi_out_send_offset >= 64)
		twi_out_send_offset = 0;
}
//Increment twi_out_add_offset
void increment_twi_out_add_offset(){
	twi_out_add_offset++;
	if (twi_out_add_offset >= 64)
		twi_out_add_offset = 0;
}



//Start condition
static inline void i2c_start(){
	//Start condition: TWCR -> 1X10X10X
	TWCR &= ~(1 << TWSTO);
	TWCR |= (1<<TWINT) | (1<<TWEN) | (1<<TWSTA);
	//Check for start condition complete is done using interrupts
}


void twi_buffer_add_address(uint8_t slave_address){
	//Disable interrupts to prevent twi_sto from being changed by TWI_ISR
	//Save global interrupts flag and disable global interrupts
	uint8_t sreg = SREG;
	cli();

		//Placing the SLA+R/W in the twi_out_buffer so its sent after START
		twi_out_buffer[twi_out_add_offset] = slave_address;
		//Send START condition before SLA+R/W
		if ((twi_sto & (1ULL << twi_out_add_offset)) == 0){
			//Send Start condition
			i2c_start();
		}
		else{
			//Repeat START condition (no need to release the bus)
			twi_sta |= (1ULL << twi_out_add_offset);
			twi_sto |= (1ULL << twi_out_add_offset);
		}
		//Incrementing the buffer offset
		increment_twi_out_add_offset();

	//Restore global interrupt flag
	SREG = sreg;
}


void twi_send (uint8_t slave_address, uint8_t * to_send, uint8_t count) {
	//Note: the count should not be greater than 64 or the buffer will be overwritten

	//The slave address is 7 bits and is followed by a R/W bit (0 for write)
	slave_address = (slave_address << 1);
	//Adding start condition and SLA+W to twi_out_buffer
	twi_buffer_add_address(slave_address);

	//Adding data to be sent to twi_out_buffer
	uint8_t i;
	for (i = 0; i < count; i++){
		twi_out_buffer[twi_out_add_offset] = to_send[i];
		increment_twi_out_add_offset();
	}
	twi_sto |= (1ULL << twi_out_add_offset);
}


/*
void twi_receive (uint8_t slave_address, uint8_t count) {
	//Note: the count should not be greater than 32 or the buffer will be overwritten
	if (count > 32)
		count = 32;

	//Resetting input offset
	twi_in_offset = 0;

	//The slave address is 7 bits and is followed by a R/W (1 for read)
	slave_address = (slave_address << 1) | 0x01;
	//Adding start condition and SLA+R to twi_out_buffer
	twi_buffer_add_address(slave_address);

	//Adding count (of data to be received from slave) to twi_out_buffer
	twi_out_buffer[twi_out_add_offset] = count;
	increment_twi_out_add_offset();
	twi_sto |= (1ULL << twi_out_add_offset);
}


//Prototype of function to be run when data is received from a slave
void twi_data_received(uint8_t count);
*/


void twi_init(int F_TWI_kHz){
	//Initializing a SCL frequency of F_TWI_kHz (max 400 kHz)
	if (F_TWI_kHz > 400) F_TWI_kHz = 400;

	//Baud rate and prescaler iare set by calculating
	//F_TWI_kHz = F_CPU / (16 + 2(TWBR).(4^Prescaler))
	TWBR = (int)((F_CPU / (1000 * F_TWI_kHz)) - 16) / (int)(2 * 4);
	TWSR &= ~((1 << TWPS0) | (1 << TWPS1));		//Prescaler set to 1 (TWPS = 0)

	TWCR = (1<<TWEN);	//Enable I2C

	TWCR |= (1 << TWIE);	//Enable I2C interrupts
}



//I2C stop condition
static inline void i2c_stop(){
	//Stop condition: TWCR -> 1X01X10X
	TWCR &= ~(1 << TWSTA);
	TWCR |= (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	//Check for stop condition complete is done using interrupts
}


//I2C write to bus
static inline void i2c_write(){
	TWDR = twi_out_buffer[twi_out_send_offset];	//Move value to I2C data register
	TWCR &= ~((1 << TWSTA) | (1 << TWSTO));		//Disable start and stop conditions
	TWCR |= (1 << TWINT) | (1 << TWEN);			//Enable I2C and clear interrupt

	//Check for successful write is done using interrupts
}



//I2C Master Transmit mode after sending (This is called in the TWI_ISR)
void i2c_master_transmit_next() {
	//Check if a STOP condition should be sent before sending the current byte
	if (twi_sto & (1ULL << twi_out_send_offset)){
		twi_sto &= ~(1ULL << twi_out_send_offset); //Set stop condition to already sent
		i2c_stop();
	}
	//Check if a START condition should be sent before sending the current byte
	if (twi_sta & (1ULL << twi_out_send_offset)){
		twi_sta &= ~(1ULL << twi_out_send_offset); //Set start condition to already sent
		i2c_start();
	}
	//Send next byte and increment count if we're not yet at the end
	else if (twi_out_send_offset != twi_out_add_offset){
		i2c_write();
		increment_twi_out_send_offset();
	}
}


/*
//I2C read from bus
static inline void i2c_read(){
	TWCR &= ~((1 << TWSTA) | (1 << TWSTO));		//Disable start and stop conditions
	TWCR |= (1 << TWINT) | (1 << TWEN);			//Enable I2C and clear interrupt

	//Getting data from TWDR will be done using interrupts
}


//I2C Master Receive mode after receiving (This is called in the TWI_ISR)
void i2c_master_receive_next() {
	//Check if the data to be received has all been received
	if ((twi_in_offset + 1) >= twi_out_buffer[twi_out_send_offset]){
		twi_in_buffer[twi_in_offset] = TWDR;	//Save final received byte to buffer
		twi_data_received(twi_in_offset + 1);	//Return data
		//Let TWI hardware continue with master transmit mode
		increment_twi_out_send_offset();
		i2c_master_transmit_next();
	}
	//Receive the next byte otherwise
	else{
		twi_in_buffer[twi_in_offset] = TWDR;	//Save previously received byte to buffer
		twi_in_offset ++;
		i2c_read();
	}
}
*/



ISR(TWI_vect) {
	i2c_master_transmit_next();
/*
	//Reading staus register and Setting prescaler bits to 0
	uint8_t twi_status_reg = TWSR & 0xF8;
	switch (twi_status_reg){
	//Master mode START condition
		case 0x08:
			//Start condition has been transmitted
			i2c_master_transmit_next();
		break;
		case 0x10:
			//Repeated Start condition has been transmitted
			i2c_master_transmit_next();
		break;
	//Master Transmit mode Address transmit
		case 0x18:
			//SLA+W transmitted and ACK received
			i2c_master_transmit_next();
		break;
		case 0x20:
			//SLA+W transmitted and NOT ACK received
			i2c_master_transmit_next();
		break;
	//Master Transmit mode Data transmit
		case 0x28:
			//Data byte transmitted and ACK received
			i2c_master_transmit_next();
		break;
		case 0x30:
			//Data byte transmitted and NOT ACK received
			i2c_master_transmit_next();
		break;
	//Multi Master system - Arbitration lost at any point
		case 0x38:
			//Arbitration lost in SLA+W or data bytes
			//TODO
		break;
	//Master Receive mode Address transmit
		case 0x40:
			//SLA+R transmitted and ACK received
			increment_twi_out_send_offset();
			i2c_master_transmit_next();
		break;
		case 0x48:
			//SLA+R transmitted and NOT ACK received
			i2c_master_receive_next();
		break;
	//Master Receive mode Data transmit
		case 0x50:
			//Data byte received and ACK returned
			i2c_master_receive_next();
		break;
		case 0x58:
			//Data byte received and NOT ACK returned
			i2c_master_receive_next();
		break;
	}
*/
}




#endif		//I2C_TWI
