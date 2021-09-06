/*
	Header: Library for I2C or TWI interfacing on the ATmega32 in mater mode

	Pins to be used:
		SCL -> PC0 -> pin 22
		SDA -> PC1 -> pin 23

	The code works synchronously awaiting each transmission
*/



#ifndef I2C_TWI
//To prevent errors if the header file is included multiple times
#define I2C_TWI



#ifndef F_CPU		//To prevent defining F_CPU multiple times
//Defining internal RC oscillator clock frequency as 8MHz
#define F_CPU 8000000UL
#endif		//F_CPU





#define I2C_MT_STA_STATUS		0x08
#define I2C_MT_RP_STA_STATUS	0x10
#define I2C_MT_SLAW_ACK_STATUS	0x18
#define I2C_MT_SLAW_NACK_STATUS	0x20
#define I2C_MT_DATA_ACK_STATUS	0x28
#define I2C_MT_DATA_NACK_STATUS	0x30
#define I2C_MT_LST_ARB_STATUS	0x38
#define I2C_MR_SLAR_ACK_STATUS	0x40
#define I2C_MR_SLAR_NACK_STATUS	0x48
#define I2C_MR_DATA_ACK_STATUS	0x50
#define I2C_MR_DATA_NACK_STATUS	0x58


//Timeouts on given transmission points
#define START_TIMEOUT_COUNT		20
#define SLA_TIMEOUT_COUNT		10
#define SEND_TIMEOUT_COUNT		5


#include <avr/io.h>




//I2C Send Start condition. Returns 0 if successful, 1 if not successful
uint8_t i2c_start(){
	//Start condition: TWCR -> 1X10X10X
	uint8_t ctrl_reg = TWCR;
	ctrl_reg &= ~(1 << TWSTO);
	ctrl_reg |= (1<<TWINT) | (1<<TWEN) | (1<<TWSTA);
	TWCR = ctrl_reg;

	//Check for transmission completed
	while(!(TWCR & (1<<TWINT)));

	//Reading status register and Setting prescaler bits to 0
	uint8_t twi_status_reg = TWSR & 0xF8;

	if ((twi_status_reg == I2C_MT_STA_STATUS) || (twi_status_reg == I2C_MT_RP_STA_STATUS))
		return 0;		//Successful
	else
		return 1;		//Unsuccessful
}


//Repeatedly send Start condition until successful or timed out
uint8_t i2c_wait_start(){
	uint8_t retries = 0;

	//Wait for a successful start condition to be sent
	while (i2c_start()){
		if (retries > START_TIMEOUT_COUNT)
			return 1;		//Timed out
		retries ++;
	}
	return 0;				//Successful
}



//I2C write to bus
uint8_t i2c_write(uint8_t data){
	TWDR = data;								//Move value to I2C data register
	uint8_t ctrl_reg = TWCR;
	ctrl_reg &= ~((1 << TWSTA) | (1 << TWSTO));	//Disable start and stop conditions
	ctrl_reg |= (1 << TWINT) | (1 << TWEN);		//Enable I2C and clear interrupt
	TWCR = ctrl_reg;

	//Wait for transmission completed
	while(!(TWCR & (1<<TWINT)));

	//Reading status register and Setting prescaler bits to 0
	uint8_t twi_status_reg = TWSR & 0xF8;

	if ((twi_status_reg == I2C_MT_SLAW_ACK_STATUS) ||
			(twi_status_reg == I2C_MT_DATA_ACK_STATUS) ||
			(twi_status_reg == I2C_MR_SLAR_ACK_STATUS))
		return 0;		//Successful
	else if (twi_status_reg == I2C_MT_LST_ARB_STATUS)
		return 2;		//Lost Arbitration (Need to resend whole "packet")
	else
		return 1;		//Unsuccessful
}



//I2C stop condition
static inline void i2c_stop(){
	//Stop condition: TWCR -> 1X01X10X
	uint8_t ctrl_reg = TWCR;
	ctrl_reg &= ~(1 << TWSTA);
	ctrl_reg |= (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	TWCR = ctrl_reg;

	//Wait for stop condition complete
	while(TWCR & (1<<TWSTO));
}


uint8_t twi_send (uint8_t slave_address, uint8_t * to_send, uint8_t count) {

	//To keep count of retries to reach a device
	uint8_t retries = 0;

	while (retries < SLA_TIMEOUT_COUNT){
		//Temporary storage byte
		uint8_t temp;

		temp = i2c_wait_start();
		if (temp) return temp;

		//The slave address is 7 bits and is followed by a R/W bit (0 for write)
		slave_address = (slave_address << 1);
		//Sending SLA+W to i2c bus
		temp = i2c_write(slave_address);

		//Stopping if sending address was unsuccessful (NACK or Lost Arbitration)
		if (temp){
			i2c_stop();
			retries ++;
			continue;
		}
	
		//Writing data to the I2C bus
		uint8_t i, byte_retries = 0;
		for (i = 0; ((i < count) && (byte_retries < SEND_TIMEOUT_COUNT)); i++){
			temp = i2c_write(to_send[i]);
			if (temp == 2){	//Arbitration lost
				i2c_stop();
				retries ++;
				break;
			}
			else if (temp){
				i --;
				byte_retries ++;
			}
		}
		if (i != count) continue;

		i2c_stop();
		return 0;		//Successfully transmitted data to slave
	}
	return 1;			//Could not send entire message to slave
}




//I2C read from bus
uint8_t i2c_read (uint8_t last){
	uint8_t ctrl_reg = TWCR;
	ctrl_reg &= ~((1 << TWSTA) | (1 << TWSTO));		//Disable start and stop conditions
	if (last)
		ctrl_reg &= ~(1 << TWEA);					//Set send NACK (so slave stops sending)
	else
		ctrl_reg |= (1 << TWEA);					//Set send ACK (so slave continues sending)
	ctrl_reg |= (1 << TWINT) | (1 << TWEN);			//Enable I2C and clear interrupt
	TWCR = ctrl_reg;

	//Returning received data
	while(!(TWCR & (1<<TWINT)));

	return TWDR;
}



uint8_t twi_receive (uint8_t slave_address, uint8_t * received, uint8_t count) {

	if (count == 0) return 1;

	//To keep count of retries to reach a device
	uint8_t retries = 0;

	while (retries < SLA_TIMEOUT_COUNT){
		//Temporary storage byte
		uint8_t temp;

		temp = i2c_wait_start();
		if (temp) return temp;

		//The slave address is 7 bits and is followed by a R/W bit (1 for read)
		slave_address = (slave_address << 1) | 0x01;
		//Sending SLA+W to i2c bus
		temp = i2c_write(slave_address);

		//Stopping if sending address was unsuccessful (NACK or Lost Arbitration)
		if (temp){
			i2c_stop();
			retries ++;
			continue;
		}

		//Reading data from the bus
		uint8_t i;
		count --;
		for (i = 0; i < count; i++){
			received[i] = i2c_read(0);
		}
		received[i] = i2c_read(1);
		i2c_stop();

		return 0;		//Successsfully received data
	}

	return 1;			//Slave couldn't be reached
}


void twi_init(int F_TWI_kHz){
	//Initializing a SCL frequency of F_TWI_kHz (max 400 kHz)
	if (F_TWI_kHz > 400) F_TWI_kHz = 400;

	//Baud rate and prescaler iare set by calculating
	//F_TWI_kHz = F_CPU / (16 + 2(TWBR).(4^Prescaler))
	TWBR = ((F_CPU / (long)(1000L * F_TWI_kHz)) - 16) / 2;
	TWSR &= ~((1 << TWPS0) | (1 << TWPS1));		//Prescaler set to 0 (TWPS = 0)
}



#endif		//I2C_TWI
