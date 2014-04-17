/*********************************************
Long Wavelength Lab - Jun/2013

This code:
- Receives data from a TWI/i2C device, and prints values
	to a serial terminal over its uart

--
Code written for ATmega328 but should be compatible with most
AVR series microcontrollers (minor modifications may be required
to register names - consult the datasheet)
--

Project supervised by Professor Keith Vanderlinde
*********************************************/

#define F_CPU 8000000

#define START			0x08	//A START condition has been transmitted 
#define REP_START		0x10	//A repeated START condition has been transmitted 
#define MR_SLA_ACK  	0x40	//SLA+R has been transmitted; ACK has been received
#define MR_SLA_NACK	0x48	//SLA+R has been transmitted; NOT ACK has been received
#define MR_DATA_ACK	0x50	//Data byte has been received; ACK has been returned 
#define MR_DATA_NACK	0x58	//Data byte has been received; NOT ACK has been returned 

#define PRESC_MASK	0xF8	//Mask for TWSR for prescale bits

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/math.h>
#include <avr/stdlib.h>

void TWI_init_master(void) // Function to initialize master
{
	//Bit rate ... what is the impact of this setting? controls the
	//period of SCL, so what? Does that affect speed? I'd assume so.
	TWBR = 0x01;
	
   //Setting prescalar bits
	//SCL freq= F_CPU/(16+2(TWBR).4^TWPS)
	TWSR = (0 << TWPS1) | (0 << TWPS0);
}

void TWI_start(void)
{
	//Clear TWI interrupt flag, put start condition on SDA, enable TWI
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	
	//Wait until start condition is transmitted (when TWINT is set)
	while(!(TWCR & (1 << TWINT)));

	//Check for the acknowledgement (0xF8 masks prescaler bits)
	//May be worth having an LED indicate if this fails	
	while((TWSR & PRESC_MASK) != START);
}

void TWI_read_address(unsigned char data)
{
   //Address and read instruction (write = 0, read = 1)
   //First 7 bits are slave address, last one R/W (^^)
	TWAR = data;
	
	//Clear TWI interrupt flag and enable TWI to start transmission of address info
	TWCR=(1<<TWINT)|(1<<TWEN);
	
	//Wait until data is transmitted (when TWINT is set)
	while (!(TWCR & (1<<TWINT)));
	
	//Check for the acknowledgement (0xF8 masks prescaler bits)
	while((TWSR & PRESC_MASK) != MR_SLA_ACK );
}

void TWI_read_data(void)
{
	TWCR=(1<<TWINT)|(1<<TWEN);    // Clear TWI interrupt flag,Enable TWI
	while (!(TWCR & (1<<TWINT))); // Wait till complete TWDR byte transmitted
	while((TWSR & PRESC_MASK) != 0x58); // Check for the acknoledgement
	recv_data=TWDR;
	PORTB=recv_data;
}


