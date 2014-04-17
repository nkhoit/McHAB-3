/*********************************************
Long Wavelength Lab - Jun/2013

This code:
- Receives data from an external source over UART (RPi sending commands
	based on PID loops)
- Uses data to set duty cycles on two timers, which control two DC motors
	connected to a L298 motor driver (used for RCS)

Purpose of code is to maintain attitude of microwave calibrator source
payload on a high altitude balloon.

--
Code written for ATmega328P but should be compatible with most
AVR series microcontrollers (minor modifications may be required
to register names - consult the datasheet)
--

Project supervised by Professor Keith Vanderlinde
*********************************************/

#define F_CPU 8000000
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

//using magic numbers to define baud rate here (overriding above definition)
//temporary solution, not recommended (value based on http://www.wormfood.net/avrbaudcalc.php)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/math.h>
#include <avr/stdlib.h>

void main( void )
{
	DDRB |= (1 << PB3);
	USART_Init(51); //initialize USART
	PWM_Init();	//initialize PWM
	sei(); //enable global interrupts

	while (1)	{
		USART_Transmit(':');
		_delay_ms(500);
		USART_Transmit(')');
		_delay_ms(500);
	}
}

//initializing the USART
void USART_Init( unsigned int ubrr)
{
	//Set baud rate
	UBRR0H = (unsigned char) (ubrr >> 8);
	UBRR0L = (unsigned char) ubrr;

	//Enable receiver and transmitter
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);

	//Set frame format: 8data, 2stop bit
	UCSR0C = (3 << UCSZ00);

	//Enable receive complete interrupt service
	UCSR0B |= (1 << RXCIE0);
}

void USART_tx_string( char *data )
{
while ((*data != '\0'))
   {
      while (!(UCSR0A & (1 <<UDRE0)));
      UDR0 = *data;
      data++;
   }   
}

//uart data tx function
void USART_Transmit( unsigned char data )
{
	//wait for empty transmit buffer
	while ( !( UCSR0A & (1 << UDRE0)) );

	//put data into input buffer
	UDR0 = data;
}

//uart data rx interrupt
ISR(USART_RX_vect)	{ 
	
	//read value into variable
	char duty_cycle = UDR0;
	
	//set duty cycle based on returned value
	PWM_dutycycle_set(duty_cycle);

	//echo it back
	//USART_Transmit(duty_cycle);

   PORTB |= (1 << PB3);
}

//note: using fast pwm mode, configured only for one output
//bits being set to 0 redundantly for extra clarity
//this is an 8 bit timer
void PWM_Init()	{

	//overriding normal port functionality to enable timer/pwm output
	//this is for fast pwm mode, and setting it up for non-inverting mode
	TCCR0A |= (0 << COM0A0) | (1 << COM0A1);

	//setting waveform generation mode to fast pwm
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (0 << WGM02);

	//setting up clock select bits to use no prescaling, and the default
	//cpu clock source
	TCCR0B |= (1 << CS00) | (0 << CS01) | (0 << CS02);

	//setting pin to output
	DDRD|=(1<<PD6);

	//setting temporary duty cycle to about 50% 
	//8 bit timer will allow values from 0 - 255
	OCR0A = 0;
}

//function to set the duty cycle
void PWM_dutycycle_set(uint8_t value)	{
	OCR0A = value * 2;
}
