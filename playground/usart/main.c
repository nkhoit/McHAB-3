#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

/* Initializes the USART (RS232 interface) */
 
void USART_init( unsigned int ubrr )
{
UBRR0H = (unsigned char)(ubrr>>8);
UBRR0L = (unsigned char)ubrr;
UCSR0B = (1 << TXEN0);     // Enable RX, TX & RX interrupt
UCSR0C = (3 << UCSZ00);    //asynchronous 8 N 1
}

/* Send some data to the serial port */

void USART_tx_string( char *data )
{
while ((*data != '\0'))
   {
      while (!(UCSR0A & (1 <<UDRE0)));
      UDR0 = *data;
      data++;
   }   
}

/*  MAIN */

int main(void)
{

USART_init(51);

_delay_ms(250);
USART_tx_string("Connected!\r\n");
_delay_ms(250);

while (1)                 // do until finished or broken
   {
      USART_tx_string("U");   //show me the test
      _delay_ms(250);         // wait .25 seconds
      USART_tx_string("T");
      _delay_ms(250);         // wait .25 seconds
   }
}
