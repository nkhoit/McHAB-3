#define F_CPU 8000000

#include <avr/io.h>
#include <avr/delay.h>

int main(void)	{
    DDRB |= (1 << PB3);

    for(;;) {
        PORTB |= (1 << PB3);
        _delay_ms(1000);
        PORTB = 0;
        _delay_ms(1000);
    }
    return 0;
}

