#include <avr/io.h>
#include <avr/delay.h>

int main()
{
	DDRB = 0xFF;
	PORTB = 1<<5; //Pin 13
	
	while (1)
	{
		PORTB ^= (1<<5);
		_delay_ms(1000);
	}
	
	return 1;
}