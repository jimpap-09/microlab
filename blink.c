#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

int main() {
	DDRD = 0xff;
	while(1) {
		PORTD = 0xff;
		_delay_ms(1000);
		PORTD = 0x00;
		_delay_ms(1000);
	}
	
}
