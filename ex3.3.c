#define F_CPU 16000000UL
#define F_PWM 62500

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>


int main(void) {
	
	unsigned int OCR1A_table[14] = {5, 25, 45, 65, 86, 107, 128,
	149, 170, 191, 212, 223, 234, 250};
	unsigned char ptr = 6;
	unsigned char mode = 0;
	DDRB = (1<<PINB1);
	DDRC = (0<<PINC1) | (0<<PINC0);
	DDRD = (0<<PIND1) | (0<<PIND2) | (0<<PIND6) | (0<<PIND7);
	PORTD = (1<<PIND1) | (1<<PIND2) | (1<<PIND6) | (1<<PIND7);

	TCCR1A = (1<<WGM10) | (1<<COM1A1);
	TCCR1B = (1<<WGM12) | (1<<CS10);

	OCR1AL = OCR1A_table[ptr];

	ADMUX = 0b01000001;
	ADCSRA = 0b10000111;
	//  ADCSRB = 0x00; => free running => delay = 25 + 13k cycles, k>=0

	while(1) {
		// start conversion
		ADCSRA|=(1<<ADSC);
		while(ADCSRA & (1<<ADSC));
		
		// mode 1
		if (!(PIND & (1<<PIND6))) {
			while(!(PIND & (1<<PIND6))) _delay_ms(10);
			ADMUX = 0b01000001;
			mode = 0;
			ADCSRA |= (1<<ADSC);
			while(ADCSRA & (1<<ADSC));
		}
		
		// mode 2
		if (!(PIND & (1<<PIND7))) {
			while(!(PIND & (1<<PIND7))) _delay_ms(10);
			ADMUX = 0b01000000;
			mode = 1;
			ADCSRA |= (1<<ADSC);
			while(ADCSRA & (1<<ADSC));
		}
		
		if(mode){
			OCR1AL = ADC/5.0;
		}
		else {
			// PD1 => DC +
			if (!(PIND & (1<<PIND1))) {
				while(!(PIND & (1<<PIND1))) _delay_ms(10);
				if(ptr != 13) OCR1AL = OCR1A_table[++ptr];
			}
			
			// PD2 => PD -
			if (!(PIND & (1<<PIND2))) {
				while(!(PIND & (1<<PIND2))) _delay_ms(10);
				if(ptr != 0) OCR1AL = OCR1A_table[--ptr];
			}
		}
	}
	return 0;
}
