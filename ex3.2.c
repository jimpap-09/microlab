#define F_CPU 16000000UL
#define F_PWM 62500

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

int main(void) {
	unsigned int ADC_values[16] = {0};

	unsigned int OCR1A_table[14] = {5, 25, 45, 65, 86, 107, 128,
	149, 170, 191, 212, 223, 234, 250};
	unsigned char ptr = 6;
	int idx = 0;
	int ADCvalue = 0;
	
	DDRB = (1<<PINB1) | (0<<PINB2) | (0<<PINB3);
	PORTB = (1<<PINB2) | (1<<PINB3);
	DDRC = (0<<PINC1);
	DDRD = 0xFF;
	
	TCCR1A = (1<<WGM10) | (1<<COM1A1);
	TCCR1B = (1<<WGM12) | (1<<CS10);
	OCR1AL = OCR1A_table[ptr];
	
	ADMUX = 0b01000001;
	ADCSRA = 0b10000111;
	//  ADCSRB = 0x00; => free running => delay = 25 + 13k cycles, k>=0

	while(1) {
		ADCSRA|=(1<<ADSC);
		while(ADCSRA & (1<<ADSC));
		ADC_values[idx++]=ADC;
		
		if (!(PINB & (1<<PINB3))) {
			while(!(PINB & (1<<PINB3))) _delay_ms(10);
			if(ptr != 13) OCR1AL = OCR1A_table[++ptr];
		}
		if (!(PINB & (1<<PINB2))) {
			while(!(PINB & (1<<PINB2))) _delay_ms(10);
			if(ptr != 0) OCR1AL = OCR1A_table[--ptr];
		}
		
		if(idx == 15) {
			for(int i=0; i<16; i++)
			ADCvalue += ADC_values[i];
			ADCvalue = ADCvalue>>4;
			idx = 0;
			
			if(ADCvalue < 200) PORTD = (1<<PIND0);
			else if (ADCvalue < 400) PORTD = (1<<PIND1);
			else if (ADCvalue < 600) PORTD = (1<<PIND2);
			else if (ADCvalue < 800) PORTD = (1<<PIND3);
			else PORTD = (1<<PIND4);
		}
		
		_delay_ms(100);
	}
	return 0;
}
