#define F_CPU 16000000UL
#define F_PWM 62500

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>


int main(void) {
	
	// DC values
	unsigned int OCR1A_table[15] = {0, 5, 30, 46, 66, 87, 107, 128,
	148, 168, 189, 209, 230, 250, 255};
	unsigned char ptr = 7;
	unsigned char mode = 0;
	
	// PB1, PD1, PD2, PD6, PD7 -> output, PC0 -> input
	DDRB = (1<<PINB1);
	DDRC = (0<<PINC0);
	DDRD = (0<<PIND1) | (0<<PIND2) | (0<<PIND6) | (0<<PIND7);
	PORTD = (1<<PIND1) | (1<<PIND2) | (1<<PIND6) | (1<<PIND7);

	// init PWM
	// fast 8-bit pwm, prescaler = 1, OC1A -> PB1 -> pwm_output
	TCCR1A = (1<<WGM10) | (1<<COM1A1);
	TCCR1B = (1<<WGM12) | (1<<CS10);

	OCR1AL = OCR1A_table[ptr];

	// REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0000 => select ADC0(pin PC0),
	// ADLAR=0 => Right adjust the ADC result
	ADMUX = 0b01000000;
	
	// ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
	// ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
	ADCSRA = 0b10000111;
	//  ADCSRB = 0x00; => free running => delay = 25 + 13k cycles, k>=0

	while(1) {
		// start conversion
		ADCSRA|=(1<<ADSC);
		while(ADCSRA & (1<<ADSC));
		
		// mode 1
		if (!(PIND & (1<<PIND6))) {
			while(!(PIND & (1<<PIND6))) _delay_ms(10);
			mode = 0;
		}
		
		// mode 2
		// PC0 -> ADC0 -> POT1
		if (!(PIND & (1<<PIND7))) {
			while(!(PIND & (1<<PIND7))) _delay_ms(10);
			mode = 1;
		}
		// if mode = 0
		if(mode){
			// DC = ADCvalue from POT1
			OCR1AL = ADC/5.0;
		}
		// if mode = 1
		else {
			OCR1AL = OCR1A_table[ptr];
			// PD1 => DC +
			if (!(PIND & (1<<PIND1))) {
				while(!(PIND & (1<<PIND1))) _delay_ms(10);
				if(ptr != 13) OCR1AL = OCR1A_table[++ptr];
			}
			
			// PD2 => DC -
			if (!(PIND & (1<<PIND2))) {
				while(!(PIND & (1<<PIND2))) _delay_ms(10);
				if(ptr != 0) OCR1AL = OCR1A_table[--ptr];
			}
		}
	}
	return 0;
}
