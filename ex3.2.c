#define F_CPU 16000000UL
#define F_PWM 62500

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

// PC1 -> ΑDC1 -> PWM_B1
int main(void) {
	// ADC values
	unsigned int ADC_values[16] = {0};

	// DC values
	unsigned int OCR1A_table[15] = {0, 5, 30, 46, 66, 87, 107, 128,
	148, 168, 189, 209, 230, 250, 255};
	unsigned char ptr = 7;
	int idx = 0;
	int ADCvalue = 0;
	
	// PB1, PORTD -> output, PB2, PB3 -> input, pull up resistors
	// PC1 -> input
	DDRB = (1<<PINB1) | (0<<PINB2) | (0<<PINB3);
	PORTB = (1<<PINB2) | (1<<PINB3);
	DDRC = (0<<PINC1);
	DDRD = 0xFF;

	// init PWM
	// fast 8-bit pwm, prescaler = 1, OC1A -> PB1 -> pwm_output
	TCCR1A = (1<<WGM10) | (1<<COM1A1);
	TCCR1B = (1<<WGM12) | (1<<CS10);
	OCR1AL = OCR1A_table[ptr];
	
	// REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0001 => select ADC1(pin PC1),
	// ADLAR=0 => Right adjust the ADC result
	ADMUX = 0b01000001;
	// ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
	// ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
	ADCSRA = 0b10000111;
	//  ADCSRB = 0x00; => free running => delay = 25 + 13k cycles, k>=0

	while(1) {
		ADCSRA|=(1<<ADSC);
		while(ADCSRA & (1<<ADSC));
		ADC_values[idx++]=ADC;
		
		// PB3 pressed -> DC +
		if (!(PINB & (1<<PINB3))) {
			while(!(PINB & (1<<PINB3))) _delay_ms(10);
			if(ptr < 13) OCR1AL = OCR1A_table[++ptr];
		}
		
		// PB2 pressed -> DC -
		if (!(PINB & (1<<PINB2))) {
			while(!(PINB & (1<<PINB2))) _delay_ms(10);
			if(ptr > 0) OCR1AL = OCR1A_table[--ptr];
		}
		
		// after 16 conversions
		// calculate ADCvalue
		// PINDx ON according to ADCvalue
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
		
		// 100 ms delay between conversions
		_delay_ms(100);
	}
	return 0;
}
