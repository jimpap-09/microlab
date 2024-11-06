#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

#define LCD_PORT PORTD							// lcd port
#define LCD_RS 2
#define LCD_E  3
#define LCD_DB4 4
#define LCD_DB5 5
#define LCD_DB6 6
#define LCD_DB7 7

#define CO_THRESHOLD 70							// CO upper limit
#define ADC_CHANNEL 2							// input ADC channel
#define SENS 0.129							// sensitivity

const char gas_detected_msg[] = "GAS DETECTED";				// messages
const char clear_msg[] = "CLEAR";

uint16_t check;								// check if ppm is CO > 70

void enable_pulse() {							// enable pulse
	LCD_PORT |= (1<<LCD_E);
	_delay_us(1);
	LCD_PORT &= ~(1<<LCD_E);
}

void write_2_nibbles (unsigned char content) {
	LCD_PORT = (LCD_PORT & 0x0f) | (content & 0xf0);		// send upper half
	enable_pulse();
	LCD_PORT = (LCD_PORT & 0x0f) | ((content<<4) & 0xf0);		// send lower half
	enable_pulse();
}

void lcd_data(unsigned char data) {					// send lcd_data
	LCD_PORT |= (1<<LCD_RS);
	write_2_nibbles(data);
	_delay_us(250);
}

void lcd_command(unsigned char command) {				// send lcd_command
	LCD_PORT &= ~(1<<LCD_RS);
	write_2_nibbles(command);
	_delay_us(250);
}

void lcd_clear_display() {						// clear display
	lcd_command(0x01);
	_delay_ms(5);
}

void lcd_init() {							// lcd_init
	_delay_ms(200);
	
	for(int i=0; i<3; i++) {
		LCD_PORT = 0x30;
		enable_pulse();
		_delay_us(250);
	}
	
	LCD_PORT = 0x20;
	enable_pulse();
	_delay_us(250);
	
	lcd_command(0x28);
	lcd_command(0x0C);
	lcd_clear_display();
	lcd_command(0x06);
}

void new_conv() {							// new conversion
	
	uint32_t Vin = ADC * 500.0 / 1024.0;				// Vin

	uint32_t dV = Vin - 0.1;					// gas_v0 = 0.1
	if(dV < 0) dV = 0;
	
	uint32_t ppm = dV / SENS;					// calculate ppm
	
	int i = 0;
	if(ppm < CO_THRESHOLD) {
		lcd_clear_display();
		while(clear_msg[i]) lcd_data(clear_msg[i++]);
	}
	else {
		lcd_clear_display();
		while(gas_detected_msg[i]) lcd_data(gas_detected_msg[i++]);
	}
	
	if(ppm < CO_THRESHOLD/5) {
		PORTB = 0x01;
		_delay_ms(100);
		check = 0;
	}
	else if(ppm < 2*CO_THRESHOLD/5) {
		PORTB = 0x03;
		_delay_ms(100);
		check = 0;
	}
	else if(ppm < 3*CO_THRESHOLD/5) {
		PORTB = 0x07;
		_delay_ms(100);
		check = 0;
	}
	else if(ppm < 4*CO_THRESHOLD/5) {
		PORTB = 0x0f;
		_delay_ms(100);
		check = 0;
	}
	else if(ppm < CO_THRESHOLD) {
		PORTB = 0x1f;
		_delay_ms(100);
		check = 0;
	}
	else {
		if(!check) {
			PORTB = 0x3f;
			check = 1;
		}
		if(ppm < 3*CO_THRESHOLD) {
			for(int i = 0; i < 1; i++) {
				_delay_ms(100);
				PORTB ^= 0x3f;
			}	
		}
		else if(ppm < 6*CO_THRESHOLD){
			for(int i = 0; i < 2; i++) {
				_delay_ms(50);
				PORTB ^= 0x3f;
			}
		}
		else {
			for(int i = 0; i < 10; i++) {
				_delay_ms(20);
				PORTB ^= 0x3f;
			}
		}
	}
}

void reset() {								// reset
	DDRD = 0xff;							// PORTD -> output
	DDRC &= ~(1<<PINC2);						// PC2 -> A2 -> POT3
	DDRB = 0xff;							// PORTB -> output

	ADMUX = (1<<REFS0) | (ADC_CHANNEL);				// Vref = 5V, ADC2 -> input
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// ADC enable, prescaler = 128 => fADC = 125KHz
	
	lcd_init();							// lcd_init
	_delay_ms(100);
	
	lcd_clear_display();						// lcd_clear_display
	_delay_us(250);
}

int main(void) {							// main
	
	reset();
	while(1) {
		ADCSRA |= (1<<ADSC);					// start ADC conversion
		while(ADCSRA & (1 << ADSC));				// while in ADC conversion do nothing
		new_conv();						// else start new conversion
	}
	return 0;
}
