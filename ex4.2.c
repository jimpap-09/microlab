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

uint16_t prevADC=0;

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

void lcd_data(unsigned char data) {					// send lcd data
	LCD_PORT |= (1<<LCD_RS);
	write_2_nibbles(data);
	_delay_us(250);
}

void lcd_command(unsigned char command) {				// send lcd command
	LCD_PORT &= ~(1<<LCD_RS);
	write_2_nibbles(command);
	_delay_us(250);
}

void lcd_clear_display() {						// clear display
	lcd_command(0x01);
	_delay_ms(5);
}

void lcd_init() {							// lcd init
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

void return_home() {							// return home
	lcd_command(0x02);
	_delay_us(1530);
}

void new_conv() {							// new conversion
	
	prevADC = ADC;
	uint32_t Vin = prevADC * 500.0 / 1024.0;			// Vin		

	int dec2 = Vin / 100;						// dec2
	int dec1 = (Vin % 100) / 10;					// dec1
	int dec0 = (Vin % 10);						// dec0

	lcd_clear_display();
	_delay_us(250);
	lcd_data('0'+ dec2);
	lcd_data('.');
	lcd_data('0'+ dec1);
	lcd_data('0'+ dec0);
}

void reset() {								// reset
	DDRD = 0xff;							// PORTD -> output
	DDRC &= ~(1<<PINC1);						// PC1 -> A1 -> POT2

	ADMUX = (1<<REFS0) | (1<<MUX0);					// Vref = 5V, ADC1 -> input
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// ADC enable, prescaler = 128 => fADC = 125KHz
	
	prevADC = 0;							// init prevADC		
	
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
		uint16_t adc_value = ADC;				// read ADC
		if (adc_value == prevADC) return_home();		// if ADC doesn't change return home
		else new_conv();					// else start new conversion
		_delay_ms(1000);					// delay 1 sec between conversions
	}
	
	return 0;
}
