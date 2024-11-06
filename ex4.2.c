#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

#define LCD_PORT PORTD
#define LCD_RS 2
#define LCD_E  3
#define LCD_DB4 4
#define LCD_DB5 5
#define LCD_DB6 6
#define LCD_DB7 7

uint16_t prevADC=0;

void enable_pulse() {
	LCD_PORT |= (1<<LCD_E);
	_delay_us(1);
	LCD_PORT &= ~(1<<LCD_E);
}

void write_2_nibbles (unsigned char content) {
	LCD_PORT = (LCD_PORT & 0x0f) | (content & 0xf0);		// send upper half
	enable_pulse();
	LCD_PORT = (LCD_PORT & 0x0f) | ((content<<4) & 0xf0);	// send lower half
	enable_pulse();
}

void lcd_data(unsigned char data) {
	LCD_PORT |= (1<<LCD_RS);
	write_2_nibbles(data);
	_delay_us(250);
}

void lcd_command(unsigned char command) {
	LCD_PORT &= ~(1<<LCD_RS);
	write_2_nibbles(command);
	_delay_us(250);
}

void lcd_clear_display() {
	lcd_command(0x01);
	_delay_ms(5);
}

void lcd_init() {
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

void return_home() {
	lcd_command(0x02);
	_delay_us(1530);
}

void new_conv() {
	
	prevADC = ADC;
	uint32_t x = prevADC * 500.0 / 1024.0;

	int dec2 = x / 100;
	int dec1 = (x % 100) / 10;
	int dec0 = (x % 10);

	lcd_clear_display();
	_delay_us(250);
	lcd_data('0'+ dec2);
	lcd_data('.');
	lcd_data('0'+ dec1);
	lcd_data('0'+ dec0);
}

void reset() {
	DDRD = 0xff;
	DDRC &= ~(1<<PINC1);

	ADMUX = (1<<REFS0) | (1<<MUX0);
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	
	prevADC = 0;
	
	lcd_init();
	_delay_ms(100);
	
	lcd_clear_display();
	_delay_us(250);
	
}

int main(void) {
	
	reset();
	while(1) {
		ADCSRA |= (1<<ADSC);
		while(ADCSRA & (1 << ADSC));
		uint16_t adc_value = ADC;
		if (adc_value == prevADC) return_home();
		else new_conv();
		_delay_ms(1000);
	}
	
	return 0;
}
