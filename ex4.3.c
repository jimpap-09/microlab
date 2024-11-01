#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define CO_THRESHOLD 70    // CO threshold in ppm
#define ADC_CHANNEL 2      // ADC channel for A2

// LCD Connections
#define LCD_PORT PORTD
#define LCD_DDR DDRD
#define RS PD2
#define EN PD3

// LCD Messages
const char gas_detected_msg[] = "GAS DETECTED";
const char clear_msg[] = "CLEAR";

// Function to initialize the ADC
void ADC_init() {
	ADMUX = (1 << REFS0) | (ADC_CHANNEL);  // Set reference voltage to AVcc, input channel to ADC2 (A2)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, set prescaler to 128
}

// Function to start an ADC conversion and wait until it completes
uint16_t ADC_read() {
	ADCSRA |= (1 << ADSC);  // Start ADC conversion
	while (ADCSRA & (1 << ADSC));  // Wait until ADSC becomes 0, indicating conversion complete
	return ADC;  // Return the 10-bit ADC result
}

// Function to calculate CO concentration in ppm from ADC result
float calculate_ppm(uint16_t adc_value) {
	float voltage = (adc_value / 1024.0) * 5.0;  // Convert ADC value to voltage
	float delta_v = voltage - 0.1;  // Subtract baseline voltage (Vgas0 = 0.1V)
	if (delta_v < 0) delta_v = 0;   // Ensure no negative values
	float ppm = (delta_v / 0.129);  // Convert voltage to ppm based on sensitivity (129 nA/ppm)
	return ppm;
}

void LED_init() {
	DDRB |= 0x3F;  // Set PB0 to PB5 as output
}

// Function to update LEDs based on concentration level
void update_LEDs(float ppm) {
	LCD_clear();
	if (ppm < 10) {
		PORTB = 0x01;  // Light up PB0 for low ppm
		} else if (ppm < 20) {
		PORTB = 0x03;  // Light up PB0, PB1
		} else if (ppm < 40) {
		PORTB = 0x07;  // Light up PB0, PB1, PB2
		} else if (ppm < 60) {
		PORTB = 0x0F;  // Light up PB0 to PB3
		} else if (ppm < 70) {
		PORTB = 0x1F;  // Light up PB0 to PB4
		} else {
		PORTB = 0x3F;  // Light up PB0 to PB5 for 70 ppm or above
	}

	// Blink LEDs if CO concentration is 70 ppm or higher
	if (ppm >= 70) {
		_delay_ms(500);  // Half a second delay for blinking effect
		PORTB ^= 0x3F;
		LCD_clear();// Toggle all LEDs on PB0 to PB5
	}
}

// LCD command function
void LCD_command(unsigned char cmnd) {
	LCD_PORT = (LCD_PORT & 0x0F) | (cmnd & 0xF0);  // Send upper nibble
	LCD_PORT &= ~(1 << RS);  // RS = 0 for command
	LCD_PORT |= (1 << EN);   // Enable pulse
	_delay_us(1);
	LCD_PORT &= ~(1 << EN);

	_delay_us(200);

	LCD_PORT = (LCD_PORT & 0x0F) | (cmnd << 4);  // Send lower nibble
	LCD_PORT |= (1 << EN);
	_delay_us(1);
	LCD_PORT &= ~(1 << EN);
	_delay_ms(2);
}

// LCD data function
void LCD_data(unsigned char data) {
	LCD_PORT = (LCD_PORT & 0x0F) | (data & 0xF0);  // Send upper nibble
	LCD_PORT |= (1 << RS);  // RS = 1 for data
	LCD_PORT |= (1 << EN);
	_delay_us(1);
	LCD_PORT &= ~(1 << EN);

	_delay_us(200);

	LCD_PORT = (LCD_PORT & 0x0F) | (data << 4);  // Send lower nibble
	LCD_PORT |= (1 << EN);
	_delay_us(1);
	LCD_PORT &= ~(1 << EN);
	_delay_ms(2);
}

// LCD initialization function
void LCD_init() {
	LCD_DDR = 0xFF;  // Set LCD port as output
	_delay_ms(20);

	LCD_command(0x02);  // Initialize LCD in 4-bit mode
	LCD_command(0x28);  // 2 line, matrix
	LCD_command(0x0C);  // Display on, cursor off
	LCD_command(0x06);  // Increment cursor
	LCD_command(0x01);  // Clear display
	_delay_ms(2);
}

// Function to clear the LCD display
void LCD_clear() {
	LCD_command(0x01);  // Clear display
	_delay_ms(2);
}

// Function to display a string on the LCD
void LCD_write_string(const char* str) {
	while (*str) {
		LCD_data(*str++);
	}
}

// Function to display the alert on the LCD
void display_alert(float ppm) {
	LCD_clear();  // Clear LCD
	if (ppm > CO_THRESHOLD) {
		LCD_write_string(gas_detected_msg);  // Display alert if ppm > threshold
		} else {
		LCD_write_string(clear_msg);  // Display clear if ppm <= threshold
	}
}


int main() {
	ADC_init();  // Initialize ADC
	LED_init();  // Initialize LEDs
	LCD_init();  // Initialize LCD

	while (1) {
		uint16_t adc_value = ADC_read();  // Read ADC value
		float ppm = calculate_ppm(adc_value);  // Calculate CO concentration

		display_alert(ppm);  // Display alert or clear on LCD
		update_LEDs(ppm);    // Update and blink LEDs based on ppm level

		_delay_ms(100);  // Check every 100 mS
	}
}
