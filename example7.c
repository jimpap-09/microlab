#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

#define PD4 4

#define PCA9555_0_ADDRESS 0x40
#define TWI_READ 1
#define TWI_WRITE 0
#define SCL_CLOCK 100000L

// TWBR0_VALUE for prescaler = 1
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2

typedef enum {
	REG_INPUT_0	= 0,
	REG_INPUT_1	= 1,
	REG_OUTPUT_0 = 2,
	REG_OUTPUT_1 = 3,
	REG_POLARITY_INV_0 = 4,
	REG_POLARITY_INV_1 = 5,
	REG_CONFIGURATION_0 = 6,
	REG_CONFIGURATION_1 = 7
} PCA9555_REGISTERS;			// pointers for read/write registers

// Master Transmitter/Receiver
#define TW_START 0x08
#define TW_REP_START 0x10

// Master Transmitter
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28

// Master Receiver
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_NACK 0x58
#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)

// LCD bits
#define LCD_RS 2
#define LCD_E  3
#define LCD_DB4 4
#define LCD_DB5 5
#define LCD_DB6 6
#define LCD_DB7 7

uint8_t IO0 = 0x00;

void twi_init(void) {
	TWSR0 = 0;					// prescaler_value = 1
	TWBR0 = TWBR0_VALUE;		// scl_clock 100KHz
}

// read one byte from the twi device and request more
unsigned char twi_readAck(void) {
	TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while(!(TWCR0 & (1 << TWINT)));
	return TWDR0;
}

// read one byte from the twi device and then stop
unsigned char twi_readNak(void) {
	TWCR0 = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR0 & (1 << TWINT)));
	return TWDR0;
}

// send address and transfer direction
unsigned char twi_start(unsigned char address) {
	uint8_t twi_status;
	TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while(!(TWCR0 & (1 << TWINT)));
	twi_status = TW_STATUS & 0xf8;
	if((twi_status != TW_START) && (twi_status != TW_REP_START)) return 1;
	TWDR0 = address;
	TWCR0 = (1 << TWINT) | (1 << TWEN);
	while(! (TWCR0 & (1 << TWINT)));
	twi_status = TW_STATUS & 0xf8;
	if((twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK)) {
		return 1;
	}
	return 0;
}

// send address and transfer direction and wait
void twi_start_wait(unsigned char address) {
	uint8_t twi_status;
	while(1) {
		TWCR0  = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
		while(! (TWCR0 & (1 << TWINT)));
		twi_status = TW_STATUS & 0xf8;
		if ((twi_status != TW_START) && (twi_status != TW_REP_START)) continue;
		TWDR0 = address;
		TWCR0 = (1 << TWINT) | (1 << TWEN);
		while(!(TWCR0 & (1 << TWINT)));
		twi_status = TW_STATUS & 0xf8;
		if((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MR_DATA_NACK)) {
			TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
			while(TWCR0 & (1 << TWSTO));
			continue;
		}
		break;
	}
}

unsigned char twi_write(unsigned char data) {
	TWDR0 = data;
	TWCR0 = (1 << TWINT) | (1 << TWEN);
	while(!(TWCR0 & (1 << TWINT)));
	if((TW_STATUS & 0xf8) != TW_MT_DATA_ACK) return 1;
	return 0;
}

// Send repeated start condition, address, transfer direction
// Return:  0 device accessible
// 1 failed to access device
unsigned char twi_rep_start(unsigned char address) {
	return twi_start(address);
}

// Terminates the data transfer and releases the twi bus
void twi_stop(void) {
	// send stop condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));
}

void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value) {
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
}

uint8_t PCA9555_0_read(PCA9555_REGISTERS reg) {
	uint8_t ret_val;
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
	ret_val = twi_readNak();
	twi_stop();
	return ret_val;
}

// enable pulse
void enable_pulse() {
	IO0 = PCA9555_0_read(REG_INPUT_0);
	PCA9555_0_write(REG_OUTPUT_0, IO0 | (1 << LCD_E));
	_delay_us(16);
	IO0 = PCA9555_0_read(REG_INPUT_0);
	PCA9555_0_write(REG_OUTPUT_0, IO0 & ~(1 << LCD_E));
}

// write 2 nibbles
void write_2_nibbles (unsigned char content) {
	IO0 = PCA9555_0_read(REG_INPUT_0);
	PCA9555_0_write(REG_OUTPUT_0, (IO0 & 0x0f) | (content & 0xf0));			// send upper half
	enable_pulse();
	IO0 = PCA9555_0_read(REG_INPUT_0);
	PCA9555_0_write(REG_OUTPUT_0, (IO0 & 0x0f) | ((content<<4) & 0xf0));	// send upper half
	enable_pulse();
}

// send lcd_data
void lcd_data(unsigned char data) {
	IO0 = PCA9555_0_read(REG_INPUT_0);
	PCA9555_0_write(REG_OUTPUT_0, IO0 | (1<<LCD_RS));
	write_2_nibbles(data);
	_delay_us(250);
}

// send lcd_command
void lcd_command(unsigned char command) {
	IO0 = PCA9555_0_read(REG_INPUT_0);
	PCA9555_0_write(REG_OUTPUT_0, IO0 & ~(1<<LCD_RS));
	write_2_nibbles(command);
	_delay_us(250);
}

// clear display
void lcd_clear_display() {
	lcd_command(0x01);
	_delay_ms(5);
}

// lcd_init
void lcd_init() {
	_delay_ms(200);
	
	for(int i=0; i<3; i++) {
		PCA9555_0_write(REG_OUTPUT_0, 0x30);
		enable_pulse();
		_delay_us(250);
	}
	
	PCA9555_0_write(REG_OUTPUT_0, 0x20);
	enable_pulse();
	_delay_us(250);
	
	lcd_command(0x28);
	lcd_command(0x0C);
	lcd_clear_display();
	lcd_command(0x06);
}

// return 0 if a device is detected, else return 1
uint8_t one_wire_reset() {
	DDRD |= (1<<PD4);
	PORTD &= ~(1<<PD4);
	_delay_us(480);
	DDRD &= ~(1<<PD4);
	_delay_us(100);
	uint8_t input = ((1<<PD4) & PIND)>>PD4;
	_delay_us(380);
	return (input==0);
}

// return the in_bit[0] = PD4 (input bit)
uint8_t one_wire_receive_bit() {
	DDRD |= (1<<PD4);
	PORTD &= ~(1<<PD4);
	_delay_us(2);
	DDRD &= ~(1<<PD4);
	_delay_us(10);
	uint8_t in_bit = ((1<<PD4) & PIND)>>PD4;
	_delay_us(49);
	return in_bit;
}

// PD4 = out_bit[0]
void one_wire_transmit_bit(uint8_t out_bit) {
	DDRD |= (1<<PD4);
	PORTD &= ~(1<<PD4);
	_delay_us(2);
	out_bit &= 0x01;
	if(out_bit == 1) PORTD |= (1<<PD4);
	else if(out_bit == 0) PORTD &= ~(1<<PD4);
	_delay_us(58);
	DDRD &= ~(1<<PD4);
	_delay_us(1);
}

// return in_byte = PD4 (8 times : LSB -> MSB)
uint8_t one_wire_receive_byte() {
	uint8_t in_byte = 0x00;
	uint8_t in_bit;
	for(int i=0; i<8; i++) {
		in_bit = one_wire_receive_bit();
		in_byte>>=1;
		if(in_bit == 0) in_byte |= 0x00;
		else in_byte |= 0x80;
	}
	return in_byte;
}

// PD4 = out_bit = out_byte[0] (8 times : LSB -> MSB)
void one_wire_transmit_byte(uint8_t out_byte) {
	uint8_t out_bit;
	for(int i=0; i<8; i++) {
		out_bit = out_byte & 1;
		out_byte>>1;
		one_wire_transmit_bit(out_bit);
	}
}

uint16_t read_temp() {
	uint16_t temp;
	uint8_t lsb = 0x00, msb = 0x80;
	
	if(one_wire_reset()) {
		one_wire_transmit_byte(0xcc);
		one_wire_transmit_byte(0x44);
		while(!one_wire_receive_bit());
		if(one_wire_reset()) {
			one_wire_transmit_byte(0xcc);
			one_wire_transmit_byte(0xbe);
			
			lsb = one_wire_receive_byte();
			msb = one_wire_receive_byte();
		}
	}

	temp = (msb << 8) | lsb;
	return temp;
}

void display_temp(uint16_t temp) {
	char buffer[16];
	
	if(temp == 0x8000) {
		lcd_clear_display();
		lcd_command(0x80);
		lcd_data("No Device");
		return;
	}
	
	float celsius = temp * 0.0625;
	int celsius_int = (int)celsius;
	int celsius_frac = (int)((celsius - celsius_int) * 1000);
	
	if(celsius < 0) {
		celsius_int = -celsius_int;
		celsius_frac = -celsius_frac;
	}
	
	lcd_clear_display();
	lcd_command(0x80);
	snprintf(buffer, sizeof(buffer), "%s%d.%03d C", (temp < 0) ? "-" : "", celsius_int, celsius_frac);
	lcd_data(buffer);
}

int main() {

	// Initialize TWI, LCD
	twi_init();
	lcd_init();

	DDRB = 0xff;
	PORTB = 1;

	// Set PCA9555 Port 0 as output
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00);

	while(1) {
		lcd_clear_display();
		PORTB = 1;
		_delay_ms(500);
		lcd_data('A');
		PORTB = 0;
		_delay_ms(1000);
	}
	return 0;
}
