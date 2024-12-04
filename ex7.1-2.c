#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

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

#define PD4 4

uint16_t prev_temp = 0x8000;
uint8_t IO0 = 0x00;

void twi_init(void) {
	TWSR0 = 0;			// prescaler_value = 1
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

// send data
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

// write data to register (reg) using twi
void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value) {
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
}

// read data from register (red) using twi
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
	PCA9555_0_write(REG_OUTPUT_0, (IO0 & 0x0f) | ((content<<4) & 0xf0));	// send lower half
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

// return 1 if a device is detected (PD4 = 0), else return 0
uint8_t one_wire_reset() {
	DDRD |= (1<<PD4);
	PORTD &= ~(1<<PD4);
	_delay_us(480);
	DDRD &= ~(1<<PD4);
	PORTD &= ~(1<<PD4);
	_delay_us(100);
	uint8_t port = PIND;
	_delay_us(380);
	if(port & (1<<PD4)) return 0;
	return 1;
}

// return the in_bit[0] = PD4 (input bit)
uint8_t one_wire_receive_bit() {
	uint8_t in_bit = 0;
	DDRD |= (1<<PD4);
	PORTD &= ~(1<<PD4);
	_delay_us(2);
	DDRD &= ~(1<<PD4);
	PORTD &= ~(1<<PD4);
	_delay_us(10);
	if(PIND & (1<<PD4)) in_bit = 1;
	_delay_us(49);
	return in_bit;
}

// PD4 = out_bit[0]
void one_wire_transmit_bit(uint8_t out_bit) {
	DDRD |= (1<<PD4);
	PORTD &= ~(1<<PD4);
	_delay_us(2);
	PORTD |= (out_bit<<PD4);
	_delay_us(58);
	DDRD &= ~(1<<PD4);
	PORTD &= ~(1<<PD4);
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
		out_byte>>=1;
		one_wire_transmit_bit(out_bit);
	}
}

// read temperature
uint16_t read_temp() {
	uint16_t temp = 0x8000;

	if(one_wire_reset()) {
		one_wire_transmit_byte(0xcc);
		one_wire_transmit_byte(0x44);			// start counting the temperature
		
		while(!one_wire_receive_bit());
		
		if(one_wire_reset()) {
			one_wire_transmit_byte(0xcc);
			one_wire_transmit_byte(0xbe);		// start reading the 16_bit temperature
			
			temp = one_wire_receive_byte();
			temp += (one_wire_receive_byte()<<8);
		}
	}
	return temp;
}

// display "No Device" message on the lcd
void display_msg(unsigned char msg[]) {
	lcd_clear_display();
	_delay_us(250);
	lcd_command(0x80);
	int i=0;
	while(msg[i]) lcd_data(msg[i++]);
}

// display temperature function
void display_temp(uint16_t temp, unsigned char msg[]) {

	prev_temp = temp;

	if(temp == 0x8000) display_msg(msg);
	else {

		int dec = (int)(temp & 0x000f) * 62.5;

		int dec1 = (dec % 1000) / 100;
		int dec2 = (dec % 100) / 10;
		int dec3 = dec % 10;

		int acc = (temp & 0x07f0) >> 4;

		int acc1 = (acc % 100) / 10;
		int acc2 = (acc % 10);

		int sign = (temp & 0xf800) ? 1 : 0;

		lcd_clear_display();
		_delay_us(250);
		if(sign) lcd_data('-');
		lcd_data('0' + acc1);
		lcd_data('0' + acc2);
		lcd_data('.');
		lcd_data('0' + dec1);
		lcd_data('0' + dec2);
		lcd_data('0' + dec3);
		lcd_data('C');
	}
}

// return_home function
void return_home() {
	lcd_command(0x02);
	_delay_us(1530);
}

int main() {
	
	// No Device message
	unsigned char msg[] = "No Device";
	
	// Initialize TWI, LCD
	twi_init();
	lcd_init();

	// Set PCA9555 Port 0 as output
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00);

	// Read the temperature
	uint16_t temp = read_temp();
	// Display it
	display_temp(temp, msg);

	// infinite loop
	while(1) {
		// read the temperature
		temp = read_temp();
		// if it remains the same dont change the lcd
		if(temp == prev_temp) return_home();
		// else display new temperature
		else {
			display_temp(temp, msg);
			_delay_ms(750);
		}
	}

	return 0;
}
