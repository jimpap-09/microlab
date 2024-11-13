#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

#define PCA9555_0_ADDRESS 0x40
#define TWI_READ 1
#define TWI_WRITE 0
#define SCL_CLOCK 100000L

#define NO_KEY_PRESSED 0xffff

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
//Return:  0 device accessible
//1 failed to access device
unsigned char twi_rep_start(unsigned char address) {
	return twi_start( address );
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

int scan_row(int row) {
	PCA9555_0_write(REG_CONFIGURATION_1, ~(1 << row));
	PCA9555_0_write(REG_OUTPUT_1, 0x00);

	// Mask for the upper 4 bits
	uint8_t btn_pressed = ~(PCA9555_0_read(REG_INPUT_1) >> 4);

	for(int col = 0; col < 4; col++) {
		if(btn_pressed & (1 << col)) return col;
	}

	// No button pressed in this row
	return -1;
}

uint16_t scan_keypad() {
	int row, col;
	for(row = 0; row < 4; row++) {
		col = scan_row(row);
		if (col != -1) {

			// Combine row and column into a single 16-bit value
			return (col << 8) | row;
		}
	}

	// Indicate no key was pressed
	return NO_KEY_PRESSED;
}

uint16_t pressed_keys = NO_KEY_PRESSED;
char prev_key = '\0';

void scan_keypad_rising_edge() {
	uint16_t pressed_keys_tempo = scan_keypad();
	// Debounce delay
	_delay_ms(20);
	uint16_t current_keys = scan_keypad();

	// if pressed_keys_tempo is valid button press
	// and is different from the previous one
	// update pressed_keys variable
	if (current_keys == pressed_keys_tempo
	&& pressed_keys_tempo != pressed_keys)
	pressed_keys = pressed_keys_tempo;
}

char keypad_to_ascii(uint16_t key_code) {

	static const char key_map[4][4] = {
		{'*', '0', '#', 'D'},
		{'7', '8', '9', 'C'},
		{'4', '5', '6', 'B'},
		{'1', '2', '3', 'A'}
	};

	// No key pressed
	if (key_code == NO_KEY_PRESSED) return '\0';
	int col = (key_code >> 8) & 0xFF;
	int row = key_code & 0xFF;

	if (row >= 0 && row < 4 && col >= 0 && col < 4) {
		return key_map[row][col];
	}
	// Invalid key press
	return '\0';
}

int main(void) {
	
	// Initialize TWI, LCD
	twi_init();
	lcd_init();

	// Set PCA9555 Port 0 as output
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00);

	while (1) {

		// Update key status
		scan_keypad_rising_edge();
		
		// return the ascii code of pressed_keys
		// to represent which button is pressed
		char key = keypad_to_ascii(pressed_keys);

		// if key has changed display it and update prev_key
		if(key != prev_key) {
			if(key == '\0') lcd_clear_display();
			else lcd_data(key);
			prev_key = key;
		}
	}
}
