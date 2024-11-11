#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <xc.h>

#define PCA9555_0_ADDRESS 0x40
#define TWI_READ 1
#define TWI_WRITE 0
#define SCL_CLOCK 100000L

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

/*
(A, B, C, D) -> (PB0-PB3)
F0 = (A'BC + B'D)' = (A+B'+C')(B+D')
F1 = (A + B + C)(BD') = ABD' + BCD' = (A + C + 1)BD' = BD'

A	B	C	D	|	F0	F1
0	0	0	0	|	1	0
0	0	0	1	|	0	0
0	0	1	0	|	1	0
0	0	1	1	|	0	0
0	1	0	0	|	1	1
0	1	0	1	|	1	0
0	1	1	0	|	0	1
0	1	1	1	|	0	0
1	0	0	0	|	1	0
1	0	0	1	|	0	0
1	0	1	0	|	1	0
1	0	1	1	|	0	0
1	1	0	0	|	1	1
1	1	0	1	|	1	0
1	1	1	0	|	1	1
1	1	1	1	|	1	0
*/

uint8_t f0(uint8_t input) {
	uint8_t A = (input >> 0) & 1;
	uint8_t B = (input >> 1) & 1;
	uint8_t C = (input >> 2) & 1;
	uint8_t D = (input >> 3) & 1;

	// F0 = (A'BC + B'D)'
	uint8_t A_ = A^0x01;			// invert A_LSB
	uint8_t B_ = B^0x01;			// invert B_LSB
	uint8_t term1 = A_ & B & C;
	uint8_t term2 = B_ & D;
	uint8_t F0 = (term1 | term2)^0x01;
	return F0;
}

uint8_t f1(uint8_t input) {
	uint8_t B = (input >> 1) & 1;
	uint8_t D = (input >> 3) & 1;

	// F1 = (A + B + C) * (B * D') = B * D'
	uint8_t D_ = D^0x01;
	uint8_t F1 = B & D_;
	return F1;
}

int main(void) {
	// Initialize TWI
	twi_init();
	
	// Set PCA9555 Port 0 as output
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00);
	
	// Set PORTB as input
	DDRB = 0x00;
	
	while(1) {
		// Read input from PORTB
		uint8_t input = PINB & 0x0F;
		
		// Compute F0 and F1
		uint8_t F0 = f0(input);
		uint8_t F1 = f1(input);
		
		// Combine F0 and F1 to write to output (bit 0 -> F0, bit 1 -> F1)
		uint8_t output = (F1 << 1) | F0;
		
		// Write output to PCA9555
		PCA9555_0_write(REG_OUTPUT_0, output);
		
		// Delay
		_delay_ms(500);
	}
}
