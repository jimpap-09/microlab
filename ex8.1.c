#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
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

#define TIMEOUT_20_sec 100

uint8_t IO0 = 0x00;
uint16_t prev_temp = 0x8000;

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

// display message on the lcd
void lcd_display_msg(unsigned char msg[]) {
	lcd_clear_display();
	_delay_us(250);
	lcd_command(0x80);
	int i=0;
	while(msg[i]) lcd_data(msg[i++]);
}

/* Routine: usart_init
Description:
This routine initializes the
usart as shown below.

------- INITIALIZATIONS -------
Baud rate: 9600 (Fck= 8MH)
Asynchronous mode
Transmitter on
Receiver on
Communication parameters: 8 Data ,1 Stop, no Parity
----------------------------------------------------
parameters: ubrr to control the BAUD.
return value: None.
*/

void usart_init(unsigned int ubrr){
	UCSR0A=0;									// (RXC0, TXC0, UDRE0) = (0, 0, 0) => no state
	UCSR0B=(1<<RXEN0)|(1<<TXEN0);				// Transmitter on, Reciever on
	UBRR0H=(unsigned char)(ubrr>>8);			// UBRR0H = UBRR0[11:8]
	UBRR0L=(unsigned char)ubrr;					// UBRR0L = UBRR0[7:0]
	UCSR0C=(3 << UCSZ00);						// USCR0C = 0b00000110
	// UMSEL0[1:0] = 00 => Asynchronous mode
	// UCSZ0[2:0] = 011 => 8-bit character size
	return;
}

// function to transmit data using usart
/* Routine: usart_transmit
Description:
This routine sends a byte of data
4
using usart.
parameters:
data: the byte to be transmitted
return value: None. */

void usart_transmit(uint8_t data) {
	while(!(UCSR0A & (1<<UDRE0)));						// wait until UDR0 is ready to receive data
	UDR0 = data;										// then transmit the data
}

// function to receive data using usart
/* Routine: usart_receive
Description:
This routine receives a byte of data
from usart.
parameters: None.
return value: the received byte */

uint8_t usart_receive() {
	while(!(UCSR0A & (1<<RXC0)));						// wait until there is data to be read
	return UDR0;										// return udr0
}

void usart_transmit_message(const char* msg) {
	int i = 0;
	while(msg[i]) usart_transmit((uint8_t)msg[i++]);	// send each byte to usart
}

int usart_receive_message() {							// receive the esp answer
	char ans[20];
	int i = 0;
	while(usart_receive() != '"');
	while(1) {
		ans[i] = usart_receive();						// receive each byte from usart
		if(ans[i] == '\n') break;
		i++;
	}
	if(ans[0]=='S') return 1;							// if the answer is Success return 1
	return 0;											// else return 0
}

void esp_ans_display(char msg[], int k) {				// display esp answer to lcd
	lcd_clear_display();
	_delay_us(250);
	lcd_command(0x80);
	lcd_data(k + '0');
	lcd_data('.');
	int i=0;
	while(msg[i]) lcd_data(msg[i++]);
}

// esp answer
char success[] = "Success";
char fail[] = "Fail";

int main() {
	
	// twi, lcd, usart init
	twi_init();
	lcd_init();
	usart_init(103);
	
	// Set IO0 as output
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00);
	
	usart_transmit_message("ESP:restart\n");		// esp restart

	if(usart_receive_message()) {					// if success
		lcd_clear_display();						// then clear lcd
		usart_transmit_message("ESP:connect\n");	// and connect
		if(usart_receive_message()) {				// receive and display the answer
			esp_ans_display(success, 1);
		}
		else {
			esp_ans_display(fail, 1);
		}
		_delay_ms(1000);													// wait 1 sec to see the answer
		lcd_clear_display();												// clear lcd again
		usart_transmit_message("ESP:url:http://192.168.1.250:5000/data");	// esp url
		if(usart_receive_message()) {										// receive and display the answer
			esp_ans_display(success, 2);
		}
		else {
			esp_ans_display(fail, 2);
		}
	}
}
