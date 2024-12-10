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

// ADC input
#define PC0 0

// TWI 1 bit wire
#define PD4 4

// Keyboard
#define NO_KEY_PRESSED 0xffff

// payload
#define TEAM_ID 36

// IoT
#define OK 0
#define NURSE_CALL 1
#define CHECK_PRESSURE 2
#define CHECK_TEMPERATURE 3
#define TEAM 6

//--------------------------------------- TWI ------------------------------------------
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
//----------------------------------------------------------------------------------------------------

//----------------------------------------------------- LCD ------------------------------------------

char empty[] = "                ";

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

// return_home function
void return_home() {
	lcd_command(0x02);
	_delay_us(1530);
}

// display new line
void lcd_display_new_line(unsigned char msg[]) {
	lcd_command(0xc0);
	int i=0;
	while(msg[i]) lcd_data(msg[i++]);
}

//-----------------------------------------------------------------------------------------------

//----------------------------------------- USART_ESP ------------------------------------------

// esp answer
char success[] = "Success";
char fail[] = "Fail";

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
	UCSR0B=(1<<RXEN0)|(1<<TXEN0);				// Transmitter on, Receiver on
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

//-----------------------------------------------------------------------------------------------------------

//------------------------------------------------- TEMPERATURE ---------------------------------------------

uint16_t prev_temp = 0x8000;
char no_device[] = "No Device";

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

// display temperature function

double calc_display_temp(uint16_t temp) {

	prev_temp = temp;
	double res_temp;

	if(temp == 0x8000) {
		lcd_display_msg(no_device);
		res_temp = (double)temp;
	}
	else {
		int dec = (int)(temp & 0x000f) * 62.5;

		int dec1 = (dec % 1000) / 100;
		int dec2 = (dec % 100) / 10;
		int dec3 = dec % 10;

		int acc = (temp & 0x07f0) >> 4;

		acc += 12;							// add 12 *C
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
		
		res_temp = (double)(acc*1.0 + dec/1000.0);
	}

	return res_temp;
}

//----------------------------------------------------------------------------------------------------


//---------------------------------- ADC -------------------------------------------------------------

/*
 ADC_INIT:
 PC0 -> A0 -> POT1 -> INPUT
 Vref = 5V, ADC0 -> INPUT
 ADC enable, prescaler = 128 => fadc = 125kHz
*/

uint16_t prev_adc = 0;

void adc_init() {
	DDRC &= ~(1<<PC0);
	ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

double calc_display_adc(uint16_t adc) {
	prev_adc = adc;
	uint32_t pressure = prev_adc * 20.0 / 1024.0;
	
	int dec1 = pressure / 10;
	int dec2 = pressure % 10;

	lcd_data('0' + dec1);
	lcd_data('0' + dec2);
	lcd_data('c');
	lcd_data('m');
	lcd_data(' ');
	lcd_data('H');
	lcd_data('2');
	lcd_data('0');
	
	return (double)pressure;
}

//-----------------------------------------------------------------------------------------------------

//---------------------------------------- KEYBOARD ---------------------------------------------------

uint16_t pressed_keys = NO_KEY_PRESSED;

char ok[] = "OK";
char nurse_call[] = "NURSE CALL";
char check_pressure[] = "CHECK PRESSURE";
char check_temperature[] = "CHECK TEMPERATURE";
char no_status_found[] = "NO STATUS FOUND";

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

void scan_keypad_rising_edge() {
	uint16_t pressed_keys_tempo = scan_keypad();
	// Debounce delay
	_delay_ms(20);
	uint16_t current_keys = scan_keypad();

	// if pressed_keys_tempo is valid button press
	// and is different from the previous one
	// update pressed_keys variable
	// and increase the counter until it reaches 2
	// then reset the counter (init to 0)
	// we display up to 2 digits
	if(current_keys == pressed_keys_tempo)
	if(pressed_keys_tempo != pressed_keys) {
		pressed_keys = pressed_keys_tempo;
	}
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

//-----------------------------------------------------------------------------------------------------
int main() {
	
	// twi, lcd, usart init
	twi_init();
	lcd_init();
	usart_init(103);
	adc_init();
	
	// Set IO0 as output
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00);
	
	// temperature
	uint16_t temp;
	double t;
	
	// pressure
	uint16_t pressure;
	double p;

	// status
	unsigned char status;

	DDRB = 0xff;
	usart_transmit_message("ESP:restart\n");		// esp restart

	if(usart_receive_message()) {					// if success
		while(1) {
/*			lcd_clear_display();						// then clear lcd

			usart_transmit_message("ESP:connect\n");	// and connect
			if(usart_receive_message())					// receive and display the answer
			esp_ans_display(success, 1);
			else esp_ans_display(fail, 1);
			_delay_ms(1000);													// wait 1 sec to see the answer
			lcd_clear_display();												// clear lcd again
			usart_transmit_message("ESP:url:http://192.168.1.250:5000/data");	// esp url
			if(usart_receive_message())											// receive and display the answer
			esp_ans_display(success, 2);
			else esp_ans_display(fail, 2);
			_delay_ms(1000);
			lcd_clear_display();
*/
			return_home();
			char key;

			while(1) {
				scan_keypad_rising_edge();					// check for pressed buttons
				key = keypad_to_ascii(pressed_keys);		// key = ascii of pressed button
				if(key == '6') break;						// if '6' is pressed call nurse
			}
			
			ADCSRA |= (1<<ADSC);						// start ADC conversion
			while(ADCSRA & (1 << ADSC));				// while in ADC conversion do nothing
			pressure = ADC;								// read pressure

			temp = read_temp();							// read temperature

			status = NURSE_CALL;						// status -> nurse call
			
			t = calc_display_temp(temp);				// display temperature
			lcd_data(' ');
			p = calc_display_adc(pressure);				// display pressure

			lcd_display_new_line(nurse_call);			// display status in new line

			while(1) {
				scan_keypad_rising_edge();
				key = keypad_to_ascii(pressed_keys);
				if(key == '#') break;					// if '#' is pressed update status
			}
			
			lcd_display_new_line(empty);				// clear second lcd line
			
			if(p < 4 || p > 12) {
				status = CHECK_PRESSURE;
				lcd_display_new_line(check_pressure);
			}
			else if(t < 34 || t > 37) {
				status = CHECK_TEMPERATURE;
				lcd_display_new_line(check_temperature);
			}
			else {
				status = OK;
				lcd_display_new_line(ok);
			}
			//_delay_ms(2000);
		}
	}
	return 0;
}
