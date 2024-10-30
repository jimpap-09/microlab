#include <avr/io.h>
#include <util/delay.h>

// CPU, PWM frequency
#define F_CPU 16000000UL
#define F_PWM 62500

int DC_VALUE;
int OCR1A_table[14] = {5, 25, 45, 65, 86, 107, 128,
                      149, 170, 191, 212, 223, 234, 250};
int ptr;

void PWM_init() {
/*
 * INIT PWM
 * f_PWM = f_CLK/[N*(1+TOP)]
 * (f_PWM, f_CLK, TOP) = (62500, 16000000, 255) => N = 1
 * (COM1A0, COM1A1) = (0, 1) => OC1A ~ PB1
 * (WGM10, WGM11) = (1, 0) => Fast PWM, 8-bit
 * (WGM12, WGM13) = (1, 0) => Fast PWM, 8-bit
 * (CS10, CS11, CS12) = (0, 0, 1) => N = 1
 */
    DDRB = (1<<PINB1) | (0<<PINB3) | (0<<PINB3);
    PORTB |= (1<<PINB3) | (1<<PINB4);
    TCCR1A = (1<<WGM10) | (1<<COM1A1);
    TCCR1B = (1<<WGM12) | (1<<CS10);
    DC_VALUE = 128;
    ptr = 6;
    OCR1A = DC_VALUE;
}

void update_PWM_Duty_Cycle(int increment) {
    if(increment)
        if(DC_VALUE == 250)
            break;
        else DC_VALUE = OCR1A_table(++ptr);
    else
        if(DC_VALUE == 5)
            break;
        else DC_VALUE = OCR1A_table(++ptr);
    OCR1A = DC_VALUE;
}

void ADC_init() {
/*
 * INIT ADC
 * MUX[3:0] = 0001 => read from analog input ADC_1 ~ PB1_PWM ~ PC1
 * DIDR0 = 11111101 => Disable all digital inputs except ADC1D
 * ADLAR = 1 => Left adjusted
 * REFS[1:0] = 01 => V_ref = 5V
 * ADEN = 1 => ADC Enable
 * ADSC = 0 => No Conversion
 * ADIE = 0 => disable ADC interrupt
 * ADPS[2:0] = 111 => f_ADC = f_CPU/128 = 125 KHz
 * DDRD[3:0] = 1 => set PORTD[3:0] as output
 */
    DDRC = (0<<PINC1);
    ADMUX = 0b01100001;
    ADCSRA = 0b10000111;
//  ADCSRB = 0x00; => free running => delay = 25 + 13k cycles, k>=0
    DDRD = 0x1F;

}

uint16_t ADC_readAverage() {
    uint16_t sum = 0;
    for(int i=0; i<16; i++) {
        ADCSRA |= (1<<ADSC);
        while(ADCSRA & (1<<ADSC));
        _delay_ms(100);
        sum += ADC;
    }
    return sum >> 4;
}

void LED_control(uint16_t ADCvalue) {
/*
 * clear PIND(0-4)
 * if 0 <= ADCvalue < 200 then PIND0 = 1
 * if 200 <= ADCvalue < 400 then PIND1 = 1
 * if 400 <= ADCvalue < 600 then PIND2 = 1
 * if 600 <= ADCvalue < 800 then PIND3 = 1
 * if 800 <= ADCvalue then PIND4 = 1
 */
    PORTD &= 0xE0;
    if(ADCvalue < 200) PORTD |= (1<<PIND0);
    else if (ADCvalue < 400) PORTD |= (1<<PIND1);
    else if (ADCvalue < 600) PORTD |= (1<<PIND2);
    else if (ADCvalue < 800) PORTD |= (1<<PIND3);
    else PORTD |= (1<<PIND4);
}

int main(void) {
    PWM_init();
    ADC_init();
    while(1) {
        uint16_t ADCvalue = ADC_readAverage();
        LED_control(ADCvalue);
        if(!(PINB & (1 << PINB3))) {
            update_PWM_Duty_Cycle(1);
        }
        if(!(PINB & (1 << PINB4))) {
            update_PWM_Duty_Cycle(0);
        }
    }
    return 0;
}
