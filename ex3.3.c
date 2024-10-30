#include <avr/io.h>
#include <util/delay.h>

// CPU, PWM frequency
#define F_CPU 16000000UL
#define F_PWM 62500

int DC_VALUE;
int OCR1A_table[14] = {5, 25, 45, 65, 86, 107, 128,
                      149, 170, 191, 212, 223, 234, 250};
int ptr;
int mode;

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
    DDRD = (0<<PIND1) | (0<<PIND2) | (0<<PIND6) | (0<<PIND7);
    PORTD |= (1<<PIND1) | (1<<PIND2) | (1<<PIND6) | (1<<PIND7);
    TCCR1A = (1<<WGM10) | (1<<COM1A1);
    TCCR1B = (1<<WGM12) | (1<<CS10);
    DC_VALUE = 128;
    ptr = 6;
    mode = 0;
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

void update_Mode (int mode) {
    if(mode) mode = 0;
    else mode = 1;
}

int main(void) {
    PWM_init();
    while(1) {

        if(!(PIND & (1 << PIND6)))
                update_Mode(0);
        
        if(!(PIND & (1 << PIND7)))
                update_Mode(1);
        
        if(mode) {
        }
        else {
            if(!(PIND & (1 << PIND1)))
                update_PWM_Duty_Cycle(1);

            if(!(PIND & (1 << PIND2)))
                update_PWM_Duty_Cycle(0);
        }
    }
    return 0;
}
