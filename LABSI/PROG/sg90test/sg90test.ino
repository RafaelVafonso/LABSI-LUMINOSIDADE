#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>

#define F_CPU 16000000UL    

void pwm_timer_servo(void){

//20ms periodo, phase-correct
DDRB |= (1 << PB1);
TCCR1A |= (1 << COM1A1) | (1 << WGM11);
TCCR1B |= (1 << WGM13) | (1 << CS11) | (1 << CS10);
ICR1 = 2500;
OCR1A = 188; // posiçao 0 do motor

}
int main(void) {
    pwm_timer_servo();
    
    while (1) {
        OCR1A = 188;
        _delay_ms(1000);

        // Spin right (assuming OCR1A = 125 for full speed one direction)
        OCR1A = 125;
        _delay_ms(2000);
        
        OCR1A = 188;
        _delay_ms(3000);

        // Spin left (assuming OCR1A = 250 for full speed the other direction)
        OCR1A = 250;
        _delay_ms(2000);
    }
    
    return 0;
}
