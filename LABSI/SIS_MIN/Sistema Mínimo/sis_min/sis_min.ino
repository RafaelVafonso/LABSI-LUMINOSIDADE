#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t contador = 0;
uint8_t valor_adc = 0;

ISR(TIMER0_COMPA_vect) {
    contador++;
    if (contador >= 125) { // 125 * 4ms = 0.5s
        PORTD ^= (1 << PD6);
        contador = 0;   
    }
}

uint8_t adc_init(){
    ADMUX = (1 << REFS0) | (1 << ADLAR); // AVcc referência, 8 bits, seleciona ADC0
    ADCSRA |= (1 << ADEN) | (1 << ADSC);// habilita e começa a conversão
    while (ADCSRA & (1 << ADSC));// espera que ADSC fique a 0          
    return ADCH; //lê os 8 bits mais significativos
}

void onda1Hz_init(void) {
    DDRD |= (1 << PD6); //PD6 (Arduino 6)
    PORTD &= ~(1 << PD6);

    // Timer0 CTC mode 
    TCCR0A = (1 << WGM01);   // CTC
    TCCR0B = (1 << CS02) | (1 << CS00); // prescaler 1024

    OCR0A = 63;
    TIMSK0 = (1 << OCIE0A); // liga interrupção
    sei(); // enable global 
}

// Inicializa PWM
void pwm_init(void) {
    DDRD |= (1 << PD3);                   // PD3 como saída (Arduino pin 3)
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); //COM2B1 para OC2B no PD3 (Fast PWM)
    TCCR2B |= (1 << CS21);                // prescaler 8 
    OCR2B = 128;                          //duty cycle inicial 50% on OCR2B
}

void inic(void){
    DDRC &= ~(1 << PC0); // ADC entrada 
    onda1Hz_init();
    pwm_init();
}

int main() {

inic();
    while (1) {
        valor_adc = adc_init();
        OCR2B = valor_adc;  //PWM OCR2B (PD3)
    }
}
