#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 16000000UL  
#define LCD_I2C_ADDR 0x20  // Endereço I2C do PCF8574T (ajuste se necessário)

#define BH1750_ADDR1 0x23  
#define BH1750_ADDR2 0x5C 
#define BH1750_POWER_ON 0x01
#define BH1750_CONT_HIGH_RES_MODE2 0x11  

// Mapeamento PCF8574T (P7=RS, P6=EN, P3=D4, P2=D5, P1=D6, P0=D7)
#define PCF_RS 0b10000000 
#define PCF_EN 0b01000000 

#define MODE_AUTOMATIC 0
#define MODE_MANUAL 1


#define BOTAO_TOUCH PC1

// --- VARIÁVEIS DE CONTROLO DE TEMPO E ESTADO (NON-BLOCKING) ---
volatile uint16_t g_delay_counter_2ms = 0; // Contador principal, decrementa a cada 2ms
volatile uint8_t g_init_state = 0;       // Máquina de estados para inicialização
volatile uint8_t g_setup_done = 0;       // Flag: 1 quando setup concluído
volatile uint8_t g_operating_mode = MODE_AUTOMATIC; 
volatile uint8_t g_mode_changed = 1;      // Flag para limpar LCD
volatile uint8_t g_prev_pc1_state = 0;    // Para detetar a borda do botão
volatile uint8_t g_mode_locked = 0;       // Bloqueio após 1º toque

volatile uint8_t g_contador = 0;
volatile char g_flag_2ms = 0;
volatile uint8_t g_display_counter = 0;

uint16_t g_lux_value = 0;
uint16_t g_servo_pwm_value = 188;

uint8_t g_sensor_count = 0;  
uint8_t g_sensor1_present = 0;
uint8_t g_sensor2_present = 0;

#define AVG_SAMPLES 10
uint16_t lux_buffer[AVG_SAMPLES] = {0};
uint8_t g_buffer_index = 0;
uint16_t g_averaged_lux = 0;

// --- FUNÇÕES DE ATRASO CRÍTICO (us) ---
// NOTA: Estas são mantidas para garantir o timing do pulso EN do LCD (requisito de hardware).
void short_delay(void) {  
   __asm__ volatile ("nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
"nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
"nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
"nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t" ::: "memory");
}

void delay_us(uint16_t us) { 
   for (uint16_t i = 0; i < us; i++) {
   for (uint8_t j = 0; j < 4; j++) {
   short_delay();
   }
   }
}

// ---  ATRASO NON-BLOCKING (BASEADO EM 2MS) ---
// Inicia um atraso dado em milissegundos. Arredonda para cima para garantir o mínimo.
void START_NB_DELAY_MS(uint16_t ms) {
g_delay_counter_2ms = (ms / 2) + 1;
 }

// Retorna 1 se o atraso terminar
uint8_t IS_DELAY_FINISHED() {
   if(g_delay_counter_2ms == 0) return 1;
   else return 0;
}

// --- Timer ISR (runs every 2ms) ---
ISR(TIMER0_COMPA_vect){
   g_flag_2ms = 1;
   
   if (g_delay_counter_2ms > 0) {
        g_delay_counter_2ms--;
    }
    
   g_contador++;
   if (g_contador >= 250) { 
   PORTD ^= (1 << PD6); 
   g_contador = 0;   
   }
   g_display_counter++;
}

ISR(PCINT1_vect) {

    uint8_t current_state = PINC & (1 << PC1); 
    uint8_t g_prev_pc1_state = 0;
    if (current_state > g_prev_pc1_state) {
        // Transição de 0 para 1 detectada 
        
        if (!g_mode_locked) {
            // Alternar Modo (apenas uma vez)
            if (g_operating_mode == MODE_AUTOMATIC) {
                g_operating_mode = MODE_MANUAL;
            } else {
                g_operating_mode = MODE_AUTOMATIC;
            }
            g_mode_changed = 1; // Sinalizar ao main loop
            g_mode_locked = 1;  // Bloquear mais mudanças
        }
    } 
    
    else if (current_state < g_prev_pc1_state) {
        // Transição de 1 para 0 detectada
        g_mode_locked = 0; // Desbloquear sistema para o próximo toque
    }
    
    // 3. Atualizar o estado anterior para a próxima comparação
    g_prev_pc1_state = current_state;
}

// --- I2C Functions ---
void inic_i2c(void){
   TWCR |= (1<< TWEN); 
   TWSR = 0;
   TWBR =72;
   DDRC &= ~((1 << PC4) | (1 << PC5)); 
   PORTC |= (1 << PC4) | (1 << PC5);
}

void i2c_start(void){
   TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
   while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
   TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_write(uint8_t data) {
   TWDR = data;
   TWCR = (1 << TWINT) | (1 << TWEN);
   while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack(void) {
   TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
   while (!(TWCR & (1 << TWINT)));
   return TWDR;
}

uint8_t i2c_read_nack(void) {
   TWCR = (1 << TWINT) | (1 << TWEN);
   while (!(TWCR & (1 << TWINT)));
   return TWDR;
}

uint8_t i2c_get_status(void){
   return TWSR & 0b11111000;
}

// --- LCD Functions ---
void i2c_lcd_send(uint8_t data) {
   i2c_start();
   i2c_write(LCD_I2C_ADDR << 1); 
   i2c_write(data);
   i2c_stop();
}

void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
   uint8_t data = 0;
   
   if (nibble & 0b1000) data |= (1 << 0); // D7 -> P0
   if (nibble & 0b0100) data |= (1 << 1); // D6 -> P1
   if (nibble & 0b0010) data |= (1 << 2); // D5 -> P2
   if (nibble & 0b0001) data |= (1 << 3); // D4 -> P3

   if (rs) data |= PCF_RS; 

   // EN high
   i2c_lcd_send(data | PCF_EN);
   delay_us(200); 

   // EN low
   i2c_lcd_send(data & ~PCF_EN);
   delay_us(200);
}

void lcd_write_byte(uint8_t data, uint8_t rs) {
   lcd_write_nibble(data >> 4, rs);  // High bits
   lcd_write_nibble(data & 0b00001111, rs);  // Low bits
}

void lcd_clear(void) {
   lcd_write_byte(0b00000001, 0); 
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
   uint8_t addr;
   if (row == 0) {
   addr = 0b10000000 + col;
    } else { 
   addr = 0b11000000 + col;
    }
    lcd_write_byte(addr, 0);
}

void lcd_write_string(const char* str) {
   while (*str) {
   lcd_write_byte(*str++, 1);
   }
}

void lcd_init_setup_cmds(void) {
   lcd_write_byte(0b00101000, 0);  // Function set: 4-bit, 2 lines, 5x8 dots
   lcd_write_byte(0b00001100, 0);  // Display on, cursor off, blink off
   lcd_write_byte(0b00000001, 0);  // Clear Display
   lcd_write_byte(0b00000110, 0);  // Entry mode: Increment cursor, no display shift
}


// --- BH1750 Functions ---
uint8_t bh1750_send(uint8_t addr, uint8_t data) {
   i2c_start();
   i2c_write(addr << 1 | 0); 
   if (i2c_get_status() != 0x18){ //SLA+W
   i2c_stop();
   return 0;  
   }
   i2c_write(data);
   if (i2c_get_status() != 0x28){//DATA+ACK
   i2c_stop();
   return 0;  
   }
   i2c_stop();
   return 1; 
}

uint16_t bh1750_read(uint8_t addr){
   uint8_t high_byte, low_byte;
   i2c_start();
   if (i2c_get_status() != 0x08){
   i2c_stop();
   return 0;  
   }
   i2c_write((addr << 1) | 1); 
   if (i2c_get_status() != 0x40){
   i2c_stop();
   return 0;  
   }
   high_byte = i2c_read_ack();
   low_byte = i2c_read_nack();
   i2c_stop();
   uint16_t data = (high_byte << 8) | low_byte;
   return (uint16_t)((float)data / 1.2);
}

void detect_sensors(void){
   g_sensor1_present = bh1750_send(BH1750_ADDR1, BH1750_POWER_ON) && bh1750_send(BH1750_ADDR1, BH1750_CONT_HIGH_RES_MODE2);
   // delay_us(1) substituído por chamada sequencial
   g_sensor2_present = bh1750_send(BH1750_ADDR2, BH1750_POWER_ON) && bh1750_send(BH1750_ADDR2, BH1750_CONT_HIGH_RES_MODE2);
   g_sensor_count = g_sensor1_present + g_sensor2_present;
}

uint16_t bh1750_read_sensors(void){
   uint16_t lux1 = 0, lux2 = 0;
   if (g_sensor1_present) lux1 = bh1750_read(BH1750_ADDR1);
   if (g_sensor2_present) lux2 = bh1750_read(BH1750_ADDR2);
   if (g_sensor_count == 1) {
   if (lux1 > 0) return lux1;
   else return lux2;
   }
   if (g_sensor_count == 2) return (uint16_t)((float)(lux1 + lux2)) / 2;
   return 0;
}

// --- Averaging Function ---
uint16_t average_lux(uint16_t new_reading) {
   lux_buffer[g_buffer_index] = new_reading;
   g_buffer_index = (g_buffer_index + 1) % AVG_SAMPLES;
   uint32_t sum = 0;
   for (uint8_t i = 0; i < AVG_SAMPLES; i++) {
   sum += lux_buffer[i];
   }
   return sum / AVG_SAMPLES;
}

// --- Timer 0 for 2ms interrupt ---
void onda1Hz_init(void) {
   DDRD |= (1 << PD6);
   PORTD &= ~(1 << PD6);
   TCCR0A = (1 << WGM01); // CTC Mode
   TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler 1024
   OCR0A = 31; // (16M / 1024) / (31+1) = 500 Hz (2ms)
   TIMSK0 = (1 << OCIE0A);
}

void pwm_Servo_init(void){
   DDRB |= (1 << PB1);
   TCCR1A |= (1 << COM1A1) | (1 << WGM11);
   TCCR1B |= (1 << WGM13) | (1 << CS11) | (1 << CS10);
   ICR1 = 2500;
   OCR1A = 188;
}
void button_inic(void){
   DDRC &= ~(1 << BOTAO_TOUCH);
   PORTC &= ~(1 << PC1);
   PCICR |= (1 << PCIE1);
   PCMSK1 |= (1 << PCINT9);
}
// --- FUNÇÃO DE INICIALIZAÇÃO DE HARDWARE (Rápida) ---
void inic(void) {
   onda1Hz_init();
   button_inic();
   // pwm_Servo_init(); // Manter ou remover conforme o necessário
   sei(); // LIGA INTERRUPÇÕES
}

// --- MÁQUINA DE ESTADOS PRINCIPAL (NON-BLOCKING) ---
void inic_non_blocking(void) {
    switch(g_init_state) {
        // ESTADO 0: Inicia I2C e Delay de Power-on
        case 0:
            inic_i2c();
            START_NB_DELAY_MS(50); // Power-on wait (50ms)
            g_init_state = 1;
            break;
            
        // ESTADO 1: Comandos de Inicialização 4-bit (1/4)
        case 1:
            if (IS_DELAY_FINISHED() == 1) {
                lcd_write_nibble(0x03, 0);
                START_NB_DELAY_MS(5); // Espera 5ms 
                g_init_state = 2;
            }
            break;

        // ESTADO 2: Comandos de Inicialização 4-bit (2/4)
        case 2:
            if (IS_DELAY_FINISHED() == 1) {
                lcd_write_nibble(0x03, 0);
                START_NB_DELAY_MS(1); // Espera 1ms 
                g_init_state = 3;
            }
            break;

        // ESTADO 3: Comandos de Inicialização 4-bit (3/4)
        case 3:
            if (IS_DELAY_FINISHED() == 1) {
                lcd_write_nibble(0x03, 0);
                START_NB_DELAY_MS(1); // Espera 1ms 
                g_init_state = 4;
            }
            break;

        // ESTADO 4: Define modo 4-bit e comandos de setup (4/4)
        case 4:
            if (IS_DELAY_FINISHED() == 1) {
                lcd_write_nibble(0x02, 0); // 4-bit mode
                lcd_init_setup_cmds(); // Comandos finais: Function Set, Display On, Clear, Entry Mode
                
                lcd_set_cursor(0, 0);
                lcd_write_string("Lux Meter V2.0");
                START_NB_DELAY_MS(1500); // Espera 1.5s
                g_init_state = 5;
            }
            break;

        // ESTADO 5: Inicia Sensores BH1750
        case 5:
            if (IS_DELAY_FINISHED() == 1) {
                lcd_clear();
                detect_sensors(); 
                START_NB_DELAY_MS(10); // Pequena espera de segurança
                g_init_state = 6;
            }
            break;

        // ESTADO 6: Display de status do sensor
        case 6:
            if (IS_DELAY_FINISHED() == 1) {
                lcd_set_cursor(0, 0);
                if (g_sensor_count == 0) lcd_write_string("No Sensor");
                else if (g_sensor_count == 1) lcd_write_string("1 Sensor Ready");
                else lcd_write_string("2 Sensors Ready");
                
                START_NB_DELAY_MS(1500); // Espera 1.5s
                g_init_state = 7;
            }
            break;

        // ESTADO 7: Fim da inicialização
        case 7:
            if (IS_DELAY_FINISHED() == 1) {
                lcd_clear();
                g_setup_done = 1; // Tudo concluído!
                g_init_state = 255; // Estado final
            }
            break;
    }
}


// --- LOOP PRINCIPAL ---
int main(void) {
   inic();
   char buffer[32];
    
   while (1) {
        
        // 1. GESTÃO DA INICIALIZAÇÃO (NON-BLOCKING)
        if (!g_setup_done) {
            inic_non_blocking();
        } 
        
        // 2. CÓDIGO OPERACIONAL PRINCIPAL (SÓ EXECUTA APÓS SETUP)
        else {
            if (g_flag_2ms) {
               g_flag_2ms = 0;
 
               // Atualização do Display/Leitura de Sensor a cada 200ms (100 * 2ms)
               if (g_display_counter >= 100) { 
               g_display_counter = 0;

               g_lux_value = bh1750_read_sensors();
               g_averaged_lux = average_lux(g_lux_value);

               lcd_set_cursor(0, 0);
               sprintf(buffer, "Lux: %4u  ", g_averaged_lux);
               lcd_write_string(buffer);

               lcd_set_cursor(1, 0);
               sprintf(buffer, "Sensors: %d", g_sensor_count);
               lcd_write_string(buffer);
               }
            }
        }
   }
   return 0;
}