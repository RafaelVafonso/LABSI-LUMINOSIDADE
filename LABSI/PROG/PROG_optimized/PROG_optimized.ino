#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// Definições de constantes (baseadas no datasheet ATmega328P e periféricos)
#define F_CPU 16000000UL  // Frequência do clock (para delays precisos)
#define LCD_I2C_ADDR 0x20  // Endereço I2C do PCF8574T (ajuste se necessário)
#define BH1750_ADDR1 0x23  // Endereço sensor BH1750 #1
#define BH1750_ADDR2 0x5C  // Endereço sensor BH1750 #2
#define BH1750_POWER_ON 0x01
#define BH1750_CONT_HIGH_RES_MODE2 0x11
#define PCF_RS 0b10000000  // Bit RS no PCF8574T
#define PCF_EN 0b01000000  // Bit EN no PCF8574T
#define BOTAO_TOUCH PC1  // Pino do botão touch (PC1)
#define MODE_AUTOMATIC 0
#define MODE_MANUAL 1
#define AVG_SAMPLES 10  // Número de amostras para média móvel

// Enum para posições do servo
typedef enum {
    FECHADA = 0,
    METADE = 1,
    ABERTA = 2,
    LUZ = 3  // Estado extra para futuro uso
} Servo_pos;

// Struct para organizar estado da aplicação (melhora manutenibilidade)
struct AppState {
    volatile uint16_t delay_counter_2ms;  // Contador de atraso não-bloqueante (2ms ticks)
    volatile uint8_t init_state;          // Estado da máquina de inicialização
    volatile uint8_t setup_done;          // Flag: setup concluído
    volatile uint8_t operating_mode;      // Modo atual (automático/manual)
    volatile uint8_t mode_changed;        // Flag para limpar LCD ao mudar modo
    volatile uint8_t prev_pc1_state;      // Estado anterior do botão (para detecção de borda)
    volatile uint8_t mode_locked;         // Bloqueio de mudança de modo
    volatile uint8_t debounce_counter;    // Contador para debouncing (50ms)
    volatile uint8_t flag_2ms;            // Flag de tick de 2ms
    volatile uint8_t display_counter;     // Contador para atualização de display
    uint16_t lux_value;                   // Valor atual de lux
    uint16_t averaged_lux;                // Valor médio de lux
    uint8_t sensor_count;                 // Número de sensores detectados
    uint8_t sensor1_present;              // Flag: sensor 1 presente
    uint8_t sensor2_present;              // Flag: sensor 2 presente
    uint16_t lux_buffer[AVG_SAMPLES];     // Buffer para média móvel
    uint8_t buffer_index;                 // Índice do buffer
    volatile uint8_t estado_atual;        // Estado atual do servo
};

// Instância global da struct (acesso otimizado)
struct AppState app = {0};

// Protótipos de funções (para organização)
void inic(void);
void inic_non_blocking(void);
void update_display(void);
void control_motor(Servo_pos pos_desejada);
uint16_t average_lux(uint16_t new_reading);
void uint_to_str(uint16_t num, char* buffer, uint8_t len);  // Função customizada para converter uint para string (substitui sprintf)

// Funções de atraso crítico (mantidas para timing do LCD, mas otimizadas)
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

// Atraso não-bloqueante baseado em ticks de 2ms
void START_NB_DELAY_MS(uint16_t ms) {
    app.delay_counter_2ms = (ms / 2) + 1;  // Arredonda para cima
}

uint8_t IS_DELAY_FINISHED(void) {
    return (app.delay_counter_2ms == 0);
}

// ISR do Timer0 (tick de 2ms para atrasos não-bloqueantes)
ISR(TIMER0_COMPA_vect) {
    app.flag_2ms = 1;
    if (app.delay_counter_2ms > 0) {
        app.delay_counter_2ms--;
    }
    app.display_counter++;
    // Toggle LED a cada 500ms (250 * 2ms)
    static volatile uint8_t led_counter = 0;
    led_counter++;
    if (led_counter >= 250) {
        PORTD ^= (1 << PD6);
        led_counter = 0;
    }
    wdt_reset();  // Reset watchdog para evitar reset
}

// ISR do botão (com debouncing)
ISR(PCINT1_vect) {
    uint8_t current_state = PINC & (1 << PC1);
    uint8_t prev_state = app.prev_pc1_state & (1 << PC1);

    if ((current_state & (1 << PC1)) && !(prev_state)) {  // Borda de subida
        if (app.debounce_counter == 0) {  // Inicia debouncing
            app.debounce_counter = 25;  // 50ms (25 * 2ms)
        }
    } else if (!(current_state & (1 << PC1)) && (prev_state)) {  // Borda de descida
        app.debounce_counter = 0;  // Cancela debouncing
    }

    app.prev_pc1_state = current_state;
}

// Funções I2C (com verificações de erro básicas para robustez)
void inic_i2c(void) {
    TWCR |= (1 << TWEN);  // Habilita TWI
    TWSR = 0;             // Prescaler 1
    TWBR = 72;            // Baud rate para 100kHz
    DDRC &= ~((1 << PC4) | (1 << PC5));  // Pinos como entrada
    PORTC |= (1 << PC4) | (1 << PC5);    // Pull-ups
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return (TWSR & 0xF8) == 0x28;  // Retorna sucesso (ACK)
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

// Funções LCD via I2C
void i2c_lcd_send(uint8_t data) {
    i2c_start();
    if (!i2c_write(LCD_I2C_ADDR << 1)) return;  // Erro: aborta
    i2c_write(data);
    i2c_stop();
}

void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = 0;
    if (nibble & 0b1000) data |= (1 << 0);  // D7 -> P0
    if (nibble & 0b0100) data |= (1 << 1);  // D6 -> P1
    if (nibble & 0b0010) data |= (1 << 2);  // D5 -> P2
    if (nibble & 0b0001) data |= (1 << 3);  // D4 -> P3
    if (rs) data |= PCF_RS;
    i2c_lcd_send(data | PCF_EN);  // EN high
    i2c_lcd_send(data & ~PCF_EN); // EN low
}

void lcd_write_byte(uint8_t data, uint8_t rs) {
    lcd_write_nibble(data >> 4, rs);
    lcd_write_nibble(data & 0x0F, rs);
}

void lcd_clear(void) {
    lcd_write_byte(0x01, 0);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcd_write_byte(addr, 0);
}

void lcd_write_string(const char* str) {
    while (*str) {
        lcd_write_byte(*str++, 1);
    }
}

void lcd_init_setup_cmds(void) {
    lcd_write_byte(0x28, 0);  // 4-bit, 2 linhas
    lcd_write_byte(0x0C, 0);  // Display on, cursor off
    lcd_write_byte(0x01, 0);  // Clear
    lcd_write_byte(0x06, 0);  // Increment cursor
}

// Funções BH1750 (com retries para robustez)
uint8_t bh1750_send(uint8_t addr, uint8_t data) {
    for (uint8_t retry = 0; retry < 3; retry++) {  // Até 3 retries
        i2c_start();
        if (i2c_write(addr << 1) && i2c_write(data)) {
            i2c_stop();
            return 1;
        }
        i2c_stop();
        delay_us(100);  // Pequena espera antes de retry
    }
    return 0;
}

uint16_t bh1750_read(uint8_t addr) {
    i2c_start();
    if (!i2c_write((addr << 1) | 1)) { i2c_stop(); return 0; }
    uint8_t high = i2c_read_ack();
    uint8_t low = i2c_read_nack();
    i2c_stop();
    uint16_t data = (high << 8) | low;
    return (uint16_t)((float)data / 1.2);  // Conversão para lux
}

void detect_sensors(void) {
    app.sensor1_present = bh1750_send(BH1750_ADDR1, BH1750_POWER_ON) &&
                          bh1750_send(BH1750_ADDR1, BH1750_CONT_HIGH_RES_MODE2);
    app.sensor2_present = bh1750_send(BH1750_ADDR2, BH1750_POWER_ON) &&
                          bh1750_send(BH1750_ADDR2, BH1750_CONT_HIGH_RES_MODE2);
    app.sensor_count = app.sensor1_present + app.sensor2_present;
}

uint16_t bh1750_read_sensors(void) {
    uint16_t lux1 = app.sensor1_present ? bh1750_read(BH1750_ADDR1) : 0;
    uint16_t lux2 = app.sensor2_present ? bh1750_read(BH1750_ADDR2) : 0;
    if (app.sensor_count == 1) return lux1 ? lux1 : lux2;
    if (app.sensor_count == 2) return (lux1 + lux2) / 2;
    return 0;
}

// Média móvel otimizada (soma acumulativa para eficiência)
uint16_t average_lux(uint16_t new_reading) {
    static uint32_t sum = 0;  // Soma acumulativa
    sum -= app.lux_buffer[app.buffer_index];
    app.lux_buffer[app.buffer_index] = new_reading;
    sum += new_reading;
    app.buffer_index = (app.buffer_index + 1) % AVG_SAMPLES;
    return sum / AVG_SAMPLES;
}

// Função customizada para converter uint16_t para string (evita sprintf)
void uint_to_str(uint16_t num, char* buffer, uint8_t len) {
    char temp[6];  // Suficiente para 65535
    uint8_t i = 0;
    if (num == 0) {
        temp[i++] = '0';
    } else {
        while (num > 0 && i < 5) {
            temp[i++] = '0' + (num % 10);
            num /= 10;
        }
    }
    // Inverte e copia para buffer
    for (uint8_t j = 0; j < i && j < len - 1; j++) {
        buffer[j] = temp[i - 1 - j];
    }
    buffer[i < len ? i : len - 1] = '\0';
}

// Inicializações de hardware
void onda1Hz_init(void) {
    DDRD |= (1 << PD6);  // LED como output
    TCCR0A = (1 << WGM01);  // CTC
    TCCR0B = (1 << CS02) | (1 << CS00);  // Prescaler 1024
    OCR0A = 31;  // 500Hz (2ms)
    TIMSK0 = (1 << OCIE0A);
}

void pwm_Servo_init(void) {
    DDRB |= (1 << PB1);  // Servo como output
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << CS11) | (1 << CS10);
    ICR1 = 2500;  // Período 20ms (50Hz)
    OCR1A = 188;  // Posição inicial
}

void button_inic(void) {
    DDRC &= ~(1 << BOTAO_TOUCH);  // Input
    PORTC |= (1 << PC1);           // Pull-up
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT9);
}

void inic(void) {
    onda1Hz_init();
    pwm_Servo_init();  // Adicionado: inicializa servo
    button_inic();
    wdt_enable(WDTO_2S);  // Watchdog 2s para detectar hangs
    sei();  // Habilita interrupções
}

// Controle do motor (com validação)
void control_motor(Servo_pos pos_desejada) {
    if (pos_desejada >= LUZ) return;  // Validação
    uint16_t valor_pwm;
    switch (pos_desejada) {
        case FECHADA: valor_pwm = 125; app.estado_atual = 0; break;
        case METADE: valor_pwm = 188; app.estado_atual = 1; break;
        case ABERTA: valor_pwm = 250; app.estado_atual = 2; break;
    }
    OCR1A = valor_pwm;
}

// Máquina de estados de inicialização (não-bloqueante)
void inic_non_blocking(void) {
    switch (app.init_state) {
        case 0:
            inic_i2c();
            START_NB_DELAY_MS(50);  // Power-on wait
            app.init_state = 1;
            break;
        case 1:
            if (IS_DELAY_FINISHED()) {
                lcd_write_nibble(0x03, 0);
                START_NB_DELAY_MS(5);
                app.init_state = 2;
            }
            break;
        case 2:
            if (IS_DELAY_FINISHED()) {
                lcd_write_nibble(0x03, 0);
                START_NB_DELAY_MS(1);
                app.init_state = 3;
            }
            break;
        case 3:
            if (IS_DELAY_FINISHED()) {
                lcd_write_nibble(0x03, 0);
                START_NB_DELAY_MS(1);
                app.init_state = 4;
            }
            break;
        case 4:
            if (IS_DELAY_FINISHED()) {
                lcd_write_nibble(0x02, 0);  // 4-bit mode
                lcd_init_setup_cmds();      // Comandos finais
                lcd_set_cursor(0, 0);
                lcd_write_string("Projeto LABSI");
                START_NB_DELAY_MS(1500);
                app.init_state = 5;
            }
            break;
        case 5:
            if (IS_DELAY_FINISHED()) {
                lcd_clear();
                detect_sensors();
                START_NB_DELAY_MS(10);
                app.init_state = 6;
            }
            break;
        case 6:
            if (IS_DELAY_FINISHED()) {
                lcd_set_cursor(0, 0);
                if (app.sensor_count == 0) lcd_write_string("Sem Sensores");
                else if (app.sensor_count == 1) lcd_write_string("1 Sensor Pronto");
                else lcd_write_string("2 Sensores Prontos");
                START_NB_DELAY_MS(1500);
                app.init_state = 7;
            }
            break;
        case 7:
            if (IS_DELAY_FINISHED()) {
                lcd_clear();
                app.setup_done = 1;  // Setup concluído
                app.init_state = 255;  // Estado final
            }
            break;
    }
}

// ... (código anterior permanece igual)

// Função para atualizar display (separada para clareza)
void update_display(void) {
    char buffer[16];  // Buffer para strings
    app.lux_value = bh1750_read_sensors();
    app.averaged_lux = average_lux(app.lux_value);

    lcd_set_cursor(0, 0);
    uint_to_str(app.averaged_lux, buffer, 5);  // Converte lux para string
    lcd_write_string("Lux: ");
    lcd_write_string(buffer);
    lcd_write_string("   ");  // Espaços para limpar caracteres antigos

    lcd_set_cursor(1, 0);
    uint_to_str(app.sensor_count, buffer, 2);  // Converte sensor_count para string
    lcd_write_string("Sensors: ");
    lcd_write_string(buffer);
}

// Loop principal
int main(void) {
    inic();  // Inicializa hardware

    while (1) {
        // 1. GESTÃO DA INICIALIZAÇÃO (NON-BLOCKING)
        if (!app.setup_done) {
            inic_non_blocking();
        } else {
            // 2. CÓDIGO OPERACIONAL PRINCIPAL (SÓ EXECUTA APÓS SETUP)
            if (app.flag_2ms) {
                app.flag_2ms = 0;

                // Processa debouncing do botão
                if (app.debounce_counter > 0) {
                    app.debounce_counter--;
                    if (app.debounce_counter == 0) {
                        // Toggle modo após debouncing
                        if (!app.mode_locked) {
                            app.operating_mode = (app.operating_mode == MODE_AUTOMATIC) ? MODE_MANUAL : MODE_AUTOMATIC;
                            app.mode_changed = 1;
                            app.mode_locked = 1;
                        }
                    }
                }

                // Atualização do Display/Leitura de Sensor a cada 200ms (100 * 2ms)
                if (app.display_counter >= 100) {
                    app.display_counter = 0;
                    update_display();  // Chama função separada
                }
            }

            // Controle do servo baseado no modo (exemplo simples; expanda conforme necessário)
            if (app.operating_mode == MODE_AUTOMATIC) {
                // Lógica automática: ajuste baseado em lux (ex.: se lux > 500, abre)
                if (app.averaged_lux > 500) {
                    control_motor(ABERTA);
                } else if (app.averaged_lux < 200) {
                    control_motor(FECHADA);
                } else {
                    control_motor(METADE);
                }
            }  // Modo manual: aguarda input adicional (não implementado aqui)

            // Modo sleep para economia de energia
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_enable();
            sleep_cpu();
            sleep_disable();
        }
    }
    return 0;
}
