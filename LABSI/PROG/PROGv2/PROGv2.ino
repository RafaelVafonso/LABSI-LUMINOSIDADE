#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#define F_CPU 16000000UL
#define BH1750_ADDR1 0x23
#define BH1750_ADDR2 0x5C
#define BH1750_POWER_ON 0x01
#define BH1750_CONT_HIGH_RES_MODE2 0x11

#define SSD1306_ADDR 0x3C
char last_line0[20] = "";
char last_line1[20] = "";
char last_line2[20] = "";
char last_line3[20] = "";
uint8_t last_bar_width = 255;

#define MODE_AUTOMATIC 0
#define MODE_MANUAL 1
#define BOTAO_TOUCH PC1
#define BOTAO_MAIS PD4 // Botão Aumentar
#define BOTAO_MENOS PD7 // Botão Diminuir
#define BUZZER_PIN PC0 // Buzzer (PC0)

// --- CONFIGURAÇÃO UART (ESP32) ---
#define BAUD 9600
#define UBRR_VAL ((F_CPU/16/BAUD)-1)

#define SONAR1_TRIG PC2
#define SONAR1_ECHO PD2
#define SONAR2_TRIG PC3
#define SONAR2_ECHO PD3
// 20 cm ~ 1156 us de echo -> com Timer1 a 4 us/tick => ~290 ticks
#define SONAR_THRESHOLD_TICKS_20CM 1000
#define LUX_STEP 50
#define LUX_MAX 1000
#define LUX_MIN 0
#define LUX_BAND_ERROR 40
#define I2C_TIMEOUT_CYCLES 2000
#define FIX_TIME_TICKS 1000
#define DEBOUNCE_TICKS 10

#define GESTURE_TIMEOUT_TICKS 500

#define BLIND1_MAX_TICKS 60000
#define BLIND2_MAX_TICKS 60000
#define STEP_CLOSE 2           // Velocidade descer
#define STEP_OPEN 2            // Velocidade subir (compensação gravidade)
#define LED_FADE_INTERVAL 100

#define GATE_PIN PB3
#define MAX_LED_PWM 255
// --- VARIÁVEIS DE CONTROLO DE TEMPO E ESTADO ---
volatile uint16_t g_delay_counter_2ms = 0; // Contador principal, decrementa a cada 2ms
volatile uint8_t g_init_state = 0; // Máquina de estados para inicialização
volatile uint8_t g_setup_done = 0; // Flag: 1 quando setup concluído
volatile uint8_t g_operating_mode = MODE_MANUAL;
volatile uint8_t g_mode_changed = 1; // Flag para limpar LCD
volatile uint8_t g_prev_pc1_state = 0; // Para detetar a borda do botão
volatile uint8_t g_mode_locked = 0; // Bloqueio após 1º toque
volatile uint16_t g_target_lux = 50;
volatile uint8_t g_contador = 0;
volatile char g_flag_2ms = 0;
volatile char g_flag_50ms = 0;
volatile uint8_t g_contador_50ms = 0;
volatile uint8_t g_display_counter = 0;
volatile uint16_t g_last_setpoint_value = 100;
volatile uint8_t g_buzzer_counter = 0;
volatile uint8_t g_fix_state = 0; // 0: Estável, 1: Ajuste Ativo, 2: Contagem (2s)
volatile uint16_t g_fix_timer = 0; // Timer de 2 segundos
// Gesto com sonares (modo manual)
volatile uint8_t g_gesture_state = 0; // 0: idle, 1: S1 primeiro, 2: S2 primeiro
volatile uint16_t g_gesture_timer = 0; // timeout do gesto
volatile uint32_t g_millis = 0;


uint8_t g_up_debounce = 0;
uint8_t g_down_debounce = 0;
uint16_t g_lux_value = 0;

uint16_t g_servo_pwm_value = 188;
uint8_t g_led_brightness = 0;

uint8_t g_sensor_count = 0;
uint8_t g_sensor1_present = 0;
uint8_t g_sensor2_present = 0;
#define AVG_SAMPLES 10
uint16_t lux_buffer[AVG_SAMPLES] = {0};
uint8_t g_buffer_index = 0;
uint16_t g_averaged_lux = 0;

typedef enum {
	FECHAR = 0,
	PARAR = 1,
	ABRIR = 2,
}
Servo_pos;
Servo_pos g_motor1_action = PARAR;
Servo_pos g_motor2_action = PARAR;


// Posições virtuais independentes
int32_t g_blind1_pos = 0; 
int32_t g_blind2_pos = 0;
int16_t g_debug_ciclos = 0;

void control_motor_1(Servo_pos action);
void control_motor_2(Servo_pos action);

uint8_t sonar_is_close(uint8_t sonar_id);
void update_manual_gestures(void);

const uint8_t font6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, // 32  ' '
    {0x00,0x00,0x5F,0x00,0x00,0x00}, // 33  '!'
    {0x00,0x07,0x00,0x07,0x00,0x00}, // 34  '"'
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, // 35  '#'
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, // 36  '$'
    {0x23,0x13,0x08,0x64,0x62,0x00}, // 37  '%'
    {0x36,0x49,0x55,0x22,0x50,0x00}, // 38  '&'
    {0x00,0x05,0x03,0x00,0x00,0x00}, // 39  '''
    {0x00,0x1C,0x22,0x41,0x00,0x00}, // 40  '('
    {0x00,0x41,0x22,0x1C,0x00,0x00}, // 41  ')'
    {0x14,0x08,0x3E,0x08,0x14,0x00}, // 42  '*'
    {0x08,0x08,0x3E,0x08,0x08,0x00}, // 43  '+'
    {0x00,0x50,0x30,0x00,0x00,0x00}, // 44  ','
    {0x08,0x08,0x08,0x08,0x08,0x00}, // 45  '-'
    {0x00,0x60,0x60,0x00,0x00,0x00}, // 46  '.'
    {0x20,0x10,0x08,0x04,0x02,0x00}, // 47  '/'
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, // 48  '0'
    {0x00,0x42,0x7F,0x40,0x00,0x00}, // 49  '1'
    {0x42,0x61,0x51,0x49,0x46,0x00}, // 50  '2'
    {0x21,0x41,0x45,0x4B,0x31,0x00}, // 51  '3'
    {0x18,0x14,0x12,0x7F,0x10,0x00}, // 52  '4'
    {0x27,0x45,0x45,0x45,0x39,0x00}, // 53  '5'
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, // 54  '6'
    {0x01,0x71,0x09,0x05,0x03,0x00}, // 55  '7'
    {0x36,0x49,0x49,0x49,0x36,0x00}, // 56  '8'
    {0x06,0x49,0x49,0x29,0x1E,0x00}, // 57  '9'
    {0x00,0x36,0x36,0x00,0x00,0x00}, // 58  ':'
    {0x00,0x56,0x36,0x00,0x00,0x00}, // 59  ';'
    {0x08,0x14,0x22,0x41,0x00,0x00}, // 60  '<'
    {0x14,0x14,0x14,0x14,0x14,0x00}, // 61  '='
    {0x00,0x41,0x22,0x14,0x08,0x00}, // 62  '>'
    {0x02,0x01,0x51,0x09,0x06,0x00}, // 63  '?'
    {0x3E,0x41,0x5D,0x59,0x4E,0x00}, // 64  '@'
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, // 65  'A'
    {0x7F,0x49,0x49,0x49,0x36,0x00}, // 66  'B'
    {0x3E,0x41,0x41,0x41,0x22,0x00}, // 67  'C'
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, // 68  'D'
    {0x7F,0x49,0x49,0x49,0x41,0x00}, // 69  'E'
    {0x7F,0x09,0x09,0x09,0x01,0x00}, // 70  'F'
    {0x3E,0x41,0x49,0x49,0x7A,0x00}, // 71  'G'
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, // 72  'H'
    {0x00,0x41,0x7F,0x41,0x00,0x00}, // 73  'I'
    {0x20,0x40,0x41,0x3F,0x01,0x00}, // 74  'J'
    {0x7F,0x08,0x14,0x22,0x41,0x00}, // 75  'K'
    {0x7F,0x40,0x40,0x40,0x40,0x00}, // 76  'L'
    {0x7F,0x02,0x0C,0x02,0x7F,0x00}, // 77  'M'
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, // 78  'N'
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, // 79  'O'
    {0x7F,0x09,0x09,0x09,0x06,0x00}, // 80  'P'
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, // 81  'Q'
    {0x7F,0x09,0x19,0x29,0x46,0x00}, // 82  'R'
    {0x46,0x49,0x49,0x49,0x31,0x00}, // 83  'S'
    {0x01,0x01,0x7F,0x01,0x01,0x00}, // 84  'T'
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, // 85  'U'
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, // 86  'V'
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, // 87  'W'
    {0x63,0x14,0x08,0x14,0x63,0x00}, // 88  'X'
    {0x07,0x08,0x70,0x08,0x07,0x00}, // 89  'Y'
    {0x61,0x51,0x49,0x45,0x43,0x00}, // 90  'Z'
    {0x00,0x7F,0x41,0x41,0x00,0x00}, // 91  '['
    {0x02,0x04,0x08,0x10,0x20,0x00}, // 92  '\'
    {0x00,0x41,0x41,0x7F,0x00,0x00}, // 93  ']'
    {0x04,0x02,0x01,0x02,0x04,0x00}, // 94  '^'
    {0x80,0x80,0x80,0x80,0x80,0x00}, // 95  '_'
    {0x00,0x03,0x05,0x00,0x00,0x00}, // 96  '`'
    {0x20,0x54,0x54,0x54,0x78,0x00}, // 97  'a'
    {0x7F,0x48,0x44,0x44,0x38,0x00}, // 98  'b'
    {0x38,0x44,0x44,0x44,0x20,0x00}, // 99  'c'
    {0x38,0x44,0x44,0x48,0x7F,0x00}, // 100 'd'
    {0x38,0x54,0x54,0x54,0x18,0x00}, // 101 'e'
    {0x08,0x7E,0x09,0x01,0x02,0x00}, // 102 'f'
    {0x0C,0x52,0x52,0x52,0x3E,0x00}, // 103 'g'
    {0x7F,0x08,0x04,0x04,0x78,0x00}, // 104 'h'
    {0x00,0x44,0x7D,0x40,0x00,0x00}, // 105 'i'
    {0x20,0x40,0x44,0x3D,0x00,0x00}, // 106 'j'
    {0x7F,0x10,0x28,0x44,0x00,0x00}, // 107 'k'
    {0x00,0x41,0x7F,0x40,0x00,0x00}, // 108 'l'
    {0x7C,0x04,0x18,0x04,0x7C,0x00}, // 109 'm'
    {0x7C,0x08,0x04,0x04,0x78,0x00}, // 110 'n'
    {0x38,0x44,0x44,0x44,0x38,0x00}, // 111 'o'
    {0x7C,0x14,0x14,0x14,0x08,0x00}, // 112 'p'
    {0x08,0x14,0x14,0x14,0x7C,0x00}, // 113 'q'
    {0x7C,0x08,0x04,0x04,0x08,0x00}, // 114 'r'
    {0x48,0x54,0x54,0x54,0x20,0x00}, // 115 's'
    {0x04,0x3F,0x44,0x40,0x20,0x00}, // 116 't'
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, // 117 'u'
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, // 118 'v'
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, // 119 'w'
    {0x44,0x28,0x10,0x28,0x44,0x00}, // 120 'x'
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, // 121 'y'
    {0x44,0x64,0x54,0x4C,0x44,0x00}, // 122 'z'
    {0x00,0x08,0x36,0x41,0x00,0x00}, // 123 '{'
    {0x00,0x00,0x7F,0x00,0x00,0x00}, // 124 '|'
    {0x00,0x41,0x36,0x08,0x00,0x00}, // 125 '}'
    {0x08,0x04,0x08,0x10,0x08,0x00}  // 126 '~'
};
// --- FUNÇÕES DE ATRASO CRÍTICO (us) ---
void short_delay(void) {
	__asm__ volatile("nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"::: "memory");
}
void delay_us(uint16_t us) {
	for(uint16_t i = 0; i < us; i++) {
		for(uint8_t j = 0; j < 4; j++) {
			short_delay();
		}
	}
}
// --- ATRASO NON-BLOCKING (BASEADO EM 2MS) ---
void START_NB_DELAY_MS(uint16_t ms) {
	g_delay_counter_2ms = (ms / 2) + 1;
}
uint8_t IS_DELAY_FINISHED() {
	if(g_delay_counter_2ms == 0) return 1;
	else return 0;
}
void buzzer_bips(uint8_t num_bips) {
	if(g_buzzer_counter == 0) {
		// Cada bip dura 400ms (200 ticks * 2ms)
		g_buzzer_counter = num_bips * 200;
	}
}
// --- Timer ISR (2ms) ---
ISR(TIMER0_COMPA_vect) {
	g_flag_2ms = 1;
	g_millis += 2;
    
	if(g_delay_counter_2ms > 0) g_delay_counter_2ms--;
	if(g_fix_state == 2 && g_fix_timer > 0) g_fix_timer--;
	if(g_gesture_timer > 0) g_gesture_timer--; 	// Timer de gesto (sonares)	
}
ISR(PCINT1_vect) {
	uint8_t current_state = PINC & (1 << PC1);
	if(current_state > g_prev_pc1_state) {
		// Transição de 0 para 1 detectada
		if(!g_mode_locked) {
			// Alternar Modo (apenas uma vez)
			if(g_operating_mode == MODE_AUTOMATIC) {
				g_operating_mode = MODE_MANUAL;
			} else {
				g_operating_mode = MODE_AUTOMATIC;
			}
			g_mode_changed = 1; // Sinalizar ao main loop
			control_motor_1(1);
			control_motor_2(1);
			buzzer_bips(1); // 1 bip na troca de modo
			g_mode_locked = 1; // Bloquear mais mudanças
		}
	} else if(current_state < g_prev_pc1_state) {
		// Transição de 1 para 0 detectada
		g_mode_locked = 0; // Desbloquear sistema para o próximo toque
	}
	g_prev_pc1_state = current_state;
}
// --- UART (ESP32) ---
void uart_init(void) {
    // Configura Baud Rate (9600)
    UBRR0H = (unsigned char)(UBRR_VAL >> 8);
    UBRR0L = (unsigned char)UBRR_VAL;
    // Habilita RX e TX
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Formato: 8 dados, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void uart_write(char c) {
    // Espera até que o registrador de dados esteja vazio
    while (!(UCSR0A & (1 << UDRE0))); // espera
    UDR0 = c;
}
void uart_write_string(const char* str) {
    while (*str) {
        uart_write(*str++); 
    }
}
uint8_t uart_available(void) {
    return (UCSR0A & (1 << RXC0));
}
char uart_read(void) {
		while (!(UCSR0A & (1<<RXC0)));
    return UDR0;
}
void check_uart_cmds(void) {
    if (uart_available()) {
        char cmd = uart_read();
        switch(cmd) {
            case 'A': if(g_operating_mode!=0){g_operating_mode=0;g_mode_changed=1;buzzer_bips(1);control_motor_1(PARAR);control_motor_2(PARAR);} break;
            case 'M': if(g_operating_mode!=1){g_operating_mode=1;g_mode_changed=1;buzzer_bips(1);control_motor_1(PARAR);control_motor_2(PARAR);set_led_brightness(0);} break;
            case 'U': if(g_operating_mode==0){
							if(g_target_lux<LUX_MAX)g_target_lux+=LUX_STEP;
							g_last_setpoint_value=g_target_lux;
							}else{
								if(g_led_brightness<255)set_led_brightness(g_led_brightness+10);
								else set_led_brightness(255);}
								break;
            case 'D': if(g_operating_mode==0){if(g_target_lux>LUX_MIN)g_target_lux-=LUX_STEP;g_last_setpoint_value=g_target_lux;}else{if(g_led_brightness>25)set_led_brightness(g_led_brightness-10);else set_led_brightness(0);} break;
            case 'O': if(g_operating_mode==1)control_motor_1(ABRIR); break;
            case 'C': if(g_operating_mode==1)control_motor_1(FECHAR); break;
            case 'S': if(g_operating_mode==1)control_motor_1(PARAR); break;
			case 'X': if(g_operating_mode==1)control_motor_2(ABRIR); break;
            case 'F': if(g_operating_mode==1)control_motor_2(FECHAR); break;
            case 'P': if(g_operating_mode==1)control_motor_2(PARAR); break;
        }
    }
}
void adjust_setpoint_control(void) {
    // --- BOTÃO MAIS (UP) COM DEBOUNCE ---
    if (PIND & (1 << BOTAO_MAIS)) {
        if (g_up_debounce < DEBOUNCE_TICKS + 1) g_up_debounce++;
    } else {
        g_up_debounce = 0;
    }

    // Dispara ação APENAS quando atinge o limiar (Single Shot)
    if (g_up_debounce == DEBOUNCE_TICKS) {
			g_up_debounce = DEBOUNCE_TICKS + 1;
        if (g_fix_state != 2) {
            if (g_last_setpoint_value < LUX_MAX) g_last_setpoint_value += LUX_STEP;
            if (g_fix_state == 0) g_fix_state = 1; // Ativa modo de ajuste
        }
    }

    // --- BOTÃO MENOS (DOWN) COM DEBOUNCE ---
    if (PIND & (1 << BOTAO_MENOS)) {
        if (g_down_debounce < DEBOUNCE_TICKS + 1) g_down_debounce++;
    } else {
        g_down_debounce = 0;
    }

    if (g_down_debounce == DEBOUNCE_TICKS) {
			g_down_debounce = DEBOUNCE_TICKS + 1;
        if (g_fix_state != 2) {
            if (g_last_setpoint_value > LUX_MIN) g_last_setpoint_value -= LUX_STEP;
            if (g_fix_state == 0) g_fix_state = 1;
        }
    }

    // --- MÁQUINA DE ESTADOS DE FIXAÇÃO (2s) ---
    // Se estiver em ajuste (1) e soltar os botões -> Conta tempo (2)
    if (g_fix_state == 1) {
        if (!(PIND & (1 << BOTAO_MAIS)) && !(PIND & (1 << BOTAO_MENOS))) {
            g_fix_timer = FIX_TIME_TICKS;
            g_fix_state = 2;
        }
    } 
    // Se estiver a contar (2) e carregar num botão -> Cancela contagem (1)
    else if (g_fix_state == 2) {
        if ((PIND & (1 << BOTAO_MAIS)) || (PIND & (1 << BOTAO_MENOS))) {
            g_fix_state = 1;
						return;
        } else if (g_fix_timer == 0) {
            // Tempo acabou: Fixa valor
            g_target_lux = g_last_setpoint_value;
            buzzer_bips(2);
            g_fix_state = 0;
        }
    }
}	
void servo_led_hybrid_control(void) {
    int16_t error = (int16_t)g_target_lux - (int16_t)g_averaged_lux;
    static uint16_t led_timer = 0;

    if(g_sensor_count == 0 || g_init_state != 255) return;

    // 1. LIMITES PRIMEIRO
    if (g_blind1_pos >= BLIND1_MAX_TICKS) control_motor_1(PARAR);
    if (g_blind1_pos <= 0)                control_motor_1(PARAR);
    if (g_blind2_pos >= BLIND2_MAX_TICKS) control_motor_2(PARAR);
    if (g_blind2_pos <= 0)                control_motor_2(PARAR);

    // 2. Se estamos dentro da banda → parar tudo
    if (abs(error) <= LUX_BAND_ERROR) {
        control_motor_1(PARAR);
        control_motor_2(PARAR);
        return;
    }

    // 3. FALTA LUZ → abrir
    if (error > LUX_BAND_ERROR) {

        if (g_blind1_pos < BLIND1_MAX_TICKS) {
            control_motor_1(ABRIR);
            control_motor_2(PARAR);
            set_led_brightness(0);
        }
        else if (g_blind2_pos < BLIND2_MAX_TICKS) {
            control_motor_1(PARAR);
            control_motor_2(ABRIR);
            set_led_brightness(0);
        }
        else {
            control_motor_1(PARAR);
            control_motor_2(PARAR);

            led_timer++;
            if (led_timer >= LED_FADE_INTERVAL) {
                led_timer = 0;
                if (g_led_brightness < MAX_LED_PWM)
                    set_led_brightness(g_led_brightness + 10);
            }
        }
    }

    // 4. LUZ A MAIS → fechar
    else {

        if (g_led_brightness > 0) {
            control_motor_1(PARAR);
            control_motor_2(PARAR);

            led_timer++;
            if (led_timer >= LED_FADE_INTERVAL) {
                led_timer = 0;
                if (g_led_brightness >= 10) set_led_brightness(g_led_brightness - 10);
                else set_led_brightness(0);
            }
        }
        else if (g_blind1_pos > 0) {
            control_motor_1(FECHAR);
            control_motor_2(PARAR);
        }
        else if (g_blind2_pos > 0) {
            control_motor_1(PARAR);
            control_motor_2(FECHAR);
        }
        else {
            control_motor_1(PARAR);
            control_motor_2(PARAR);
        }
    }
}
// --- I2C Functions ---
void inic_i2c(void) {
	TWCR |= (1 << TWEN);
	TWSR = 0;
	TWBR = 12;
	DDRC &= ~((1 << PC4) | (1 << PC5));
	PORTC |= (1 << PC4) | (1 << PC5);
}
void i2c_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	uint16_t timeout = 0;
	while(!(TWCR & (1 << TWINT)) && timeout < I2C_TIMEOUT_CYCLES) {
		timeout++;
	}
}
void i2c_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}
void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	uint16_t timeout = 0;
	while(!(TWCR & (1 << TWINT)) && timeout < I2C_TIMEOUT_CYCLES) {
		timeout++;
	}
}
uint8_t i2c_read_ack(void) {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	uint16_t timeout = 0;
	while(!(TWCR & (1 << TWINT)) && timeout < I2C_TIMEOUT_CYCLES) {
		timeout++;
	}
	return TWDR;
}
uint8_t i2c_read_nack(void) {
	TWCR = (1 << TWINT) | (1 << TWEN);
	uint16_t timeout = 0;
	while(!(TWCR & (1 << TWINT)) && timeout < I2C_TIMEOUT_CYCLES) {
		timeout++;
	}
	return TWDR;
}
uint8_t i2c_get_status(void) {
	return TWSR & 0b11111000;
}
void ssd1306_cmd(uint8_t cmd) {
	i2c_start(); // write mode
	i2c_write(SSD1306_ADDR << 1);// Co = 0, D/C = 0 (cmd)
	i2c_write(0x00); 
	i2c_write(cmd);
	i2c_stop(); 
}
void ssd1306_data(uint8_t data) {
	i2c_start();
	i2c_write(SSD1306_ADDR << 1);// Co = 0, D/C = 1 (data)
	i2c_write(0x40); 
	i2c_write(data); 
	i2c_stop(); 
}
void ssd1306_init(void) {
    ssd1306_cmd(0xAE); // Display OFF
    ssd1306_cmd(0x20); ssd1306_cmd(0x02); // Page Addressing
    ssd1306_cmd(0xB0); // Start Page 0
    ssd1306_cmd(0xC8); // COM Output Scan Dec
    ssd1306_cmd(0x00); // Low Col
    ssd1306_cmd(0x10); // High Col
    ssd1306_cmd(0x40); // Start Line 0
    ssd1306_cmd(0x81); ssd1306_cmd(0xCF); // Contrast
    ssd1306_cmd(0xA1); // Segment Remap
    ssd1306_cmd(0xA6); // Normal
    ssd1306_cmd(0xA8); ssd1306_cmd(0x3F); // Mux
    ssd1306_cmd(0x8D); ssd1306_cmd(0x14); // Charge Pump
    ssd1306_cmd(0xAF); // Display ON
}
void ssd1306_clear(void) {
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_cmd(0xB0 + page); // page address
        ssd1306_cmd(0x00);        // low column
        ssd1306_cmd(0x10);        // high column
        for (uint8_t col = 0; col < 128; col++) {
            ssd1306_data(0x00);
        }
    }
}
void ssd1306_clear_line(uint8_t row) {
	ssd1306_set_cursor(row, 0); 
	for(uint8_t i = 0; i < 128; i++) 
	ssd1306_data(0x00); 
}
void ssd1306_char(char c) {
    if (c < 32 || c > 127) c = '?';
    for (uint8_t i = 0; i < 6; i++) {
        ssd1306_data(font6x8[c - 32][i]);
    }
}
void ssd1306_print(const char *str) {
    while (*str) {
        ssd1306_char(*str++);
    }
}
void ssd1306_print_xy(uint8_t row, uint8_t col, const char *str) {
    ssd1306_set_cursor(row, col);
    ssd1306_print(str);
}
void ssd1306_set_cursor(uint8_t row, uint8_t col) {
    ssd1306_cmd(0xB0 + row);          // page (0–7)
    ssd1306_cmd(0x00 + (col & 0x0F)); // low column
    ssd1306_cmd(0x10 + (col >> 4));   // high column
}
const char* motor_state_to_str(Servo_pos s) {
  switch(s) {
    case ABRIR:  return "ABRIR";
    case FECHAR: return "FECHAR";
    default:     return "PARAR";
  }
}
void oled_write_line(uint8_t row, const char *text, char *last_buffer) { 
  if(strcmp(text, last_buffer) == 0) return; // nada mudou → não redesenha 
  strcpy(last_buffer, text); 
  ssd1306_set_cursor(row, 0); 
  ssd1306_print(" "); // limpa com espaços 
  ssd1306_set_cursor(row, 0); 
  ssd1306_print(text); 
}
void oled_update_status(void) {
    char line[20];

    // --- Linha 0: Lux atual ---
    sprintf(line, "LUX: %4u lx", g_averaged_lux);
    oled_write_line(0, line, last_line0);

    // --- Linha 1: Modo ---
    if(g_operating_mode == MODE_AUTOMATIC)
        oled_write_line(1, "MODO: AUTOMATICO", last_line1);
    else
        oled_write_line(1, "MODO: MANUAL    ", last_line1);

    // --- Linha 2: Estado dos motores ---
    sprintf(line, "M1:%s M2:%s",
        motor_state_to_str(g_motor1_action),
        motor_state_to_str(g_motor2_action)
    );
    oled_write_line(2, line, last_line2);
    // Linha 3: separador
    ssd1306_set_cursor(3, 0);
    ssd1306_print("----------------");
    // --- Linha 4: Barra de luz ---
    oled_draw_lux_bar(g_averaged_lux);
    sprintf(line, "Ciclos: %4u", g_debug_ciclos);
    oled_write_line(4, line, last_line3);
}
void oled_draw_lux_bar(uint16_t lux) {
    uint32_t scaled = (uint32_t)lux * 128;
    uint8_t width = scaled / LUX_MAX;
    if(width > 127) width = 127;

    if(width == last_bar_width)
        return; // barra igual → não redesenha

    last_bar_width = width;

    ssd1306_set_cursor(4, 0);
    for(uint8_t i = 0; i < 128; i++) {
        ssd1306_data(i < width ? 0xFF : 0x00);
    }
}
// --- BH1750 Functions ---
uint8_t bh1750_send(uint8_t addr, uint8_t data) {
	i2c_start();
	i2c_write(addr << 1 | 0);
	if(i2c_get_status() != 0x18) { //SLA+W
		i2c_stop();
		return 0;
	}
	i2c_write(data);
	if(i2c_get_status() != 0x28) { //DATA+ACK
		i2c_stop();
		return 0;
	}
	i2c_stop();
	return 1;
}
uint16_t bh1750_read(uint8_t addr) {
	uint8_t high_byte, low_byte;
	i2c_start();
	if(i2c_get_status() != 0x08) {
		i2c_stop();
		return 0;
	}
	i2c_write((addr << 1) | 1);
	if(i2c_get_status() != 0x40) {
		i2c_stop();
		return 0;
	}
	high_byte = i2c_read_ack();
	low_byte = i2c_read_nack();
	i2c_stop();
	uint16_t data = (high_byte << 8) | low_byte;
	uint16_t lux = (uint16_t)((float) data / 1.2);
	if(lux > 65500) lux = 65500;
	return lux;
}
void detect_sensors(void) {
	g_sensor1_present = bh1750_send(BH1750_ADDR1, BH1750_POWER_ON) && bh1750_send(BH1750_ADDR1, BH1750_CONT_HIGH_RES_MODE2);
	g_sensor2_present = bh1750_send(BH1750_ADDR2, BH1750_POWER_ON) && bh1750_send(BH1750_ADDR2, BH1750_CONT_HIGH_RES_MODE2);
	g_sensor_count = g_sensor1_present + g_sensor2_present;
}
uint16_t bh1750_read_sensors(void) {
	uint16_t lux1 = 0, lux2 = 0;
	if(g_sensor1_present) lux1 = bh1750_read(BH1750_ADDR1);
	if(g_sensor2_present) lux2 = bh1750_read(BH1750_ADDR2);
	if(g_sensor_count == 1) {
		if(lux1 > 0) return lux1;
		else return lux2;
	}
	if(g_sensor_count == 2) return (uint16_t)((float)(lux1 + lux2)) / 2;
	return 0;
}
// --- Averaging Function ---
uint16_t average_lux(uint16_t new_reading) {
	lux_buffer[g_buffer_index] = new_reading;
	g_buffer_index = (g_buffer_index + 1) % AVG_SAMPLES;
	uint32_t sum = 0;
	for(uint8_t i = 0; i < AVG_SAMPLES; i++) {
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
void pwm_led_init(void){
	DDRB |= (1 << GATE_PIN);
	PORTB &= ~(1 << GATE_PIN);
	TCCR2A = (1 << WGM20) | (1 << COM2A1);
	TCCR2B =  (1 << CS20) | (1 << CS21);
	OCR2A = 0;
}
void pwm_Servo_init(void) {
	DDRB |= (1 << PB1) | (1 << PB2);
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1)| (1 << WGM11);//PC
	TCCR1B |= (1 << WGM13) | (1 << CS11) | (1 << CS10) | (1 << ICNC1); // prescaler 64, noise cancelling
	ICR1 = 2500; // 20 ms (50 Hz)
	OCR1A = 188;
	OCR1B = 188;
}
void buttons_inic(void) {
	// Botão touch PC1
	DDRC &= ~(1 << BOTAO_TOUCH);
	PORTC &= ~(1 << BOTAO_TOUCH);
	// Botões +/- em PD4 e PD7
	DDRD &= ~((1 << BOTAO_MAIS) | (1 << BOTAO_MENOS));
	PORTD &= ~((1 << BOTAO_MAIS) | (1 << BOTAO_MENOS));
	// BUZZER
	DDRC |= (1 << BUZZER_PIN);
	PORTC &= ~(1 << BUZZER_PIN);
	// Sonares: TRIG como saída, ECHO como entrada
	DDRC |= (1 << SONAR1_TRIG) | (1 << SONAR2_TRIG);
	DDRD &= ~((1 << SONAR1_ECHO) | (1 << SONAR2_ECHO));
	PORTD &= ~((1 << SONAR1_ECHO) | (1 << SONAR2_ECHO)); // sem pull-up
	// Interrupção de PC1 (botão touch)
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT9);
}
void control_motor_1(Servo_pos action) {
    g_motor1_action = action;

    uint16_t valor_pwm;
    switch(action) {
        case ABRIR:
            valor_pwm = 175;
            break;
        case PARAR:
            valor_pwm = 188;
            break;
        case FECHAR:
            valor_pwm = 198;
            break;
    }
    OCR1B = valor_pwm;
}
void control_motor_2(Servo_pos action) {
    g_motor2_action = action;

	uint16_t valor_pwm;
	switch(action) {
		case ABRIR:
			valor_pwm = 169;
			break;
		case PARAR:
			valor_pwm = 188;
			break;
		case FECHAR:
			valor_pwm = 197;
			break;
	}
	OCR1A = valor_pwm;
}
void manual_led_control(void) {
	static uint8_t repeat_delay = 0;
	    if (repeat_delay > 0) {
        repeat_delay--;
        return;
    	}

    if (PIND & (1 << BOTAO_MAIS)) {
        if (g_led_brightness < 255) {
				set_led_brightness(g_led_brightness + 5);
				repeat_delay = 3;
				}else set_led_brightness(255);  
    }
    if (PIND & (1 << BOTAO_MENOS)) {
        if (g_led_brightness > 0) {
				set_led_brightness(g_led_brightness - 5);
				repeat_delay = 3;
				}else set_led_brightness(0);
    }
}
void set_led_brightness(uint8_t brightness) {
    if (brightness > MAX_LED_PWM) brightness = MAX_LED_PWM;
    OCR2A = brightness;
    g_led_brightness = brightness;
}
uint8_t sonar_is_close(uint8_t sonar_id){
    static uint32_t last_trigger_time_1 = 0;
    static uint32_t last_trigger_time_2 = 0;
    static uint8_t stable_state_1 = 0;
    static uint8_t stable_state_2 = 0;

    uint32_t *last_time = (sonar_id == 1) ? &last_trigger_time_1 : &last_trigger_time_2;
    uint8_t *stable_state = (sonar_id == 1) ? &stable_state_1 : &stable_state_2;

    uint8_t trig_pin = (sonar_id == 1) ? SONAR1_TRIG : SONAR2_TRIG;
    uint8_t echo_pin = (sonar_id == 1) ? SONAR1_ECHO : SONAR2_ECHO;

    // 1) Espera não-bloqueante entre triggers (60ms de silêncio)
    if (g_millis - *last_time < 60) {
        return *stable_state;
    }
    *last_time = g_millis;

    // 2) Trigger de 10us
    PORTC &= ~(1 << trig_pin); _delay_us(2);
    PORTC |=  (1 << trig_pin); _delay_us(10);
    PORTC &= ~(1 << trig_pin);

    // 3) Espera NÃO-BLOQUEANTE pelo echo subir (Timeout curto de ~2ms)
    // Se o sensor estiver desconectado, sai rápido sem travar o código.
    uint16_t timeout = 0;
    while (!(PIND & (1 << echo_pin))) {
        if (++timeout > 16000) {   
            return *stable_state; // Falha ou nada detetado
        }
    }

    // 4) Medição BLOQUEANTE (Precisão máxima)
    // Medimos a largura do pulso. Como só nos interessa "perto" (<30cm),
    // isto dura no máximo 1 a 2 ms.
    uint16_t cycles = 0;
    while (PIND & (1 << echo_pin)) {
        cycles++;
        if (cycles > 10000) break; // Break de segurança (se > 50cm, sai)
    }

    // ---> DEBUG: Guarda o valor para mostrar no OLED <---
    // Apenas guardamos o do Sonar 1 para não piscar demasiado o ecrã, 
    // ou podes fazer uma média/alternância se quiseres.
    if(sonar_id == 1 || (sonar_id == 2 && cycles > 100)) { 
        g_debug_ciclos = cycles; 
    }

    // 5) Threshold calibrado
    // Verifica se "cycles" é menor que o limite (perto) E maior que ruído (50)
    uint8_t reading = (cycles < SONAR_THRESHOLD_TICKS_20CM && cycles > 50) ? 1 : 0;

    // 6) Atualiza estado
    *stable_state = reading;    
    return *stable_state;
}
void update_manual_gestures(void){

    static uint8_t s1_prev = 0;
    static uint8_t s2_prev = 0;

    // Leitura atual dos sonares
    uint8_t s1 = sonar_is_close(1);
    uint8_t s2 = sonar_is_close(2);

    // Timeout de segurança
    if (g_gesture_timer > 0) {
        g_gesture_timer--;
        // Se o tempo acabar e não completou, reseta. 
        // Se estiver no estado 3 (bloqueio), o timer serve apenas de debounce.
        if (g_gesture_timer == 0 && g_gesture_state != 0 && g_gesture_state != 3) {
            g_gesture_state = 0;
            s1_prev = s1; s2_prev = s2;
            return;
        }
    }

    switch (g_gesture_state)
    {
        // ESTADO 0: À espera (Repouso)
        case 0:
            if (s1 && !s1_prev) {      // Entrou na Esquerda
                g_gesture_state = 1;
                g_gesture_timer = GESTURE_TIMEOUT_TICKS;
            } 
            else if (s2 && !s2_prev) { // Entrou na Direita
                g_gesture_state = 2;
                g_gesture_timer = GESTURE_TIMEOUT_TICKS;
            }
            break;

        // ESTADO 1: Iniciou na Esquerda -> Espera Direita
        case 1:
            if (s2) { // Não precisa de borda, basta presença
                control_motor_1(FECHAR); // Ou FECHAR, dependendo da tua lógica
                control_motor_2(FECHAR);
                g_gesture_state = 3;    // Vai para Bloqueio
                g_gesture_timer = GESTURE_TIMEOUT_TICKS; 
            }
            break;

        // ESTADO 2: Iniciou na Direita -> Espera Esquerda
        case 2:
            if (s1) {
                control_motor_1(ABRIR);
                control_motor_2(ABRIR);
                g_gesture_state = 3;    // Vai para Bloqueio
                g_gesture_timer = GESTURE_TIMEOUT_TICKS;
            }
            break;

        // ESTADO 3: Bloqueio (Espera mãos saírem)
        // Isto impede que o estore pare/inverta imediatamente se mantiveres a mão.
        case 3:
            if (!s1 && !s2) {
                // Só liberta o sistema quando não houver obstáculos
                g_gesture_state = 0;
                g_gesture_timer = 0;
            }
            break;
    }

    s1_prev = s1;
    s2_prev = s2;
}

// --- FUNÇÃO DE INICIALIZAÇÃO DE HARDWARE ---
void inic(void) {
	uart_init();
	onda1Hz_init();
	pwm_led_init();
	pwm_Servo_init();
	buttons_inic();
	sei();

}
// --- MÁQUINA DE ESTADOS PRINCIPAL (NON-BLOCKING) ---
void inic_non_blocking(void) {
  switch(g_init_state) {

    case 0:
        inic_i2c();
        START_NB_DELAY_MS(10);   // pequeno delay opcional
        g_init_state = 1;
        break;
    case 1:
        if(IS_DELAY_FINISHED()) {
            ssd1306_init();
            ssd1306_clear();
            ssd1306_set_cursor(0, 0);
            ssd1306_print("Projeto LABSI");
            START_NB_DELAY_MS(1500);
            g_init_state = 2;
        }
        break;
    case 2:
        if(IS_DELAY_FINISHED()) {
            detect_sensors();
            ssd1306_clear();
            ssd1306_set_cursor(0, 0);
            if(g_sensor_count == 0) {
                ssd1306_print("Sem Sensores");
            } 
            else if(g_sensor_count == 1) {
                ssd1306_print("1 Sensor Pronto");
            } 
            else {
                ssd1306_print("2 Sensores Prontos");
            }
            START_NB_DELAY_MS(1500);
            g_init_state = 3;
        }
        break;
    case 3:
        if(IS_DELAY_FINISHED()) {
            ssd1306_clear();
            g_setup_done = 1;
            g_init_state = 255;
            buzzer_bips(3);
        }
        break;
  }
}
int main(void) {
inic();
char buffer[32];
while(1) {
	if(!g_setup_done) {
		inic_non_blocking();
	} else {
        if(g_flag_50ms){
            g_flag_50ms = 0;
                // 1. LEITURA DE LUX e MÉDIA
			    g_lux_value = bh1750_read_sensors();
			    g_averaged_lux = average_lux(g_lux_value);
            
				oled_update_status();
				char op_mode_char = (g_operating_mode == MODE_AUTOMATIC) ? 'A' : 'M';
				uint16_t disp_set = (g_fix_state != 0) ? g_last_setpoint_value : g_target_lux;
				
				char motor1_char = 'E';
				if (g_motor1_action == ABRIR) motor1_char = 'A';
				else if (g_motor1_action == FECHAR) motor1_char = 'F';
				else motor1_char = 'P';
				char motor2_char = 'E';
				if (g_motor2_action == ABRIR) motor2_char = 'A';
				else if (g_motor2_action == FECHAR) motor2_char = 'F';
				else motor2_char = 'P';
				// 2. Envia para o ESP32 (Protocolo: D:Modo,Set,Lux,Motor1,Motor2,Ticks1,Ticks2)
				char serial_buffer[40];
				sprintf(serial_buffer, "D:%c,%u,%u,%c,%c,%ld,%ld\n", 
				        op_mode_char, disp_set, g_averaged_lux, motor1_char,motor2_char, g_blind1_pos, g_blind2_pos);
				uart_write_string(serial_buffer);
			
        }
		if(g_flag_2ms) {
			g_flag_2ms = 0;
            g_display_counter++;
            g_contador++;
	        if(g_contador >= 250) {
		        PORTD ^= (1 << PD6);
		        g_contador = 0;
	        }   
            g_contador_50ms++;
            if(g_contador_50ms >= 25){
                g_flag_50ms = 1;
                g_contador_50ms = 0;
            }
            // Controlo do Buzzer
	        if(g_buzzer_counter > 0) {
	        	g_buzzer_counter--;
	        	// Alterna o Buzzer (PC0) a cada 50ms (25 ticks)
	        	if(g_buzzer_counter % 25 == 0) {
	        		PORTC ^= (1 << BUZZER_PIN);
	        	}
	        } else {
	        	PORTC &= ~(1 << BUZZER_PIN); // Desliga o Buzzer
	        }
            // Rastreamento Posição
            if(g_motor1_action==ABRIR && g_blind1_pos<BLIND1_MAX_TICKS) g_blind1_pos+=STEP_OPEN;
            else if(g_motor1_action==FECHAR && g_blind1_pos>0) g_blind1_pos-=STEP_CLOSE;

            if(g_motor2_action==ABRIR && g_blind2_pos<BLIND2_MAX_TICKS) g_blind2_pos+=STEP_OPEN;
            else if(g_motor2_action==FECHAR && g_blind2_pos>0) g_blind2_pos-=STEP_CLOSE;
            /*
            if (g_blind1_pos >= BLIND1_MAX_TICKS) control_motor_1(PARAR);
            if (g_blind1_pos <= 0)                control_motor_1(PARAR);
            if (g_blind2_pos >= BLIND2_MAX_TICKS) control_motor_2(PARAR);
            if (g_blind2_pos <= 0)                control_motor_2(PARAR);
            */
			// 2. Controlo dependendo do modo
			static uint8_t sonar_tick = 0;
            check_uart_cmds();
			if(g_operating_mode == MODE_AUTOMATIC) {
				adjust_setpoint_control();
				servo_led_hybrid_control();
			} else {
				manual_led_control();
				sonar_tick++;
					if(sonar_tick >= 50) { // ~100 ms
						sonar_tick = 0;
						update_manual_gestures();
					}
			}
		}
	}
}
    return 0;
}