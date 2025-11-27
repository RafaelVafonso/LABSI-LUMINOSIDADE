#include <avr/io.h>

#include <avr/interrupt.h>

#include <stdio.h>

#include <stdlib.h>

#define F_CPU 16000000 UL
#define LCD_I2C_ADDR 0x20 // Endereço I2C do PCF8574T (ajuste se necessário)
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
#define BOTAO_MAIS PD4 // Botão Aumentar
#define BOTAO_MENOS PD7 // Botão Diminuir
#define BUZZER_PIN PC0 // Buzzer (PC0)

#define SONAR1_TRIG PD0
#define SONAR1_ECHO PD2
#define SONAR2_TRIG PD1
#define SONAR2_ECHO PD3
// 20 cm ~ 1156 us de echo -> com Timer1 a 4 us/tick => ~290 ticks
#define SONAR_THRESHOLD_TICKS_20CM 300
#define LUX_STEP 50
#define LUX_MAX 1000
#define LUX_MIN 0
#define LUX_BAND_ERROR 30
#define I2C_TIMEOUT_CYCLES 20000
#define FIX_TIME_TICKS 20

#define GESTURE_TIMEOUT_TICKS 500
// --- VARIÁVEIS DE CONTROLO DE TEMPO E ESTADO ---
volatile uint16_t g_delay_counter_2ms = 0; // Contador principal, decrementa a cada 2ms
volatile uint8_t g_init_state = 0; // Máquina de estados para inicialização
volatile uint8_t g_setup_done = 0; // Flag: 1 quando setup concluído
volatile uint8_t g_operating_mode = MODE_AUTOMATIC;
volatile uint8_t g_mode_changed = 1; // Flag para limpar LCD
volatile uint8_t g_prev_pc1_state = 0; // Para detetar a borda do botão
volatile uint8_t g_mode_locked = 0; // Bloqueio após 1º toque
volatile uint16_t g_target_lux = 150;
volatile uint8_t g_contador = 0;
volatile char g_flag_2ms = 0;
volatile uint8_t g_display_counter = 0;
volatile uint16_t g_last_setpoint_value = 150;
volatile uint8_t g_buzzer_counter = 0;
volatile uint8_t g_fix_state = 0; // 0: Estável, 1: Ajuste Ativo, 2: Contagem (2s)
volatile uint16_t g_fix_timer = 0; // Timer de 2 segundos
// Gesto com sonares (modo manual)
volatile uint8_t g_gesture_state = 0; // 0: idle, 1: S1 primeiro, 2: S2 primeiro
volatile uint16_t g_gesture_timer = 0; // timeout do gesto

uint8_t g_up_debounce = 0;
uint8_t g_down_debounce = 0;
uint16_t g_lux_value = 0;
uint16_t g_servo_pwm_value = 188;
uint8_t g_sensor_count = 0;
uint8_t g_sensor1_present = 0;
uint8_t g_sensor2_present = 0;
#define AVG_SAMPLES 10
uint16_t lux_buffer[AVG_SAMPLES] = {
	0
};
uint8_t g_buffer_index = 0;
uint16_t g_averaged_lux = 0;
// Controlo Motor
typedef enum {
	FECHAR = 0,
		PARAR = 1,
		ABRIR = 2,
}
Servo_pos;
uint8_t g_estado_atual;
Servo_pos pos_desejada;

void control_motor(Servo_pos pos_desejada);

uint8_t sonar_is_close(uint8_t sonar_id);
void update_manual_gestures(void);
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
	if(g_delay_counter_2ms > 0) {
		g_delay_counter_2ms--;
	}
	if(g_fix_state == 2 && g_fix_timer > 0) {
		g_fix_timer--;
	}
	// Timer de gesto (sonares)
	if(g_gesture_timer > 0) {
		g_gesture_timer--;
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
	g_contador++;
	if(g_contador >= 250) {
		PORTD ^= (1 << PD6);
		g_contador = 0;
	}
	g_display_counter++;

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
			control_motor(1);
			//buzzer_bips(1); // 1 bip na troca de modo
			g_mode_locked = 1; // Bloquear mais mudanças
		}
	} else if(current_state < g_prev_pc1_state) {
		// Transição de 1 para 0 detectada
		g_mode_locked = 0; // Desbloquear sistema para o próximo toque
	}
	g_prev_pc1_state = current_state;
}
// --- LÓGICA DE CONTROLO DE SETPOINT (+/-) ---
void adjust_setpoint_control(void) {
	const uint8_t DEBOUNCE_DELAY = 10; // 20ms
	// 1. Botão UP (Aumentar)
	if((PIND & (1 << BOTAO_MAIS))) {
		if(g_up_debounce == 0) {
			g_up_debounce = DEBOUNCE_DELAY;
		} else if(g_up_debounce == 1) {
			if(g_fix_state != 2) {
				if(g_last_setpoint_value < LUX_MAX) {
					g_last_setpoint_value += LUX_STEP;
					if(g_last_setpoint_value > LUX_MAX) g_last_setpoint_value = LUX_MAX;
				}
				if(g_fix_state == 0) {
					g_fix_state = 1;
				}
			}
			g_up_debounce = 0;
		}
	} else {
		g_up_debounce = 0;
	}
	// 2. Botão DOWN (Diminuir)
	if((PIND & (1 << BOTAO_MENOS))) {
		if(g_down_debounce == 0) {
			g_down_debounce = DEBOUNCE_DELAY;
		} else if(g_down_debounce == 1) {
			if(g_fix_state != 2) {
				if(g_last_setpoint_value > LUX_MIN) {
					g_last_setpoint_value -= LUX_STEP;
					if(g_last_setpoint_value < LUX_MIN) g_last_setpoint_value = LUX_MIN;
				}
				if(g_fix_state == 0) {
					g_fix_state = 1;
				}
			}
			g_down_debounce = 0;
		}
	} else {
		g_down_debounce = 0;
	}
	// 3. Lógica de Debounce (Decrementa)
	if(g_up_debounce > 0) g_up_debounce--;
	if(g_down_debounce > 0) g_down_debounce--;
	// 4. MÁQUINA DE ESTADOS DE FIXAÇÃO (Contagem de 2 Segundos)
	else {
		if(g_fix_state == 1) {
			g_fix_timer = FIX_TIME_TICKS;
			g_fix_state = 2; // Transição para Contagem
		}
	}
	if(g_fix_state == 2) {
		// CONTAGEM (2s)
		if(g_fix_timer == 0) {
			g_target_lux = g_last_setpoint_value; // Fixa o valor
			//buzzer_bips(2); // 2 bips para sinalizar bloqueio
			g_fix_state = 0; // Volta a Estável
		}
	}
}
void servo_control_automatic(void) {
	int16_t error = (int16_t) g_target_lux - (int16_t) g_averaged_lux;
	// Se o sistema ainda está a inicializar o BH1750, não atuar
	if(g_sensor_count == 0 || g_init_state != 255) return;
	if(abs(error) <= LUX_BAND_ERROR) {
		control_motor(PARAR);
		return;
	} else if(error > LUX_BAND_ERROR) {
		// Erro positivo -> abrir
		if(g_estado_atual < ABRIR) {
			control_motor(ABRIR);
		}
	} else {
		// Erro negativo -> fechar
		if(g_estado_atual > FECHAR) {
			control_motor(FECHAR);
		}
	}
}
// --- I2C Functions ---
void inic_i2c(void) {
	TWCR |= (1 << TWEN);
	TWSR = 0;
	TWBR = 72;
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
// --- LCD Functions ---
void i2c_lcd_send(uint8_t data) {
	i2c_start();
	i2c_write(LCD_I2C_ADDR << 1);
	i2c_write(data);
	i2c_stop();
}
void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
	uint8_t data = 0;
	if(nibble & 0b1000) data |= (1 << 0); // D7 -> P0
	if(nibble & 0b0100) data |= (1 << 1); // D6 -> P1
	if(nibble & 0b0010) data |= (1 << 2); // D5 -> P2
	if(nibble & 0b0001) data |= (1 << 3); // D4 -> P3
	if(rs) data |= PCF_RS;
	// EN high
	i2c_lcd_send(data | PCF_EN);
	delay_us(200);
	// EN low
	i2c_lcd_send(data & ~PCF_EN);
	delay_us(200);
}
void lcd_write_byte(uint8_t data, uint8_t rs) {
	lcd_write_nibble(data >> 4, rs); // High bits
	lcd_write_nibble(data & 0b00001111, rs); // Low bits
}
void lcd_clear(void) {
	lcd_write_byte(0b00000001, 0);
}
void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t addr;
	if(row == 0) {
		addr = 0b10000000 + col;
	} else {
		addr = 0b11000000 + col;
	}
	lcd_write_byte(addr, 0);
}
void lcd_write_string(const char * str) {
	while( * str) {
		lcd_write_byte( * str++, 1);
	}
}
void lcd_init_setup_cmds(void) {
	lcd_write_byte(0b00101000, 0); // Function set: 4-bit, 2 lines, 5x8 dots
	lcd_write_byte(0b00001100, 0); // Display on, cursor off, blink off
	lcd_write_byte(0b00000001, 0); // Clear Display
	lcd_write_byte(0b00000110, 0); // Entry mode: Increment cursor, no display shift
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
	return (uint16_t)((float) data / 1.2);
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
void pwm_Servo_init(void) {
	DDRB |= (1 << PB1);
	TCCR1A |= (1 << COM1A1) | (1 << WGM11);//PC
	TCCR1B |= (1 << WGM13) | (1 << CS11) | (1 << CS10) | (1 << ICNC1); // prescaler 64, noise cancelling
	ICR1 = 2500; // 20 ms (50 Hz)
	OCR1A = 188;
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
	DDRD |= (1 << SONAR1_TRIG) | (1 << SONAR2_TRIG);
	DDRD &= ~((1 << SONAR1_ECHO) | (1 << SONAR2_ECHO));
	PORTD &= ~((1 << SONAR1_ECHO) | (1 << SONAR2_ECHO)); // sem pull-up
	// Interrupção de PC1 (botão touch)
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT9);
}
void control_motor(Servo_pos pos_desejada) {
	uint16_t valor_pwm;
	switch(pos_desejada) {
		case FECHAR:
			valor_pwm = 125;
			g_estado_atual = FECHAR;
			break;
		case PARAR:
			valor_pwm = 188;
			g_estado_atual = PARAR;
			break;
		case ABRIR:
			valor_pwm = 250;
			g_estado_atual = ABRIR;
			break;
	}
	OCR1A = valor_pwm;
}
// verifica se há objeto < ~20 cm ---
uint8_t sonar_is_close(uint8_t sonar_id) {
	uint8_t trig_pin;
	uint8_t echo_pin;
	DDRD |= (1 << trig_pin);
  DDRD &= ~(1 << echo_pin);

	if(sonar_id == 1) {
		trig_pin = SONAR1_TRIG;
		echo_pin = SONAR1_ECHO;
	} else {
		trig_pin = SONAR2_TRIG;
		echo_pin = SONAR2_ECHO;
	}
	// Trigger: pulso de 10 us
	PORTD &= ~(1 << trig_pin);
	delay_us(2);
	PORTD |= (1 << trig_pin);
	delay_us(10);
	PORTD &= ~(1 << trig_pin);
	// Espera echo subir (com timeout simples)
	uint16_t timeout = 0;
	while(!(PIND & (1 << echo_pin)) && timeout < 5000) {
		timeout++;
	}
	if(timeout >= 5000) {
		return 0; // sem resposta
	}
	// Mede largura do pulso de echo usando TCNT1 (4 us por tick)
	uint16_t start = TCNT1;
	timeout = 0;
	while((PIND & (1 << echo_pin)) && timeout < 5000) {
		timeout++;
	}
	uint16_t end = TCNT1;
	// Calcula diferença de ticks considerando wrap em ICR1 (5000)
	uint16_t ticks;
	if(end >= start) {
		ticks = end - start;
	} else {
		ticks = (ICR1 - start) + end;
	}
	// Se ticks < limite -> menos de ~20 cm
	if(ticks < SONAR_THRESHOLD_TICKS_20CM) {
		return 1;
	} else {
		return 0;
	}
}
// --- Lógica de gestos no modo MANUAL ---
// S1 (baixo) -> S2 (cima)   => ABRIR
// S2 (cima)  -> S1 (baixo)  => FECHAR
void update_manual_gestures(void) {
	static uint8_t s1_prev = 0;
	static uint8_t s2_prev = 0;
	uint8_t s1 = sonar_is_close(1);
	uint8_t s2 = sonar_is_close(2);
	// Reset do gesto se o tempo expirar
	if(g_gesture_timer == 0 && g_gesture_state != 0) {
		g_gesture_state = 0;
	}
	// Estados:
	// 0: idle, 1: S1 primeiro, 2: S2 primeiro
	if(g_gesture_state == 0) {
		// Procura primeira ativação
		if(s1 && !s1_prev) {
			g_gesture_state = 1; // S1 primeiro (de baixo para cima)
			g_gesture_timer = GESTURE_TIMEOUT_TICKS;
		} else if(s2 && !s2_prev) {
			g_gesture_state = 2; // S2 primeiro (de cima para baixo)
			g_gesture_timer = GESTURE_TIMEOUT_TICKS;
		}
	} else if(g_gesture_state == 1) {
		// Já detetou S1, agora espera S2
		if(s2) {
			// Gesto de BAIXO para CIMA -> ABRIR
			control_motor(ABRIR);
			g_gesture_state = 0;
			g_gesture_timer = 0;
		}
	} else if(g_gesture_state == 2) {
		// Já detetou S2, agora espera S1
		if(s1) {
			// Gesto de CIMA para BAIXO -> FECHAR
			control_motor(FECHAR);
			g_gesture_state = 0;
			g_gesture_timer = 0;
		}
	}
	s1_prev = s1;
	s2_prev = s2;
}
// --- FUNÇÃO DE INICIALIZAÇÃO DE HARDWARE ---
void inic(void) {
	onda1Hz_init();
	pwm_Servo_init();
	buttons_inic();
	sei();
}
// --- MÁQUINA DE ESTADOS PRINCIPAL (NON-BLOCKING) ---
void inic_non_blocking(void) {
	switch(g_init_state) {
		case 0:
			inic_i2c();
			START_NB_DELAY_MS(50); // Power-on wait (50ms)
			g_init_state = 1;
			break;
		case 1:
			if(IS_DELAY_FINISHED() == 1) {
				lcd_write_nibble(0x03, 0);
				START_NB_DELAY_MS(5);
				g_init_state = 2;
			}
			break;
		case 2:
			if(IS_DELAY_FINISHED() == 1) {
				lcd_write_nibble(0x03, 0);
				START_NB_DELAY_MS(1);
				g_init_state = 3;
			}
			break;
		case 3:
			if(IS_DELAY_FINISHED() == 1) {
				lcd_write_nibble(0x03, 0);
				START_NB_DELAY_MS(1);
				g_init_state = 4;
			}
			break;
		case 4:
			if(IS_DELAY_FINISHED() == 1) {
				lcd_write_nibble(0x02, 0); // 4-bit mode
				lcd_init_setup_cmds();
				lcd_set_cursor(0, 0);
				lcd_write_string("Projeto LABSI");
				START_NB_DELAY_MS(1500);
				g_init_state = 5;
			}
			break;
		case 5:
			if(IS_DELAY_FINISHED() == 1) {
				lcd_clear();
				detect_sensors();
				START_NB_DELAY_MS(10);
				g_init_state = 6;
			}
			break;
		case 6:
			if(IS_DELAY_FINISHED() == 1) {
				lcd_set_cursor(0, 0);
				if(g_sensor_count == 0) {
					lcd_write_string("Sem Sensores");
				} else if(g_sensor_count == 1) {
					lcd_write_string("1 Sensor Pronto");
				} else {
					lcd_write_string("2 Sensores");
					lcd_set_cursor(1, 0);
					lcd_write_string("Prontos");
				}
				START_NB_DELAY_MS(1500);
				g_init_state = 7;
			}
			break;
		case 7:
			if(IS_DELAY_FINISHED() == 1) {
				lcd_clear();
				g_setup_done = 1;
				g_init_state = 255;
			}
			break;
	}
}
// --- LOOP PRINCIPAL ---
int main(void) {
	inic();
	char buffer[32];
	while(1) {
		// 1. GESTÃO DA INICIALIZAÇÃO (NON-BLOCKING)
		if(!g_setup_done) {
			inic_non_blocking();
		} else {
			// 2. CÓDIGO OPERACIONAL PRINCIPAL (SÓ EXECUTA APÓS SETUP)
			if(g_flag_2ms) {
				g_flag_2ms = 0;
				// 1. LEITURA DE LUX e MÉDIA
				g_lux_value = bh1750_read_sensors();
				g_averaged_lux = average_lux(g_lux_value);
				// 2. Controlo dependendo do modo
				static uint8_t sonar_tick = 0;
				if(g_operating_mode == MODE_AUTOMATIC) {
					adjust_setpoint_control();
					servo_control_automatic();
				} else {
					// MODO MANUAL: reconhecimento de gestos com sonares
					sonar_tick++;
						if(sonar_tick >= 25) { // ~50 ms
							sonar_tick = 0;
							update_manual_gestures();
						}
				}
				// Estado do motor para debug em LCD
				char motor_status = 'E';
				switch(g_estado_atual) {
					case ABRIR:
						motor_status = 'A';
						break;
					case PARAR:
						motor_status = 'P';
						break;
					case FECHAR:
						motor_status = 'F';
						break;
				}
				// 3. ATUALIZAÇÃO DO DISPLAY (200ms)
				if(g_display_counter >= 100) {
					g_display_counter = 0;
					// Define valor de display (Preview vs Fixo)
					uint16_t display_setpoint = g_target_lux;
					if(g_fix_state != 0) {
						display_setpoint = g_last_setpoint_value;
					}
					sprintf(buffer, "MODO:%s SET:%4u",
						(g_operating_mode == MODE_AUTOMATIC) ? "A" : "M", display_setpoint);
					lcd_set_cursor(0, 0);
					lcd_write_string(buffer);
					sprintf(buffer, "Lux:%4u Motor:%c", g_averaged_lux, motor_status);
					lcd_set_cursor(1, 0);
					lcd_write_string(buffer);
				}
			}
		}
	}
	return 0;
}