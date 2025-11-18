#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 16000000UL    
#define LCD_I2C_ADDR 0x20  // Endereço I2C do PCF8574T (ajuste se necessário)

#define BH1750_ADDR1 0x23  //Default I2C address (0x23) ADDR GND
#define BH1750_ADDR2 0x5C // ADDR VCC
#define BH1750_POWER_DOWN 0x00
#define BH1750_POWER_ON 0x01
#define BH1750_CONT_HIGH_RES_MODE2 0x11  // Continuous measurement, 0.5 lux resolution, 180ms typical time

#define PCF_RS 0b10000000 // Register Select (P0)
#define PCF_EN 0b01000000 // Enable (P1)

volatile uint8_t contador = 0;
volatile char flag_2ms = 0;
volatile uint8_t display_counter = 0;
uint16_t lux_value = 0;
uint16_t servo_pwm_value = 188;

//Detect BH1750
uint8_t sensor_count = 0;  // 0, 1, ou 2
uint8_t sensor1_present = 0;
uint8_t sensor2_present = 0;

// Averaging: Buffer
#define AVG_SAMPLES 10
uint16_t lux_buffer[AVG_SAMPLES] = {0};
uint8_t buffer_index = 0;
uint16_t averaged_lux = 0;

// --- Utility Delays ---
void short_delay(void) {  // ~1 us
    __asm__ volatile (
        "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
        "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
        "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
        "nop\n\t" "nop\n\t" "nop\n\t" "nop\n\t"
        ::: "memory"
    );
}

void delay_us(uint16_t us) {
    for (uint16_t i = 0; i < us; i++) {
        uint8_t j;
        for (j = 0; j < 4; j++) {
            short_delay();
        }
    }
}

void delay_ms(uint8_t ms) {
    for (uint8_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

// --- Timer ISR (runs every 2ms) ---
ISR(TIMER0_COMPA_vect){
    flag_2ms = 1;
    contador++;
    if (contador >= 250) { // 250 * 2ms = 0.5s
        PORTD ^= (1 << PD6); // Toggle PD6 for 1Hz signal check
        contador = 0;   
    }
    display_counter++;
}

void inic_i2c(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
uint8_t i2c_get_status(void);

// Pins: P0=RS, P1=EN, P2=D4, P3=D5, P4=D6, P5=D7

void i2c_lcd_send(uint8_t data) {
    i2c_start();
    i2c_write(LCD_I2C_ADDR << 1); // SLA+W
    i2c_write(data);
    i2c_stop();
}


void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = 0;

    
    if (nibble & 0b1000) data |= (1 << 0); // D7 -> P7
    if (nibble & 0b0100) data |= (1 << 1); // D6 -> P6
    if (nibble & 0b0010) data |= (1 << 2); // D5 -> P5
    if (nibble & 0b0001) data |= (1 << 3); // D4 -> P4

    if (rs) {
        data |= PCF_RS; // RS -> P0
    }

    // EN high
    i2c_lcd_send(data | PCF_EN);  // Set EN (P1)
    delay_us(200);

    // EN low
    i2c_lcd_send(data & ~PCF_EN);  // Clear EN (P1)
    delay_us(200);
}

void lcd_write_byte(uint8_t data, uint8_t rs) {
    lcd_write_nibble(data >> 4, rs);  // High bits
    lcd_write_nibble(data & 0b00001111, rs);  // Low bits

    if (rs) {
        delay_us(100);  // Data write delay
    } else {
        delay_ms(2);    // Command delay
    }
}

void lcd_clear(void) {
    lcd_write_byte(0b00000001, 0); 
    delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr;
        if (row == 0) {
            addr = 0b10000000 + col;
}       else { // row = 1 
            addr = 0b11000000 + col;
}
lcd_write_byte(addr, 0);
}

void lcd_write_string(const char* str) {
    while (*str) {
        lcd_write_byte(*str++, 1);
    }
}

void lcd_init(void) {
    delay_ms(50);  // Power-on wait
    
    // 4-bit init sequence (send as commands, rs=0)
    lcd_write_nibble(0x03, 0); delay_ms(5);
    lcd_write_nibble(0x03, 0); delay_us(100);
    lcd_write_nibble(0x03, 0); delay_us(100);
    lcd_write_nibble(0x02, 0); delay_us(100);  // 4-bit mode
    
    lcd_write_byte(0b00101000, 0);  // Function set: 4-bit, 2 lines, 5x8 dots
    lcd_write_byte(0b00001100, 0);  // Display on, cursor off, blink off
    lcd_write_byte(0b00000001, 0);  // Clear Display
    delay_ms(2);
    lcd_write_byte(0b00000110, 0);  // Entry mode: Increment cursor, no display shift
}

// --- I2C Functions ---
void inic_i2c(void){
    TWCR |= (1<< TWEN); //twi controla os pinos scl e sda
    TWSR = 0;// prescaler a 1
    TWBR =72;// SCL a 100khz
    DDRC &= ~((1 << PC4) | (1 << PC5)); 
    PORTC |= (1 << PC4) | (1 << PC5);
}

void i2c_start(void){
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))); //espera pelo set da flag TWINT
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

// --- BH1750 Functions ---
uint8_t bh1750_send(uint8_t addr, uint8_t data) {
    i2c_start();
    i2c_write(addr << 1 | 0); // SLA+W
    i2c_write(data);
    i2c_stop();
    delay_ms(10);
    return 1; // assume success
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
    sensor1_present = bh1750_send(BH1750_ADDR1, BH1750_POWER_ON) && bh1750_send(BH1750_ADDR1, BH1750_CONT_HIGH_RES_MODE2);
    short_delay();
    sensor2_present = bh1750_send(BH1750_ADDR2, BH1750_POWER_ON) && bh1750_send(BH1750_ADDR2, BH1750_CONT_HIGH_RES_MODE2);
    sensor_count = sensor1_present + sensor2_present;
    delay_ms(200);
}

uint16_t bh1750_read_sensors(void){
    uint16_t lux1 = 0, lux2 = 0;
    if (sensor1_present) lux1 = bh1750_read(BH1750_ADDR1);
    if (sensor2_present) lux2 = bh1750_read(BH1750_ADDR2);
    if (sensor_count == 1) {
        if (lux1 > 0) return lux1;
        else return lux2;
    }
    if (sensor_count == 2) return (uint16_t)((float)(lux1 + lux2)) / 2;
    return 0;
}

void bh1750_init(void) {
    detect_sensors();
}

// --- Averaging Function ---
uint16_t average_lux(uint16_t new_reading) {
    lux_buffer[buffer_index] = new_reading;
    buffer_index = (buffer_index + 1) % AVG_SAMPLES;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < AVG_SAMPLES; i++) {
        sum += lux_buffer[i];
    }
    return sum / AVG_SAMPLES;
}

// --- Servo PWM Init ---
void pwm_Servo_init(void){
    DDRB |= (1 << PB1);
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << CS11) | (1 << CS10);
    ICR1 = 2500;
    OCR1A = 188;
}

// --- Timer 0 for 2ms interrupt ---
void onda1Hz_init(void) {
    DDRD |= (1 << PD6);
    PORTD &= ~(1 << PD6);
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS02) | (1 << CS00);
    OCR0A = 31;
    TIMSK0 = (1 << OCIE0A);
    sei();
}

void inic(void) {
    onda1Hz_init();
    inic_i2c();
    lcd_init();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_write_string("Lux Meter V1.0");
    delay_ms(1500);
    lcd_clear();
    bh1750_init();
    lcd_set_cursor(0, 0);
    if (sensor_count == 0) {
        lcd_write_string("No Sensor");
        delay_ms(1500);
    } else if (sensor_count == 1) {
        lcd_write_string("1 Sensor Ready");
        delay_ms(1500);
    } else {
        lcd_write_string("2 Sensors Ready");
        delay_ms(1500);
    }
    delay_ms(1500);
    lcd_clear();
}

int main(void) {
    inic();
    char buffer[32];
    while (1) {
        if (flag_2ms) {
            flag_2ms = 0;
            if (display_counter >= 100) {
                display_counter = 0;
                lux_value = bh1750_read_sensors();
                averaged_lux = average_lux(lux_value);
                lcd_set_cursor(0, 0);
                sprintf(buffer, "Lux: %4u    ", averaged_lux);
                lcd_write_string(buffer);
                lcd_set_cursor(1, 0);
                sprintf(buffer, "Sensors: %d", sensor_count);
                lcd_write_string(buffer);
            }
        }
    }
    return 0;
}
