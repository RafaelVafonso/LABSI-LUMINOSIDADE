#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 16000000UL    

// LCD PIN MAPPINGS (4-bit mode)
#define LCD_RS PORTD |= (1 << PD7)  // RS Pin 7
#define LCD_RS_LOW PORTD &= ~(1 << PD7)

#define LCD_EN PORTB |= (1 << PB0)  // EN Pin 8
#define LCD_EN_LOW PORTB &= ~(1 << PB0)

#define LCD_D4 PORTD |= (1 << PD0)  // D4 Pin 0
#define LCD_D4_LOW PORTD &= ~(1 << PD0)

#define LCD_D5 PORTD |= (1 << PD1)  // D5 Pin 1
#define LCD_D5_LOW PORTD &= ~(1 << PD1)

#define LCD_D6 PORTD |= (1 << PD4)  // D6 Pin 4
#define LCD_D6_LOW PORTD &= ~(1 << PD4)

#define LCD_D7 PORTD |= (1 << PD5)  // D7 Pin 5
#define LCD_D7_LOW PORTD &= ~(1 << PD5)


#define BH1750_ADDR1 0x23  //Default I2C address (0x23) ADDR GND
#define BH1750_ADDR2 0x5C // ADDR VCC
#define BH1750_POWER_DOWN 0x00
#define BH1750_POWER_ON 0x01
#define BH1750_CONT_HIGH_RES_MODE2 0x11  // Continuous measurement, 0.5 lux resolution, 180ms typical time


volatile uint8_t contador = 0;
volatile char flag_2ms = 0;
volatile uint8_t display_counter = 0;
uint16_t lux_value = 0;

//Detect BH1750
uint8_t sensor_count = 0;  // 0, 1, ou 2
uint8_t sensor1_present = 0;
uint8_t sensor2_present = 0;

// Averaging: Buffer for last 5 readings
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

// --- LCD Functions ---
void lcd_pulse_en(void) {
    LCD_EN;
    short_delay();  // 1 us high level
    LCD_EN_LOW;
    delay_us(50);  // Low level during 50 us
}
void lcd_write_data(uint8_t bits_on) {
    if (bits_on & 0b00000001) LCD_D4; else LCD_D4_LOW;
    if (bits_on & 0b00000010) LCD_D5; else LCD_D5_LOW;
    if (bits_on & 0b00000100) LCD_D6; else LCD_D6_LOW;
    if (bits_on & 0b00001000) LCD_D7; else LCD_D7_LOW;
    lcd_pulse_en();
}
void lcd_write_byte(uint8_t data, uint8_t rs) {
    if (rs) LCD_RS; else LCD_RS_LOW;
    lcd_write_data(data >> 4);  // High bits
    lcd_write_data(data & 0b00001111);  // Low bits
    if(rs){
        delay_us(100);
    }else delay_ms(2);
}
void lcd_init(void) {
    // Setup LCD control and data pins as outputs
    DDRB |= (1 << PB0);  // EN output
    DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD4) | (1 << PD5) | (1 << PD7);  // D4-D7 and RS outputs
    
    delay_ms(50);  // Power-on wait
    
    // 4-bit init sequence
    LCD_RS_LOW;// Instruction
    lcd_write_data(0b00000011); delay_ms(5);
    lcd_write_data(0b00000011); delay_us(100);
    lcd_write_data(0b00000011); delay_us(100);
    lcd_write_data(0b00000010); delay_us(100);  //4-bit 
    
    lcd_write_byte(0b00101000, 0);  // Function set: 4-bit, 2 lines, 5x8 dots

    lcd_write_byte(0b00001100, 0);  // Display on, cursor off, blink off
    lcd_write_byte(0b00000001, 0);  // Clear Display
    delay_ms(2);  // Clear takes time
    lcd_write_byte(0b00000110, 0);  // Entry mode: Increment cursor, no display shift
}
void lcd_clear(void) {
    lcd_write_byte(0b00000001, 0); delay_ms(2);
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

void inic_i2c(void){
    TWCR |= (1<< TWEN); //twi controla os pinos scl e sda
    TWSR = 0;// prescaler a 1
    TWBR =17;// SCL a 320khz
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
    // Load SLA_W into TWI data register
    TWDR = data;
    // Start transmission
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack(void) {
    // Set TWINT (clear flag), TWEN, and TWEA (send ACK after receiving byte)
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    
    // Wait for the TWINT flag to be set, indicating byte reception is complete
    while (!(TWCR & (1 << TWINT)));
    
    // Return the received data from the TWI data register
    return TWDR;
}

uint8_t i2c_read_nack(void) {
    // Set TWINT (clear flag), TWEN, but DO NOT set TWEA (send NACK after receiving byte)
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    // Wait for the TWINT flag to be set, indicating byte reception is complete
    while (!(TWCR & (1 << TWINT)));
    
    // Return the received data
    return TWDR;
}

uint8_t i2c_get_status(void){
    return TWSR & 0b11111000;
}

uint8_t bh1750_send(uint8_t addr, uint8_t data){
    i2c_start();
        if(i2c_get_status() != 0x08){ // check for start
            i2c_stop();
            return 0;
        }

    i2c_write(addr << 1 | 0);
        if(i2c_get_status() != 0x18){//SLA+W, ACK received
            i2c_stop();
            return 0;
        }

    i2c_write(data);
        if (i2c_get_status() != 0x28){ // Data byte transmitted, ACK received
            i2c_stop();
            return 0;            
        } 
i2c_stop();
return 1;
}
uint16_t bh1750_read(uint8_t addr){
    uint8_t high_byte, low_byte;
    
    i2c_start();
    if (i2c_get_status() != 0x08){ // check for start
            i2c_stop();
            return 0;            
        }
    
    i2c_write((addr << 1) | 1);  // SLA+R 
    if (i2c_get_status() != 0x40){ //SLA+R has been transmitted; ACK received
            i2c_stop();
            return 0;            
        }
    
    high_byte = i2c_read_ack();  // Read high byte, send ACK
    low_byte = i2c_read_nack();  // Read low byte, send NACK (end of read)
    
    i2c_stop();
    
    // Calculate lux: (raw_data) / 1.2
    uint16_t data = (high_byte << 8) | low_byte;

    return (uint16_t)((float)data / 1.2);
}
void detect_sensors(void){
    sensor1_present = bh1750_send(BH1750_ADDR1, BH1750_POWER_ON) && bh1750_send(BH1750_ADDR1, BH1750_CONT_HIGH_RES_MODE2);
    sensor2_present = bh1750_send(BH1750_ADDR2, BH1750_POWER_ON) && bh1750_send(BH1750_ADDR2, BH1750_CONT_HIGH_RES_MODE2);
    sensor_count = sensor1_present + sensor2_present;
}
uint16_t bh1750_read_sensors(void){
    uint16_t lux1 = 0, lux2 = 0;
    if (sensor1_present) lux1 = bh1750_read(BH1750_ADDR1);
    if (sensor2_present) lux2 = bh1750_read(BH1750_ADDR2);
    
    if (sensor_count == 1) {
        
        if (lux1 > 0) {
            return lux1;
        } else {
            
            return lux2;
        }
    }
    if (sensor_count == 2) return(uint16_t)((float)(lux1 + lux2)) / 2;
    return 0;  // No sensors
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
// --- Timer 0 for 2ms interrupt (for timing and toggling PD6) ---
void onda1Hz_init(void) {
    DDRD |= (1 << PD6); // PD6 as output (LED check)
    PORTD &= ~(1 << PD6); // Initial state low

    // Timer0 CTC mode (Clear Timer on Compare Match)
    TCCR0A = (1 << WGM01);   // CTC mode
    TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler 1024
    // OCR0A = (F_CPU / (Prescaler * Target_Freq)) - 1
    // OCR0A = (16000000 / (1024 * 500)) - 1 = 31.25 - 1 = 30
    OCR0A = 31; // 31 clock cycles for a 2ms interrupt (500Hz)
    TIMSK0 = (1 << OCIE0A); // Enable Output Compare Match A Interrupt
    sei(); // Enable global interrupts
}

void inic(void) {
    onda1Hz_init();
    
    lcd_init();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_write_string("Lux Meter V1.0");
    delay_ms(1500);
    
    inic_i2c(); // TWI/I2C initialization
    bh1750_init(); // BH1750 power on and mode set

    lcd_clear();
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
    char buffer[32];  // Increased buffer size to prevent overflow
    while (1) {
        if (flag_2ms) {
            flag_2ms = 0;  // Clear flag            
            
            // Update every 200ms (100 * 2ms). 
            // This ensures the 180ms measurement time for High-Res mode is met.
            if (display_counter >= 100) { 
                display_counter = 0;
                
                // Read BH1750 lux value
                lux_value = bh1750_read_sensors();
                averaged_lux = average_lux(lux_value);    
                // Line 1: Display Lux
                lcd_set_cursor(0, 0);
                // Use %4d to pad with spaces and " lx" for units
                sprintf(buffer, "Lux: %4u", averaged_lux);
                lcd_write_string(buffer);
                
                // Clear the second line
                lcd_set_cursor(1, 0);
                sprintf(buffer, "Sensors: %d", sensor_count);
                lcd_write_string(buffer); // 16 spaces
            }
        }
    }
    return 0;
}