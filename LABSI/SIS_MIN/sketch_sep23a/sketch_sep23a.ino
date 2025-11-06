#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h> // Include standard delay header for convenience

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

// BH1750 Defines
#define BH1750_ADDR 0x23  // Default I2C address (0x46 for R/W)
#define BH1750_POWER_DOWN 0x00
#define BH1750_POWER_ON 0x01
#define BH1750_CONT_HIGH_RES_MODE 0x10  // Continuous measurement, 1 lux resolution, 180ms typical time

volatile uint8_t contador = 0;
volatile char flag_2ms = 0;
uint8_t display_counter = 0;
uint16_t lux_value = 0;

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
void lcd_write_nibble(uint8_t nibble) {
    if (nibble & 0b00000001) LCD_D4; else LCD_D4_LOW;
    if (nibble & 0b00000010) LCD_D5; else LCD_D5_LOW;
    if (nibble & 0b00000100) LCD_D6; else LCD_D6_LOW;
    if (nibble & 0b00001000) LCD_D7; else LCD_D7_LOW;
    lcd_pulse_en();
}
void lcd_write_byte(uint8_t data, uint8_t rs) {
    if (rs) LCD_RS; else LCD_RS_LOW;
    lcd_write_nibble(data >> 4);  // High nibble
    lcd_write_nibble(data & 0x0F);  // Low nibble
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
    LCD_RS_LOW;
    lcd_write_nibble(0x03); delay_ms(5);
    lcd_write_nibble(0x03); delay_us(100);
    lcd_write_nibble(0x03); delay_us(100);
    lcd_write_nibble(0x02); delay_us(100);  // Switch to 4-bit mode
    
    lcd_write_byte(0x28, 0);  // Function set: 4-bit, 2 lines, 5x8 dots
    lcd_write_byte(0x0C, 0);  // Display on, cursor off, blink off
    lcd_write_byte(0x01, 0);  // Clear Display
    delay_ms(2);  // Clear takes time
    lcd_write_byte(0x06, 0);  // Entry mode: Increment cursor, no display shift
}
void lcd_clear(void) {
    lcd_write_byte(0x01, 0); delay_ms(2);
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

// --- I2C (TWI) Functions ---
void i2c_init(void) {
    // Set SCL frequency to ~100kHz (for F_CPU=16MHz, Prescaler=1)
    // TWBR = (F_CPU / SCL - 16) / (2 * Prescaler)
    // TWBR = (16000000 / 100000 - 16) / 2 = 72
    TWSR = 0;  // Prescaler 1
    TWBR = 72;  // Bit rate register for ~100kHz
    TWCR = (1 << TWEN);  // Enable TWI
}
void i2c_start(void) {
    // Send START condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}
void i2c_stop(void) {
    // Send STOP condition
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}
void i2c_write(uint8_t data) {
    // Load data into TWI data register
    TWDR = data;
    // Start transmission
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}
uint8_t i2c_read_ack(void) {
    // Start receiving with ACK after reception
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}
uint8_t i2c_read_nack(void) {
    // Start receiving with NACK after reception (last byte)
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}
uint8_t i2c_get_status(void) {
    return TWSR & 0x11111000;
}

// Helper to send a command to the BH1750
uint8_t bh1750_tx(uint8_t data) {
    i2c_start();
    if (i2c_get_status() != 0x08) return 0; // START condition transmitted
    
    i2c_write((BH1750_ADDR << 1) | 0); // SLA+W
    if (i2c_get_status() != 0x18) return 0; // SLA+W transmitted, ACK received
    
    i2c_write(data); // Command
    if (i2c_get_status() != 0x28) return 0; // Data byte transmitted, ACK received
    
    i2c_stop();
    return 1;
}

void bh1750_init(void) {
    // Send Power Up
    bh1750_tx(BH1750_POWER_ON);
    delay_ms(10); // Short power-up delay
    // Set Continuous High Resolution Mode
    bh1750_tx(BH1750_CONT_HIGH_RES_MODE);
}

// BH1750 read function
uint16_t read_bh1750(void) {
    // Assumes BH1750 is already in Continuous High-Res Mode (0x10) from init.
    // We only perform the read sequence here.
    uint16_t lux = 0;
    uint8_t high_byte, low_byte;
    
    i2c_start(); // START
    if (i2c_get_status() != 0x08) return 0; 
    
    i2c_write((BH1750_ADDR << 1) | 1);  // SLA+R (0x47)
    if (i2c_get_status() != 0x40) {  // SLA+R transmitted, ACK received
        i2c_stop();
        return 0;  
    }
    
    high_byte = i2c_read_ack();  // Read high byte, send ACK
    low_byte = i2c_read_nack();  // Read low byte, send NACK (end of read)
    
    i2c_stop();
    
    // Calculate lux: (raw_data) / 1.2
    // To avoid floating point, use integer math: (raw * 10) / 12
    uint16_t raw = (high_byte << 8) | low_byte;
    lux = (raw * 10) / 12;
    return lux;
}
    
void inic(void) {
    onda1Hz_init(); // Initialize Timer0 for 2ms interrupts
    i2c_init();  // Initialize I2C (TWI) communication
    bh1750_init(); // Send power-up and continuous measurement mode command
    lcd_init();  // Initialize LCD
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_write_string("Lux Meter V1.0");  // Boot message
    lcd_set_cursor(1, 0);
    lcd_write_string("BH1750 Ready");
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
                lux_value = read_bh1750();
                
                // Line 1: Display Lux
                lcd_set_cursor(0, 0);
                // Use %5d to pad with spaces and " lx" for units
                sprintf(buffer, "Luminosidade: %5d lx", lux_value);
                lcd_write_string(buffer);
                
                // Clear the second line (optional, for aesthetics)
                lcd_set_cursor(1, 0);
                lcd_write_string("                "); // 16 spaces
            }
        }
    }
    return 0;
}