#include <Arduino.h>
#include "SPI.h"
#include <avr/interrupt.h>
#include <avr/io.h>

#define wait_for_completion while(!(SPSR & (1<<SPIF)));

void initSPI(){
    // set the SS, MOSI, and SCLK pin as output
    DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2);
    // DDRB |= (1 << DDB0); // Set SS (PB0) as output
    // DDRB |= (1 << DDB1); // Set SCK (PB1) as output
    // DDRB |= (1 << DDB2); // Set MOSI (PB2) as output

    // set SS high at first
    PORTB |= (1 << PORTB0);
    // PORTB |= (1 << PORTB0); // Set SS (PB0) high initially (deselect slave)

    // enable the  SPI, master mode, CPOL, CPHA, default clock, and fosc/128
    //ADXL345 CPOL = 1, CPHA = 1
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << CPOL)| (1 << CPHA)| (1 << SPR1) | (1 << SPR0);
    // SPCR |= (1 << SPE);  // Enable SPI
    // SPCR |= (1 << MSTR); // Set as Master
    // SPCR |= (1 << CPOL); // Clock Polarity (CPOL=1: SCK high when idle)
    // SPCR |= (1 << CPHA); // Clock Phase (CPHA=1: Sample on trailing edge)
    // SPCR |= (1 << SPR1) | (1 << SPR0); // SPI Clock Rate (fosc/128)
}

void spiWriteByte(uint8_t data) {
    SPDR = data;
    // Load data into SPI Data Register (SPDR) for transmission
    while (!(SPSR & (1 << SPIF))); // Wait for transmission complete
    // Wait until SPIF flag is set, indicating transmission completion
}

void write_Max7219(unsigned char address,unsigned char data)
{
    PORTB &= ~(1 << PB0);          // SS low to start transmission
    // PORTB &= ~(1 << PB0);          // Select MAX7219 by setting SS low
    spiWriteByte(address);          // Send register address
    // spiWriteByte(address);          // Send the register address
    spiWriteByte(data);            // Send data
    // spiWriteByte(data);            // Send the data
    PORTB |= (1 << PB0);           // SS high to end transmission
    // PORTB |= (1 << PB0);           // Deselect MAX7219 by setting SS high
}

void init_MAX7219(void)
{
    initSPI(); // Initialize SPI
    write_Max7219(0x09, 0x00); //decoding ：BCD
    // write_Max7219(0x09, 0x00); // Set decode mode (no decode)
    write_Max7219(0x0a, 0x03); //brightness
    // write_Max7219(0x0a, 0x03); // Set intensity (brightness)
    write_Max7219(0x0b, 0x07); //scanlimit；8 LEDs
    // write_Max7219(0x0b, 0x07); // Set scan limit (display all digits)
    write_Max7219(0x0c, 0x01); //power-down mode：0，normal mode：1
    // write_Max7219(0x0c, 0x01); // Set display on
    write_Max7219(0x0f, 0x00); //test display：1；EOT，display：0
    // write_Max7219(0x0f, 0x00); // Disable display test
}

// Display 8-byte pattern
void printByte(const uint8_t pattern[8]) {
    for (int i = 0; i < 8; i++) {
        write_Max7219(i + 1, pattern[i]);
        // write_Max7219(i + 1, pattern[i]); // Send row data to MAX7219
    }
}
