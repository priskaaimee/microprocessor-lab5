#include <Arduino.h>
#include "SPI.h"
#include <avr/interrupt.h>
#include <avr/io.h>

#define wait_for_completion while(!(SPSR & (1<<SPIF)));


void initSPI(){
// set the SS, MOSI, and SCLK pin as output
DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2);

// set the MISO pin as input
DDRB &= ~(1 << DDB3);
// set SS high at first
PORTB |= (1 << PORTB0);
// enable the interrupt, SPI, master mode, CPOL, CPHA, default clock, and fosc/128
SPCR |= (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << CPOL)| (1 << CPHA)| (1 << SPR1) | (1 << SPR0);

}

// Send one byte over SPI
void spiWriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        PORTB &= ~(1 << PB1); // CLOCK TO LOW
   
        if (data & 0x80) {
            PORTB |= (1 << PB2); // Set DIN high
        } else {
            PORTB &= ~(1 << PB2); // Set DIN low
        }
        data <<= 1;
        PORTB |= (1 << PB1); // Clocl TO HIGH
      }
}

void write_Max7219(unsigned char address,unsigned char data)
{
    PORTB &= ~(1 << PB0);         // SS low to start transmission
    spiWriteByte(address);           // Send register address
    spiWriteByte(data);           // Send data
    PORTB |= (1 << PB0);          // SS high to end transmission
}

void init_MAX7219(void)
{
    initSPI(); // Initialize SPI
    write_Max7219(0x09, 0x00); //decoding ：BCD
    write_Max7219(0x0a, 0x03); //brightness
    write_Max7219(0x0b, 0x07); //scanlimit；8 LEDs
    write_Max7219(0x0c, 0x01); //power-down mode：0，normal mode：1
    write_Max7219(0x0f, 0x00); //test display：1；EOT，display：0
}

// Display 8-byte pattern
void printByte(const uint8_t pattern[8]) {
    for (int i = 0; i < 8; i++) {
        write_Max7219(i + 1, pattern[i]);
    }
}
