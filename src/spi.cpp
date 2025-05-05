#include <Arduino.h>
#include "SPI.h"
#include <avr/interrupt.h>
#include <avr/io.h>

#define wait_for_completion while(!(SPSR & (1<<SPIF))); // Macro to wait for SPI transfer completion. Checks the SPIF flag in SPSR.

void initSPI(){
    // set the SS, MOSI, and SCLK pin as output
    DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2);
    //   DDRB |= (1 << DDB0);  // Set PB0 (SS) as output
    //   DDRB |= (1 << DDB1);  // Set PB1 (SCK) as output
    //   DDRB |= (1 << DDB2);  // Set PB2 (MOSI) as output

    // set the MISO pin as input
    DDRB &= ~(1 << DDB3);
    //   DDRB &= ~(1 << DDB3); // Set PB3 (MISO) as input

    // set SS high at first
    PORTB |= (1 << PORTB0);
    //   PORTB |= (1 << PORTB0); // Set PB0 (SS) high initially (deselect any slave)

    // enable the interrupt, SPI, master mode, CPOL, CPHA, default clock, and fosc/128
    SPCR |= (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << CPOL)| (1 << CPHA)| (1 << SPR1) | (1 << SPR0);
    //   SPCR |= (1 << SPIE);  // Enable SPI interrupt (not used in this code)
    //   SPCR |= (1 << SPE);   // Enable SPI
    //   SPCR |= (1 << MSTR);  // Set SPI to Master mode
    //   SPCR |= (1 << CPOL);  // Set Clock Polarity (CPOL = 1: SCK high when idle)
    //   SPCR |= (1 << CPHA);  // Set Clock Phase (CPHA = 1: Sample on rising edge)
    //   SPCR |= (1 << SPR1) | (1 << SPR0); // Set SPI clock rate to fosc/128 (SPR1=1, SPR0=1)
}

// Send one byte over SPI
void spiWriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {   // Loop through each bit of the byte
        PORTB &= ~(1 << PB1);   // CLOCK TO LOW
        //   PORTB &= ~(1 << PB1);   // Set SCK (PB1) low before sending bit

        if (data & 0x80) {   // Check the most significant bit (MSB)
            PORTB |= (1 << PB2);   // Set DIN high
            //   PORTB |= (1 << PB2);   // If MSB is 1, set MOSI (PB2) high
        } else {
            PORTB &= ~(1 << PB2);   // Set DIN low
            //   PORTB &= ~(1 << PB2);   // If MSB is 0, set MOSI (PB2) low
        }
        data <<= 1;   // Shift data left by one bit to process the next bit
        PORTB |= (1 << PB1);   // Clocl TO HIGH
        //   PORTB |= (1 << PB1);   // Set SCK (PB1) high to clock the bit
    }
}

void write_Max7219(unsigned char address,unsigned char data)
{
    PORTB &= ~(1 << PB0);            // SS low to start transmission
    //   PORTB &= ~(1 << PB0);            // Set SS (PB0) low to select the MAX7219
    spiWriteByte(address);            // Send register address
    //   spiWriteByte(address);            // Send the register address to the MAX7219
    spiWriteByte(data);            // Send data
    //   spiWriteByte(data);            // Send the data to be written to the register
    PORTB |= (1 << PB0);             // SS high to end transmission
    //   PORTB |= (1 << PB0);             // Set SS (PB0) high to deselect the MAX7219
}

void init_MAX7219(void)
{
    initSPI(); // Initialize SPI
    write_Max7219(0x09, 0x00); //decoding ：BCD
    //   write_Max7219(0x09, 0x00); // Set decode mode to no-decode (register 0x09, value 0x00)
    write_Max7219(0x0a, 0x03); //brightness
    //   write_Max7219(0x0a, 0x03); // Set display brightness (register 0x0A, value 0x03)
    write_Max7219(0x0b, 0x07); //scanlimit；8 LEDs
    //   write_Max7219(0x0b, 0x07); // Set scan limit to display all 8 digits/rows (register 0x0B, value 0x07)
    write_Max7219(0x0c, 0x01); //power-down mode：0，normal mode：1
    //   write_Max7219(0x0c, 0x01); // Set display to normal mode (register 0x0C, value 0x01)
    write_Max7219(0x0f, 0x00); //test display：1；EOT，display：0
    //   write_Max7219(0x0f, 0x00); // Disable display test mode (register 0x0F, value 0x00)
}

// Display 8-byte pattern
void printByte(const uint8_t pattern[8]) {
    for (int i = 0; i < 8; i++) {   // Loop through each byte in the pattern (each row of the matrix)
        write_Max7219(i + 1, pattern[i]); // Write the byte to the corresponding digit/row register
        //   write_Max7219(i + 1, pattern[i]); // Write the byte to the MAX7219 (registers 1-8 control the rows)
    }
}
