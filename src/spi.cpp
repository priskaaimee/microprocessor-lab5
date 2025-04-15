#include <Arduino.h>
#include "SPI.h"
#include <avr/interrupt.h>
#include <avr/io.h>

#define DDR_SPI DDRB

#define DD_SS DDB0

//Line for clock signal
#define DD_SCK DDB1

#define DD_MOSI DDB2

//set the line for the master to select which slave to send data to
#define SPI_PORT PORTB

#define SPI_SS_BIT PORTB0


void SPI_MASTER_Init() {
    DDR_SPI = (1 << DD_MOSI) | (1 << DD_SCK) | (1 << DD_SS);

    SPI_PORT |= (1 << SPI_SS_BIT);

    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (1 << SPR1) | (1 << SPR0);
}

// Function to send a single byte using a the SPI communication

void spi_send_byte (uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1 << SPIF)));
}

void max7219_send_data(uint8_t address, uint8_t data) {
    PORTB &= ~(1 << SPI_SS_BIT); // Select the slave (active low)
    spi_send_byte (address); 
    spi_send_byte (data);
    PORTB |= (1 << SPI_SS_BIT); // Deselect the slave (active high)
}

// Initialize the MAX7219 8x8 LED Matrix
void max7219_init() {
    max7219_send_data(0x0C, 0x01); // Normal operation (exit shutdown)
    max7219_send_data(0x09, 0x00); // Decode mode: No decode
    max7219_send_data(0x0A, 0x0F); // Intensity: Max
    max7219_send_data(0x0B, 0x07); // Scan limit: Display all 8 rows
    max7219_send_data(0x0F, 0x00); // Display test: Off
}

// Display a smiley face on the LED matrix
void display_smiley() {
    uint8_t smiley[8] = {
        0b00111100,
        0b01000010,
        0b10100101,
        0b10000001,
        0b10100101,
        0b10011001,
        0b01000010,
        0b00111100
    };

    for (uint8_t i = 0; i < 8; i++) {
        max7219_send_data(i + 1, smiley[i]);
    }
}

// Display a frowny face on the LED matrix
void display_frowny() {
    uint8_t frowny[8] = {
        0b00111100,
        0b01000010,
        0b10100101,
        0b10000001,
        0b10011001,
        0b10100101,
        0b01000010,
        0b00111100
    };

    for (uint8_t i = 0; i < 8; i++) {
        max7219_send_data(i + 1, frowny[i]);
    }
}