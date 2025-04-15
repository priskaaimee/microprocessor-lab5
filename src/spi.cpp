#include <Arduino.h>
#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define DDR_SPI DDRB

#define DD_SS DDB0

#define DD_SCK DDB1

#define DD_MOSI DDB2


#define SPI_PORT PORTB
#define SPI_SS_BIT PORTB0

void SPI_MASTER_Init() {
    // Set MOSI, SCK as Output
    DDR_SPI |= (1 << DD_MOSI) | (1 << DD_SCK) | (1 << DD_SS);
    // Set SS high
    SPI_PORT |= (1 << SPI_SS_BIT);
    // Enable SPI, Set as Master
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (1 << SPR1); // Prescaler: F_CPU/16
}