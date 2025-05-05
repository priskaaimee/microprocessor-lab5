#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

void initSPI();
void spiWriteByte(uint8_t data);
void write_Max7219(unsigned char address, unsigned char data);
void init_MAX7219(void);
void printByte(const uint8_t pattern[8]);

#endif
