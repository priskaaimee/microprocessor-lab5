#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

static const uint8_t SMILEFACE[8] = {0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C};
static const uint8_t SADFACE[8]   = {0x3C, 0x42, 0xA5, 0x81, 0x99, 0xA5, 0x42, 0x3C};

void initSPI();
void spiWriteByte(uint8_t data);
void write_Max7219(unsigned char address, unsigned char data);
void init_MAX7219(void);
void printByte(const uint8_t pattern[8]);

#endif