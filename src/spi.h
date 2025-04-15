#ifndef SPI_H
#define SPI_H

#include <stdint.h>

void SPI_MASTER_Init();
void spi_send_byte(uint8_t data);
void max7219_send_data(uint8_t address, uint8_t data);
void max7219_init();
void display_smiley();
void display_frowny();

#endif