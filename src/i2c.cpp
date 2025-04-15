#include "i2c.h"
#include <avr/io.h>


void initI2C() {
    TWSR = 0x00; // Prescaler = 1
    TWBR = 0x48; // Set bit rate ~100kHz
    TWCR = (1 << TWEN); // Enable TWI

}

void startI2C_Trans(unsigned char SLA) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
    TWDR = (SLA << 1);
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));


}

void stopI2C_Trans(){
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void write(unsigned char data){

    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}


void writeTo(unsigned char SLA, unsigned char REGADDRESS, unsigned char data){
    startI2C_Trans(SLA);
    write(REGADDRESS);
    write(data);
    stopI2C_Trans();
}


void readFrom(unsigned char SLA, unsigned char MEMADDRESS){
    // Write mode to select register
    startI2C_Trans(SLA);
    write(MEMADDRESS);
    stopI2C_Trans();

    // Restart with read mode
    startI2C_Trans(SLA | 0x01); // read bit
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    unsigned char data = TWDR;
    stopI2C_Trans();
    
}

unsigned char Read_data(){
    return TWDR;
}