#ifndef I2C_H
#define I2C_H

void initI2C();
void startI2C_Trans(unsigned char SLA);
void stopI2C_Trans();
void writeTo(unsigned char SLA, unsigned char REGADDRESS, unsigned char data);
void write(unsigned char data);
void readFrom(unsigned char SLA, unsigned char MEMADDRESS);
unsigned char readData();

#endif