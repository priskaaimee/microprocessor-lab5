#include "i2c.h"
#include <avr/io.h>
#include "Arduino.h"
#include "timer.h"

#define WAIT_TRANSFER() while (!(TWCR & (1 << TWINT))) // check the flag to determine when the transfer is done (done when flag = 0)
/* TWSTA untuk start condition
// TWINT untuk flag
// TWEA untuk enable ack
// TWSTO untuk stop condition
// TWEN enable
// TWIE interrupt
*/

void initI2C()
{

    PRR0 &= ~(1 << PRTWI); // turn on I2C

    // set prescalar 1 meaning  4^1 = 4 (01)
    TWSR |= (1 << TWPS0);
    TWSR &= ~(1 << TWPS1);
    TWBR = 0xC6;                        // 10KHz SCL clock, (198 in decimal)
    PORTD |= (1 << PD0);                // pull-up for SDA to ensure reliable data transfer
    PORTD |= (1 << PD1);                // pull-up for SCL to ensure reliable data transfer
    TWCR |= (1 << TWINT) | (1 << TWEN); // enable two wire interface
}

void StartI2C_Trans(unsigned char SLA)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // TWSTA set 1 to initaite start
    WAIT_TRANSFER();                                  // wait until transmission is done

    TWDR = (SLA << 1);                 // set the slave address and shift it by 1  to be transmitted
    TWCR = (1 << TWINT) | (1 << TWEN); // begin the transmission
    WAIT_TRANSFER();                   // wait until transmission is done
}

void StopI2C_Trans()
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // TWSTO set 1 to stop transmit
}

void Write(unsigned char data)
{
    TWDR = data;                       // data will be sent to the slave register
    TWCR = (1 << TWINT) | (1 << TWEN); // enable Twin wire interface
    WAIT_TRANSFER();                   // wait until data successfully transfered
}
void Read_from(unsigned char SLA, unsigned char MEMADDRESS)
{
    StartI2C_Trans(SLA);                              // set the slave address
    Write(MEMADDRESS);                                // send memory register to slave
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // TWSTA set 1 to initaite start
    WAIT_TRANSFER();                                  // wait until transmission is done

    TWDR = (SLA << 1) | 0x01;                        // set the slave address and shift it by 1 and add read bit (master will read) that will be transmitted
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // begin the transmission and ACK by master
    WAIT_TRANSFER();                                 // wait until transmission is done

    TWCR = (1 << TWINT) | (1 << TWEN); // send NACk meaning done reading
    WAIT_TRANSFER();                   // wait until NACK is finished

    StopI2C_Trans(); // stop the transmission
}
unsigned char Read_data()
{
    return TWDR; // return what is inside of TWDR
}

void initAccel() // initialize the I2C connection with MPU 6050
{
    // write to power config to wake MPU 6050 up
    StartI2C_Trans(MPU6050_ADDR);
    Write(PWR_MGMT_1);
    Write(0x00); // write bit
    StopI2C_Trans();

    // write to acceleration config to set the sensitivity of the accelerometer to +- 2g
    StartI2C_Trans(MPU6050_ADDR);
    Write(ACCEL_CONF);
    Write(0x00); // write bit
    StopI2C_Trans();
}
