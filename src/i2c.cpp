#include "i2c.h"
#include <avr/io.h>
#include "Arduino.h" // While not strictly necessary for AVR I2C, it's often included for Arduino compatibility and Serial debugging
#include "timer.h" // Include the custom timer library for delay functions

#define WAIT_TRANSFER() while (!(TWCR & (1 << TWINT))) // Macro to wait for I2C transfer completion. Waits until TWINT flag is set.
/* TWSTA untuk start condition
// TWINT untuk flag
// TWEA untuk enable ack
// TWSTO untuk stop condition
// TWEN enable
// TWIE interrupt
*/
// Multi-line comment (Indonesian) explaining TWCR register bits:
// TWSTA: Set to 1 to initiate a START condition.
// TWINT: TWI Interrupt Flag. Set by hardware after an event. Clear by writing '1'.
// TWEA: TWI Enable Acknowledge. Enables sending/receiving ACKs.
// TWSTO: Set to 1 to initiate a STOP condition.
// TWEN: TWI Enable. Enables the TWI interface.
// TWIE: TWI Enable Interrupt. Enables interrupts.

void initI2C()
{
    PRR0 &= ~(1 << PRTWI); // turn on I2C
    // Clear the PRTWI bit in PRR0 to enable power to the TWI module.

    // set prescalar 1 meaning  4^1 = 4 (01)
    TWSR |= (1 << TWPS0);
    TWSR &= ~(1 << TWPS1);
    // Configure the TWI Status Register (TWSR) to set the prescaler.
    // TWPS0 = 1, TWPS1 = 0 sets the prescaler to 4.

    TWBR = 0xC6;                        // 10KHz SCL clock, (198 in decimal)
    // Set the TWI Bit Rate Register (TWBR) to control the SCL clock frequency.
    // The value 0xC6 (198 decimal) is calculated to achieve approximately 10kHz SCL.

    PORTD |= (1 << PD0);                        // pull-up for SDA to ensure reliable data transfer
    PORTD |= (1 << PD1);                        // pull-up for SCL to ensure reliable data transfer
    // Enable internal pull-up resistors on SDA (PD0) and SCL (PD1) pins.
    // These are crucial for I2C to work correctly.

    TWCR |= (1 << TWINT) | (1 << TWEN); // enable two wire interface
    // Enable the TWI interface (TWEN) and clear the TWINT flag.
}

void StartI2C_Trans(unsigned char SLA)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // TWSTA set 1 to initaite start
    // Initiate a START condition on the I2C bus.
    // Set TWSTA, TWEN, and clear TWINT.

    WAIT_TRANSFER();                                     // wait until transmission is done
    // Wait for the START condition to be transmitted.

    TWDR = (SLA << 1);                                   // set the slave address and shift it by 1  to be transmitted
    // Load the slave address (SLA) into the TWI Data Register (TWDR).
    // Shift left by 1 to make space for the R/W bit (0 for write).

    TWCR = (1 << TWINT) | (1 << TWEN); // begin the transmission
    // Start transmitting the slave address.

    WAIT_TRANSFER();                                     // wait until transmission is done
    // Wait for the slave address to be sent and ACK/NACK to be received.
}

void StopI2C_Trans()
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // TWSTO set 1 to stop transmit
    // Send a STOP condition to terminate the I2C communication.
    // Set TWSTO, TWEN, and clear TWINT.
}

void Write(unsigned char data)
{
    TWDR = data;                                   // data will be sent to the slave register
    // Load the data to be sent into the TWDR.

    TWCR = (1 << TWINT) | (1 << TWEN); // enable Twin wire interface
    // Transmit the data byte.

    WAIT_TRANSFER();                                     // wait until data successfully transfered
    // Wait for the data byte to be transmitted and ACK/NACK to be received.
}

void Read_from(unsigned char SLA, unsigned char MEMADDRESS)
{
    StartI2C_Trans(SLA);                                     // set the slave address
    // Initiate a write transaction to send the memory address to read from.

    Write(MEMADDRESS);                                     // send memory register to slave
    // Send the memory address (MEMADDRESS) to the slave.

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // TWSTA set 1 to initaite start
    // Send a repeated START condition to switch to read mode.

    WAIT_TRANSFER();                                     // wait until transmission is done
    // Wait for the repeated START condition to be sent.

    TWDR = (SLA << 1) | 0x01;                                   // set the slave address and shift it by 1 and add read bit (master will read) that will be transmitted
    // Load the slave address (SLA) into TWDR for reading.
    // Shift left by 1 and set the LSB to 1 to indicate a read operation.

    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // begin the transmission and ACK by master
    // Start reading from the slave. Enable ACK (TWEA) to acknowledge received bytes.

    WAIT_TRANSFER();                                      // wait until transmission is done
    // Wait for the slave to send a byte and for ACK to be sent.

    TWCR = (1 << TWINT) | (1 << TWEN); // send NACk meaning done reading
    // Send a NACK to indicate the master is done reading.

    WAIT_TRANSFER();                                      // wait until NACK is finished
    // Wait for the NACK to be sent.

    StopI2C_Trans(); // stop the transmission
    // Send a STOP condition to end the transaction.
}

unsigned char Read_data()
{
    return TWDR; // return what is inside of TWDR
    // Return the data received from the slave (stored in TWDR).
}

void initAccel() // initialize the I2C connection with MPU 6050
{
    // write to power config to wake MPU 6050 up
    StartI2C_Trans(MPU6050_ADDR);
    Write(PWR_MGMT_1);
    Write(0x00); // write bit
    StopI2C_Trans();
    // Initialize the MPU 6050:
    // 1. Start I2C communication with the MPU 6050.
    // 2. Write to the PWR_MGMT_1 register to wake the MPU 6050.
    // 3. Send 0x00 to select the internal oscillator.
    // 4. Stop I2C communication.

    // write to acceleration config to set the sensitivity of the accelerometer to +- 2g
    StartI2C_Trans(MPU6050_ADDR);
    Write(ACCEL_CONF);
    Write(0x00); // write bit
    StopI2C_Trans();
    // Configure the accelerometer:
    // 1. Start I2C communication.
    // 2. Write to the ACCEL_CONF register.
    // 3. Send 0x00 to set the accelerometer range to +/- 2g.
    // 4. Stop I2C communication.
}
