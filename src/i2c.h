#include <avr/io.h>

#ifndef I2C_H
#define I2C_H

// MPU6050 address and registers
// Arduino will communicate through these register
#define MPU6050_ADDR 0x68 // MPU address to communicate via  I2C (AD0 is set to LOW)
#define PWR_MGMT_1 0x6B   // MPU power management address via  I2C
#define ACCEL_XOUT_H 0x3B // MPU x-axis output first 8 bits via  I2C
#define ACCEL_XOUT_L 0x3C // MPU x-axis output last 8 bits via  I2C
#define ACCEL_YOUT_H 0x3D // MPU y-axis output first 8 bits via  I2C
#define ACCEL_YOUT_L 0x3E // MPU y-axis output last 8 bits via  I2C
#define ACCEL_ZOUT_H 0x3F // MPU z-axis output first 8 bits via  I2C
#define ACCEL_ZOUT_L 0x40 // MPU z-axis output last 8 bits via  I2C
#define ACCEL_CONF 0x1C   // MPU acceleration configuration via I2C


void initI2C();
void StartI2C_Trans(unsigned char SLA);
void StopI2C_Trans();
void Write(unsigned char data);
void Read_from(unsigned char SLA, unsigned char MEMADDRESS);
unsigned char Read_data();
void initAccel();

#endif