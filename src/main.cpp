#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include "i2c.h"
#include "pwm.h"
#include "timer.h"
#include "switch.h"

// Declare SPI-related functions from spi.cpp
void SPI_MASTER_Init();
void display_smiley();
void display_frowny();

// MPU6050 constants
#define MPU6050_ADDRESS 0x68
#define POWER_MGMT_1 0x6B
#define THRESHOLD_ANGLE 45.0
#define PI 3.14159265

// State enums
enum AlarmState { ALARM_OFF, ALARM_ON };
enum FaceState { SMILEY, FROWNY };

// Function declarations
void updateDisplay(FaceState state);
float getAccelAngle(float a1, float a2, float a3);

int main() {
    // Init peripherals
    initI2C();
    SPI_MASTER_Init();
    initPWM();
    initSwitchPD2();
    initTimer1();
    sei(); // Enable global interrupts

    // Wake up MPU6050
    startI2C_Trans(MPU6050_ADDRESS << 1);
    write(POWER_MGMT_1);
    write(0x00);
    stopI2C_Trans();

    AlarmState alarmState = ALARM_OFF;
    FaceState faceState = SMILEY;

    while (1) {
        int16_t x_val, y_val, z_val;
        float x_accel, y_accel, z_accel;
        float roll, pitch;

        // --- Read X axis ---
        Read_from(MPU6050_ADDRESS << 1, 0x3B); // ACCEL_XOUT_H
        x_val = Read_data();
        Read_from(MPU6050_ADDRESS << 1, 0x3C); // ACCEL_XOUT_L
        x_val = (x_val << 8) | Read_data();
        x_accel = x_val / 16384.0;

        // --- Read Y axis ---
        Read_from(MPU6050_ADDRESS << 1, 0x3D); // ACCEL_YOUT_H
        y_val = Read_data();
        Read_from(MPU6050_ADDRESS << 1, 0x3E); // ACCEL_YOUT_L
        y_val = (y_val << 8) | Read_data();
        y_accel = y_val / 16384.0;

        // --- Read Z axis ---
        Read_from(MPU6050_ADDRESS << 1, 0x3F); // ACCEL_ZOUT_H
        z_val = Read_data();
        Read_from(MPU6050_ADDRESS << 1, 0x40); // ACCEL_ZOUT_L
        z_val = (z_val << 8) | Read_data();
        z_accel = (z_val / 16384.0) - 0.07; // adjust for calibration

        // --- Calculate roll and pitch in degrees ---
        roll = atan2(y_accel, sqrt(x_accel * x_accel + z_accel * z_accel)) * 180 / PI;
        pitch = atan2(x_accel, sqrt(y_accel * y_accel + z_accel * z_accel)) * 180 / PI;

        StopI2C_Trans();

        // Tilt detection
        bool tilted = (fabs(roll) > THRESHOLD_ANGLE || fabs(pitch) > THRESHOLD_ANGLE);

        // Alarm logic
        if (tilted && alarmState == ALARM_OFF) {
            alarmState = ALARM_ON;
            faceState = FROWNY;
            startBuzzer();
        }

        if (alarmState == ALARM_ON && IsSwitchPressed()) {
            alarmState = ALARM_OFF;
            faceState = SMILEY;
            stopBuzzer();
        }

        // Display state
        updateDisplay(faceState);

        delayMs(100);
    }
}

void updateDisplay(FaceState state) {
    if (state == SMILEY) {
        display_smiley();
    } else {
        display_frowny();
    }
}
