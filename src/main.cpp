#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
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
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define THRESHOLD 15000  // ~45 degree threshold (example)

// State enums
enum AlarmState { ALARM_OFF, ALARM_ON };
enum FaceState { SMILEY, FROWNY };

// Helper functions
int16_t readAxis(uint8_t highReg);
void updateDisplay(FaceState state);

int main() {
    // Init peripherals
    InitI2C();
    SPI_MASTER_Init(); // use friend's SPI function
    InitPWM();
    InitSwitch();
    InitTimer1();
    sei(); // Enable global interrupts

    // Wake up MPU6050
    StartI2C_Trans(MPU6050_ADDRESS << 1);
    Write(POWER_MGMT_1);
    Write(0x00);
    StopI2C_Trans();

    AlarmState alarmState = ALARM_OFF;
    FaceState faceState = SMILEY;

    while (1) {
        // Read Y and Z axes
        int16_t accY = readAxis(ACCEL_YOUT_H);
        int16_t accZ = readAxis(ACCEL_ZOUT_H);

        // Debug prints (optional - use only if Serial enabled)
        // Serial.print("Y: "); Serial.print(accY);
        // Serial.print(" Z: "); Serial.println(accZ);

        // Check if tilt exceeds threshold
        bool aboveThreshold = (abs(accY) > THRESHOLD || abs(accZ) > THRESHOLD);

        // Alarm trigger logic
        if (aboveThreshold && alarmState == ALARM_OFF) {
            alarmState = ALARM_ON;
            faceState = FROWNY;
            StartChirp();
        }

        // Button press to silence
        if (alarmState == ALARM_ON && IsSwitchPressed()) {
            alarmState = ALARM_OFF;
            faceState = SMILEY;
            StopChirp();
        }

        // Update display based on state
        updateDisplay(faceState);

        Delay_ms(100);
    }
}

int16_t readAxis(uint8_t highReg) {
    uint8_t high = Read_from(MPU6050_ADDRESS << 1, highReg);
    uint8_t low = Read_from(MPU6050_ADDRESS << 1, highReg + 1);
    return (int16_t)((high << 8) | low);
}

void updateDisplay(FaceState state) {
    if (state == SMILEY) {
        display_smiley(); // from spi.cpp
    } else {
        display_frowny(); // from spi.cpp
    }
}
