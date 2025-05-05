#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2c.h"
#include "pwm.h"
#include "timer.h"
#include "spi.h"
#include "switch.h"
#include <HardwareSerial.h> // Include the library for Serial communication

// Enum for states of the button press state machine
typedef enum {
  wait_press,        // Waiting for button press
  debounce_press,    // Debouncing after button press
  wait_release,      // Waiting for button release
  debounce_release,  // Debouncing after button release
} current_state;

volatile current_state state = wait_press; // Start in the waiting-for-press state
volatile uint8_t btn = 0; // Flag to indicate button press   

// State machine for displaying smiley or frowny face
enum FaceState { SMILE, SAD };
volatile FaceState faceState = SMILE;  // Start with the smiley face displayed

// Alarm state to control the buzzer
enum AlarmState { ALARM_OFF, ALARM_ON };
AlarmState alarmState = ALARM_OFF; // Start with the alarm off

// Threshold for tilt detection in degrees
#define THRESHOLD_ANGLE 45.0

int main() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  init_MAX7219();     // Initialize the 8x8 LED matrix
  initTimer1();       // Initialize Timer1 for delays
  initSwitchPD3();    // Initialize the button on pin PD3
  initPWM();          // Initialize PWM for the buzzer
  initI2C();          // Initialize I2C communication
  initAccel();        // Initialize the MPU6050 accelerometer
  sei();              // Enable global interrupts

  printByte(SMILEFACE); // Display the smiley face on the LED matrix initially

  int16_t x = 0, y = 0, z = 0;            // Variables to store raw accelerometer readings
  double x_acc = 0, y_acc = 0, z_acc = 0; // Variables to store normalized accelerometer readings

  // Variables to smooth the accelerometer readings using a low-pass filter
  static double smooth_roll = 0;
  static double smooth_pitch = 0;

  while (1) {
    // Ensure the button is debounced before proceeding
    if (state != wait_press) {
      state = wait_press; // Reset the state to wait for the next press
      delayMs(50);        // Add a small delay for debouncing
    }

    // Read accelerometer data from the MPU6050
    Read_from(MPU6050_ADDR, ACCEL_XOUT_H); // Read the high byte of the X-axis
    x = Read_data();
    Read_from(MPU6050_ADDR, ACCEL_XOUT_L); // Read the low byte of the X-axis
    x = (x << 8) | Read_data();            // Combine high and low bytes
    x_acc = (x / 16384.0);                 // Normalize the X-axis reading

    Read_from(MPU6050_ADDR, ACCEL_YOUT_H); // Read the high byte of the Y-axis
    y = Read_data();
    Read_from(MPU6050_ADDR, ACCEL_YOUT_L); // Read the low byte of the Y-axis
    y = (y << 8) | Read_data();            // Combine high and low bytes
    y_acc = (y / 16384.0);                 // Normalize the Y-axis reading

    Read_from(MPU6050_ADDR, ACCEL_ZOUT_H); // Read the high byte of the Z-axis
    z = Read_data();
    Read_from(MPU6050_ADDR, ACCEL_ZOUT_L); // Read the low byte of the Z-axis
    z = (z << 8) | Read_data();            // Combine high and low bytes
    z_acc = (z / 16384.0);                 // Normalize the Z-axis reading

    StopI2C_Trans(); // Stop I2C communication to prevent overflow

    // Print raw accelerometer readings to the serial monitor
    Serial.print("x: ");
    Serial.println(x);
    Serial.print(" y: ");
    Serial.println(y);
    Serial.print(" z: ");
    Serial.println(z);

    // Calculate roll and pitch angles in radians
    double roll = atan2(y_acc, sqrt(x_acc * x_acc + z_acc * z_acc));         // Roll angle based on Y-axis
    double pitch = atan2(x_acc, sqrt(y_acc * y_acc + z_acc * z_acc)) - 0.12; // Pitch angle based on X-axis

    // Apply a low-pass filter to smooth sudden spikes in roll and pitch
    smooth_roll = 0.9 * smooth_roll + 0.1 * roll;
    smooth_pitch = 0.9 * smooth_pitch + 0.1 * pitch;

    // Print roll and pitch angles to the serial monitor
    Serial.print("roll: ");
    Serial.println(roll);
    Serial.print("pitch: ");
    Serial.println(pitch);

    // Check if the tilt exceeds the threshold angle (45 degrees)
    if (smooth_roll > 0.785 || smooth_roll < -0.785 || smooth_pitch > 0.785 || smooth_pitch < -0.785) {
      faceState = SAD;       // Display a sad face on the LED matrix
      alarmState = ALARM_ON; // Turn on the alarm (buzzer)
    } else {
      faceState = SMILE;     // Display a smiley face on the LED matrix
    }

    // If the alarm is on, chirp the buzzer
    if (alarmState) {
      chirpBuzzer(0.9); // Generate a chirping sound with the buzzer
    }

    // Update the LED matrix based on the face state
    if (faceState == SMILE) {
      printByte(SMILEFACE); // Show the smiley face
    } else {
      printByte(SADFACE);   // Show the sad face
    }
  }
}

// Interrupt Service Routine for the button press (INT3)
ISR(INT3_vect) {
  state = wait_release; // Change the state to wait for button release
  stopBuzzer();         // Turn off the buzzer
  delayMs(100);         // Add a delay for debouncing
}
