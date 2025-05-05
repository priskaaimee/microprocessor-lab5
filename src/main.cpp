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
    debounce_press,    // Debouncing after press
    wait_release,      // Waiting for button release
    debounce_release,  // Debouncing after release
} current_state;

volatile current_state state = wait_press; // Start in waiting-for-press state
volatile uint8_t btn = 0; // Flag to indicate button press (not actively used in this version)

// State machine for displaying smiley or frowny face
enum FaceState { SMILE, SAD };
volatile FaceState faceState = SMILE;  // Initialize faceState to SMILE

static const uint8_t SMILEFACE[8] = {0x3C, 0x42, 0xA5, 0x81, 0xA5, 0x99, 0x42, 0x3C}; // Data for the smiley face display
static const uint8_t SADFACE[8]   = {0x3C, 0x42, 0xA5, 0x81, 0x99, 0xA5, 0x42, 0x3C};  // Data for the sad face display

// Alarm state
enum AlarmState { ALARM_OFF, ALARM_ON };
AlarmState alarmState = ALARM_OFF; // Initialize alarmState to ALARM_OFF (buzzer off)

// Threshold for tilt detection (45 degrees in radians)
#define THRESHOLD_ANGLE 0.785  // Define the threshold angle as 45 degrees in radians (approximately)

int main() {
    Serial.begin(9600); // begin serial monitor
    // Initialize the serial communication at 9600 baud rate for debugging output.

    initTimer1();
    // Initialize Timer 1, which is used for the delayMs function (millisecond delays).

    init_MAX7219();
    // Initialize the MAX7219 LED matrix driver.

    initSwitchPD3();
    // Initialize the push button switch connected to pin PD3.

    initPWM();
    // Initialize the PWM functionality for the buzzer.

    initI2C(); // initialize I2C communication
    // Initialize the I2C communication module.

    initAccel(); // initialize I2C communication with MPU 6050 accelerometer
    // Initialize the MPU 6050 accelerometer.

    sei(); // Enable global interrupts
    // Enable global interrupts, allowing the interrupt service routine (ISR) for the button press to function.

    printByte(SMILEFACE); // show the smiley face
    // Display the smiley face on the LED matrix as the initial state.

    int16_t x = 0, y = 0, z = 0;         // variable to store the accelerometer reading
    // Declare 16-bit integer variables to store the raw accelerometer readings from the MPU 6050.

    double x_acc = 0, y_acc = 0, z_acc = 0; // variable to count angle dgeree
    // Declare double-precision floating-point variables to store the accelerometer readings in g-force units.

    // variable to smooth the reading
    static double smooth_roll = 0;
    static double smooth_pitch = 0;
    // Declare static double-precision floating-point variables to store the smoothed roll and pitch angles.
    // Static ensures these variables retain their values across function calls.

    while (1) { // Main program loop, runs indefinitely
        if (state != wait_press) // make sure button is debounced
        {
            state = wait_press;
            delayMs(50);
        }
        // This if statement implements a simple debouncing mechanism for the push button.
        // If the button state is not wait_press, it means a button event is being processed.
        // It resets the state to wait_press and introduces a 50ms delay to ignore rapid transitions.

        Read_from(MPU6050_ADDR, ACCEL_XOUT_H); // reading value of x (first half)
        x = Read_data();
        Read_from(MPU6050_ADDR, ACCEL_XOUT_L); // read the value of x (second half)
        x = (x << 8) | Read_data();             // data needs to be shifted to fit second half
        x_acc = (x / 16384.0);                 // normalize the data by deviding with 2^14 (2^14 is the range of Accelerometer)
        // These lines read the X-axis accelerometer data from the MPU 6050.
        // ACCEL_XOUT_H and ACCEL_XOUT_L are registers in the MPU 6050.
        // The high and low bytes are combined to form a 16-bit value, which is then normalized to g-force units.

        Read_from(MPU6050_ADDR, ACCEL_YOUT_H); // reading value of y (first half)
        y = Read_data();
        Read_from(MPU6050_ADDR, ACCEL_YOUT_L); // read the value of y (second half)
        y = (y << 8) | Read_data();             // data needs to be shifted to fit second half
        y_acc = (y / 16384.0);                 // normalize the data by deviding with 2^14 (2^14 is the range of Accelerometer)
        // These lines read the Y-axis accelerometer data, similar to the X-axis.

        Read_from(MPU6050_ADDR, ACCEL_ZOUT_H); // reading value of z (first half)
        z = Read_data();
        Read_from(MPU6050_ADDR, ACCEL_ZOUT_L); // read the value of z (second half)
        z = (z << 8) | Read_data();             // data needs to be shifted to fit second half
        z_acc = (z / 16384.0);                 // normalize the data by deviding with 2^14 (2^14 is the range of Accelerometer)
        // These lines read the Z-axis accelerometer data, similar to the X and Y axes.

        StopI2C_Trans(); // to stop the device from overflowing
        // Stop the I2C transmission to prevent potential issues with the I2C bus.

        // printing values of the MPU 6050 accelerometer
        Serial.print("x: ");
        Serial.println(x);
        Serial.print(" y: ");
        Serial.println(y);
        Serial.print(" z: ");
        Serial.println(z);
        // Print the raw accelerometer values (x, y, z) to the serial monitor for debugging.

        double roll = atan2(y_acc, sqrt(x_acc * x_acc + z_acc * z_acc));         // count the degree respect to y
        double pitch = atan2(x_acc, sqrt(y_acc * y_acc + z_acc * z_acc)) - 0.12; // count the degree respect to x
        // Calculate the roll and pitch angles from the accelerometer data using the atan2 function.
        // A small offset (0.12) is subtracted from the pitch, likely for calibration.

        // low pass filter to counter sudden spike
        smooth_roll = 0.9 * smooth_roll + 0.1 * roll;
        smooth_pitch = 0.9 * smooth_pitch + 0.1 * pitch;
        // Apply a simple low-pass filter to smooth the roll and pitch values, reducing noise and jitter.
        // The filter gives more weight to the previous smoothed value (0.9) and less to the current raw value (0.1).

        // printing value of degree
        Serial.print("roll: ");
        Serial.println(roll);
        Serial.print("pitch: ");
        Serial.println(pitch);
        // Print the calculated roll and pitch angles to the serial monitor.

        if (smooth_roll > 0.785 || smooth_roll < -0.785 || smooth_pitch > 0.785 || smooth_pitch < -0.785) { // check  if the degree is more than 45 degree or less than -45 degree (45 degree to the other side)
            faceState = SAD; // make the 8x8 LED matrix to show SAD face
            alarmState = ALARM_ON;      // change the buzzer state to true (starting the chirping buzzer)
        } else {
            faceState = SMILE; // make 8x8 LED matrix to show SMILE face
        }
        // Check if the absolute value of the smoothed roll or pitch angle exceeds the threshold (45 degrees, or approximately 0.785 radians).
        // If it does, set the faceState to SAD and the alarmState to ALARM_ON.
        // Otherwise, set the faceState to SMILE.

        if (alarmState) { // check to turn on buzzer
            chirpBuzzer(0.9); // buzzer chirping
        }
        // If the alarmState is ALARM_ON, call the chirpBuzzer function to activate the buzzer.

        if (faceState == SMILE) {
            printByte(SMILEFACE); // show the smiley face
        } else {
            printByte(SADFACE); // show the sad face
        }
        // Display either the SMILEFACE or SADFACE on the LED matrix based on the current faceState.
    }
}

ISR(INT3_vect){
    state = wait_release; // change the state of button
    stopBuzzer();                    // turn off the buzzer
    //  Buzzer = false;                     // change the buzzer state to false (stopping the repeat chirping buzzer)
    delayMs(100);                        // debounce the button
    alarmState = ALARM_OFF; // change the alarm state to false (turning off the buzzer)
}
// Interrupt Service Routine (ISR) for external interrupt INT3 (connected to the push button).
// This ISR is triggered when the button is pressed (falling edge).
// It sets the button state to wait_release, stops the buzzer, introduces a debouncing delay, and turns off the alarm.
