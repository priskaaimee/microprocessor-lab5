#include <avr/io.h>       
#include <util/delay.h>    
#include <avr/interrupt.h>   
#include "i2c.h"           
#include "pwm.h"            
#include "timer.h"         
#include "spi.h"             
#include "switch.h"          
#include <HardwareSerial.h> 

// --- State Machine for Button Press Detection ---
// This section defines the different states the system goes through when detecting a button press.
typedef enum {
    wait_press,         // The system is idle, waiting for the button to be pressed.
    debounce_press,     // A brief delay to ignore rapid bouncing of the button contacts after a press is detected.
    wait_release,       // The button is currently pressed, and the system is waiting for it to be released.
    debounce_release,   // A brief delay to ignore rapid bouncing of the button contacts after the button is released.
} current_state;

volatile current_state state = wait_press; // Declares a volatile variable to store the current state of the button press detection, initialized to 'wait_press'.
                                         // 'volatile' is used because this variable can be changed by an interrupt routine.
volatile uint8_t btn = 0;                // Declares a volatile flag (0 or 1) to indicate if the button has been pressed (though it's not actively used in the main loop).

// --- State Machine for Displaying Face on LED Matrix ---
// This section defines the states for controlling what is displayed on the 8x8 LED matrix.
enum FaceState { SMILE, SAD };
volatile FaceState faceState = SMILE;    // Declares a volatile variable to store the current face to display, initialized to 'SMILE'.

// --- State Machine for Alarm (Buzzer) Control ---
// This section defines the states for turning the buzzer on or off.
enum AlarmState { ALARM_OFF, ALARM_ON };
AlarmState alarmState = ALARM_OFF;       // Declares a variable to store the current alarm state, initialized to 'ALARM_OFF'.

// --- Threshold for Tilt Detection ---
// This constant defines the angle (in degrees, later converted to radians) beyond which a "tilt" is considered detected.
#define THRESHOLD_ANGLE 45.0

int main() {
    Serial.begin(9600);      // Initializes serial communication with a baud rate of 9600 bits per second.
                             // This allows the microcontroller to send data to a connected computer for debugging or monitoring.
    init_MAX7219();         // Calls a function (defined in 'spi.h' or a related file) to initialize the MAX7219 chip,
                             // which controls the 8x8 LED matrix display.
    initTimer1();            // Calls a function (defined in 'timer.h') to initialize Timer1. Timers are used for various tasks,
                             // including generating precise delays or timing events.
    initSwitchPD3();         // Calls a function (defined in 'switch.h') to configure the digital pin PD3 as an input for the button.
                             // It likely sets up any necessary pull-up or pull-down resistors.
    initPWM();               // Calls a function (defined in 'pwm.h') to initialize Pulse Width Modulation on a specific pin.
                             // PWM is used to generate an analog-like signal to control the buzzer's volume or produce sound.
    initI2C();               // Calls a function (defined in 'i2c.h') to initialize the I2C communication protocol.
                             // I2C is used to communicate with the MPU6050 accelerometer.
    initAccel();             // Calls a function (likely defined in a separate accelerometer-related file) to initialize the MPU6050 sensor.
                             // This involves configuring its registers for reading acceleration data.
    sei();                   // Enables global interrupts. This allows the microcontroller to respond to events like button presses
                             // by executing Interrupt Service Routines (ISRs).

    printByte(SMILEFACE);    // Calls a function (likely defined in 'spi.h') to send a specific byte pattern (representing a smiley face)
                             // to the LED matrix for display. 'SMILEFACE' is likely a pre-defined constant.

    int16_t x = 0, y = 0, z = 0;          // Declares integer variables to store the raw acceleration readings from the MPU6050 for the X, Y, and Z axes.
    double x_acc = 0, y_acc = 0, z_acc = 0; // Declares double-precision floating-point variables to store the normalized acceleration values.

    // Variables to smooth the accelerometer readings using a low-pass filter.
    // This helps to reduce noise and sudden jumps in the readings, providing a more stable tilt detection.
    static double smooth_roll = 0;
    static double smooth_pitch = 0;

    while (1) { // This is the main program loop that runs indefinitely.
        // --- Button Debouncing (Simplified in Main Loop) ---
        // This section ensures that the system doesn't react to the rapid bouncing of the button contacts.
        // Instead of a dedicated state machine in the main loop, it briefly waits after a non-'wait_press' state.
        if (state != wait_press) {
            state = wait_press; // Resets the button state machine to wait for a new press after processing the previous one.
            delayMs(50);        // Introduces a 50-millisecond delay to allow the button contacts to stabilize.
        }

        // --- Read Accelerometer Data ---
        // This section reads the raw acceleration data from the MPU6050 sensor using I2C communication.
        Read_from(MPU6050_ADDR, ACCEL_XOUT_H); // Sends a command to the MPU6050 to read the high byte of the X-axis acceleration data.
        x = Read_data();                     // Reads the received data byte and stores it in the 'x' variable.
        Read_from(MPU6050_ADDR, ACCEL_XOUT_L); // Sends a command to read the low byte of the X-axis acceleration data.
        x = (x << 8) | Read_data();          // Reads the low byte and combines it with the high byte (shifted left by 8 bits) to form a 16-bit signed integer.
        x_acc = (x / 16384.0);                // Normalizes the raw X-axis reading by dividing it by the sensitivity factor (16384 for +/- 2g range).

        Read_from(MPU6050_ADDR, ACCEL_YOUT_H); // Reads the high byte of the Y-axis acceleration.
        y = Read_data();
        Read_from(MPU6050_ADDR, ACCEL_YOUT_L); // Reads the low byte of the Y-axis acceleration.
        y = (y << 8) | Read_data();
        y_acc = (y / 16384.0);                // Normalizes the Y-axis reading.

        Read_from(MPU6050_ADDR, ACCEL_ZOUT_H); // Reads the high byte of the Z-axis acceleration.
        z = Read_data();
        Read_from(MPU6050_ADDR, ACCEL_ZOUT_L); // Reads the low byte of the Z-axis acceleration.
        z = (z << 8) | Read_data();
        z_acc = (z / 16384.0);                // Normalizes the Z-axis reading.

        StopI2C_Trans(); // Calls a function (defined in 'i2c.h') to stop the current I2C transmission.
                         // This is good practice to ensure the I2C bus is free for other communications.

        // --- Output Accelerometer Data to Serial Monitor ---
        // This section sends the raw accelerometer readings to the serial monitor for debugging or visualization.
        Serial.print("x: ");
        Serial.println(x);
        Serial.print(" y: ");
        Serial.println(y);
        Serial.print(" z: ");
        Serial.println(z);

        // --- Calculate Roll and Pitch Angles ---
        // This section calculates the roll and pitch angles (in radians) based on the accelerometer readings.
        // These calculations use trigonometric functions (atan2 and sqrt) to determine the device's orientation.
        double roll = atan2(y_acc, sqrt(x_acc * x_acc + z_acc * z_acc));         // Calculates the roll angle (rotation around the X-axis).
        double pitch = atan2(x_acc, sqrt(y_acc * y_acc + z_acc * z_acc)) - 0.12; // Calculates the pitch angle (rotation around the Y-axis), with a small offset.

        // --- Apply Low-Pass Filter for Smoothing ---
        // This section applies a simple low-pass filter to the calculated roll and pitch angles.
        // This helps to smooth out any sudden noise or fluctuations in the sensor readings.
        smooth_roll = 0.9 * smooth_roll + 0.1 * roll;   // Updates the smoothed roll value using a weighted average of the previous smoothed value and the current reading.
        smooth_pitch = 0.9 * smooth_pitch + 0.1 * pitch; // Updates the smoothed pitch value similarly.

        // --- Output Smoothed Angles to Serial Monitor ---
        Serial.print("roll: ");
        Serial.println(roll);
        Serial.print("pitch: ");
        Serial.println(pitch);

        // --- Tilt Detection and Face/Alarm Control ---
        // This section checks if the smoothed roll or pitch angles exceed the defined threshold.
        // If a tilt is detected, it sets the face to sad and turns the alarm on.
        if (smooth_roll > 0.785 || smooth_roll < -0.785 || smooth_pitch > 0.785 || smooth_pitch < -0.785) {
            faceState = SAD;         // Sets the face state to 'SAD' to display a frowny face.
            alarmState = ALARM_ON;   // Sets the alarm state to 'ALARM_ON' to activate the buzzer.
        } else {
            faceState = SMILE;       // If no tilt is detected, sets the face state back to 'SMILE'.
        }

        // --- Buzzer Control ---
        // This section checks the alarm state and activates the buzzer if it's turned on.
        if (alarmState) {
            chirpBuzzer(0.9); // Calls a function (defined in 'pwm.h') to generate a chirping sound on the buzzer.
                             // The '0.9' likely controls the frequency or duty cycle of the PWM signal.
        } else {
            stopBuzzer();      // Ensures the buzzer is off when the alarm state is 'ALARM_OFF'.
        }

        // --- Update LED Matrix Display ---
        // This section checks the face state and displays the corresponding face on the LED matrix.
        if (faceState == SMILE) {
            printByte(SMILEFACE); // Calls the function to display the smiley face pattern.
        } else {
            printByte(SADFACE);   // Calls the function to display the sad face pattern.
        }
    }
}

// --- Interrupt Service Routine for Button Press (INT3) ---
// This function is automatically executed when an interrupt occurs on the INT3 pin (connected to the button).
ISR(INT3_vect) {
    state = wait_release; // Immediately changes the button state to 'wait_release' as a press has been detected.
    stopBuzzer();       // Calls the function to turn off the buzzer when the button is pressed.
    delayMs(100);       // Introduces a 100-millisecond software delay for debouncing within the ISR.
                        // This helps to prevent multiple interrupts from being triggered by a single button press.
}
