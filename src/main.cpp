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
volatile uint8_t btn = 0; // Flag to indicate button press   

// State machine for displaying smiley or frowny face
enum FaceState { SMILE, SAD };
volatile FaceState faceState = SMILE;  


// Alarm state
enum AlarmState { ALARM_OFF, ALARM_ON };
AlarmState alarmState = ALARM_OFF;

// Threshold for tilt detection
#define THRESHOLD_ANGLE 45.0


int main() {
    Serial.begin(9600); // begin serial monitor
    init_MAX7219();
    initTimer1();
    initSwitchPD3();
    initPWM();
    initI2C();
    initSwitchPD3();
    initAccel(); // initialize I2C communication with MPU 6050 accelerometer
    sei(); // Enable global interrupts

    printByte(SMILEFACE);


  int16_t x = 0, y = 0, z = 0;            // variable to store the accelerometer reading
  double x_acc = 0, y_acc = 0, z_acc = 0; // variable to count angle dgeree

  // variable to smooth the reading
  static double smooth_roll = 0;
  static double smooth_pitch = 0;

    while (1) {
        if (state != wait_press) // make sure button is debounced
    {
        state = wait_press;
      delayMs(50);
    }

        Read_from(MPU6050_ADDR, ACCEL_XOUT_H); // reading value of x (first half)
    x = Read_data();
    Read_from(MPU6050_ADDR, ACCEL_XOUT_L); // read the value of x (second half)
    x = (x << 8) | Read_data();            // data needs to be shifted to fit second half
    x_acc = (x / 16384.0);                 // normalize the data by deviding with 2^14 (2^14 is the range of Accelerometer)

    Read_from(MPU6050_ADDR, ACCEL_YOUT_H); // reading value of y (first half)
    y = Read_data();
    Read_from(MPU6050_ADDR, ACCEL_YOUT_L); // read the value of y (second half)
    y = (y << 8) | Read_data();            // data needs to be shifted to fit second half
    y_acc = (y / 16384.0);                 // normalize the data by deviding with 2^14 (2^14 is the range of Accelerometer)

    Read_from(MPU6050_ADDR, ACCEL_ZOUT_H); // reading value of z (first half)
    z = Read_data();
    Read_from(MPU6050_ADDR, ACCEL_ZOUT_L); // read the value of z (second half)
    z = (z << 8) | Read_data();            // data needs to be shifted to fit second half
    z_acc = (z / 16384.0);                 // normalize the data by deviding with 2^14 (2^14 is the range of Accelerometer)

    StopI2C_Trans(); // to stop the device from overflowing
    // printing values of the MPU 6050 accelerometer
    Serial.print("x: ");
    Serial.println(x);
    Serial.print(" y: ");
    Serial.println(y);
    Serial.print(" z: ");
    Serial.println(z);

    double roll = atan2(y_acc, sqrt(x_acc * x_acc + z_acc * z_acc));         // count the degree respect to y
    double pitch = atan2(x_acc, sqrt(y_acc * y_acc + z_acc * z_acc)) - 0.12; // count the degree respect to x

    // low pass filter to counter sudden spike
    smooth_roll = 0.9 * smooth_roll + 0.1 * roll;
    smooth_pitch = 0.9 * smooth_pitch + 0.1 * pitch;


    // printing value of degree
    Serial.print("roll: ");
    Serial.println(roll);
    Serial.print("pitch: ");
    Serial.println(pitch);

    if (smooth_roll > 0.45 || smooth_roll < -0.45 || smooth_pitch > 0.45 || smooth_pitch < -0.45) // check  if the degree is more than 45 degree or less than -45 degree (45 degree to the other side)
    {
        faceState = SAD; // make the 8x8 LED matrix to show SAD face
        alarmState = ALARM_ON;    // change the buzzer state to true (starting the chirping buzzer)
    }
    else
    {
        faceState = SMILE; // make 8x8 LED matrix to show SMILE face
    }
    if (alarmState) // check to turn on buzzer
    {
      chirpBuzzer(0.9); // buzzer chirping
    }
    if (faceState == SMILE) {
        printByte(SMILEFACE); // show the smiley face
    } else {
        printByte(SADFACE); // show the sad face
    }
  }
}

ISR(INT3_vect){
    state = wait_release; // change the state of button
  stopBuzzer();               // turn off the buzzer
//   Buzzer = false;                // change the buzzer state to false (stopping the repeat chirping buzzer)
  delayMs(100);                   // debounce the button
}
