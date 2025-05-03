#include "timer.h"

#include <avr/io.h>


void initPWM() {
    // Set buzzer pin as output
    DDRE |= (1 << PE3);

    TCCR3B &= ~(1 << WGM33);               // set WGM3 to 0
    TCCR3A |= (1 << WGM31) | (1 << WGM30); // set WGM1 and WGM0 to 1
    TCCR3B |= (1 << WGM32);                // set WGM2 to 1

    // Set non-inverting mode for OCR3A (clear on compare match, set on bottom)
    TCCR3A |= (1 << COM3A1);
    TCCR3A &= ~(1 << COM3A0);

    // Set presclar to 1 for Fast PWM, creating constant signal
    TCCR3B |= (1 << CS30);
    TCCR3B &= ~((1 << CS31) | (1 << CS32));

    OCR3A = 0;  // Initial duty cycle 0
}

void changeDutyCycle(float dutyCycle) {
    OCR3A = 1023 * dutyCycle;
}

void chirpBuzzer(float dutyCycle) // for buzzer to chirp (on, off,on,off)
{
    changeDutyCycle(dutyCycle); //start buzzer
    delayMs(100);          // delay for chirp
    changeDutyCycle(0);    // no sound in buzzer
    delayMs(100);          // delay for chirp
}

void stopBuzzer() {
    OCR3A = 0; // No output
}