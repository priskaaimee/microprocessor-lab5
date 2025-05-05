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

    ICR3 = 1000; // Set PWM frequency to ~2kHz → ICR3 = 2000000 / 2000 = 1000

    OCR3A = 0;  // Initial duty cycle 0
}

void changeDutyCycle(float dutyCycle) {
    OCR3A = (uint16_t)(ICR3 * dutyCycle); // set output compare based on TOP
}

void playFrequency(float frequency, float dutyCycle) {
    // Use prescaler = 8, so timer clock = 2 MHz
    const float timerClock = 2000000.0;

    // Calculate TOP value for desired frequency
    uint16_t top = (uint16_t)(timerClock / frequency);

    ICR3 = top; // Set frequency
    OCR3A = (uint16_t)(top * dutyCycle); // Set duty cycle
}

void chirpBuzzer(float dutyCycle) {// for buzzer to chirp (on, off,on,off)
    TCCR3A |= (1 << COM3A1);
    for (int i = 1000; i < 4000; i++) {
        playFrequency(i + 15, dutyCycle); // Play frequency from 1kHz to 4kHz
        delayMs(1); // Delay for 100ms
    }
    for (int i = 4000; i > 1015; i--) {
        playFrequency(i - 15, dutyCycle); // Play frequency from 4kHz to 1kHz
        delayMs(1); // Delay for 100ms
    }
}

void stopBuzzer() {
    OCR3A = 0; // No output
    TCCR3A &= ~(1 << COM3A1);
}
