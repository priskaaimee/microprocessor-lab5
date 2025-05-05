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

    // Set Fast PWM using ICR3 as TOP
    TCCR3A |= (1 << WGM31);
    TCCR3A &= ~(1 << WGM30);
    TCCR3B |= (1 << WGM32) | (1 << WGM33);

    // Non-inverting mode
    TCCR3A |= (1 << COM3A1);
    TCCR3A &= ~(1 << COM3A0);

    // Prescaler = 8
    TCCR3B &= ~((1 << CS32) | (1 << CS30));
    TCCR3B |= (1 << CS31);

    ICR3 = top; // Set frequency
    OCR3A = (uint16_t)(top * dutyCycle); // Set duty cycle
}

void chirpBuzzer(float dutyCycle) {// for buzzer to chirp (on, off,on,off)
    playFrequency(1000, 0.5); // Play 1kHz tone at 50% duty cycle
    delayMs(500);
    playFrequency(2000, 0.5); // Play 2kHz tone
    delayMs(500);
}

void stopBuzzer() {
    OCR3A = 0; // No output
}
