#include <avr/io.h>
#include <util/delay.h>

// Constants
#define BUZZER_PIN PE5 // OC2B (on ATmega2560)

void initPWM() {
    // Set buzzer pin as output
    DDRD |= (1 << BUZZER_PIN);

    // Set Timer2 to Fast PWM mode, non-inverting on OC2B
    TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM
    TCCR2B |= (1 << WGM22) | (1 << CS21); // prescaler 8

    // Initial compare value for chirp (will be varied)
    OCR2A = 255; // TOP
    OCR2B = 127; // 50% duty cycle
}

void startBuzzer() {
    for (int freq = 100; freq <= 800; freq += 20) {
        uint16_t top = (16000000UL / (8UL * freq)) - 1;
        if (top > 255) top = 255; // clamp
        OCR2A = top;
        OCR2B = top / 2; // 50% duty
        _delay_ms(50); // Delay to create chirp ramp
    }
}

void stopBuzzer() {
    OCR2B = 0; // No output
}