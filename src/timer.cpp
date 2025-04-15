#include "timer.h"

void initTimer1(){
    // CTC mode
    TCCR1A &= ~(1 << WGM10);
    TCCR1A &= ~(1 << WGM11);
    TCCR1B |= (1 << WGM12);
    TCCR1B &= ~(1 << WGM13);

    // set prescalar to 8
    TCCR1B |= (1 << CS01) | (1 << CS00);
    TCCR1B &= ~(1 << CS02);
}

void delayMs(unsigned int delay){
    int prescaler = 8;
    OCR1A = ((0.000001 * 16000000) / prescaler) - 1; // Set OCR1A to generate 1 µs delay with 16 MHz clock

    for (unsigned int i = 0; i < delay; i++) {
        TCNT1 = 0; // Reset timer count to 0

        // Wait until the compare match flag (OCF1A) is set
        while (!(TIFR1 & (1 << OCF1A)));

        // Clear OCF1A by writing 1 to it
        TIFR1 |= (1 << OCF1A);
    }
}