#include "switch.h"
#include <avr/io.h>

/*
 * Initializes pull-up resistor on PB3 and sets it into input mode
 */
void initSwitchPD2(){
    PORTD |= (1 << PD2); //set pull-up resistor on PD2
    DDRD &= ~(1 << DDD2); //set PD2 as input

    PCICR |= (1 << PCIE0); //enable pin change interrupt 0
    PCMSK0 |= (1 << PCINT2); //enable pin change interrupt on PD2  
}

void silence(){
    //silence the buzzer
    PORTB &= ~(1 << PB0); //set PB0 low
    DDRB &= ~(1 << DDB0); //set PB0 as input
}