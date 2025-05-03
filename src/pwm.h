#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>


void initPWM();
void changeDutyCycle(float dutyCycle);
void chirpBuzzer(float dutyCycle); // for buzzer to chirp (on, off,on,off)
void stopBuzzer();

#endif