#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

#include "include/linmath.h"
#include "include/i2csoft.h"

#define FL_MOTOR_PIN DDD3
#define FR_MOTOR_PIN DDB1

#define BL_MOTOR_PIN DDD5
#define BR_MOTOR_PIN DDD6

void setupMotors(){
    DDRD |= _BV(FL_MOTOR_PIN);
    DDRB |= _BV(FR_MOTOR_PIN);
    DDRD |= _BV(BL_MOTOR_PIN);
    DDRD |= _BV(BR_MOTOR_PIN);

    OCR0A = 0;
    OCR0B = 0;
    TCCR0A |= (_BV(COM0A1) | _BV(WGM00) | _BV(WGM01));
    TCCR0B |= _BV(CS01);

    OCR2A = 0;
    OCR2B = 0;
    TCCR2A |= (_BV(COM2A1) | _BV(WGM20) | _BV(WGM21));
    TCCR2B |= _BV(CS21);
}

int main(void){
    setupMotors();
    int testpower = 0;
    while(1){
        testpower++;
        if (testpower > 100){
            testpower = 0;
            OCR0A = testpower;
            OCR0B = testpower;
            OCR2A = testpower;
            OCR2A = testpower;
        }
        _delay_ms(20);
    }
}
