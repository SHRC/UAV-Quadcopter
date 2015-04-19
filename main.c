#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>

#include "include/linmath.h"
#include "include/i2csoft.h"

#define FL_MOTOR_PIN DDD3
#define FR_MOTOR_PIN DDB3

#define BL_MOTOR_PIN DDD5
#define BR_MOTOR_PIN DDD6

typedef enum {
    FL_MOTOR = 0, //Pin 3
    FR_MOTOR, //Pin 5
    RL_MOTOR, //Pin 6
    RR_MOTOR //Pin 11
} MOTOR_NUM;

void setupMotors(){
    DDRD |= _BV(FL_MOTOR_PIN);
    DDRB |= _BV(FR_MOTOR_PIN);
    DDRD |= _BV(BL_MOTOR_PIN);
    DDRD |= _BV(BR_MOTOR_PIN);

    OCR0A = 0;
    OCR0B = 0;
    TCCR0A |= (_BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01));
    TCCR0B |= _BV(CS01);

    OCR2A = 0;
    OCR2B = 0;
    TCCR2A |= (_BV(COM2A1) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21));
    TCCR2B |= _BV(CS21);
}

void setMotorPower(MOTOR_NUM n_motor, int power){
    switch(n_motor){
    case FL_MOTOR: OCR2B = power;
    case FR_MOTOR: OCR0B = power;
    case RL_MOTOR: OCR0A = power;
    case RR_MOTOR: OCR2A = power;
    }
}

int main(void){
    SoftI2CInit();
    setupMotors();
    int testpower = 0;
    while(1){
        testpower++;
        if (testpower > 256){
            testpower = 0;
        }
        setMotorPower(FR_MOTOR, testpower);
        setMotorPower(FL_MOTOR, testpower);
        setMotorPower(RR_MOTOR, testpower);
        setMotorPower(RL_MOTOR, testpower);
        _delay_ms(200);
    }
}
