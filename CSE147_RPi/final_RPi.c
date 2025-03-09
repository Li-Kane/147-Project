#include "final_RPi.h"
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>

void init_shared_variable(SharedVariable* sv) {
    sv->bProgramExit = 0;
    sv->state = RUNNING;
    //sv->ROTstate = CLOCKWISE;
}

void ledInit(void) {
    softPwmCreate(PIN_SMD_RED, 0, 0xff);
    softPwmCreate(PIN_SMD_GRN, 0, 0xff);
    softPwmCreate(PIN_SMD_BLU, 0, 0xff);

    pinMode(PIN_DIP_RED, OUTPUT);
    pinMode(PIN_DIP_GRN, OUTPUT);
}

void init_sensors(SharedVariable* sv) {
    //pinMode(PIN_ALED, OUTPUT);
    pinMode(PIN_BUTTON, INPUT);
    int button;
    pinMode(PIN_LASER, OUTPUT);
    pinMode(PIN_SONIC_ECHO, INPUT);
    pinMode(PIN_SONIC_TRIG, OUTPUT);
    //pinMode(PIN_ROTARY_CLK, INPUT);
    //int A;
    //pinMode(PIN_ROTARY_DT, INPUT);
    //int B;
    //int pLast = digitalRead(PIN_ROTARY_CLK);
    float dist;
    long travelTime;
    long startTime;
    ledInit();
}

// Button (DONE)
void body_button(SharedVariable* sv) {
    int button = READ(PIN_BUTTON);
    if(button == 0){
        switch(sv->state){
            case RUNNING:
                sv->state = PAUSE;
                break;
            case PAUSE:
                sv->state = RUNNING;
                break;
        }
    }
}

// DIP two-color LED (DONE)
void body_twocolor(SharedVariable* sv) {
    switch(sv->state){
        case RUNNING:
            digitalWrite(PIN_DIP_GRN, LOW);
            digitalWrite(PIN_DIP_RED, HIGH);
            break;
        case PAUSE:
            digitalWrite(PIN_DIP_GRN, LOW);
            digitalWrite(PIN_DIP_RED, LOW);
            break;
    }
}

// SMD RGB LED (DONE)
void body_rgbcolor(SharedVariable* sv) {
    switch(sv->state){
        case RUNNING:
            softPwmWrite(PIN_SMD_RED, 0x00);
            softPwmWrite(PIN_SMD_GRN, 0xff);
            softPwmWrite(PIN_SMD_BLU, 0x00);
            break;
        case PAUSE:
            softPwmWrite(PIN_SMD_RED, 0x00);
            softPwmWrite(PIN_SMD_GRN, 0x00);
            softPwmWrite(PIN_SMD_BLU, 0x00);
            break;
    }
}

// LASER (DONE)
void body_laser(SharedVariable* sv) {
    switch(sv->state){
        case RUNNING:
            digitalWrite(PIN_LASER, HIGH);
            break;
        case PAUSE:
            digitalWrite(PIN_LASER, LOW);
            break;
    }
}

// SONIC SENSOR (ALMOST DONE? NEED TO FIX DELAY STATEMENTS?)
void body_sonic(SharedVariable* sv) {
    digitalWrite(PIN_SONIC_TRIG, 0);
	delayMicroseconds(500);
	digitalWrite(PIN_SONIC_TRIG, 1);
        delayMicroseconds(10);
        digitalWrite(PIN_SONIC_TRIG, 0);
	printf("Waiting for echo \n");
	while(digitalRead(PIN_SONIC_ECHO) == 0){
        }
	printf("echo reading went high \n");
	long startTime = micros();
        while(digitalRead(PIN_SONIC_ECHO) == 1){
	}
    long travelTime = micros() - startTime;
    float dist = travelTime / 58;
	printf("Measured a dist of %f\n", dist);
	delayMicroseconds(100);
}