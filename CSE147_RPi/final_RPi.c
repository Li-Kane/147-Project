#include "final_RPi.h"
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>

// Initialize the shard variable for the states that can be changed and read by all sensors
void init_shared_variable(SharedVariable* sv) {
    sv->bProgramExit = 0;
    sv->state = RUNNING;
    sv->nanoState = FAR;
}

// Initialize the LEDs
void ledInit(void) {
    softPwmCreate(PIN_SMD_RED, 0, 0xff);
    softPwmCreate(PIN_SMD_GRN, 0, 0xff);
    softPwmCreate(PIN_SMD_BLU, 0, 0xff);

    pinMode(PIN_DIP_RED, OUTPUT);
    pinMode(PIN_DIP_GRN, OUTPUT);
}

// Initialize the sensors and run the LED initialization method
void init_sensors(SharedVariable* sv) {
    pinMode(PIN_BUTTON, INPUT);
    int button;
    pinMode(PIN_LASER, OUTPUT);
    pinMode(PIN_SONIC_ECHO, INPUT);
    pinMode(PIN_SONIC_TRIG, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_NANO, INPUT);
    float dist;
    long travelTime;
    long startTime;
    int nano;
    ledInit();
}

// Button - When the button is pressed, we cahnge states from the opposite of the state that we are currently in. 
// This is used to control the LEDs and the laser emitter on the glove
void body_button(SharedVariable* sv) {
    int button = digitalRead(PIN_BUTTON);
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

// Jetson Nano INPUT - This is used to get a value from the Jetson Nano that tells if where we are pointing is too close.
// We change states if we are close or far and this will trigger the active buzzer
void body_nano(SharedVariable* sv){
    int nano = digitalRead(PIN_NANO);
    if(nano == 1){
	sv->nanoState = CLOSE;
    }
    else{
	sv->nanoState = FAR;
    }
}

// DIP two-color LED - This is used as the RED LED on the glove
// If we are in the running state then show red, otherwise turn off
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

// SMD RGB LED - This is used as the Green LED on the glove
// If we are in the running state then show green, otehrwise turn off
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

// LASER - This is used for showing the user in real life what they are exactly pointing at.
// It allows the user to understand what the measurement results are from.
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

// SONIC SENSOR - This is used to test distance using the ultrasonic sensor.
void body_sonic(SharedVariable* sv) {
    // We first need to write Low, then wait for a certain amount of time to have a clear signal
    digitalWrite(PIN_SONIC_TRIG, 0);
	delayMicroseconds(500);
    // We set the TRIG pin high so that we output a signal to recieve later, we
	digitalWrite(PIN_SONIC_TRIG, 1);
    delayMicroseconds(10);
    //We set the TRIG pin low again so that we can identify the signal that is 10 ms long
    digitalWrite(PIN_SONIC_TRIG, 0);
	printf("Waiting for echo \n");
    //We now sit here while we wait for the echo to read HIGH, meaning we have recieved the echo
	while(digitalRead(PIN_SONIC_ECHO) == 0){
    }
	printf("echo reading went high \n");
	long startTime = micros();
    //We now wait again for that signal to end
    while(digitalRead(PIN_SONIC_ECHO) == 1){
	}
    //Here we calculate the sitance by measuring from when we first recieve the signal to when the signal ends
    //We divide by 58 to get the distance in mm
	long travelTime = micros() - startTime;
    float dist = travelTime / 58;
	printf("Measured a dist of %f\n", dist);
	delayMicroseconds(1000000);
}

// Active Buzzer - This outputs a buzzing noise when we read that we are pointing 
// somewhere that is under the threshold defined in the jetson nano, we simplay use 
// the signal recieved from the nano
void body_buzzer(SharedVariable* sv) {
	switch(sv->nanoState){
		case CLOSE:
			digitalWrite(PIN_BUZZER, HIGH);
			break;
		case FAR:
			digitalWrite(PIN_BUZZER, LOW);
			break;
	}
}
